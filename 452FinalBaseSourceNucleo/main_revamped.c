/*
 * main_revamped.c
 *
 *      Author: Timothy Kenendy
 *      Purpose: Read codec input, apply effects, write to output
 *      Notes:
 *      -Toggling originally done with GPIO interrupt, now standard input to allow multiple efx
 *      -Effects include distortion, delay, stereo reverb, tremolo and chorus
 *      -The first four are implemented for the four cardinal directions
 *      	available as input from Wii processing
 *     - Last effect (chorus) produces output on initial use, but any clearing/switching
 *      	to another effect followed by return to chorus causes hardfault
 *
 *      MAIN TO-DOs IN FUTURE WORK:
 *      1) Debug serial communication via USART2 from Raspberry Pi
 *      	a) correctly recieve and interpret integer sent
 *      	b) scale integer to decimal between 0 and 1
 *      	c) apply multiplicatively to various parameters (gain, delay length, etc)
 *      2) Debug Chorus
 *      	a) using high quality debug tool, track memory in program
 *      	b) locate hardfault on stack
 *      	c) fix to protect RAM during execution
 *      3) Debug delay shifting
 *      	a) identify logic not allowing delay to shift
 */

#include <stdio.h>
#include <stdint.h>
#include "stm32f7xx_hal.h"
#include "arm_math.h"
#include "lowpass.h"
#include "highpass.h"

#include "main.h"

#define ASIZE 		244
#define BLOCK_SIZE 1
//#define PI 3.14159265
#define Dd 48000
#define Da 45000
#define Db 24000
#define DAP 80000
#define DP 40000
#define D1 2500
#define D2 300
#define D3 4500
#define D4 5000
#define D5 1500
#define D6 1200
#define L 5000
#define R 5000
#define ChL 4800
#define ChR 4800
#define aS 0.88
#define Dtremolo 192
#define Dchorus 4800
//our sampling rate
short fs = 48000;
//for uart
USART_HandleTypeDef huart2;
uint8_t data[10];
//delay ints
const int d = 24000;
int d1,d2;

//array for tremolo
float wtremolo[Dtremolo];

//array for chorus
float wchorus[Dchorus];

//buffer for delay
float wb[Db+1];

//buffer for left stereo reverb delay
float wL[L+1];

//buffer for right stereo reverb delay
float wR[R+1];

//buffer for left chorus delay
float wcL[ChL+1];

//buffer for right chorus delay
float wcR[ChR+1];

//pointers for all buffers
float *pb, *pL, *pR, *pcL, *pcR;

//ints for wave iterators
int qsin1;
int qsin2;
int qchorus1;
int qchorus2;

//flags
char UFlag = 0;
char DFlag = 0;
char LFlag = 0;
char RFlag = 0;
char DelayFlag = 0;
char LPFilter = 0;
char HPFilter = 0;

 //effect params
const float a=0.67,b=0.33,c=2048,alpha=0.7,beta=0.3;
float s0,sD;
const float b0=1, b1=0.6, b2=0.6, b3=0.6, b4=0.6;
const float a1=0.7, a2=0.7, a3=0.5, a4=0.3, a5=0.7, a6=0.7;
const float aL=0,aR=0,bL=0.7,bR=0.7,cL=0.6,cR=0.6,dL=0.5,dR=0.5;

float bin = 0;
float A = 2000;
float f1 = 250;
float f2 = 10;

void Startup(void);
void CODEC_read(int16_t *, int16_t *);	// read left/right values from ADC
void CODEC_write(int16_t, int16_t);		// write left/right value to DAC
void Pin_E10(uint16_t value);
//tims init funcs
void initializeEffectParameters(void);
static void MX_USART2_UART_Init(void);

//effect helper funcs
int16_t thresh3(int16_t in);
float *pwrap(int D, float *w, float *p);
int qwrap(int D, int q);
float wavgen(int D, float *w, float A, float F, int *q);
//effect algorithm funcs
void distortion3(int16_t inL, int16_t inR, int16_t *outL, int16_t *outR);
void delay(int16_t inL, int16_t inR, int16_t* outL, int16_t* outR, char flag);
//Prototypes left here for debugging...new delay works for now
//void delayleft(int16_t in, int16_t* out);
//void delayright(int16_t in, int16_t* out);
void reverbstereo(int16_t inL, int16_t inR, int16_t* outL, int16_t* outR);
void tremolo(int16_t inL, int16_t inR, int16_t* outL, int16_t* outR);
void chorus(int16_t inL, int16_t inR, int16_t* outL, int16_t* outR);

//arm functions for fft
void arm_fir_fast_q15(
  const arm_fir_instance_q15 * S,
  q15_t * pSrc,
  q15_t * pDst,
  uint32_t blockSize);

arm_status arm_fir_init_q15(
  arm_fir_instance_q15 * S,
  uint16_t numTaps,
  q15_t * pCoeffs,
  q15_t * pState,
  uint32_t blockSize);
static void MX_GPIO_Init(void);
void readInputs(void);

void main(void)
{
	int16_t right, left; //AIC inputs
	int16_t outL, outR; //outputs
	int16_t buffL, buffR; //buffer for multiples
	int16_t outLLP, outRLP;
	int16_t outLHP, outRHP;

		// LOW PASS Filter
			q15_t ARM_buffer_LP[LPL+BLOCK_SIZE];
			arm_fir_instance_q15 LPF;
			arm_status LPstatus;

		// HIGH PASS Filter
			q15_t ARM_buffer_HP[HPL+BLOCK_SIZE];
			arm_fir_instance_q15 HPF;
			arm_status HPstatus;
			uint16_t ntapsLP = LPL;
				uint16_t ntapsHP = HPL;
	Startup();
	MX_USART2_UART_Init();
	MX_GPIO_Init();
	initializeEffectParameters();
	//LP INIT
		if ((ntapsLP&0x1) != 0) ntapsLP++; // if odd, add one ... add into coefficient list!!!

		LPstatus = arm_fir_init_q15(	// initialize the filter structure
					&LPF,
					ntapsLP,		// number of taps === assumes odd makes even
					(q15_t *) &LP[0],		// points to coefficients
					(q15_t *) &ARM_buffer_LP[0], // points to data buffer
					(uint32_t) BLOCK_SIZE);			// block size
		//HP INIT
			if ((ntapsHP&0x1) != 0) ntapsHP++; // if odd, add one ... add into coefficient list!!!

				HPstatus = arm_fir_init_q15(	// initialize the filter structure
							&HPF,
							ntapsHP,		// number of taps === assumes odd makes even
							(q15_t *) &HP[0],		// points to coefficients
							(q15_t *) &ARM_buffer_HP[0], // points to data buffer
							(uint32_t) BLOCK_SIZE);			// block size
		//LED_Green_On();


	while(1)
	{
		int i;
		//does not work, bin not applied anywhere
		HAL_USART_Receive_IT(&huart2,data,10);
		bin = (float) data[0] / 100;
		//small delay
		if(i>=ASIZE) i=0;
		Pin_E10(1);
		readInputs();
		CODEC_read(&left, &right);

		//flag logic; flags set via gpio interrupt
		//Up flag logic with other flags
		if(UFlag != 0)
		{
			if(DFlag != 0)
			{
				distortion3(buffL,buffR,&outL,&outR);
				reverbstereo(left,right,&buffL,&buffR);
			}
			else if(LFlag != 0)
			{
				distortion3(left,right,&buffL,&buffR);

				//if(DelayFlag != 0){
				delay(buffL,buffR,&outL,&outR,DelayFlag);
				//outR = buffR;
				//delay shifting disabled for multiple efx
				/*}
				else{
				outL = buffL;
				delayright(buffR,&outR);}
				*/
			}
			else if(RFlag != 0)
			{
				distortion3(left,right,&buffL,&buffR);
				tremolo(buffL,buffR,&outL,&outR);
			}
			else {distortion3(left,right,&outL,&outR);}
		}

		else if(DFlag != 0)
		{
			if(UFlag != 0)
			{
				reverbstereo(left,right,&buffL,&buffR);
				distortion3(buffL,buffR,&outL,&outR);

			}
			else if(LFlag != 0)
			{
				reverbstereo(left,right,&buffL,&buffR);

				//if(DelayFlag != 0){
				delay(buffL,buffR,&outL,&outR,DelayFlag);
				//delayleft(buffL,&outL);
				//outR = buffR;
				//delay shifting disabled for multiple efx
				/*}
				else{
				outL = buffL;
				delayright(buffR,&outR);}
				*/
			}
			else if(RFlag != 0)
			{
				reverbstereo(left,right,&buffL,&buffR);
				tremolo(buffL,buffR,&outL,&outR);
			}
			else{reverbstereo(left,right,&outL,&outR);}
		}

		else if(LFlag != 0)
		{
			delay(left,right,&outL,&outR,DelayFlag);
			/*
			if(DelayFlag != 0){
			delayleft(left,&outL);
			outR = right;
			}
			else{
			outL = left;
			delayright(right,&outR);
			}
			*/
		}

		else if(RFlag != 0)
		{
			tremolo(left,right,&outL,&outR);
			//chorus(left,right,&outL,&outR);
		}

		else
		{
			outL = left;
			outR = right;
		}
		//apply arm fir
		//LP output
		if(LPFilter != 0)
		{
				arm_fir_fast_q15(
					&LPF,
					(q15_t *) &outL,	// points to input
					(q15_t *) &outLLP,	// points to output
					(uint32_t) BLOCK_SIZE);	// block size of 1 //**
				arm_fir_fast_q15(
					&LPF,
					(q15_t *) &outR,	// points to input
					(q15_t *) &outRLP,	// points to output
					(uint32_t) BLOCK_SIZE);	// block size of 1 //**
				CODEC_write(outLLP,outRLP);
		}

				//HP output
		else if(HPFilter != 0)
		{
				arm_fir_fast_q15(
					&HPF,
					(q15_t *) &outL,	// points to input
					(q15_t *) &outLHP,	// points to output
					(uint32_t) BLOCK_SIZE);	// block size of 1 //**
				arm_fir_fast_q15(
					&HPF,
					(q15_t *) &outR,	// points to input
					(q15_t *) &outRHP,	// points to output
					(uint32_t) BLOCK_SIZE);	// block size of 1 //**
				CODEC_write(outLHP,outRHP);
		}
		else
		{
				CODEC_write(outL,outR);
		}
		 Pin_E10(0);
		i++;
	}
}
void readInputs()
{
	UFlag = HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_2);
	DFlag = HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_3);
	LFlag = HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_9);
	RFlag = HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_11);
	DelayFlag = HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_10);
	LPFilter = HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_7);
	HPFilter = HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_8);
}
void initializeEffectParameters()
{
	int i,n;
//initialize delay lines, set pointers
for (n=0; n<=Db; n++) {wb[n] = 0;}
for (n=0; n<=L; n++) {wL[n] = 0;}
for (n=0; n<=R; n++) {wR[n] = 0;}
for (n=0; n<=ChL; n++) {wcL[n] = 0;}
for (n=0; n<=ChR; n++) {wcR[n] = 0;}
pb = wb; pL = wL; pR = wR; pcL = wcL; pcR = wcR;
//initialize waveforms, set iterators
for (i=0; i<Dtremolo; i++) wtremolo[i] = sin(2*PI*i/Dtremolo);
for (i=0; i<Dchorus; i++) wchorus[i] = sin(2*PI*i/Dchorus);
qsin1 = 0;
qsin2 = 0;
qchorus1 = 0;
qchorus2 = 10;
}

float *pwrap(int D, float *w, float *p)
{
if (p > w+D)
p -= D+1;
if (p < w)
p += D+1;
return p;
}
// ------------------------------------------------------
// qwrap.c - circular index wrapping
// Usage: q_new = qwrap(D,q);
// -------------------------------------
int qwrap(int D, int q)
{
if (q > D)
q -= D + 1;
if (q < 0)
q += D + 1;
return q;
}
// -------------------------------------
//old delays left for debugging if new delay fails
/*
void delayleft(int16_t in, int16_t* out){
    float x = (float) in; // work with left input only
    float y = *pwrap(Dd,wb,pb+d); // delayed output
    *pb = x; // delay-line input
    pb = pwrap(Dd,wb,--pb); // backshift pointer
    *out = (int16_t) y;
}
void delayright(int16_t in, int16_t* out){
    float x = (float) in; // work with right input only
    float y = *pwrap(Dd,wb,pb+d); // delayed output
    *pb = x; // delay-line input
    pb = pwrap(Dd,wb,--pb); // backshift pointer
    *out = (int16_t) y;
}
*/
void delay(int16_t inL, int16_t inR, int16_t* outL, int16_t* outR, char flag){
    //if delay flag, the left speaker is delayed
    if(flag != 0)
    {
    float xl = (float) inL; // work with left input only
    float yl = *pwrap(Dd,wb,pb+d); // delayed output
    *pb = xl; // delay-line input
    pb = pwrap(Dd,wb,--pb); // backshift pointer
    *outL = (int16_t) yl;
    *outR = inR;
    }
    //else the right speaker is delayed
    else
    {
    float xr = (float) inR; // work with left input only
    float yr = *pwrap(Dd,wb,pb+d); // delayed output
    *pb = xr; // delay-line input
    pb = pwrap(Dd,wb,--pb); // backshift pointer
    *outR = (int16_t) yr;
    *outL = inL;
    }

}
// ------------------------------------------------------------------------------------
// stereo reverb
//
void reverbstereo(int16_t inL, int16_t inR, int16_t* outL, int16_t* outR)
{
	float sL, sR;
		float xL = (float) inL;
		float xR = (float) inR;
		float yL,yR;
	sL = *pwrap(L,wL,pL+L);
	sR = *pwrap(R,wR,pR+R);
	yL = cL*xL + sL;
	yR = cR*xR + sR;
	*pL = bL*xL + aL*sL + dR*sR;
	*pR = bR*xR + aR*sR + dL*sL;
	pL = pwrap(L,wL,--pL);
	pR = pwrap(R,wR,--pR);
	*outL = (int16_t) yL;
	*outR = (int16_t) yR;
}
// -------------------------------------------------
// wavgen.c - wavetable generator
// Usage: y = wavgen(D,w,A,F,&q);
// ------------------------------------------------
float wavgen(int D, float *w, float A, float F, int *q) //w[] = sin(2*PI*i/D), i = 0:D-1
{
float y, c=D*F;
y = A * w[*q];
*q = qwrap(D-1, (int) (*q+c));
return y;
}

void tremolo(int16_t inL, int16_t inR, int16_t* outL, int16_t* outR)
{
    float y1,y2;
    float x1 = (float) inL;
    float x2 = (float) inR;
    y1 = beta * x1 + alpha * wavgen(Dtremolo, wtremolo, x1, f1/fs, &qsin1);
    y2 = beta * x2 + alpha * wavgen(Dtremolo, wtremolo, x2, f1/fs, &qsin2);
    *outL = (int16_t) y1;
    *outR = (int16_t) y2;
}


void chorus(int16_t inL, int16_t inR, int16_t* outL, int16_t* outR)
{
	float y1,y2,sL1,sL2,sR1,sR2;
	float x1 = (float) inL;
	float x2 = (float) inR;
	d1 = (int) ceil(wavgen(Dchorus, wchorus, 200, f1/fs, &qchorus1));
	d2 = (int) ceil(wavgen(Dchorus, wchorus, 200, f2/fs, &qchorus2));
	sL1 = *pwrap(ChL,wcL,pcL+d1); // delayed output
	sR1 = *pwrap(ChR,wcR,pcR+d1); // delayed output
	sL2 = *pwrap(ChL,wcL,pcL+d2); // delayed output
	sR2 = *pwrap(ChR,wcR,pcR+d2); // delayed output
	y1 =  0.2 * x1 + 0.6 * sL2 + 0.4 * sL1;
	y2 =  0.2 * x2 + 0.6 * sR2 + 0.4 * sR1;
	*pcL = x1; // delay-line input
	*pcR = x2; // delay-line input
	 pcL = pwrap(ChL,wcL,--pcL); // backshift pointer
	 pcR = pwrap(ChR,wcR,--pcR); // backshift pointer
	 *outL = (int16_t) y1;
	 *outR = (int16_t) y2;
}


void distortion3(int16_t inL, int16_t inR, int16_t *outL, int16_t *outR){
	*outL =  (int16_t) ((float) thresh3(inL) ); //+ ((1-gain) * inL);
		//*outL = *outL >> 1;
	*outR =  (int16_t) ((float) thresh3(inR) );
		//*outR = *outR >> 1;
}

int16_t thresh3(int16_t in)
{
	return ((in >> 12) << 12);
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */

  //__HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
/*
  //Configure GPIO pins : PB9
  GPIO_InitStruct.Pin = GPIO_PIN_9;//GPIO_PIN_12|GPIO_PIN_3|GPIO_PIN_4|
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
*/
  /*Configure GPIO pins : PC9 PC10 PC11 */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;//|GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;//IT_RISING_FALLING;//
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PG2 PG3 */
      GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
      GPIO_InitStruct.Mode = GPIO_MODE_INPUT;//IT_RISING_FALLING;//
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
  /*Configure GPIO pins : PF7 PF8 */
      GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8;
      GPIO_InitStruct.Mode = GPIO_MODE_INPUT;//IT_RISING_FALLING;//
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);


      //  Initialize GPIO port E test pins
      	//
      	__GPIOE_CLK_ENABLE();
      	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
      	GPIO_InitStruct.Pin =  GPIO_PIN_7;//|GPIO_PIN_14|GPIO_PIN_12|GPIO_PIN_10;
      	GPIO_InitStruct.Pull = GPIO_NOPULL;
      	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
      	HAL_GPIO_Init(GPIOE,&GPIO_InitStruct);

      /* EXTI interrupt init*/
        HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(EXTI2_IRQn);

        HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(EXTI3_IRQn);

        HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

        HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

static void MX_USART2_UART_Init(void)
{


  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_RX;
  //huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  //huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  //huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_USART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}
//TODO: THE GRAVEYARD, AKA ALL OLD CODE
//Not needed, interrupt handling moved to stm32f7xx_it.c
/*
void HAL_GPIO_EXTI_Callback ( uint16_t GPIO_Pin )
	{
	//int n;

	if( GPIO_Pin == GPIO_PIN_2 ){
			UFlag = HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_2);
		}

	 if( GPIO_Pin == GPIO_PIN_3 ){
			DFlag = HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_3);
		}
	else if( GPIO_Pin == GPIO_PIN_9 ){
			LFlag = HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_9);
		}
	else if( GPIO_Pin == GPIO_PIN_10 ){
			DelayFlag = HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_10);
		}
	else if( GPIO_Pin == GPIO_PIN_11 ){
			RFlag = HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_11);
		}
	}
*/

/*
a = 0.67;
	b = (1-a);
	c = 2048;
	alpha = a;
	beta = b;
	int n;
	aL = 0;
	bL = aL;
	bL = 0.8;
	bR = bL;
	cL = 0.5;
	cR = cL;
	dL = 0.5;
	dR = dL;
*/

//float wPlain[DP+1], *pPlain;

//float wAllPass[DAP+1], *pAllPass;
//float wPlain[DP+1], *pPlain;
/*
float w1[D1+1], *p1;
float w2[D2+1], *p2;
float w3[D3+1], *p3;
float w4[D4+1], *p4;
float w5[D5+1], *p5;
float w6[D6+1], *p6;
*/
/*
void flanger(int16_t inL, int16_t inR, int16_t* outL, int16_t* outR)
{
    float wd = 2*PI*F1;
    float sd1,sd2;
    float x1 = (float) inL;
    float x2 = (float) inR;
    d = (1 - cos(wd*n))*D/2; // automatically cast to int, wd = 2*PI*fd/fs
    if (++n>=L) n=0; // L = 16000 to allow fd = 0.5 Hz
    sd = *pwrap(D,w,p+d); // extract d-th state relative to p
    y = x + a*sd; // output
    *p = y; // delay-line input
    p = pwrap(D,w,--p); // backshift pointer
    *outL = *outR = (short) y;
    return;
}
*/
// -------------------------------------------------
/*
interrupt void isr()
{
float y; // filter input & output
//read_inputs(&xL, &xR); // codec inputs are not used
y = wavgen(D, w, A, f/fs, &q);
yL = yR = (short) y;
write_outputs(yL,yL);
return;
}
*/
/*
void reverbdelay(int16_t inL, int16_t inR, int16_t* outL, int16_t* outR)
{
	float x = (float) inL; // process left channel only
	sD = *pwrap(Dd,w,p+Dd); // extract states relative to p
	float y = c*x + sD; // output sample
	*p = b*x + a*sD; // delay-line input
	p = pwrap(Dd,w,--p); // backshift pointer
	*outL = *outR = (int16_t) y;
	 // write outputs to codec
	return;
}

void multitapdelay(int16_t inL, int16_t inR, int16_t* outL, int16_t* outR) // sample processing algorithm - interrupt service routine
{
float x, s0, s1, s2, y;
x = (float) inL; // process left channel only
s1 = *pwrap(D1+D2, wMultiTap, pMultiTap+D1);
s2 = *pwrap(D1+D2, wMultiTap, pMultiTap+D1+D2);
y = b0*x + b1*s1 + b2*s2;
s0 = x + a1*s1 + a2*s2;
*p = s0;
p = pwrap(D1+D2, w, --p);
*outL = *outR = (int16_t) y;
}
*/
/*
void distortion(int16_t inL, int16_t inR, int16_t *outL, int16_t *outR){
	*outL =  (int16_t) thresh(inL);
		*outL = *outL >> 1;
	*outR = (int16_t) thresh(inR);
		*outR = *outR >> 1;
}
void distortion1(int16_t inL, int16_t inR, int16_t *outL, int16_t *outR){
	*outL =  thresh1(inL);
		*outL = *outL << 1;
	*outR =  thresh1(inR);
		*outR = *outR << 1;
}
void distortion2(int16_t inL, int16_t inR, int16_t *outL, int16_t *outR){
	*outL =  thresh2(inL);
		*outL = *outL << 1;
	*outR = inR;
		*outR = *outR << 1;
}
*/
/*
float thresh(int16_t x)
{
	float y;
	float xc = x/c; // this y is local to f()
	y = x * (1 - b * xc *xc);

	if(x<c) y=a*c;
	if(x<-c) y = -a*c;

	return y;
}
int16_t thresh1(int16_t in)
{
	float y;
	float x = (float) in;
	float xc = x/c; // this y is local to f()
	y = x * (1 - b * xc *xc);

	if(x<c) y=a*c;
	if(x<-c) y = -a*c;

	return (int16_t) y;
}
int16_t thresh2(int16_t in)
{
	float y;
	float x = (float) in;

	if(x>0) y= 1 - (float) exp(-x);
	else y = -1 + (float) exp(x);

	return (int16_t) y;
}
*/
//or (n=0; n<=DP; n++) {wPlain[n] = 0;}
//    pPlain = wPlain;
/*
for (n=0; n<=D1+D2; n++) // initialize circular buffer to zero
    {wMultiTap[n] = 0;}
    for (n=0; n<=DAP; n++) // initialize circular buffer to zero
    {wAllPass[n] = 0;}
    pAllPass = wAllPass;
    for (n=0; n<=DP; n++) // initialize circular buffer to zero
    {wPlain[n] = 0;}
    pPlain = wPlain;
	for (n=0; n<=D1; n++) w1[n] = 0; // initialize buffers to zero
	*/
/*
// -------------------------------------------------------------------------------
// allpass_tr.c - allpass reverb with circular delay line - transposed realization
// -------------------------------------------------------------------------------
float reverballpass(int DAllPass, float *wAllPass, float *pAllPass, float a, int16_t inL)
{
    float x = (float) inL;
    float y, sD;
    sD = *pwrap(DAllPass,wAllPass,pAllPass+DAllPass);
    y = sD - a*x;
    *pAllPass = x + a*y;
    pAllPass = pwrap(DAllPass,wAllPass,--pAllPass);
    return y; //CAST TO INT16_T in MAIN IF USED ALONE
}
// ------------------------------------------------------
// plain.c - plain reverb with circular delay line
// ------------------------------------------------------

float reverbplain(int DPlain, float *wPlain, float *pPlain, float a, int16_t inL)
{
    float x = (float) inL;
    float y, sD;
    sD = *pwrap(DPlain,wPlain,pPlain+DPlain);
    y = x + a * sD;
    *pPlain = y;
    pPlain = pwrap(DPlain,wPlain,--pPlain);
    return y; //CAST TO INT16_T in MAIN IF USED ALONE
}
*/
// -------------------------------------------------------------------------------
// ------------------------------------------------------
// --------------------------------------------------------------------
// schroeder.c - Schroeder’s reverb algorithm using circular buffers
// --------------------------------------------------------------------



// ----------------------------------------------------------------------------------
/*
void reverbschroeder(int16_t in, int16_t* out)
    {
        float x = (float) in;
        // ------------------------------------------------------------
        // x1,x2,x3,x4=plain(xi,di,pi,wi,ai)
		float x1 = reverbplain(D1,w1,p1,a1,x);
		float x2 = reverbplain(D2,w2,p2,a2,x);
		float x3 = reverbplain(D3,w3,p3,a3,x);
		float x4 = reverbplain(D4,w4,p4,a4,x);
		// x5 = bi*xi
		float x5 = x1*b1 + x2*b2 + x3*b3 + x4*b4;
		// x6 = allpass x5
		float x6 = reverballpass(D5,w5,p5,a5,x5);
		// y = allpass x6
		float y = reverballpass(D6,w6,p6,a6,x6);
        // ------------------------------------------------------------
        *out = (int16_t) y;
    }
*/
//jonk
/*
	  //INIT FIR STRUCTURE
		//Priming the PUMP	.. // circular array with arithmetic boundary checks
			//PUMP FOR OLD FIR
		for(i = 0; i < (ASIZE); i++)
			{
				CODEC_read(&left, &right);
				in[i] = right;
			}
*/

