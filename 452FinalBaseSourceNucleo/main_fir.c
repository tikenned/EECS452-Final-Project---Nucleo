/*
 * fir_filter.c
 *
 *      Author: GSI
 */

#include <stdio.h>
#include <stdint.h>
#include "arm_math.h"
#include "lowpass.h"
//#include "lowpasshitap.h"

#define ASIZE 		244
#define BLOCK_SIZE 1

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

void Pin_E10(uint16_t value);


int16_t in [ASIZE];
const int32_t mask = ASIZE - 1;

/* OLD FIR FUNC; KEEP FOR DEBUG
int16_t FIR(uint16_t i)
{
	int32_t sum;
	uint16_t j;
	uint16_t index;

	Pin_E10(1);

	sum=1;

	//The actual filter work
	for(j=0; j<LPL; j++)
	{
		index = (ASIZE + i - j) & mask;
		sum += (int32_t)in[index] * (int32_t)LP[j];
	}
	sum = sum + 0x00004000;			// So we round rather than truncate.

	Pin_E10(0);

	return (int16_t) (sum >> 15);  	// Conversion from 32 Q30 to 16 Q15.

} */


void Startup(void);
void CODEC_read(int16_t *, int16_t *);	// read left/right values from ADC
void CODEC_write(int16_t, int16_t);		// write left/right value to DAC

void main(void)
{
	char on;
	uint16_t i;
	int16_t gain = 0xBBBBBBBBBBBBBBBB;
	int16_t right, left; //AIC inputs
	int16_t outL, outR;
	int32_t sample;

	//int D = 2500;
	//float w[D+1], *p;
	//float sD;

	// LOW PASS Filter
		q15_t ARM_buffer_LP[LPL+BLOCK_SIZE];
		arm_fir_instance_q15 LPF;
		arm_status status;

		uint16_t ntapsLP = LPL;

	Startup();
	//INIT FIR STRUCTURE
	if ((ntapsLP&0x1) != 0) ntapsLP++; // if odd, add one ... add into coefficient list!!!

		status = arm_fir_init_q15(	// initialize the filter structure
					&LPF,
					ntapsLP,		// number of taps === assumes odd makes even
					(q15_t *) &LP[0],		// points to coefficients
					(q15_t *) &ARM_buffer_LP[0], // points to data buffer
					(uint32_t) BLOCK_SIZE);			// block size
		if (status !=ARM_MATH_SUCCESS) {
			LED_Red_On();
			while(1);
		}

		initializeEffectParameters();
		// Indicate Successful Init
		LED_Green_On();
	//Priming the PUMP	.. // circular array with arithmetic boundary checks
	//PUMP FOR OLD FIR
	/*for(i = 0; i < (ASIZE); i++)
	{
		CODEC_read(&left, &right);
		in[i] = right;
	}
	*/
	while(1)
	{
		LED_Green_Toggle();

		if(i>=ASIZE) i=0;
		CODEC_read(&left, &right);
		distortion(&left,&right,&outL,&outR);
		//reverb(&right,&outR,D,&w,&p,sD);

		//apply arm fir
		/*
		arm_fir_fast_q15(
				&LPF,
				(q15_t *) &right,	// points to input
				(q15_t *) &out,	// points to output
				(uint32_t) BLOCK_SIZE);	// block size of 1 //**
*/
		//apply distortion TEST
		/*arm_mult_q15(
				&left, //points to input
				&gain,  //points to gain value
				&out,
				(uint32_t) BLOCK_SIZE);*/
		//OLD FIR FOR DEBUG
		//in[i] = right;
		//out = FIR(i);
		//out = symclip(right);
		//right = out;
		//POSTFILTER:
		//left = out;
		//right = out;
		//right = out;
		 if(UserButton())
		 {CODEC_write(left,right);}
		 else
		{CODEC_write(outL, outR);}
		i++;
	}
}
