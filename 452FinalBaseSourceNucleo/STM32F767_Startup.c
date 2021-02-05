// File name: STM32F746_Startup.c

#include <stdint.h>

void Pins_initialize();
void CODEC_initialize(void);
void CODEC_run(void);
void Pin_E12(uint16_t value);
void Pin_E14(uint16_t value);
#include "stm32f7xx_hal.h"

SAI_HandleTypeDef hsai_BlockA1;
SAI_HandleTypeDef hsai_BlockB1;


void Startup(void)
{
	Pins_initialize();				// configure GPIO pins
	// TODO: Uncomment the two lines below when you are on the FIR section
	CODEC_initialize();				// initialize CODEC
	CODEC_run();					// start CODEC running
}

// DAC wants more data

int16_t ramp;

static int16_t DAC_value[2];

void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef * hsai)
{
	Pin_E14(1);
	//ramp+=1365;		// generate 1 kHz ramp for testing
	//DAC_value[0] = DAC_value[1] = ramp;
	HAL_SAI_Transmit_IT(&hsai_BlockA1, (uint8_t*)&DAC_value[0], 2);	// send to DAC
	Pin_E14(0);
}

void CODEC_write(int16_t left, int16_t right)
{
	DAC_value[0] = left;
	DAC_value[1] = right;
	Pin_E14(0);
}

// ADC has data to go

static int16_t volatile ADC_ready_flag, ADC_value[2], left_value, right_value;

void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef * hsai)
{
	Pin_E12(1);
	HAL_SAI_Receive_IT(&hsai_BlockB1, (uint8_t*)&ADC_value[0], 2);
	left_value = ADC_value[0]; right_value = ADC_value[1];
	ADC_ready_flag = 1;
	Pin_E12(0);
}

void CODEC_read(int16_t *right, int16_t *left)
{
	while (ADC_ready_flag == 0);
	*left = left_value;
	*right = right_value;
	ADC_ready_flag = 0;
	Pin_E12(0);
}
