/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f7xx_hal.h"
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "sgtl5000.h"
#include "math.h"

I2C_HandleTypeDef hi2c2;

SAI_HandleTypeDef hsai_BlockA1;
SAI_HandleTypeDef hsai_BlockB1;

/* Private function prototypes -----------------------------------------------*/
//void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_SAI1_Init(void);
HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Master_Receive(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);
//HAL_StatusTypeDef x(SAI_HandleTypeDef *hsai, uint8_t* pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_DeInit(I2C_HandleTypeDef *hi2c);

//------------------------------------------
uint16_t i2c2_error_code, i2c2_error_address;

void i2c2_err(uint16_t code, uint16_t address)
{
	i2c2_error_code = code;
	i2c2_error_address = address;
	while(1);
}

volatile int ctr = 0;
uint16_t write_ctr, modify_ctr;

void read_register(uint16_t address, uint16_t * value)
{
	uint8_t reg_address[] = {0, 0};
	uint8_t reg_value[] = {0, 0};
	uint16_t r;

	reg_address[0] = address>>8; reg_address[1] = address;
	r = HAL_I2C_Master_Transmit(&hi2c2, 0x0A<<1, reg_address, 2, 1000);
	if (r != 0) i2c2_err(r, address);
	r = HAL_I2C_Master_Receive(&hi2c2, (0x0A<<1), reg_value, 2, 1000);
	if (r != 0) i2c2_err(r, address);
	*value = (uint16_t)reg_value[0]<<8 | (uint16_t)reg_value[1];
}

void write_register(uint16_t address, uint16_t value)
{
	uint8_t write_array[] = {0, 0, 0, 0};
	uint16_t r;

	write_array[0] = address>>8; write_array[1] = address;
	write_array[2] = value>>8; write_array[3] = value;
	do {r = HAL_I2C_Master_Transmit(&hi2c2, 0x0A<<1, write_array, 4, 1000);} while(r != HAL_OK);
	//if (r != 0) i2c2_err(r, address);
	write_ctr++;
}

void modify_register(uint16_t address, uint16_t clear_mask, uint16_t set_mask)
{
	uint16_t value;

	read_register(address, &value);
	value = (value & clear_mask) | set_mask;
	write_register(address, value);
	modify_ctr++;
}

void sgtl5000_init()
{
	write_ctr = modify_ctr = 0;
	write_register(SGTL5000_CHIP_LINREG_CTRL, 0x0008);			// set VDD to 1.2V
	write_register(SGTL5000_CHIP_ANA_POWER, 0x7260); 			// power up internal regulator
	write_register(SGTL5000_CHIP_LINREG_CTRL, 0x006C);			// configure charge pump
	write_register(SGTL5000_CHIP_REF_CTRL, 0x004E);				// set up bias current
	write_register(SGTL5000_CHIP_LINE_OUT_CTRL, 0x322);			// set up line out reference
	write_register(SGTL5000_CHIP_REF_CTRL, 0x004F);				// slow ramp up to minimize pop (bit 0)
	write_register(SGTL5000_CHIP_SHORT_CTRL, 0x1106);			// headphone short detect
	write_register(SGTL5000_CHIP_ANA_CTRL, 0x0133);				// zero-cross detect
	write_register(SGTL5000_CHIP_ANA_POWER, 0x6AFF);			// power analog I/O
	write_register(SGTL5000_CHIP_DIG_POWER, 0x0073);			// power digital I/O
	write_register(SGTL5000_CHIP_LINE_OUT_VOL, 0x0505);			// set line out level

	modify_register(SGTL5000_CHIP_CLK_CTRL, ~0x000C, 2<<2);		// set SYS_FS clock to 48kHz
	modify_register(SGTL5000_CHIP_CLK_CTRL, ~0x0003, 0);		// set MCLK_FREQ to 256*Fs
	modify_register(SGTL5000_CHIP_I2S_CTRL,	~0x0080, 1<<7); 	// set up as I2S master
	modify_register(SGTL5000_CHIP_ANA_POWER, ~0x0400, 1<<10); 	// power the PLL
	modify_register(SGTL5000_CHIP_ANA_POWER, ~0x0100, 1<<8); 	// power VCOAMP
	modify_register(SGTL5000_CHIP_PLL_CTRL, ~0xF800, 16<<11);	// divider integer part
	modify_register(SGTL5000_CHIP_PLL_CTRL, ~0x007F, 786<<0);	// divider fractional part

	// set up the STTL5000 input/output routing
#define ADC_2_DAC (0<<4)
#define I2S_2_DAC (1<<4)

#define ADC_2_I2S (0<<0)
#define I2S_2_I2S (1<<0)

	write_register(SGTL5000_CHIP_SSS_CTRL, ADC_2_I2S | I2S_2_DAC);	// ADC-> Nucleo -> DAC
	//write_register(SGTL5000_CHIP_SSS_CTRL, ADC_2_DAC);
	//write_register(SGTL5000_CHIP_SSS_CTRL, I2S_2_I2S);
	//
	modify_register(SGTL5000_CHIP_ANA_CTRL, ~0x0040, 0);		// DAC to headphone
	modify_register(SGTL5000_DAP_CTRL, ~0x0001, 0x0001);		// enable the DAP .. need to set up individual pass throughs

	// set up I2S
	modify_register(SGTL5000_CHIP_I2S_CTRL, ~0x0030, 3<<4);		// 16 bit I2S data length

	// set up volumes and then un-mute
	write_register(SGTL5000_CHIP_ANA_ADC_CTRL, 0x0000);			// set ADC l&r analog volume default
	modify_register(SGTL5000_CHIP_MIC_CTRL, ~0x0003, 0x0001);	// set mic gain to 20 dB
	write_register(SGTL5000_CHIP_ANA_HP_CTRL, 0x7F7F);			// set HP l&r min volume
	modify_register(SGTL5000_CHIP_ANA_CTRL, ~0x0010, 1<<4);		// unmute HP

	// add loop to ramp up HP volume .. DO

	// Lineout and DAC volume control
	modify_register(SGTL5000_CHIP_ANA_CTRL, ~0x0100, 0<<8);		// un-mute lineout?
	write_register(SGTL5000_CHIP_DAC_VOL, 0x3C3C);				// set DAC volume 0dB
	modify_register(SGTL5000_CHIP_ADCDAC_CTRL, ~0x0004, 0<<2);	// un-mute DAC left
	modify_register(SGTL5000_CHIP_ADCDAC_CTRL, ~0x0008, 0<<3);	// un-mute DAC right
	modify_register(SGTL5000_CHIP_ANA_CTRL, ~0x0004, 1<<2);		// line in to ADC
	modify_register(SGTL5000_CHIP_ANA_CTRL, ~0x0001, 0<0);		// un-mute ADC
}

//volatile uint16_t value[2];
volatile int16_t DAC_value[2], ADC_value[2];
//volatile left_value, right_value;  // check if really is left and right!!!!

static uint32_t loop_ctr = 0, value_ctr = 0;

#define NTABLE 16

volatile uint32_t tx_ctr = 0, rx_ctr = 0;
int16_t table[NTABLE];

void HAL_SAI_ErrorCallback(SAI_HandleTypeDef * hsai)
{
	while(1);
}

#define PI 3.14159265
#define FS 48000;


void CODEC_initialize(void)
{

  /* Initialize all configured peripherals */
  MX_GPIO_Init();	// init GPIO pins
  MX_I2C2_Init();	// init I2C2
  sgtl5000_init();	// init the NXP (Freescale) CODEC
  MX_SAI1_Init();	// init serial audio interface

  for (ctr=0; ctr<NTABLE; ctr++) {
	  table[ctr]=32767*sin(2*PI*ctr/NTABLE);
  }
  ctr = 0;
  loop_ctr = 0;
  value_ctr = 0;
}

//static volatile int16_t DAC_value[2], ADC_value[2];

void CODEC_run(void)
{
	HAL_SAI_Receive_IT(&hsai_BlockB1, (uint8_t*)&DAC_value[0], 2);
	HAL_SAI_Transmit_IT(&hsai_BlockA1, (uint8_t*)&DAC_value[0], 2);	// echo onto DAC
}


/* I2C2 init function */
static void MX_I2C2_Init(void)
{

	if (hi2c2.Instance == I2C2) HAL_I2C_DeInit(&hi2c2);

  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x20404768;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analog filter
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SAI1 init function */
static void MX_SAI1_Init(void)
{

  hsai_BlockA1.Instance = SAI1_Block_A;
  hsai_BlockA1.Init.AudioMode = SAI_MODESLAVE_TX;
  hsai_BlockA1.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockA1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockA1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockA1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockA1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockA1.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockA1.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  if (HAL_SAI_InitProtocol(&hsai_BlockA1, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_16BIT, 2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  hsai_BlockB1.Instance = SAI1_Block_B;
  hsai_BlockB1.Init.AudioMode = SAI_MODESLAVE_RX;
  hsai_BlockB1.Init.Synchro = SAI_SYNCHRONOUS;
  hsai_BlockB1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockB1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockB1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockB1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockB1.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockB1.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  if (HAL_SAI_InitProtocol(&hsai_BlockB1, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_16BIT, 2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PC1   ------> ETH_MDC
     PA1   ------> ETH_REF_CLK
     PA2   ------> ETH_MDIO
     PA7   ------> ETH_CRS_DV
     PC4   ------> ETH_RXD0
     PC5   ------> ETH_RXD1
     PB13   ------> ETH_TXD1
     PD8   ------> USART3_TX
     PD9   ------> USART3_RX
     PA8   ------> USB_OTG_FS_SOF
     PA9   ------> USB_OTG_FS_VBUS
     PA10   ------> USB_OTG_FS_ID
     PA11   ------> USB_OTG_FS_DM
     PA12   ------> USB_OTG_FS_DP
     PG11   ------> ETH_TX_EN
     PG13   ------> ETH_TXD0
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
  GPIO_InitStruct.Pin = RMII_MDC_Pin|RMII_RXD0_Pin|RMII_RXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin RMII_CRS_DV_Pin */
  GPIO_InitStruct.Pin = RMII_REF_CLK_Pin|RMII_MDIO_Pin|RMII_CRS_DV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RMII_TXD1_Pin */
  GPIO_InitStruct.Pin = RMII_TXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : STLK_RX_Pin STLK_TX_Pin */
  GPIO_InitStruct.Pin = STLK_RX_Pin|STLK_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_SOF_Pin USB_ID_Pin USB_DM_Pin USB_DP_Pin */
  GPIO_InitStruct.Pin = USB_SOF_Pin|USB_ID_Pin|USB_DM_Pin|USB_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_TX_EN_Pin RMII_TXD0_Pin */
  GPIO_InitStruct.Pin = RMII_TX_EN_Pin|RMII_TXD0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
