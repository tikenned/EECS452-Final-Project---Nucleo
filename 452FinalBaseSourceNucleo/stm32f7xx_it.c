/**
  ******************************************************************************
  * @file    stm32f7xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
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
#include "stm32f7xx_hal.h"
#include "stm32f7xx.h"
#include "stm32f7xx_it.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern SAI_HandleTypeDef hsai_BlockA1;
extern SAI_HandleTypeDef hsai_BlockB1;
extern char UFlag,DFlag,LFlag,RFlag,DelayFlag;

/******************************************************************************/
/*            Cortex-M7 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f7xx.s).                    */
/******************************************************************************/


// @brief This function handles SAI1 global interrupt.
void SAI1_IRQHandler(void)
{
  /* USER CODE BEGIN SAI1_IRQn 0 */

	/* USER CODE END SAI1_IRQn 0 */
	//HAL_SAI_IRQHandler(&hsai_BlockA1);
	if ((hsai_BlockA1.Instance->SR) & SAI_xSR_FREQ) HAL_SAI_IRQHandler(&hsai_BlockA1);  // KM
	if ((hsai_BlockB1.Instance->SR) & SAI_xSR_FREQ) HAL_SAI_IRQHandler(&hsai_BlockB1);  // KM

  /* USER CODE BEGIN SAI1_IRQn 1 */

  /* USER CODE END SAI1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/**
  * @brief This function handles System tick timer.
  */
//Not needed, after timing testing move was made to GPIO input mode to allow multiple effects
/*
void EXTI2_IRQHandler(void)
{

if(HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_2))
		{
		UFlag = 1;
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,UFlag);
		}
else
		{
		UFlag = 0;
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,UFlag);
		}

  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);

}

// @brief This function handles EXTI line3 interrupt.

void EXTI3_IRQHandler(void)
{

	if(HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_3))
			{
			DFlag = 1;
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,DFlag);
			}
	else
			{
			DFlag = 0;
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,DFlag);
			}

  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);

}
void EXTI9_5_IRQHandler(void)
{

	if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_9))
				{
				LFlag = 1;
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,LFlag);
				}
		else
				{
				LFlag = 0;
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,LFlag);
				}

	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);

}
 // @brief This function handles EXTI line[15:10] interrupts.

void EXTI15_10_IRQHandler(void)
{

	if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_10))
					{
					DelayFlag = 1;
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,DelayFlag);
					}
			else
					{
					DelayFlag = 0;
					//HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,DelayFlag);
					}
	if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_11))
					{
					RFlag = 1;
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,RFlag);
					}
			else
					{
					RFlag = 0;
					//HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,RFlag);
					}

	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_11);
 // HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
}
*/
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
