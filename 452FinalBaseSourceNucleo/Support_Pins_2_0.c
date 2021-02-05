// File name: Pin_suport_2_0.c
//
// 06Sep2014 .. initial version .. K.Metzger
// 29Oct2014 .. removed implemented functions .. K.Metzger
// 27Apr2015 .. updated to new HAL .. KM
// 25Jan2016 .. added Port C pins to support 4-bit DAC .. KM
// 01Feb2016 .. fixed bug in LED_Red_Off .. KM
// 04Jan2018 .. modified for EECS 452 Nucleo .. KM

#ifdef STM32F746xx
#include "stm32f7xx.h"
#include "stm32f7xx_hal.h"
#endif

#include <stdint.h>

static GPIO_InitTypeDef GPIOB_InitStructure;
static GPIO_InitTypeDef GPIOC_InitStructure;
static GPIO_InitTypeDef GPIOE_InitStructure;

// Initializes:  LED pins
//               push button
//               test output pins

void Pins_initialize()
{
	//  Initialize GPIO port B LED pins
	//
	__GPIOB_CLK_ENABLE();
	GPIOB_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	// pin assignment value.
	GPIOB_InitStructure.Pin = GPIO_PIN_0 | GPIO_PIN_7 | GPIO_PIN_14;
	GPIOB_InitStructure.Pull = GPIO_NOPULL;
	GPIOB_InitStructure.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOB,&GPIOB_InitStructure);

	// Initialize GIPO port C pin 13 .. user switch
	//
	__GPIOC_CLK_ENABLE();
	GPIOC_InitStructure.Pin = GPIO_PIN_13;
	GPIOC_InitStructure.Mode = GPIO_MODE_INPUT;
	GPIOC_InitStructure.Speed = GPIO_SPEED_MEDIUM;
	GPIOC_InitStructure.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOC, &GPIOC_InitStructure);

	//  Initialize GPIO port E test pins
	//
	__GPIOE_CLK_ENABLE();
	GPIOE_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIOE_InitStructure.Pin =  GPIO_PIN_15|GPIO_PIN_14|GPIO_PIN_12|GPIO_PIN_10;
	GPIOE_InitStructure.Pull = GPIO_NOPULL;
	GPIOE_InitStructure.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOE,&GPIOE_InitStructure);
}

// TODO: Fill out the LED functions below with your answers from the prelab
void LED_Green_On()
{
	GPIOB->ODR |= GPIO_PIN_0;
}

void LED_Green_Off()
{
	GPIOB->ODR &= ~GPIO_PIN_0;
}

void LED_Green_Toggle()
{
	GPIOB->ODR ^= GPIO_PIN_0;
}

void LED_Red_On()
{
	GPIOB->ODR |= GPIO_PIN_14;
}

void LED_Red_Off()
{
	GPIOB->ODR &= ~GPIO_PIN_14;
}

void LED_Red_Toggle()
{
	GPIOB->ODR ^= GPIO_PIN_14;
}

void LED_Blue_On()
{
	GPIOB->ODR |= GPIO_PIN_7;
}

void LED_Blue_Off()
{
	GPIOB->ODR &= ~GPIO_PIN_7;
}

void LED_Blue_Toggle()
{
	GPIOB->ODR ^= GPIO_PIN_7;
}

// End TODO: Fill out LED Functions

// test pins

void Pin_E10(uint16_t value)
{
	if (value != 0) GPIOE->ODR |= GPIO_PIN_10;
	else GPIOE->ODR &= ~GPIO_PIN_10;
}

void Pin_E12(uint16_t value)
{
	if (value != 0) GPIOE->ODR |= GPIO_PIN_12;
	else GPIOE->ODR &= ~GPIO_PIN_12;
}

void Pin_E14(uint16_t value)
{
	if (value != 0) GPIOE->ODR |= GPIO_PIN_14;
	else GPIOE->ODR &= ~GPIO_PIN_14;
}

void Pin_E15(uint16_t value)
{
	if (value != 0) GPIOE->ODR |= GPIO_PIN_15;
	else GPIOE->ODR &= ~GPIO_PIN_15;

}

uint16_t UserButton()
{
	return GPIOC->IDR & 0x13;
}


