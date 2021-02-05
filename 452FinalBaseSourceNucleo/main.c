/*
 * This file is part of the µOS++ distribution.
 *   (https://github.com/micro-os-plus)
 * Copyright (c) 2014 Liviu Ionescu.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom
 * the Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

// ----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"

#include "stm32f7xx.h"
#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_gpio.h"
#include "stm32f7xx_hal_rcc_ex.h"



// ----------------------------------------------------------------------------
//
// Standalone STM32F7 empty sample (trace via DEBUG).
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the DEBUG output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

void Startup(void);
void LED_Red_On();
void LED_Red_Off();
void LED_Green_On();
void LED_Green_Off();
void LED_Blue_On();
void LED_Blue_Off();

int
main(int argc, char* argv[])
{
	  Startup();

	  // Start this at 0, will change later in lab
	  int loop_select = 1;

	  // Infinite loop
	  while (1)
	    {
		  // TODO: Student Loop
		  if (loop_select == 0) {
	       // TODO: Fill in your code from the pre-lab here
	       trace_printf ("cycle\n");
		  }
		  // Instructor Loop
		  else {
			  LED_Red_On();
			  HAL_Delay(500);
			  trace_printf("%p\n", GPIOB->ODR);
			  LED_Red_Off();
			  LED_Green_On();
			  HAL_Delay(500);
			  trace_printf("%p\n", GPIOB->ODR);
			  LED_Green_Off();
			  LED_Blue_On();
			  HAL_Delay(500);
			  trace_printf("%p\n", GPIOB->ODR);
			  LED_Blue_Off();
			  trace_printf("cycle\n");
		  }
	    }
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------