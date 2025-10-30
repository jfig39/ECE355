//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------
// School: University of Victoria, Canada.
// Course: ECE 355 "Microprocessor-Based Systems".
// This is template code for Part 2 of Introductory Lab, UPDATED.
//
// See "system/include/cmsis/stm32f051x8.h" for register/bit definitions.
// See "system/src/cmsis/vectors_stm32f051x8.c" for handler declarations.
// ----------------------------------------------------------------------------

#include <stdio.h>
#include "diag/Trace.h"
#include "cmsis/cmsis_device.h"

// ----------------------------------------------------------------------------
//
// STM32F0 empty sample (trace via $(trace)).
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the $(trace) output,
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


/* Definitions of registers and their bits are
   given in system/include/cmsis/stm32f051x8.h */


/* Clock prescaler for TIM2 timer: no prescaling */
#define myTIM2_PRESCALER ((uint16_t)0x0000)
/* Maximum possible setting for overflow */
#define myTIM2_PERIOD ((uint32_t)0xFFFFFFFF)

void myGPIOB_Init(void);
void myTIM2_Init(void);
void myEXTI_Init(void);


// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------

/* Tracks whether the first edge has started the timer (1) or we're idle (0). */
static volatile uint8_t timerRunning = 0;

/*** Call this function to boost the STM32F0xx clock to 48 MHz ***/

void SystemClock48MHz( void )
{
//
// Disable the PLL
//
    RCC->CR &= ~(RCC_CR_PLLON);
//
// Wait for the PLL to unlock
//
    while (( RCC->CR & RCC_CR_PLLRDY ) != 0 );
//
// Configure the PLL for 48-MHz system clock
//
    RCC->CFGR = 0x00280000;
//
// Enable the PLL
//
    RCC->CR |= RCC_CR_PLLON;
//
// Wait for the PLL to lock
//
    while (( RCC->CR & RCC_CR_PLLRDY ) != RCC_CR_PLLRDY );
//
// Switch the processor to the PLL clock source
//
    RCC->CFGR = ( RCC->CFGR & (~RCC_CFGR_SW_Msk)) | RCC_CFGR_SW_PLL;
//
// Update the system with the new clock frequency
//
    SystemCoreClockUpdate();

}

/*****************************************************************/


int
main(int argc, char* argv[])
{

SystemClock48MHz();

trace_printf("This is Part 2 of Introductory Lab...\n");
trace_printf("System clock: %u Hz\n", SystemCoreClock);

    /* Enable SYSCFG clock (for EXTI port mapping) */
RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;

myGPIOB_Init(); /* Initialize I/O port PB */
myTIM2_Init(); /* Initialize timer TIM2 */
myEXTI_Init(); /* Initialize EXTI */

while (1)
{
// Nothing is going on here...
}

return 0;

}


void myGPIOB_Init()
{
/* Enable clock for GPIOB peripheral */
RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

/* Configure PB2 as input */
GPIOB->MODER &= ~GPIO_MODER_MODER2;

/* Ensure no pull-up/pull-down for PB2 */
GPIOB->PUPDR &= ~GPIO_PUPDR_PUPDR2;
}


void myTIM2_Init()
{
/* Enable clock for TIM2 peripheral */
RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

/* Configure TIM2: buffer auto-reload, upcounting.
  (Do not start here; start/stop from EXTI ISR.) */
TIM2->CR1 = TIM_CR1_ARPE;  // ARPE=1, upcount (DIR=0)

/* Set clock prescaler value (48 MHz timer clock) */
TIM2->PSC = myTIM2_PRESCALER;

/* Set auto-reloaded delay (max) */
TIM2->ARR = myTIM2_PERIOD;

/* Update timer registers */
TIM2->EGR = TIM_EGR_UG;

/* Optional: enable overflow interrupt to detect very slow signals */
NVIC_SetPriority(TIM2_IRQn, 0);
NVIC_EnableIRQ(TIM2_IRQn);
TIM2->DIER |= TIM_DIER_UIE;

/* DO NOT enable CEN here; we'll gate it on edges. */
}


void myEXTI_Init()
{
/* Map EXTI2 line to PB2: EXTICR[0] bits 11:8 = 0001 (Port B) */
SYSCFG->EXTICR[0] = (SYSCFG->EXTICR[0] & ~(0xF << 8)) | (0x1 << 8);

/* EXTI2 line interrupts: set rising-edge trigger */
EXTI->RTSR |= EXTI_RTSR_TR2;
//EXTI->FTSR |= EXTI_FTSR_TR2;

/* Unmask interrupts from EXTI2 line */
EXTI->IMR  |= EXTI_IMR_IM2;

/* Clear any stale EXTI2 pending flag */
EXTI->PR   |= EXTI_PR_PR2;

/* Assign EXTI2 interrupt priority = 0 in NVIC, then enable */
NVIC_SetPriority(EXTI2_3_IRQn, 0);
NVIC_EnableIRQ(EXTI2_3_IRQn);
}


/* This handler is declared in system/src/cmsis/vectors_stm32f051x8.c */
void TIM2_IRQHandler()
{
/* Check if update interrupt flag is indeed set */
if ((TIM2->SR & TIM_SR_UIF) != 0)
{
trace_printf("\n*** Overflow! (input too slow) ***\n");

/* Clear update interrupt flag */
TIM2->SR &= ~TIM_SR_UIF;

/* Stop timer and reset state */
TIM2->CR1 |= TIM_CR1_CEN;
//timerRunning = 0;
}
}


/* This handler is declared in system/src/cmsis/vectors_stm32f051x8.c */
void EXTI2_3_IRQHandler()
{

/* Check if EXTI2 interrupt pending flag is indeed set */
if ((EXTI->PR & EXTI_PR_PR2) != 0)
{
if (!timerRunning)
{
	//trace_printf("Start Timer");
/* First edge:
  - Clear count register
  - Start timer */
TIM2->CNT = 0;
TIM2->CR1 |= TIM_CR1_CEN;
//printf("Start Clock");
timerRunning = 1;
}
else
{
	//trace_printf("Stop Timer");
	volatile uint32_t ticks = TIM2->CNT;
	TIM2->CR1 &= ~TIM_CR1_CEN;
	//TIM2->CR1 |= TIM_CR1_CEN;
	timerRunning = 0;
/* Second edge:
  - Stop timer
  - Read count
  - Compute and print period/frequency (integer arithmetic) */
//TIM2->CR1 &= ~TIM_CR1_CEN;


if (ticks != 0U)
{
/* f = timer_clk / ticks ; T_us = ticks * 1e6 / timer_clk */

	volatile double f_Hz = (SystemCoreClock / ticks)+1;

//uint32_t T_us = (ticks * 1000000u) / SystemCoreClock;
	volatile double T_us = (1/f_Hz)*1e6;
trace_printf("Period: %.0lf us, Freq: %.0lf Hz\n", T_us, f_Hz);

}
else
{
trace_printf("Too fast (ticks=0)\n");
}

//timerRunning = 0;
}

/* 2. Clear EXTI2 interrupt pending flag (write 1 to clear). */
EXTI->PR |= EXTI_PR_PR2;
}
}


#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
