//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------
// School: University of Victoria, Canada.
// Course: ECE 355 "Microprocessor-Based Systems".
// This is tutorial code for Part 1 of Introductory Lab.
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
/* Delay count for TIM2 timer: 1/4 sec at 48 MHz */
#define myTIM2_PERIOD ((uint32_t)12000000)

#define DEBOUNCE_MS 50


void myGPIOA_Init(void);
void myGPIOC_Init(void);
void myTIM2_Init(void);
void myEXTI_Init();
//void TIM2_IRQHandler();
void EXTI0_1_Handler(void);

void tim3_init_1ms_tick(void);
void timer_sleep(uint16_t ms);
//Global Flags
static volatile uint16_t last_button_time = 0;





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


	// By customizing __initialize_args() it is possible to pass arguments,
	// for example when running tests with semihosting you can pass various
	// options to the test.
	// trace_dump_args(argc, argv);

	// Send a greeting to the trace device (skipped on Release).
	trace_puts("Push Btn Test");

	// The standard output and the standard error should be forwarded to
	// the trace device. For this to work, a redirection in _write.c is
	// required.


	// At this stage the system clock should have already been configured
	// at high speed.
	trace_printf("System clock: %u Hz\n", SystemCoreClock);

        RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;  /* Enable SYSCFG clock */

	myGPIOA_Init();		/* Initialize I/O port PA */
	//myGPIOC_Init();		/* Initialize I/O port PC */
	myEXTI_Init();
	//myTIM2_Init();		/* Initialize timer TIM2 */
	tim3_init_1ms_tick();  //Initalize 1ms timer


	while(1){}


}


void myGPIOA_Init()
{
	/* Enable clock for GPIOA peripheral */
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	/* Configure PA0 as input */
	GPIOA->MODER &= ~(GPIO_MODER_MODER0);
	// Enable pullups (PUPDR0 = 01)
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR0;
	GPIOA->PUPDR |= ~GPIO_PUPDR_PUPDR0_0;
}




void myGPIOC_Init()
{
	/* Enable clock for GPIOC peripheral */
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

	/* Configure PC8 and PC9 as outputs */
	GPIOC->MODER |= (GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0);
	/* Ensure push-pull mode selected for PC8 and PC9 */
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_8 | GPIO_OTYPER_OT_9);
	/* Ensure high-speed mode for PC8 and PC9 */
	GPIOC->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR8 | GPIO_OSPEEDER_OSPEEDR9);
	/* Ensure no pull-up/pull-down for PC8 and PC9 */
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR8 | GPIO_PUPDR_PUPDR9);
}

void myEXTI_Init()
{
	/* Map EXTI2 line to PA0 */
	// Relevant register: SYSCFG->EXTICR[0]
	SYSCFG->EXTICR[0] &= ~(SYSCFG_EXTICR1_EXTI0_Msk);
	SYSCFG ->EXTICR[0] |= (SYSCFG_EXTICR1_EXTI0_PA );
	/* EXTI2 line interrupts: set rising-edge trigger */
	//Disable both edges first
	  EXTI->RTSR &= ~EXTI_RTSR_TR0;
	  EXTI->FTSR &= ~EXTI_FTSR_TR0;


	// Relevant register: EXTI->RTSR
	EXTI->RTSR |= EXTI_FTSR_TR0;

	/* Unmask interrupts from EXTI2 line */
	// Relevant register: EXTI->IMR
	  EXTI->IMR  |= EXTI_IMR_MR0;
	  EXTI->PR   |= EXTI_PR_PR0;

	/* Assign EXTI2 interrupt priority = 0 in NVIC */
	// Relevant register: NVIC->IP[2], or use NVIC_SetPriority
	NVIC_SetPriority(EXTI0_1_IRQn, 0);
	NVIC_EnableIRQ(EXTI0_1_IRQn);
	/* Enable EXTI2 interrupts in NVIC */
	// Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
}

void EXTI0_1_IRQHandler(void)
{
    if (EXTI->PR & EXTI_PR_PR0) {
        EXTI->PR = EXTI_PR_PR0;   // write-1-to-clear (assignment is fine)

        //Read current time in ms from TIM3
        uint16_t now = (uint16_t)TIM3->CNT;

        // Time since lass press
        uint16_t delta =(uint16_t)(now - last_button_time);

        if (delta >= DEBOUNCE_MS){
        	last_button_time = now;

        	trace_printf("btn pressed \n");
        }
    }
}



void tim3_init_1ms_tick(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    TIM3->PSC = 48000 - 1; // 48 MHz / 48000 = 1 kHz (1 ms tick)
    TIM3->ARR = 0xFFFF;
    TIM3->EGR = TIM_EGR_UG;
    TIM3->CR1 = TIM_CR1_CEN;
}

void timer_sleep(uint16_t ms) {
    uint16_t start = (uint16_t) TIM3->CNT;
    while ((uint16_t) ((uint16_t) TIM3->CNT - start) < ms) {
        __NOP();
    }
}


/* This handler is declared in system/src/cmsis/vectors_stm32f051x8.c */
//void TIM2_IRQHandler()
//{
////	uint16_t LEDstate;
////
////	/* Check if update interrupt flag is indeed set */
////	if ((TIM2->SR & TIM_SR_UIF) != 0)
////	{
////		/* Read current PC output and isolate PC8 and PC9 bits */
////		LEDstate = GPIOC->ODR & ((uint16_t)0x0300);
////		if (LEDstate == 0)	/* If LED is off, turn it on... */
////		{
////			/* Set PC8 or PC9 bit */
////			GPIOC->BSRR = blinkingLED;
////		}
////		else			/* ...else (LED is on), turn it off */
////		{
////			/* Reset PC8 or PC9 bit */
////			GPIOC->BRR = blinkingLED;
////		}
////
////		TIM2->SR &= ~(TIM_SR_UIF);	/* Clear update interrupt flag */
////		TIM2->CR1 |= TIM_CR1_CEN;	/* Restart stopped timer */
////	}
//}

///* This handler is declared in system/src/cmsis/vectors_stm32f051x8.c */
//void TIM2_IRQHandler()
//{
///* Check if update interrupt flag is indeed set */
//if ((TIM2->SR & TIM_SR_UIF) != 0)
//{
//trace_printf("btn pressed!\n");
//
///* Clear update interrupt flag */
//TIM2->SR &= ~TIM_SR_UIF;
//
///* Stop timer and reset state */
//TIM2->CR1 |= TIM_CR1_CEN;
////timerRunning = 0;
//}
//}


#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
