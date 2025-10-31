//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------
// School: University of Victoria, Canada.
// Course: ECE 355 "Microprocessor-Based Systems".
// This is template code for Part 2 of Introductory Lab.
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
#define VDD 2.968 //Need to adjust for what stm system power is
#define ADC_bits (4095)
#define VoltsPerBit VDD/ADC_bits


//------Functions---------------------------------------------------------------

void myGPIOA_Init(void);

//	1. RCC: Enable Port A, Enable ADC
//	2. PA1 -> analog configuration
//	3. ADC1 -> CFGR1[5:3] = 000, [13:12] = 11;	ADC configure register 1
//	4. ADC1 -> SMPR[2:0] = 111, ADC1 -> CHSELR[1] = 1
//	5. ADC1 -> CR[0] = 1

//------------------------------------------------------------------------------

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

        RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;  /* Enable SYSCFG clock */ // RCC_APB2ENR[0] = 1

	myGPIOA_Init();		/* Initialize I/O port PA */
	uint32_t pot_ADC;
	float pot_V= 0;

	while (1)
	{

	//Start ADC
		ADC1->CR |= ADC_CR_ADSTART; //Start ADC ADSTART = 1
	//Check if ADC is ready
		while(ADC_ISR_ADRDY != 1){}
		//Get ADC Value
		pot_ADC = ADC1->DR;
		// Convert ADC to Voltage
		pot_V = pot_ADC * VoltsPerBit;
		// print ADC Val

		trace_printf("Pot ADC:  %u \t\t Pot Voltage:  %.3f V \n", pot_ADC, pot_V);

	}

	return 0;

}


void myGPIOA_Init()
{
	//	1. RCC: Enable Port A, Enable ADC
	//	2. PA1 -> analog configuration
	//	3. ADC1 -> CFGR1[5:3] = 000, [13:12] = 11;	ADC configure register 1
	//	4. ADC1 -> SMPR[2:0] = 111, ADC1 -> CHSELR[1] = 1
	//	5. ADC1 -> CR[0] = 1

	// step 1, Enable clock for GPIOA peripheral
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;		// 0x00020000 = 0b 0000 0000 0000 0010 0000 0000 0000 0000,	bit 17, RCC_AHBENR[17] = 1

	// Enable ADC
	RCC->APB2ENR |= RCC_APB2ENR_ADCEN;

	// step 2, Configure PA as ANALOG
	GPIOA->MODER |= GPIO_MODER_MODER2;		// 0x00000030 = 0b ... 0011 0000

	// step 3, 0x ... 0011 0000 00OO O000
	ADC1->CFGR1 &= ~ADC_CFGR1_RES;		// bits [4:3] = 00
	ADC1->CFGR1 &= ~ADC_CFGR1_ALIGN;	// bit [5] = 0
	ADC1->CFGR1 |= ADC_CFGR1_OVRMOD;	// bit [12] = 1
	ADC1->CFGR1 |= ADC_CFGR1_CONT;		// bit [13] = 1

	// step 4, sampling time register, channel selection register
	ADC1->SMPR |= ADC_SMPR_SMP;				// bits [2:0] = 111
	ADC1->CHSELR |= ADC_CHSELR_CHSEL1;		// bit [1] = 1

	// step 5, ADC enable
	ADC1->CR |= ADC_CR_ADEN;		// bit [0] = 1, ADC enable = 1

}


#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
