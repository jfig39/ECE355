//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------

#include <stdio.h>
#include "diag/Trace.h"
#include <string.h>

#include "cmsis/cmsis_device.h"
#include "stm32f0xx_hal_spi.h"
//#include "timer.h"

// ----------------------------------------------------------------------------
//
// STM32F0 led blink sample (trace via $(trace)).
//
// In debug configurations, demonstrate how to print a greeting message
// on the trace device. In release configurations the message is
// simply discarded.
//
// To demonstrate POSIX retargetting, reroute the STDOUT and STDERR to the
// trace device and display messages on both of them.
//
// Then demonstrates how to blink a led with 1Hz, using a
// continuous loop and SysTick delays.
//
// On DEBUG, the uptime in seconds is also displayed on the trace device.
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the $(trace) output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//
// The external clock frequency is specified as a preprocessor definition
// passed to the compiler via a command line option (see the 'C/C++ General' ->
// 'Paths and Symbols' -> the 'Symbols' tab, if you want to change it).
// The value selected during project creation was HSE_VALUE=48000000.
//
/// Note: The default clock settings take the user defined HSE_VALUE and try
// to reach the maximum possible system clock. For the default 8MHz input
// the result is guaranteed, but for other values it might not be possible,
// so please adjust the PLL settings in system/src/cmsis/system_stm32f0xx.c
//

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

/*** This is partial code for accessing LED Display via SPI interface. ***/

//...
// Display Defines
#define OLED_COL_OFFSET 2 //

//Global Variables
unsigned int Freq = 0;  // Example: measured frequency value (global variable)
unsigned int Res = 0;   // Example: measured resistance value (global variable)
static volatile uint8_t timerRunning = 0;

//ADC Defines
/* Clock prescaler for TIM2 timer: no prescaling */
#define myTIM2_PRESCALER ((uint16_t)0x0000)
/* Maximum possible setting for overflow */
#define myTIM2_PERIOD ((uint32_t)0xFFFFFFFF)
#define VDD 2.968 //Need to adjust for what stm system power is
#define ADC_bits (4095)
#define VoltsPerBit VDD/ADC_bits

//Display Functions
void oled_Write(unsigned char);
void oled_Write_Cmd(unsigned char);
void oled_Write_Data(unsigned char);
void oled_config(void);
void refresh_OLED(void);

//timer3 functions
static void tim3_init_1ms_tick(void);
static void timer_sleep(uint16_t ms);

//TIM2 Functions
void myTIM2_Init(void);

//EXTI functions
void EXTI0_ub_Init(void);
void EXTI0_1_IRQHandler(void);
void EXTI2_fgen_Init(void);

//set page and columns
static inline void oled_SetPage(uint8_t page);
static inline void oled_SetColumn(uint8_t col);
static void oled_DrawChar(uint8_t page, uint8_t col, unsigned char c);
static void oled_DrawStrings(uint8_t page, uint8_t col, const unsigned char *s);

//GPIO init functions
void myGPIOA_Init(void);
void myGPIOB_Init(void);
void myGPIOC_Init(void);

SPI_HandleTypeDef SPI_Handle;

/* Clock prescaler for TIM2 timer: no prescaling */
#define myTIM2_PRESCALER ((uint16_t)0x0000)
/* Delay count for TIM2 timer: 1/4 sec at 48 MHz */
#define myTIM2_PERIOD ((uint32_t)12000000)

//
// LED Display initialization commands
//
unsigned char oled_init_cmds[] = { 0xAE, 0x20, 0x00, 0x40, 0xA0 | 0x01, 0xA8,
		0x40 - 1, 0xC0 | 0x08, 0xD3, 0x00, 0xDA, 0x32, 0xD5, 0x80, 0xD9, 0x22,
		0xDB, 0x30, 0x81, 0xFF, 0xA4, 0xA6, 0xAD, 0x30, 0x8D, 0x10, 0xAE | 0x01,
		0xC0, 0xA0 };

//
// Character specifications for LED Display (1 row = 8 bytes = 1 ASCII character)
// Example: to display '4', retrieve 8 data bytes stored in Characters[52][X] row
//          (where X = 0, 1, ..., 7) and send them one by one to LED Display.
// Row number = character ASCII code (e.g., ASCII code of '4' is 0x34 = 52)
//
unsigned char Characters[][8] = { { 0b00000000, 0b00000000, 0b00000000,
		0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000 },  // SPACE
		{ 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
				0b00000000, 0b00000000, 0b00000000 },  // SPACE
		{ 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
				0b00000000, 0b00000000, 0b00000000 },  // SPACE
		{ 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
				0b00000000, 0b00000000, 0b00000000 },  // SPACE
		{ 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
				0b00000000, 0b00000000, 0b00000000 },  // SPACE
		{ 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
				0b00000000, 0b00000000, 0b00000000 },  // SPACE
		{ 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
				0b00000000, 0b00000000, 0b00000000 },  // SPACE
		{ 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
				0b00000000, 0b00000000, 0b00000000 },  // SPACE
		{ 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
				0b00000000, 0b00000000, 0b00000000 },  // SPACE
		{ 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
				0b00000000, 0b00000000, 0b00000000 },  // SPACE
		{ 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
				0b00000000, 0b00000000, 0b00000000 },  // SPACE
		{ 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
				0b00000000, 0b00000000, 0b00000000 },  // SPACE
		{ 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
				0b00000000, 0b00000000, 0b00000000 },  // SPACE
		{ 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
				0b00000000, 0b00000000, 0b00000000 },  // SPACE
		{ 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
				0b00000000, 0b00000000, 0b00000000 },  // SPACE
		{ 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
				0b00000000, 0b00000000, 0b00000000 },  // SPACE
		{ 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
				0b00000000, 0b00000000, 0b00000000 },  // SPACE
		{ 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
				0b00000000, 0b00000000, 0b00000000 },  // SPACE
		{ 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
				0b00000000, 0b00000000, 0b00000000 },  // SPACE
		{ 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
				0b00000000, 0b00000000, 0b00000000 },  // SPACE
		{ 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
				0b00000000, 0b00000000, 0b00000000 },  // SPACE
		{ 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
				0b00000000, 0b00000000, 0b00000000 },  // SPACE
		{ 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
				0b00000000, 0b00000000, 0b00000000 },  // SPACE
		{ 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
				0b00000000, 0b00000000, 0b00000000 },  // SPACE
		{ 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
				0b00000000, 0b00000000, 0b00000000 },  // SPACE
		{ 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
				0b00000000, 0b00000000, 0b00000000 },  // SPACE
		{ 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
				0b00000000, 0b00000000, 0b00000000 },  // SPACE
		{ 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
				0b00000000, 0b00000000, 0b00000000 },  // SPACE
		{ 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
				0b00000000, 0b00000000, 0b00000000 },  // SPACE
		{ 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
				0b00000000, 0b00000000, 0b00000000 },  // SPACE
		{ 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
				0b00000000, 0b00000000, 0b00000000 },  // SPACE
		{ 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
				0b00000000, 0b00000000, 0b00000000 },  // SPACE
		{ 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
				0b00000000, 0b00000000, 0b00000000 },  // SPACE
		{ 0b00000000, 0b00000000, 0b01011111, 0b00000000, 0b00000000,
				0b00000000, 0b00000000, 0b00000000 },  // !
		{ 0b00000000, 0b00000111, 0b00000000, 0b00000111, 0b00000000,
				0b00000000, 0b00000000, 0b00000000 },  // "
		{ 0b00010100, 0b01111111, 0b00010100, 0b01111111, 0b00010100,
				0b00000000, 0b00000000, 0b00000000 },  // #
		{ 0b00100100, 0b00101010, 0b01111111, 0b00101010, 0b00010010,
				0b00000000, 0b00000000, 0b00000000 },  // $
		{ 0b00100011, 0b00010011, 0b00001000, 0b01100100, 0b01100010,
				0b00000000, 0b00000000, 0b00000000 },  // %
		{ 0b00110110, 0b01001001, 0b01010101, 0b00100010, 0b01010000,
				0b00000000, 0b00000000, 0b00000000 },  // &
		{ 0b00000000, 0b00000101, 0b00000011, 0b00000000, 0b00000000,
				0b00000000, 0b00000000, 0b00000000 },  // '
		{ 0b00000000, 0b00011100, 0b00100010, 0b01000001, 0b00000000,
				0b00000000, 0b00000000, 0b00000000 },  // (
		{ 0b00000000, 0b01000001, 0b00100010, 0b00011100, 0b00000000,
				0b00000000, 0b00000000, 0b00000000 },  // )
		{ 0b00010100, 0b00001000, 0b00111110, 0b00001000, 0b00010100,
				0b00000000, 0b00000000, 0b00000000 },  // *
		{ 0b00001000, 0b00001000, 0b00111110, 0b00001000, 0b00001000,
				0b00000000, 0b00000000, 0b00000000 },  // +
		{ 0b00000000, 0b01010000, 0b00110000, 0b00000000, 0b00000000,
				0b00000000, 0b00000000, 0b00000000 },  // ,
		{ 0b00001000, 0b00001000, 0b00001000, 0b00001000, 0b00001000,
				0b00000000, 0b00000000, 0b00000000 },  // -
		{ 0b00000000, 0b01100000, 0b01100000, 0b00000000, 0b00000000,
				0b00000000, 0b00000000, 0b00000000 },  // .
		{ 0b00100000, 0b00010000, 0b00001000, 0b00000100, 0b00000010,
				0b00000000, 0b00000000, 0b00000000 },  // /
		{ 0b00111110, 0b01010001, 0b01001001, 0b01000101, 0b00111110,
				0b00000000, 0b00000000, 0b00000000 },  // 0
		{ 0b00000000, 0b01000010, 0b01111111, 0b01000000, 0b00000000,
				0b00000000, 0b00000000, 0b00000000 },  // 1
		{ 0b01000010, 0b01100001, 0b01010001, 0b01001001, 0b01000110,
				0b00000000, 0b00000000, 0b00000000 },  // 2
		{ 0b00100001, 0b01000001, 0b01000101, 0b01001011, 0b00110001,
				0b00000000, 0b00000000, 0b00000000 },  // 3
		{ 0b00011000, 0b00010100, 0b00010010, 0b01111111, 0b00010000,
				0b00000000, 0b00000000, 0b00000000 },  // 4
		{ 0b00100111, 0b01000101, 0b01000101, 0b01000101, 0b00111001,
				0b00000000, 0b00000000, 0b00000000 },  // 5
		{ 0b00111100, 0b01001010, 0b01001001, 0b01001001, 0b00110000,
				0b00000000, 0b00000000, 0b00000000 },  // 6
		{ 0b00000011, 0b00000001, 0b01110001, 0b00001001, 0b00000111,
				0b00000000, 0b00000000, 0b00000000 },  // 7
		{ 0b00110110, 0b01001001, 0b01001001, 0b01001001, 0b00110110,
				0b00000000, 0b00000000, 0b00000000 },  // 8
		{ 0b00000110, 0b01001001, 0b01001001, 0b00101001, 0b00011110,
				0b00000000, 0b00000000, 0b00000000 },  // 9
		{ 0b00000000, 0b00110110, 0b00110110, 0b00000000, 0b00000000,
				0b00000000, 0b00000000, 0b00000000 },  // :
		{ 0b00000000, 0b01010110, 0b00110110, 0b00000000, 0b00000000,
				0b00000000, 0b00000000, 0b00000000 },  // ;
		{ 0b00001000, 0b00010100, 0b00100010, 0b01000001, 0b00000000,
				0b00000000, 0b00000000, 0b00000000 },  // <
		{ 0b00010100, 0b00010100, 0b00010100, 0b00010100, 0b00010100,
				0b00000000, 0b00000000, 0b00000000 },  // =
		{ 0b00000000, 0b01000001, 0b00100010, 0b00010100, 0b00001000,
				0b00000000, 0b00000000, 0b00000000 },  // >
		{ 0b00000010, 0b00000001, 0b01010001, 0b00001001, 0b00000110,
				0b00000000, 0b00000000, 0b00000000 },  // ?
		{ 0b00110010, 0b01001001, 0b01111001, 0b01000001, 0b00111110,
				0b00000000, 0b00000000, 0b00000000 },  // @
		{ 0b01111110, 0b00010001, 0b00010001, 0b00010001, 0b01111110,
				0b00000000, 0b00000000, 0b00000000 },  // A
		{ 0b01111111, 0b01001001, 0b01001001, 0b01001001, 0b00110110,
				0b00000000, 0b00000000, 0b00000000 },  // B
		{ 0b00111110, 0b01000001, 0b01000001, 0b01000001, 0b00100010,
				0b00000000, 0b00000000, 0b00000000 },  // C
		{ 0b01111111, 0b01000001, 0b01000001, 0b00100010, 0b00011100,
				0b00000000, 0b00000000, 0b00000000 },  // D
		{ 0b01111111, 0b01001001, 0b01001001, 0b01001001, 0b01000001,
				0b00000000, 0b00000000, 0b00000000 },  // E
		{ 0b01111111, 0b00001001, 0b00001001, 0b00001001, 0b00000001,
				0b00000000, 0b00000000, 0b00000000 },  // F
		{ 0b00111110, 0b01000001, 0b01001001, 0b01001001, 0b01111010,
				0b00000000, 0b00000000, 0b00000000 },  // G
		{ 0b01111111, 0b00001000, 0b00001000, 0b00001000, 0b01111111,
				0b00000000, 0b00000000, 0b00000000 },  // H
		{ 0b01000000, 0b01000001, 0b01111111, 0b01000001, 0b01000000,
				0b00000000, 0b00000000, 0b00000000 },  // I
		{ 0b00100000, 0b01000000, 0b01000001, 0b00111111, 0b00000001,
				0b00000000, 0b00000000, 0b00000000 },  // J
		{ 0b01111111, 0b00001000, 0b00010100, 0b00100010, 0b01000001,
				0b00000000, 0b00000000, 0b00000000 },  // K
		{ 0b01111111, 0b01000000, 0b01000000, 0b01000000, 0b01000000,
				0b00000000, 0b00000000, 0b00000000 },  // L
		{ 0b01111111, 0b00000010, 0b00001100, 0b00000010, 0b01111111,
				0b00000000, 0b00000000, 0b00000000 },  // M
		{ 0b01111111, 0b00000100, 0b00001000, 0b00010000, 0b01111111,
				0b00000000, 0b00000000, 0b00000000 },  // N
		{ 0b00111110, 0b01000001, 0b01000001, 0b01000001, 0b00111110,
				0b00000000, 0b00000000, 0b00000000 },  // O
		{ 0b01111111, 0b00001001, 0b00001001, 0b00001001, 0b00000110,
				0b00000000, 0b00000000, 0b00000000 },  // P
		{ 0b00111110, 0b01000001, 0b01010001, 0b00100001, 0b01011110,
				0b00000000, 0b00000000, 0b00000000 },  // Q
		{ 0b01111111, 0b00001001, 0b00011001, 0b00101001, 0b01000110,
				0b00000000, 0b00000000, 0b00000000 },  // R
		{ 0b01000110, 0b01001001, 0b01001001, 0b01001001, 0b00110001,
				0b00000000, 0b00000000, 0b00000000 },  // S
		{ 0b00000001, 0b00000001, 0b01111111, 0b00000001, 0b00000001,
				0b00000000, 0b00000000, 0b00000000 },  // T
		{ 0b00111111, 0b01000000, 0b01000000, 0b01000000, 0b00111111,
				0b00000000, 0b00000000, 0b00000000 },  // U
		{ 0b00011111, 0b00100000, 0b01000000, 0b00100000, 0b00011111,
				0b00000000, 0b00000000, 0b00000000 },  // V
		{ 0b00111111, 0b01000000, 0b00111000, 0b01000000, 0b00111111,
				0b00000000, 0b00000000, 0b00000000 },  // W
		{ 0b01100011, 0b00010100, 0b00001000, 0b00010100, 0b01100011,
				0b00000000, 0b00000000, 0b00000000 },  // X
		{ 0b00000111, 0b00001000, 0b01110000, 0b00001000, 0b00000111,
				0b00000000, 0b00000000, 0b00000000 },  // Y
		{ 0b01100001, 0b01010001, 0b01001001, 0b01000101, 0b01000011,
				0b00000000, 0b00000000, 0b00000000 },  // Z
		{ 0b01111111, 0b01000001, 0b00000000, 0b00000000, 0b00000000,
				0b00000000, 0b00000000, 0b00000000 },  // [
		{ 0b00010101, 0b00010110, 0b01111100, 0b00010110, 0b00010101,
				0b00000000, 0b00000000, 0b00000000 },  // back slash
		{ 0b00000000, 0b00000000, 0b00000000, 0b01000001, 0b01111111,
				0b00000000, 0b00000000, 0b00000000 },  // ]
		{ 0b00000100, 0b00000010, 0b00000001, 0b00000010, 0b00000100,
				0b00000000, 0b00000000, 0b00000000 },  // ^
		{ 0b01000000, 0b01000000, 0b01000000, 0b01000000, 0b01000000,
				0b00000000, 0b00000000, 0b00000000 },  // _
		{ 0b00000000, 0b00000001, 0b00000010, 0b00000100, 0b00000000,
				0b00000000, 0b00000000, 0b00000000 },  // `
		{ 0b00100000, 0b01010100, 0b01010100, 0b01010100, 0b01111000,
				0b00000000, 0b00000000, 0b00000000 },  // a
		{ 0b01111111, 0b01001000, 0b01000100, 0b01000100, 0b00111000,
				0b00000000, 0b00000000, 0b00000000 },  // b
		{ 0b00111000, 0b01000100, 0b01000100, 0b01000100, 0b00100000,
				0b00000000, 0b00000000, 0b00000000 },  // c
		{ 0b00111000, 0b01000100, 0b01000100, 0b01001000, 0b01111111,
				0b00000000, 0b00000000, 0b00000000 },  // d
		{ 0b00111000, 0b01010100, 0b01010100, 0b01010100, 0b00011000,
				0b00000000, 0b00000000, 0b00000000 },  // e
		{ 0b00001000, 0b01111110, 0b00001001, 0b00000001, 0b00000010,
				0b00000000, 0b00000000, 0b00000000 },  // f
		{ 0b00001100, 0b01010010, 0b01010010, 0b01010010, 0b00111110,
				0b00000000, 0b00000000, 0b00000000 },  // g
		{ 0b01111111, 0b00001000, 0b00000100, 0b00000100, 0b01111000,
				0b00000000, 0b00000000, 0b00000000 },  // h
		{ 0b00000000, 0b01000100, 0b01111101, 0b01000000, 0b00000000,
				0b00000000, 0b00000000, 0b00000000 },  // i
		{ 0b00100000, 0b01000000, 0b01000100, 0b00111101, 0b00000000,
				0b00000000, 0b00000000, 0b00000000 },  // j
		{ 0b01111111, 0b00010000, 0b00101000, 0b01000100, 0b00000000,
				0b00000000, 0b00000000, 0b00000000 },  // k
		{ 0b00000000, 0b01000001, 0b01111111, 0b01000000, 0b00000000,
				0b00000000, 0b00000000, 0b00000000 },  // l
		{ 0b01111100, 0b00000100, 0b00011000, 0b00000100, 0b01111000,
				0b00000000, 0b00000000, 0b00000000 },  // m
		{ 0b01111100, 0b00001000, 0b00000100, 0b00000100, 0b01111000,
				0b00000000, 0b00000000, 0b00000000 },  // n
		{ 0b00111000, 0b01000100, 0b01000100, 0b01000100, 0b00111000,
				0b00000000, 0b00000000, 0b00000000 },  // o
		{ 0b01111100, 0b00010100, 0b00010100, 0b00010100, 0b00001000,
				0b00000000, 0b00000000, 0b00000000 },  // p
		{ 0b00001000, 0b00010100, 0b00010100, 0b00011000, 0b01111100,
				0b00000000, 0b00000000, 0b00000000 },  // q
		{ 0b01111100, 0b00001000, 0b00000100, 0b00000100, 0b00001000,
				0b00000000, 0b00000000, 0b00000000 },  // r
		{ 0b01001000, 0b01010100, 0b01010100, 0b01010100, 0b00100000,
				0b00000000, 0b00000000, 0b00000000 },  // s
		{ 0b00000100, 0b00111111, 0b01000100, 0b01000000, 0b00100000,
				0b00000000, 0b00000000, 0b00000000 },  // t
		{ 0b00111100, 0b01000000, 0b01000000, 0b00100000, 0b01111100,
				0b00000000, 0b00000000, 0b00000000 },  // u
		{ 0b00011100, 0b00100000, 0b01000000, 0b00100000, 0b00011100,
				0b00000000, 0b00000000, 0b00000000 },  // v
		{ 0b00111100, 0b01000000, 0b00111000, 0b01000000, 0b00111100,
				0b00000000, 0b00000000, 0b00000000 },  // w
		{ 0b01000100, 0b00101000, 0b00010000, 0b00101000, 0b01000100,
				0b00000000, 0b00000000, 0b00000000 },  // x
		{ 0b00001100, 0b01010000, 0b01010000, 0b01010000, 0b00111100,
				0b00000000, 0b00000000, 0b00000000 },  // y
		{ 0b01000100, 0b01100100, 0b01010100, 0b01001100, 0b01000100,
				0b00000000, 0b00000000, 0b00000000 },  // z
		{ 0b00000000, 0b00001000, 0b00110110, 0b01000001, 0b00000000,
				0b00000000, 0b00000000, 0b00000000 },  // {
		{ 0b00000000, 0b00000000, 0b01111111, 0b00000000, 0b00000000,
				0b00000000, 0b00000000, 0b00000000 },  // |
		{ 0b00000000, 0b01000001, 0b00110110, 0b00001000, 0b00000000,
				0b00000000, 0b00000000, 0b00000000 },  // }
		{ 0b00001000, 0b00001000, 0b00101010, 0b00011100, 0b00001000,
				0b00000000, 0b00000000, 0b00000000 },  // ~
		{ 0b00001000, 0b00011100, 0b00101010, 0b00001000, 0b00001000,
				0b00000000, 0b00000000, 0b00000000 }   // <-
};

void SystemClock48MHz(void) {
//
// Disable the PLL
//
	RCC->CR &= ~(RCC_CR_PLLON);
//
// Wait for the PLL to unlock
//
	while (( RCC->CR & RCC_CR_PLLRDY) != 0)
		;
//
// Configure the PLL for a 48MHz system clock
//
	RCC->CFGR = 0x00280000;

//
// Enable the PLL
//
	RCC->CR |= RCC_CR_PLLON;

//
// Wait for the PLL to lock
//
	while (( RCC->CR & RCC_CR_PLLRDY) != RCC_CR_PLLRDY)
		;

//
// Switch the processor to the PLL clock source
//
	RCC->CFGR = ( RCC->CFGR & (~RCC_CFGR_SW_Msk)) | RCC_CFGR_SW_PLL;

//
// Update the system with the new clock frequency
//
	SystemCoreClockUpdate();

}

int main(int argc, char *argv[]) {
	//Systems Setup
	HAL_Init();
	SystemClock48MHz();
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN; /* Enable SYSCFG clock */ // RCC_APB2ENR[0] = 1
	//Timer Init
	tim3_init_1ms_tick();
	oled_config();	//Display Init
	refresh_OLED();


	//EXTI Init

	timer_sleep(100);
	myTIM2_Init(); 		// Initialize timer TIM2
	EXTI0_ub_Init();	// Initialize User Button external interrupt

	EXTI2_fgen_Init();	// Initialize Function Generator external interrupt





	//GPIOs Init
	myGPIOA_Init(); /* Initialize I/O port PA */
	myGPIOB_Init(); 	// Initialize I/O port PB
	myGPIOC_Init(); 	// Initialize I/O port PB

	uint32_t pot_ADC;
	float pot_V = 0;

	//DAC Init

	// Enable perifer clock for GPIO
	RCC->AHBENR |= (RCC_AHBENR_GPIOAEN);
	// Pin A4: analog mode. (PA4 = DAC1, Channel 1)
	//Set GPIOA to output mode
	GPIOA->MODER &= ~(0x3 << (4 * 2));
	GPIOA->MODER |= (0x3 << (4 * 2));
	//Enable clock for DAC
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;
	// Enable DAC
	DAC->CR |= DAC_CR_EN1;

	while (1) {

		//Start ADC
		ADC1->CR |= ADC_CR_ADSTART; //Start ADC ADSTART = 1
		//Check if ADC is ready
		while (ADC_ISR_ADRDY != 1) {
		}
		//Get ADC Value
		pot_ADC = ADC1->DR;

		//Put ADV value into DAC
		DAC->DHR12R1 = pot_ADC;
		// Convert ADC to Voltage
		pot_V = pot_ADC * VoltsPerBit;
		// print ADC Val

		//trace_printf("Pot ADC:  %u \t\t Pot Voltage:  %.3f V \n", pot_ADC,
				//pot_V);

		Res = pot_V * (5000 / VDD);
		refresh_OLED();

	}

}

//
// LED Display Functions
//

void refresh_OLED(void) {
	NVIC_DisableIRQ(EXTI2_3_IRQn);
	// Buffer size = at most 16 characters per PAGE + terminating '\0'
	unsigned char Buffer[17];

	snprintf(Buffer, sizeof(Buffer), "R: %5u Ohms", Res);
	/* Buffer now contains your character ASCII codes for LED Display
	 - select PAGE (LED Display line) and set starting SEG (column)
	 - for each c = ASCII code = Buffer[0], Buffer[1], ...,
	 send 8 bytes in Characters[c][0-7] to LED Display
	 */

	//...
	oled_DrawStrings(0, 0, Buffer);

	snprintf(Buffer, sizeof(Buffer), "F: %5u Hz", Freq);
	/* Buffer now contains your character ASCII codes for LED Display
	 - select PAGE (LED Display line) and set starting SEG (column)
	 - for each c = ASCII code = Buffer[0], Buffer[1], ...,
	 send 8 bytes in Characters[c][0-7] to LED Display
	 */

	//...
	oled_DrawStrings(1, 0, Buffer);

	/* Wait for ~100 ms (for example) to get ~10 frames/sec refresh rate
	 - You should use TIM3 to implement this delay (e.g., via polling)
	 */

	//...
	NVIC_EnableIRQ(EXTI2_3_IRQn);
	timer_sleep(100);


}

void oled_Write_Cmd(unsigned char cmd) {
	//... // make PB8 = CS# = 1
	GPIOB->BSRR |= GPIO_BSRR_BS_8;

	//... // make PB9 = D/C# = 0
	GPIOB->BSRR |= GPIO_BSRR_BR_9;

	//... // make PB8 = CS# = 0
	GPIOB->BSRR |= GPIO_BSRR_BR_8;

	oled_Write(cmd);

	//... // make PB8 = CS# = 1
	GPIOB->BSRR |= GPIO_BSRR_BS_8;

}

void oled_Write_Data(unsigned char data) {
	//... // make PB8 = CS# = 1
	GPIOB->BSRR |= GPIO_BSRR_BS_8;

	//... // make PB9 = D/C# = 1
	GPIOB->BSRR |= GPIO_BSRR_BS_9;

	//... // make PB8 = CS# = 0
	GPIOB->BSRR |= GPIO_BSRR_BR_8;

	oled_Write(data);

	//... // make PB8 = CS# = 1
	GPIOB->BSRR |= GPIO_BSRR_BS_8;

}

void oled_Write(unsigned char Value) {

	/* Wait until SPI2 is ready for writing (TXE = 1 in SPI2_SR) */

	while (!(SPI2->SR & SPI_SR_TXE)) {
	} //wait till SPI beffer is empty

	//...

	/* Send one 8-bit character:
	 - This function also sets BIDIOE = 1 in SPI2_CR1
	 */
	HAL_SPI_Transmit(&SPI_Handle, &Value, 1, HAL_MAX_DELAY);

	/* Wait until transmission is complete (TXE = 1 in SPI2_SR) */
	while (!(SPI2->SR & SPI_SR_TXE)) {
	}
	while (SPI2->SR & SPI_SR_BSY) {
	}

	//...

}

void oled_config(void) {

// Don't forget to enable GPIOB clock in RCC
// Don't forget to configure PB13/PB15 as AF0
// Don't forget to enable SPI2 clock in RCC

	RCC->AHBENR |= RCC_AHBENR_GPIOBEN; //  0x00040000 0b ... 0100 0000 0000 0000 0000, bit 18, RCC_AHBENR[18] = 1
	RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;	// SPI2 enable

	// step 2, Port B -> AF0 for PB13, PB15
	GPIOB->MODER |= GPIO_MODER_MODER13_1;
	GPIOB->MODER |= GPIO_MODER_MODER15_1;
	GPIOB->AFR[1] = 0;// selecting the 0th alternate function for high end Port B pins
	GPIOB->MODER |= GPIO_MODER_MODER8_0;	// out
	GPIOB->MODER |= GPIO_MODER_MODER9_0;	// out
	GPIOB->MODER |= GPIO_MODER_MODER11_0;	// out

	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_8 | GPIO_OTYPER_OT_9 | GPIO_OTYPER_OT_11); //set GPIOs outputs as push pull CS, D/C, RES
	GPIOB->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR13 | GPIO_OSPEEDR_OSPEEDR15
			| GPIO_OSPEEDR_OSPEEDR8 | //Set SPI bus to slow
			GPIO_OSPEEDR_OSPEEDR9 | GPIO_OSPEEDR_OSPEEDR11);

	SPI_Handle.Instance = SPI2;

	SPI_Handle.Init.Direction = SPI_DIRECTION_1LINE;
	SPI_Handle.Init.Mode = SPI_MODE_MASTER;
	SPI_Handle.Init.DataSize = SPI_DATASIZE_8BIT;
	SPI_Handle.Init.CLKPolarity = SPI_POLARITY_LOW;
	SPI_Handle.Init.CLKPhase = SPI_PHASE_1EDGE;
	SPI_Handle.Init.NSS = SPI_NSS_SOFT;
	SPI_Handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
	SPI_Handle.Init.FirstBit = SPI_FIRSTBIT_MSB;
	SPI_Handle.Init.CRCPolynomial = 7;

//
// Initialize the SPI interface
//
	HAL_SPI_Init(&SPI_Handle);

//
// Enable the SPI
//
	__HAL_SPI_ENABLE(&SPI_Handle);

	/* Reset LED Display (RES# = PB11):
	 //set up timer


	 - make pin PB11 = 0, wait for a few ms
	 - make pin PB11 = 1, wait for a few ms
	 */
	//...
	GPIOB->BSRR |= GPIO_BSRR_BR_11; 	//make pin PB11 = 0, wait for a few ms
	timer_sleep(10);				//need to make this TIM3 (using hal timer)
	GPIOB->BSRR |= GPIO_BSRR_BS_11; 	//make pin PB11 = 1, wait for a few ms
	timer_sleep(10);

//
// Send initialization commands to LED Display
//
	for (unsigned int i = 0; i < sizeof(oled_init_cmds); i++) {
		oled_Write_Cmd(oled_init_cmds[i]);
	}

	/* Fill LED Display data memory (GDDRAM) with zeros:
	 - for each PAGE = 0, 1, ..., 7
	 set starting SEG = 0
	 call oled_Write_Data( 0x00 ) 128 times
	 */

	//...
	// After init commands in oled_config()
	for (uint8_t page = 0; page < 8; page++) {
		oled_SetPage(page);
		oled_SetColumn(0);
		for (uint16_t i = 0; i < 128; i++) {
			oled_Write_Data(0x00);
		}
	}

}

//Timer Functions
// ~~~ Timer 2 Initialization and IRQ Handler ~~~

void myTIM2_Init() {

	// Enable clock for TIM2 peripheral
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	// Configure TIM2: buffer auto-reload, upcounting.
	// (Do not start here; start/stop from EXTI ISR.)
	TIM2->CR1 = TIM_CR1_ARPE;  // ARPE=1, upcount (DIR=0)

	// Set clock prescaler value (48 MHz timer clock)
	TIM2->PSC = myTIM2_PRESCALER;

	// Set auto-reloaded delay (max)
	TIM2->ARR = myTIM2_PERIOD;

	// Update timer registers
	TIM2->EGR = TIM_EGR_UG;

	// Optional: enable overflow interrupt to detect very slow signals
	NVIC_SetPriority(TIM2_IRQn, 0);
	NVIC_EnableIRQ(TIM2_IRQn);
	TIM2->DIER |= TIM_DIER_UIE;
}

void TIM2_IRQHandler() {

	// Check if update interrupt flag is indeed set
	if ((TIM2->SR & TIM_SR_UIF) != 0) {

		trace_printf("\n*** Overflow! (input too slow) ***\n");

		// Clear update interrupt flag
		TIM2->SR &= ~TIM_SR_UIF;

		// Stop timer and reset state
		TIM2->CR1 |= TIM_CR1_CEN;		//timerRunning = 0;
	}
}

// ----------------------------------------------------------------------------
// Tiny TIM3-based blocking delay (~1 ms resolution)
// ----------------------------------------------------------------------------

static void tim3_init_1ms_tick(void) {
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	TIM3->PSC = 48000 - 1; // 48 MHz / 48000 = 1 kHz (1 ms tick)
	TIM3->ARR = 0xFFFF;
	TIM3->EGR = TIM_EGR_UG;
	TIM3->CR1 = TIM_CR1_CEN;
}

static void timer_sleep(uint16_t ms) {
	uint16_t start = (uint16_t) TIM3->CNT;
	while ((uint16_t) ((uint16_t) TIM3->CNT - start) < ms) {
		__NOP();
	}
}

//EXTI functions
// ~~~ EXTI0 (User Button) Initialization and IRQ Handler ~~~

void EXTI0_ub_Init() {

	// Map EXTI0 line to PA0
	SYSCFG->EXTICR[0] &= ~(SYSCFG_EXTICR1_EXTI0_Msk);
	SYSCFG->EXTICR[0] |= (SYSCFG_EXTICR1_EXTI0_PA);

	// EXTI0 line interrupts: set rising-edge trigger
	//Disable both edges first
	EXTI->RTSR &= ~EXTI_RTSR_TR0;
	EXTI->FTSR &= ~EXTI_FTSR_TR0;
	EXTI->RTSR |= EXTI_FTSR_TR0;

	// Unmask interrupts from EXTI0 line
	EXTI->IMR |= EXTI_IMR_MR0;
	EXTI->PR |= EXTI_PR_PR0;

	// Assign EXTI0 interrupt priority = 0 in NVIC
	NVIC_SetPriority(EXTI0_1_IRQn, 0);

	// Enable EXTI0 interrupts in NVIC
	NVIC_EnableIRQ(EXTI0_1_IRQn);
}

void EXTI0_1_IRQHandler(void) {

	if (EXTI->PR & EXTI_PR_PR0) {

		EXTI->PR = EXTI_PR_PR0;   // write-1-to-clear (assignment is fine)
		trace_printf("btn pressed\n");
		// GPIOC->ODR ^= (1u << 8); // optional: visible LED proof if PC8 is an output
	}
}

// ----------------------------------------------------------

// ~~~ EXTI2 (Function Generator) Initialization and IRQ Handler ~~~

void EXTI2_fgen_Init() {

	// Map EXTI2 line to PB2: EXTICR[0] bits 11:8 = 0001 (Port B) */
	SYSCFG->EXTICR[0] |= (SYSCFG->EXTICR[0] & ~(0xF << 8)) | (0x1 << 8);

	// EXTI2 line interrupts: set rising-edge trigger
	EXTI->RTSR |= EXTI_RTSR_TR2;

	// Unmask interrupts from EXTI2 line
	EXTI->IMR |= EXTI_IMR_IM2;

	// Clear any stale EXTI2 pending flag
	EXTI->PR |= EXTI_PR_PR2;

	// Assign EXTI2 interrupt priority = 0 in NVIC, then enable
	NVIC_SetPriority(EXTI2_3_IRQn, 0);
	NVIC_EnableIRQ(EXTI2_3_IRQn);
}

void EXTI2_3_IRQHandler() {

	/* Check if EXTI2 interrupt pending flag is indeed set */
	if ((EXTI->PR & EXTI_PR_PR2) != 0) {

		if (!timerRunning) {

			//trace_printf("Start Timer");
			// First edge:
			// - Clear count register
			// - Start timer
			TIM2->CNT = 0;
			TIM2->CR1 |= TIM_CR1_CEN;
			//printf("Start Clock");
			timerRunning = 1;
		} else {

			volatile uint32_t ticks = TIM2->CNT;
			TIM2->CR1 &= ~TIM_CR1_CEN;

			timerRunning = 0;
			// Second edge:
			// - Stop timer
			// - Read count
			// - Compute and print period/frequency (integer arithmetic)
			//TIM2->CR1 &= ~TIM_CR1_CEN;

			if (ticks != 0U) {
				// f = timer_clk / ticks ; T_us = ticks * 1e6 / timer_clk

				volatile double f_Hz = (SystemCoreClock / ticks) + 1;
				volatile double T_us = (1 / f_Hz) * 1e6;
				//trace_printf("Period: %.0lf us, Freq: %.0lf Hz\n", T_us, f_Hz);
				Freq = f_Hz;
			} else {

				trace_printf("Too fast (ticks=0)\n");
			}

			//timerRunning = 0;
		}

		// 2. Clear EXTI2 interrupt pending flag (write 1 to clear).
		EXTI->PR |= EXTI_PR_PR2;
	}
}

// -----------------------------------------------------------------

// -----------------

//Display commands
static inline void oled_SetPage(uint8_t page) {
	oled_Write_Cmd(0xB0 | (page & 0x07)); // Page 0..7
}

static inline void oled_SetColumn(uint8_t col) {
	col += OLED_COL_OFFSET;
	oled_Write_Cmd(0x00 | (col & 0x0F));       // lower nibble
	oled_Write_Cmd(0x10 | ((col >> 4) & 0x0F)); // upper nibble
}

static void oled_DrawChar(uint8_t page, uint8_t col, unsigned char c) {
	oled_SetPage(page);
	oled_SetColumn(col);

	for (uint8_t k = 0; k < 8; k++) {
		oled_Write_Data(Characters[(uint8_t) c][k]);
	}
}

static void oled_DrawStrings(uint8_t page, uint8_t col, const unsigned char *s) {
	uint8_t x = col;
	for (size_t i = 0; s[i] != '\0' && x <= 120; i++) {
		oled_DrawChar(page, x, s[i]);
		x += 8;
	}
}

//ADC Function Definitions
void myGPIOA_Init() {
	//	1. RCC: Enable Port A, Enable ADC
	//	2. PA1 -> analog configuration
	//	3. ADC1 -> CFGR1[5:3] = 000, [13:12] = 11;	ADC configure register 1
	//	4. ADC1 -> SMPR[2:0] = 111, ADC1 -> CHSELR[1] = 1
	//	5. ADC1 -> CR[0] = 1

	// step 1, Enable clock for GPIOA peripheral
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;// 0x00020000 = 0b 0000 0000 0000 0010 0000 0000 0000 0000,	bit 17, RCC_AHBENR[17] = 1

	// Enable ADC
	RCC->APB2ENR |= RCC_APB2ENR_ADCEN;

	// step 2, Configure PA as ANALOG
	GPIOA->MODER |= GPIO_MODER_MODER2;		// 0x00000030 = 0b ... 0011 0000

	//Joey's Code Start
	// Configure PA0 as input
	GPIOA->MODER &= ~(GPIO_MODER_MODER0);

	// Enable pullups (PUPDR0 = 01)
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR0;
	GPIOA->PUPDR |= ~GPIO_PUPDR_PUPDR0_0;

	//Joey's Code End

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

void myGPIOB_Init() {

	// Enable clock for GPIOB peripheral
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

	// Configure PB2 as input
	GPIOB->MODER &= ~GPIO_MODER_MODER2;

	// Ensure no pull-up/pull-down for PB2
	GPIOB->PUPDR &= ~GPIO_PUPDR_PUPDR2;
}

void myGPIOC_Init() {

	// Enable clock for GPIOC peripheral
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

	// Configure PC8 and PC9 as outputs
	GPIOC->MODER |= (GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0);

	// Ensure push-pull mode selected for PC8 and PC9
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_8 | GPIO_OTYPER_OT_9);

	// Ensure high-speed mode for PC8 and PC9
	GPIOC->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR8 | GPIO_OSPEEDER_OSPEEDR9);

	// Ensure no pull-up/pull-down for PC8 and PC9
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR8 | GPIO_PUPDR_PUPDR9);
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
