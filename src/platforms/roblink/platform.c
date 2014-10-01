/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2011  Black Sphere Technologies Ltd.
 * Written by Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* This file implements the platform specific functions for the STM32
 * implementation.
 */

#include <platform.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/stm32/adc.h>

#include <jtag_scan.h>
#include <usbuart.h>

#include <ctype.h>

uint8_t running_status;
volatile uint32_t timeout_counter;

jmp_buf fatal_error_jmpbuf;

static void morse_update(void);

#ifdef DEBUG_EARLY
int _write(int file, void* ptr, int len)
{
    uint8_t* uptr = (uint8_t*)ptr;
    
    for(int i = 0 ; i < len ; ++i)
    {
        usart_send_blocking(USBUSART, uptr[i]);
    }
    
    return len;
}
#endif

bool platform_target_get_power(void) 
{
    return gpio_get(PWR_BR_PORT, PWR_BR_PIN);
}

void platform_target_set_power(bool power)
{
}

int platform_hwversion(void)
{
    return 0;
}

int platform_init(void)
{
	rcc_clock_setup_in_hse_8mhz_out_72mhz();

	/* Enable peripherals */
	rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_USBEN);
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPAEN);
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPBEN);
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_AFIOEN);
	rcc_peripheral_enable_clock(&RCC_AHBENR, RCC_AHBENR_CRCEN);
    
    AFIO_MAPR = 0x2000000;
    
    gpio_set_mode(LED_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, LED_UART | LED_IDLE_RUN | LED_ERROR);

	/* Setup GPIO ports */
	gpio_clear(USB_PU_PORT, USB_PU_PIN); // pull-up disabled
    gpio_set_mode(USB_PU_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, USB_PU_PIN); 

    // JTAG pins
	gpio_set_mode(JTAG_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, TMS_PIN | TCK_PIN | TDI_PIN);
	
    /* This needs some fixing... */
	/* Toggle required to sort out line drivers... */
	//gpio_port_write(GPIOA, 0x8100);
	//gpio_port_write(GPIOB, 0x2000);

	//gpio_port_write(GPIOA, 0x8180);
	//gpio_port_write(GPIOB, 0x2002);
    
    
	/* Enable SRST output. Original uses a NPN to pull down, so setting the
	 * output HIGH asserts. Mini is directly connected so use open drain output
	 * and set LOW to assert.
	 */
	platform_srst_set_val(false);
	gpio_set_mode(SRST_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_OPENDRAIN, SRST_PIN);

    /* Enable internal pull-up on PWR_BR so that we don't drive
     TPWR locally or inadvertently supply power to the target. */
    gpio_set (PWR_BR_PORT, PWR_BR_PIN); 
    gpio_set_mode(PWR_BR_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, PWR_BR_PIN);

	/* Setup heartbeat timer */
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
	systick_set_reload(900000);	/* Interrupt us at 10 Hz */
	SCB_SHPR(11) &= ~((15 << 4) & 0xff);
	SCB_SHPR(11) |= ((14 << 4) & 0xff);
	systick_interrupt_enable();
	systick_counter_enable();

    // TGT_PWR sensing
    gpio_clear(GPIOB, GPIO0);
	gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO0);
    
    // RS485 transmitter control
    gpio_set_mode(RS485_CONTROL_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, RS485_CONTROL_PIN);
    gpio_clear(RS485_CONTROL_PORT, RS485_CONTROL_PIN);
	
	/* Drive pull-up high if VBUS connected */
    gpio_set(USB_PU_PORT, USB_PU_PIN);
    gpio_set_mode(USB_PU_PORT, GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, USB_PU_PIN);

	SCB_VTOR = 0x2000;	// Relocate interrupt vector table here

	cdcacm_init();
	usbuart_init();
    
    DEBUG("Platform ready\r\n");

    jtag_scan(NULL);

	return 0;
}

void platform_srst_set_val(bool assert)
{
    gpio_set_val(SRST_PORT, SRST_PIN, !assert);
}

void platform_delay(uint32_t delay)
{
	timeout_counter = delay;
	while(timeout_counter);
}

void sys_tick_handler(void)
{
	if(running_status)
		gpio_toggle(LED_PORT, LED_IDLE_RUN);

	if(timeout_counter)
		timeout_counter--;

	morse_update();
}


/* Morse code patterns and lengths */
static const struct {
	uint16_t code;
	uint8_t bits;
} morse_letter[] = {
	{        0b00011101,  8}, // 'A' .-
	{    0b000101010111, 12}, // 'B' -...
	{  0b00010111010111, 14}, // 'C' -.-.
	{      0b0001010111, 10}, // 'D' -..
	{            0b0001,  4}, // 'E' .
	{    0b000101110101, 12}, // 'F' ..-.
	{    0b000101110111, 12}, // 'G' --.
	{      0b0001010101, 10}, // 'H' ....
	{          0b000101,  6}, // 'I' ..
	{0b0001110111011101, 16}, // 'J' .---
	{    0b000111010111, 12}, // 'K' -.-
	{    0b000101011101, 12}, // 'L' .-..
	{      0b0001110111, 10}, // 'M' --
	{        0b00010111,  8}, // 'N' -.
	{  0b00011101110111, 14}, // 'O' ---
	{  0b00010111011101, 14}, // 'P' .--.
	{0b0001110101110111, 16}, // 'Q' --.-
	{      0b0001011101, 10}, // 'R' .-.
	{        0b00010101,  8}, // 'S' ...
	{          0b000111,  6}, // 'T' -
	{      0b0001110101, 10}, // 'U' ..-
	{    0b000111010101, 12}, // 'V' ...-
	{    0b000111011101, 12}, // 'W' .--
	{  0b00011101010111, 14}, // 'X' -..-
	{0b0001110111010111, 16}, // 'Y' -.--
	{  0b00010101110111, 14}, // 'Z' --..
};


const char *morse_msg;
static const char * volatile morse_ptr;
static char morse_repeat;

void morse(const char *msg, char repeat)
{
	morse_msg = morse_ptr = msg;
	morse_repeat = repeat;
	SET_ERROR_STATE(0);
}

static void morse_update(void)
{
	static uint16_t code;
	static uint8_t bits;

	if(!morse_ptr) return;

	if(!bits) {
		char c = *morse_ptr++;
		if(!c) {
			if(morse_repeat) {
				morse_ptr = morse_msg;
				c = *morse_ptr++;
			} else {
				morse_ptr = 0;
				return;
			}
		}
		if((c >= 'A') && (c <= 'Z')) {
			c -= 'A';
			code = morse_letter[(int)c].code;
            bits = morse_letter[(int)c].bits;
		} else {
			code = 0; bits = 4;
		}
	}
	SET_ERROR_STATE(code & 1);
	code >>= 1; bits--;
}

const char *platform_target_voltage(void)
{
    return gpio_get(PWR_BR_PORT, PWR_BR_PIN) != 0 ? "OK" : "ABSENT!";
}

void assert_boot_pin(void)
{
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO12);
	gpio_clear(GPIOB, GPIO12);
}

void setup_vbus_irq(void)
{

}

void platform_485transmit(bool enable)
{
    if(enable == true)
    {
        gpio_set(RS485_CONTROL_PORT, RS485_CONTROL_PIN);
    }
    else
    {
        gpio_clear(RS485_CONTROL_PORT, RS485_CONTROL_PIN);
    }
}
