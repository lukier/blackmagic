/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2013 Gareth McMullin <gareth@blacksphere.co.nz>
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

#include <string.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/scb.h>

#include "usbdfu.h"
#define STK_CTRL_CLKSOURCE      (1 << 2)
#define STK_CTRL_CLKSOURCE_LSB      2
#define STK_CTRL_CLKSOURCE_AHB_DIV8 0
#define STK_CTRL_CLKSOURCE_AHB      1


static uint8_t rev;
static uint16_t led_idle_run;
static uint32_t led2_state = 0;

uint32_t app_address = 0x08002000;

static int stlink_test_nrst(void)
{
	/* Test if JRST/NRST is pulled down*/
	uint16_t nrst;
	uint16_t pin;
	uint32_t systick_value;

	systick_set_clocksource(STK_CTRL_CLKSOURCE_AHB_DIV8);
	systick_set_reload(0xffffff); /* no underflow for about 16.7 seconds*/
	systick_counter_enable();
	/* systick ist now running with 1 MHz, systick counts down */

	systick_value = systick_get_value();
	while (systick_get_value() > (systick_value - 1000)); /* Wait 1 msec*/
	
	pin = GPIO0;
	led_idle_run = GPIO4;
	
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, led_idle_run);
    
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPAEN);
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO2);
	gpio_set(GPIOA, GPIO2); // set SRST
	systick_value = systick_get_value();
	while (systick_get_value() > (systick_value - 20000)); // Wait 20 msec
	nrst = gpio_get(GPIOA, GPIO2);
	systick_counter_disable();
	return (nrst) ? 1 : 0;
}

void dfu_detach(void)
{
	/* Disconnect USB cable by resetting USB Device
	   and pulling USB_DP low*/
    rcc_peripheral_reset(&RCC_APB1RSTR, RCC_APB1ENR_USBEN);
    rcc_peripheral_clear_reset(&RCC_APB1RSTR, RCC_APB1ENR_USBEN);
    rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_USBEN);
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPAEN);
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO8);
    gpio_set(GPIOA, GPIO8); // pullup enabled
    
	scb_reset_system();
}

int main(void)
{
	/* Check the force bootloader pin*/
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPAEN);
	if(gpio_get(GPIOB, GPIO12) == 1) // BOOT not used
		dfu_jump_app_if_valid();

	dfu_protect(DFU_MODE);

	rcc_clock_setup_in_hse_8mhz_out_72mhz();
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
	systick_set_reload(900000);

        /* Handle USB disconnect/connect */
	/* Just in case: Disconnect USB cable by resetting USB Device
         * and pulling USB_DP low
         * Device will reconnect automatically as Pull-Up is hard wired*/
    rcc_peripheral_reset(&RCC_APB1RSTR, RCC_APB1ENR_USBEN);
    rcc_peripheral_clear_reset(&RCC_APB1RSTR, RCC_APB1ENR_USBEN);
    rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_USBEN);
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPAEN);
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO8);
    gpio_set(GPIOA, GPIO8); // pullup enabled

	systick_interrupt_enable();
	systick_counter_enable();

	dfu_init(&stm32f103_usb_driver, DFU_MODE);

	dfu_main();
}

void sys_tick_handler(void)
{
	if (rev == 0) {
		gpio_toggle(GPIOB, led_idle_run);
	} else {
		if (led2_state & 1) {
			gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, led_idle_run);
		} else {
			gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, led_idle_run);
		}
		led2_state++;
	}
}

