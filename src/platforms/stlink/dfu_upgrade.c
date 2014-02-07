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

#define STK_CTRL_CLKSOURCE      (1 << 2)
#define STK_CTRL_CLKSOURCE_LSB      2
#define STK_CTRL_CLKSOURCE_AHB_DIV8 0
#define STK_CTRL_CLKSOURCE_AHB      1

#include "usbdfu.h"
uint32_t app_address = 0x08000000;

static uint8_t rev;
static uint16_t led_idle_run;
static uint32_t led2_state = 0;

void dfu_detach(void)
{
	/* Disconnect USB cable by resetting USB Device
	   and pulling USB_DP low*/
    /* Disconnect USB cable by resetting USB Device and pulling USB_DP low*/
    rcc_peripheral_reset(&RCC_APB1RSTR, RCC_APB1ENR_USBEN);
    rcc_peripheral_clear_reset(&RCC_APB1RSTR, RCC_APB1ENR_USBEN);
    rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_USBEN);
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPAEN);
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO8);
    gpio_set(GPIOA, GPIO8); // pullup enabled
    
	/* Pull (T_NRST) low */
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPAEN);
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_OPENDRAIN, GPIO1);
	gpio_clear(GPIOA, GPIO1);
	SCB_VTOR = 0;
	scb_reset_core();
}

void stlink_set_rev(void)
{
	int i;

	/* First, get Board revision by pulling PC13/14 up. Read
	 *  11 for ST-Link V1, e.g. on VL Discovery, tag as rev 0
	 *  10 for ST-Link V2, e.g. on F4 Discovery, tag as rev 1
	 */
    rev = 1;

    led_idle_run = GPIO4;
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, led_idle_run);
}

int main(void)
{

	rcc_clock_setup_in_hse_8mhz_out_72mhz();

        stlink_set_rev();

	systick_set_clocksource(STK_CTRL_CLKSOURCE_AHB_DIV8);
	systick_set_reload(900000);

        dfu_protect(UPD_MODE);

        /* Handle USB disconnect/connect */
	/* Just in case: Disconnect USB cable by resetting USB Device
         * and pulling USB_DP low
         * Device will reconnect automatically as Pull-Up is hard wired*/
    rcc_peripheral_reset(&RCC_APB1RSTR, RCC_APB1ENR_USBEN);
    rcc_peripheral_clear_reset(&RCC_APB1RSTR, RCC_APB1ENR_USBEN);
    rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_USBEN);
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPAEN);
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO8);
    gpio_clear(GPIOA, GPIO8); // pullup disabled

	systick_interrupt_enable();
	systick_counter_enable();

	dfu_init(&stm32f103_usb_driver, UPD_MODE);

	dfu_main();
}

void sys_tick_handler(void)
{
	if (rev == 0) {
		gpio_toggle(GPIOB, led_idle_run);
	} else {
		if (led2_state & 1) {
			gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, led_idle_run);
			gpio_set(GPIOB, led_idle_run);
		} else {
			gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, led_idle_run);
		}
		led2_state++;
	}
}
