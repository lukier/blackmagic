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
#include <libopencm3/stm32/usart.h>
#include "usbdfu.h"
#include "platform.h"

// main stack memory
uint8_t _main_stack[MAIN_STACK_SIZE] __attribute__ ((section(".stack.main")));

uint32_t app_address = 0x08002000;

void dfu_detach(void)
{
    // USB device must detach, we just reset... 
    scb_reset_system();
}

int main(void)
{       
    // Check the force bootloader pin
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPBEN);
    if(gpio_get(GPIOB, GPIO12))
    {
        dfu_jump_app_if_valid();
    }

    dfu_protect(DFU_MODE);

    rcc_clock_setup_in_hse_8mhz_out_72mhz();
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
    systick_set_reload(900000);

    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPAEN);
    rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_USBEN);

    systick_interrupt_enable();
    systick_counter_enable();

    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO5); // led config?
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO4); // led config?
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO3); // led config?

    dfu_init(&stm32f103_usb_driver, DFU_MODE);

    gpio_set(GPIOA, GPIO8); // USB pullup
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO8);

    dfu_main();
}

void sys_tick_handler(void)
{
    gpio_toggle(GPIOB, GPIO5); /* LED2 on/off */
    gpio_toggle(GPIOB, GPIO4); /* LED2 on/off */
    gpio_toggle(GPIOB, GPIO3); /* LED2 on/off */
}

