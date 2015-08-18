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
#include <morse.h>

#include <cdcacm.h>

#include <ctype.h>

jmp_buf fatal_error_jmpbuf;

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

void platform_init(void)
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

    platform_timing_init();
    cdcacm_init();
    usbuart_init();
    
    //DEBUG("Platform ready\r\n");

    jtag_scan(NULL);
}

void platform_srst_set_val(bool assert)
{
    gpio_set_val(SRST_PORT, SRST_PIN, !assert);
}

const char *platform_target_voltage(void)
{
    return gpio_get(PWR_BR_PORT, PWR_BR_PIN) != 0 ? "OK" : "ABSENT!";
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

void platform_request_boot(void)
{
    /* Disconnect USB cable by resetting USB Device and pulling USB_DP low*/
    rcc_periph_reset_pulse(RST_USB);
    rcc_periph_clock_enable(RCC_USB);
    rcc_periph_clock_enable(RCC_GPIOA);
    gpio_clear(GPIOA, GPIO12);
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_OPENDRAIN, GPIO12);
    
    /* Assert bootloader pin */
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO12);
    gpio_clear(GPIOB, GPIO12);
    
    uint32_t crl = GPIOA_CRL;
    rcc_periph_clock_enable(RCC_GPIOA);
    /* Enable Pull on GPIOA1. We don't rely on the external pin
     * really pulled, but only on the value of the CNF register
     * changed from the reset value
     */
    crl &= 0xffffff0f;
    crl |= 0x80;
    GPIOA_CRL = crl;
}
