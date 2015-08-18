#ifndef __PLATFORM_H
#define __PLATFORM_H

#include <stdint.h>
#include <libopencm3/cm3/common.h>
#include <libopencm3/stm32/memorymap.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/usb/usbd.h>

#include <setjmp.h>
#include <alloca.h>

#include "gpio.h"
#include "timing.h"

#define PLATFORM_HAS_TRACESWO
#define BOARD_IDENT       "Black Magic Probe (RobLink), (Firmware " FIRMWARE_VERSION ")"
#define BOARD_IDENT_DFU   "Black Magic (Upgrade) for RobLink, (Firmware " FIRMWARE_VERSION ")"
#define BOARD_IDENT_UPD   "Black Magic (DFU Upgrade) for RobLink, (Firmware " FIRMWARE_VERSION ")"
#define DFU_IDENT         "Black Magic Firmware Upgrade (RobLink)"
#define DFU_IFACE_STRING  "@Internal Flash   /0x08000000/1*016Ka,3*016Kg,1*064Kg,7*128Kg"
#define UPD_IFACE_STRING  "@Internal Flash   /0x08000000/8*001Kg"

/* Hardware definitions... */
#define JTAG_PORT               GPIOA
#define TDI_PORT                JTAG_PORT
#define TMS_PORT                JTAG_PORT
#define TCK_PORT                JTAG_PORT
#define TDO_PORT                JTAG_PORT
#define TDI_PIN                 GPIO3
#define TMS_PIN                 GPIO4
#define TCK_PIN                 GPIO5
#define TDO_PIN                 GPIO6

#define SWDIO_PORT              JTAG_PORT
#define SWCLK_PORT              JTAG_PORT
#define SWDIO_PIN               TMS_PIN
#define SWCLK_PIN               TCK_PIN

#define TRST_PORT               GPIOA
#define TRST_PIN                GPIO1
#define PWR_BR_PORT             GPIOB
#define PWR_BR_PIN              GPIO0
#define SRST_PORT               GPIOA
#define SRST_PIN                GPIO2
#define SRST_SET_VAL(x)         platform_srst_set_val(x)

#define LED_PORT                GPIOB
#define LED_PORT_UART           GPIOB
#define LED_UART                GPIO4
#define LED_IDLE_RUN            GPIO3
#define LED_ERROR               GPIO5

#define TMS_SET_MODE()          gpio_set_mode(TMS_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, TMS_PIN);
#define SWDIO_MODE_FLOAT()      gpio_set_mode(SWDIO_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, SWDIO_PIN);
#define SWDIO_MODE_DRIVE()      gpio_set_mode(SWDIO_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, SWDIO_PIN);

#define USB_PU_PORT             GPIOA
#define USB_PU_PIN              GPIO8
#define USB_DRIVER              stm32f103_usb_driver
#define USB_IRQ                 NVIC_USB_LP_CAN_RX0_IRQ
#define USB_ISR                 usb_lp_can_rx0_isr
/* Interrupt priorities.  Low numbers are high priority.
 * For now USART1 preempts USB which may spin while buffer is drained.
 * TIM3 is used for traceswo capture and must be highest priority.
 */
#define IRQ_PRI_USB             (2 << 4)
#define IRQ_PRI_USBUSART        (1 << 4)
#define IRQ_PRI_USBUSART_TIM    (3 << 4)
#define IRQ_PRI_TRACE           (0 << 4)

#define USBUSART                USART3
#define USBUSART_CR1            USART3_CR1
#define USBUSART_SR             USART3_SR
#define USBUSART_IRQ            NVIC_USART3_IRQ
#define USBUSART_APB_ENR        RCC_APB1ENR
#define USBUSART_CLK_ENABLE     RCC_APB1ENR_USART3EN
#define USBUSART_PORT           GPIOB
#define USBUSART_PORT_APB_ENR   RCC_APB2ENR
#define USBUSART_PORT_ENABLE    RCC_APB2ENR_IOPAEN
#define USBUSART_TX_PIN         GPIO10
#define USBUSART_ISR            usart1_isr
#define USBUSART_TIM            TIM4
#define USBUSART_TIM_CLK_EN()   rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_TIM4EN)
#define USBUSART_TIM_IRQ        NVIC_TIM4_IRQ
#define USBUSART_TIM_ISR        tim4_isr
#define UART_PIN_SETUP()        gpio_set_mode(USBUSART_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, USBUSART_TX_PIN);

#define TRACE_TIM               TIM3
#define TRACE_TIM_CLK_EN()      rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_TIM3EN)
#define TRACE_IRQ               NVIC_TIM3_IRQ
#define TRACE_ISR               tim3_isr

#define SET_RUN_STATE(state)    {running_status = (state);}
#define SET_IDLE_STATE(state)    {gpio_set_val(LED_PORT, LED_IDLE_RUN, state);}
#define SET_ERROR_STATE(state)    {gpio_set_val(LED_PORT, LED_ERROR, state);}

#define RS485_CONTROL_PORT      GPIOB
#define RS485_CONTROL_PIN       GPIO14
#define RS485_INTERRUPT_PORT    GPIOB
#define RS485_INTERRUPT_PIN     GPIO15

/* Use newlib provided integer only stdio functions */
#define sscanf siscanf
#define sprintf siprintf
#define vasprintf vasiprintf

void platform_init(void);
const char *platform_target_voltage(void);
void platform_485transmit(bool enable);
void platform_srst_set_val(bool assert);
void platform_request_boot(void);
int platform_hwversion(void);
bool platform_target_get_power(void);
void platform_target_set_power(bool power);

#endif // __PLATFORM_H