#ifndef __PLATFORM_H
#define __PLATFORM_H

#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/usb/usbd.h>

#include <setjmp.h>
#include <alloca.h>

#include "gdb_packet.h"

#define INLINE_GPIO
#define CDCACM_PACKET_SIZE      64
#define PLATFORM_HAS_TRACESWO
#define BOARD_IDENT             "Black Magic Probe"
#define BOARD_IDENT_DFU         "Black Magic Probe (Upgrade)"
#define BOARD_IDENT_UPD         "Black Magic Probe (DFU Upgrade)"
#define DFU_IDENT               "Black Magic Firmware Upgrade"
#define DFU_IFACE_STRING        "@Internal Flash   /0x08000000/8*001Ka,120*001Kg"
#define UPD_IFACE_STRING        "@Internal Flash   /0x08000000/8*001Kg"

extern usbd_device *usbdev;
#define CDCACM_GDB_ENDPOINT     1
#define CDCACM_UART_ENDPOINT    3

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
#define PWR_BR_PIN              GPIO1
#define SRST_PORT               GPIOA
#define SRST_PIN                GPIO2

#define USB_PU_PORT             GPIOA
#define USB_PU_PIN              GPIO8

#define LED_PORT                GPIOB
#define LED_PORT_UART           GPIOB
#define LED_UART                GPIO4
#define LED_IDLE_RUN            GPIO3
#define LED_ERROR               GPIO5

#define TMS_SET_MODE()          gpio_set_mode(TMS_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, TMS_PIN);
#define SWDIO_MODE_FLOAT()      gpio_set_mode(SWDIO_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, SWDIO_PIN);
#define SWDIO_MODE_DRIVE()      gpio_set_mode(SWDIO_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, SWDIO_PIN);

#define UART_PIN_SETUP()        gpio_set_mode(USBUSART_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, USBUSART_TX_PIN);

#define SRST_SET_VAL(x)         platform_srst_set_val(x)

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

#define USBUSART                USART1
#define USBUSART_CR1            USART1_CR1
#define USBUSART_IRQ            NVIC_USART1_IRQ
#define USBUSART_APB_ENR        RCC_APB2ENR
#define USBUSART_CLK_ENABLE     RCC_APB2ENR_USART1EN
#define USBUSART_PORT           GPIOA
#define USBUSART_PORT_APB_ENR   RCC_APB2ENR
#define USBUSART_PORT_ENABLE    RCC_APB2ENR_IOPAEN
#define USBUSART_TX_PIN         GPIO9
#define USBUSART_ISR            usart1_isr
#define USBUSART_TIM            TIM4
#define USBUSART_TIM_CLK_EN()   rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_TIM4EN)
#define USBUSART_TIM_IRQ        NVIC_TIM4_IRQ
#define USBUSART_TIM_ISR        tim4_isr

#define TRACE_TIM               TIM3
#define TRACE_TIM_CLK_EN()      rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_TIM3EN)
#define TRACE_IRQ               NVIC_TIM3_IRQ
#define TRACE_ISR               tim3_isr

//#define DEBUG(...) printf(__VA_ARGS__)
#define DEBUG(...)

extern uint8_t running_status;
extern volatile uint32_t timeout_counter;

extern jmp_buf fatal_error_jmpbuf;

extern const char *morse_msg;

#define gpio_set_val(port, pin, val) do {    \
    if(val)                    \
        gpio_set((port), (pin));    \
    else                    \
        gpio_clear((port), (pin));    \
} while(0)

#define SET_RUN_STATE(state)    {running_status = (state);}
#define SET_IDLE_STATE(state)    {gpio_set_val(LED_PORT, LED_IDLE_RUN, state);}
#define SET_ERROR_STATE(state)    {gpio_set_val(LED_PORT, LED_ERROR, state);}

#define PLATFORM_SET_FATAL_ERROR_RECOVERY()    {setjmp(fatal_error_jmpbuf);}
#define PLATFORM_FATAL_ERROR(error)    do {         \
    if(running_status) gdb_putpacketz("X1D");    \
        else gdb_putpacketz("EFF");        \
    running_status = 0;                \
    target_list_free();                \
    morse("TARGET LOST.", 1);            \
    longjmp(fatal_error_jmpbuf, (error));        \
} while (0)

int platform_init(void);
void morse(const char *msg, char repeat);
const char *platform_target_voltage(void);
int platform_hwversion(void);
void platform_delay(uint32_t delay);

/* <cdcacm.c> */
void cdcacm_init(void);
/* Returns current usb configuration, or 0 if not configured. */
int cdcacm_get_config(void);
int cdcacm_get_dtr(void);

/* <platform.h> */
void uart_usb_buf_drain(uint8_t ep);

/* Use newlib provided integer only stdio functions */
#define sscanf siscanf
#define sprintf siprintf
#define vasprintf vasiprintf

#ifdef INLINE_GPIO
static inline void _gpio_set(uint32_t gpioport, uint16_t gpios)
{
    GPIO_BSRR(gpioport) = gpios;
}
#define gpio_set _gpio_set

static inline void _gpio_clear(uint32_t gpioport, uint16_t gpios)
{
    GPIO_BRR(gpioport) = gpios;
}
#define gpio_clear _gpio_clear

static inline uint16_t _gpio_get(uint32_t gpioport, uint16_t gpios)
{
    return (uint16_t)GPIO_IDR(gpioport) & gpios;
}
#define gpio_get _gpio_get
#endif

#endif

#define disconnect_usb() gpio_set_mode(USB_PU_PORT, GPIO_MODE_INPUT, 0, USB_PU_PIN);
void assert_boot_pin(void);
void setup_vbus_irq(void);
void platform_srst_set_val(bool assert);
