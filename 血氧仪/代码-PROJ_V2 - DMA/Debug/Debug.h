#ifndef __DEBUG_H
#define __DEBUG_H

#include "CW32l031.h"                   // Device header
#include "cw32l031_rcc.h"
#include "cw32l031_gpio.h"
#include "cw32l031_uart.h"

//UARTx
#define  DEBUG_USARTx                   CW_UART1
#define  DEBUG_USART_CLK                RCC_APB2_PERIPH_UART1
#define  DEBUG_USART_APBClkENx          RCC_APBPeriphClk_Enable2
#define  DEBUG_USART_BaudRate           9600
#define  DEBUG_USART_UclkFreq           64000000

//UARTx GPIO
#define  DEBUG_USART_GPIO_CLK           RCC_AHB_PERIPH_GPIOA
#define  DEBUG_USART_TX_GPIO_PORT       CW_GPIOA
#define  DEBUG_USART_TX_GPIO_PIN        GPIO_PIN_8
#define  DEBUG_USART_RX_GPIO_PORT       CW_GPIOA
#define  DEBUG_USART_RX_GPIO_PIN        GPIO_PIN_9

//GPIO AF
#define  DEBUG_USART_AFTX               PA08_AFx_UART1TXD()
#define  DEBUG_USART_AFRX               PA09_AFx_UART1RXD()


void UART_Configuration(void);


#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line);
#endif

#endif
