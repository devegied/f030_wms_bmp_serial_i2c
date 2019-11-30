/**
  ******************************************************************************
  * @file           : buffered_uart.c
  * @brief          : buffered UART API implementation
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 devegied.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by devegied under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
#include "buffered_uart.h"
void uart_write_and_flush(const uint8_t *buf, int len, uartTypeDef *uart,uint8_t flush){
    while (len) {
		if (fifo_put(uart->tx, *buf) == 1U) {//byte successfuly added to fifo
			++buf;
			--len;
		} else {
			break;
		}
        if(flush)
		    /* enable the "transmit register empty" interrupt to start things flowing */
		    SET_BIT(uart->huart->Instance->CR1, USART_CR1_TXEIE);//equivalent to __HAL_UART_ENABLE_IT(uart->huart, UART_IT_TXE);
	}
}
/* writes to transmitter fifo, can loose bytes if fifo is full, starts transmition with the first added byte */
void uart_write(const char *buf, int len, uartTypeDef *uart){
    uart_write_and_flush((const uint8_t *)buf,len,uart,1U);
}
/* writes to transmitter fifo, can loose bytes if fifo is full, does not start transmition */
void uart_buffer(const char *buf, int len, uartTypeDef *uart){
    uart_write_and_flush((const uint8_t *)buf,len,uart,0U);
}
/* writes to transmitter fifo, can loose byte if fifo is full, does not start transmition */
void uart_buffer_byte(const uint8_t buf, uartTypeDef *uart){
    uart_write_and_flush(&buf,1,uart,0U);
}
void uart_flush(uartTypeDef *uart){
    if(!fifo_isEmpty(uart->tx))
        SET_BIT(uart->huart->Instance->CR1, USART_CR1_TXEIE);//equivalent to __HAL_UART_ENABLE_IT(uart->huart, UART_IT_TXE);
}
