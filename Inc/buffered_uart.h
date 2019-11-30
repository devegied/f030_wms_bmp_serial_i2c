/**
  ******************************************************************************
  * @file           : buffered_uart.h
  * @brief          : Header for buffered_uart.c file.
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BUFFERED_UART_H
#define __BUFFERED_UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f0xx_hal.h"
#include "fifo.h"

typedef struct __uartTypeDef {
  UART_HandleTypeDef *huart;
  FIFO *tx;
} uartTypeDef;

void uart_write(const char *buf, int len, uartTypeDef *uart);
void uart_buffer(const char *buf, int len, uartTypeDef *uart);
void uart_buffer_byte(const uint8_t buf, uartTypeDef *uart);
void uart_flush(uartTypeDef *uart);

#ifdef __cplusplus
}
#endif

#endif /* __BUFFERED_UART_H */

/************************ (C) COPYRIGHT devegied *****END OF FILE****/
