/**
  ******************************************************************************
  * @file           : serial.h
  * @brief          : UART for RS232
  * @author         : Piotr Rzeszut
  ******************************************************************************
  */
#ifndef __SERIAL_H
#define __SERIAL_H

#include "main.h"
#include <string.h>

void UART_Init(UART_HandleTypeDef *huart);
bool UART_GetChar(UART_HandleTypeDef *huart, char* c);
bool UART_PutChar(UART_HandleTypeDef *huart, char c);
size_t UART_WriteData(UART_HandleTypeDef *huart, const void *data, size_t dataSize);
size_t UART_WriteString(UART_HandleTypeDef *huart, const char *string);

void UART_RxISR(UART_HandleTypeDef *huart);
void UART_TxISR(UART_HandleTypeDef *huart);

#endif /* __SERIAL_H */
