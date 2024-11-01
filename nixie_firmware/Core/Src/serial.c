/**
  ******************************************************************************
  * @file           : serial.c
  * @brief          : UART for RS232
  * @author         : Piotr Rzeszut
  ******************************************************************************
  */
#include "serial.h"
#include "ring_buffer.h"
#include "stm32f100xb.h"

extern UART_HandleTypeDef huart2;

RingBuffer TxBuffer[1];
RingBuffer RxBuffer[1];

#define BUFFER_SIZE 50

uint8_t DataBuffer[2][BUFFER_SIZE];

static uint8_t UART_GetIndex(UART_HandleTypeDef *huart)
{
  if (huart == &huart2)
  {
    return 0;
  }
  return 0;
}

void UART_RxISR(UART_HandleTypeDef *huart)
{
  if (huart->Instance->SR & USART_SR_RXNE)
  {
    __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_RXNE);
    char uhdata = (char) READ_REG(huart->Instance->DR);
    uint8_t idx = UART_GetIndex(huart);
    RingBuffer_PutChar(&RxBuffer[idx], uhdata);
  }
}

void UART_TxISR(UART_HandleTypeDef *huart)
{
  if (huart->Instance->SR & USART_SR_TXE)
  {
    __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_TXE);
    char uhdata;
    uint8_t idx = UART_GetIndex(huart);
    if(RingBuffer_GetChar(&TxBuffer[idx], &uhdata))
    {
      huart->Instance->DR = uhdata;
    }
    else
    {
      ATOMIC_CLEAR_BIT(huart->Instance->CR1, USART_CR1_TXEIE);
    }
  }
}

bool UART_GetChar(UART_HandleTypeDef *huart, char* c)
{
  uint8_t idx = UART_GetIndex(huart);
  return RingBuffer_GetChar(&RxBuffer[idx], c);
}

bool UART_PutChar(UART_HandleTypeDef *huart, char c)
{
  uint8_t idx = UART_GetIndex(huart);
  if (RingBuffer_PutChar(&TxBuffer[idx], c))
  {
    ATOMIC_SET_BIT(huart->Instance->CR1, USART_CR1_TXEIE);
    return true;
  }
  else
  {
    return false;
  }
}

size_t UART_WriteData(UART_HandleTypeDef *huart, const void *data, size_t dataSize){
	size_t i = 0;
	while(i<dataSize){
		if(UART_PutChar(huart, ((char*)data)[i])) i++;
	}
	return i;
}

size_t UART_WriteString(UART_HandleTypeDef *huart, const char *string){
	return UART_WriteData(huart, string, strlen(string));
}

void UART_Init(UART_HandleTypeDef *huart)
{
  uint8_t idx = UART_GetIndex(huart);

  RingBuffer_Init(&TxBuffer[idx], DataBuffer[idx * 2 + 0], BUFFER_SIZE, sizeof(char));
  RingBuffer_Init(&RxBuffer[idx], DataBuffer[idx * 2 + 1], BUFFER_SIZE, sizeof(char));

  ATOMIC_SET_BIT(huart->Instance->CR1, USART_CR1_RXNEIE);
}
