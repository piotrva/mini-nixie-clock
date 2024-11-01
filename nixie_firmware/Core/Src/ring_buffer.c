/* Includes ------------------------------------------------------------------*/
#include <assert.h>
#include "ring_buffer.h"
#include "stm32f1xx.h"


bool RingBuffer_Init(RingBuffer *ringBuffer, uint8_t *dataBuffer, size_t dataBufferSize, size_t elementSize) 
{
	assert(ringBuffer);
	assert(dataBuffer);
	assert(dataBufferSize > 0);
	assert(elementSize > 0);
	assert((dataBufferSize % elementSize) == 0);
	
	if ((ringBuffer) && (dataBuffer) && (dataBufferSize > 0)) {
	  	ringBuffer->dataBuffer = dataBuffer;
		ringBuffer->dataBufferSize = dataBufferSize;
		ringBuffer->elementSize = elementSize;
		ringBuffer->dataBufferLen = 0;
		ringBuffer->tailIdx = 0;
		
		return true;
	}
	
	return false;
}

bool RingBuffer_Clear(RingBuffer *ringBuffer)
{
	assert(ringBuffer);
	
	if (ringBuffer) {
		ringBuffer->dataBufferLen = 0;
		ringBuffer->tailIdx = 0;
		return true;
	}
	return false;
}

bool RingBuffer_IsEmpty(const RingBuffer *ringBuffer)
{
  assert(ringBuffer);	
	
	if (ringBuffer) {
		return ringBuffer->dataBufferLen == 0;
	}
	
	return true;
	
}

size_t RingBuffer_GetLen(const RingBuffer *ringBuffer)
{
	assert(ringBuffer);
	
	if (ringBuffer) {
		return ringBuffer->dataBufferLen / ringBuffer->elementSize;
	}
	return 0;
	
}

size_t RingBuffer_GetCapacity(const RingBuffer *ringBuffer)
{
	assert(ringBuffer);
	
	if (ringBuffer) {
		return ringBuffer->dataBufferSize / ringBuffer->elementSize;
	}
	return 0;	
}


bool RingBuffer_PutChar(RingBuffer *ringBuffer, char c)
{
	assert(ringBuffer);
	
	if (ringBuffer) {
		uint32_t prim = __get_PRIMASK();
		if (prim) __disable_irq();
		if(ringBuffer->dataBufferLen == ringBuffer->dataBufferSize){
			if (prim) __enable_irq();
			return false;
		}
		unsigned int writeIndex = (ringBuffer->tailIdx + ringBuffer->dataBufferLen) % ringBuffer->dataBufferSize;
		ringBuffer->dataBuffer[writeIndex] = (uint8_t)c;
		ringBuffer->dataBufferLen++;
		if (prim) __enable_irq();
		return true;
		
	}
	return false;
}

bool RingBuffer_GetChar(RingBuffer *ringBuffer, char *c)
{
	assert(ringBuffer);
	assert(c);
	
  	if ((ringBuffer) && (c)) {
	  	uint32_t prim = __get_PRIMASK();
		if (prim) __disable_irq();
		if(ringBuffer->dataBufferLen == 0){
			if (prim) __enable_irq();
			return false;
		}
		*c = (char)(ringBuffer->dataBuffer[ringBuffer->tailIdx]);
		ringBuffer->dataBufferLen--;
		ringBuffer->tailIdx = (ringBuffer->tailIdx + 1) % ringBuffer->dataBufferSize;
		if (prim) __enable_irq();
		return true;
	}
	return false;
}

bool RingBuffer_PutElement(RingBuffer *ringBuffer, void* element)
{
	assert(ringBuffer);
	assert(element);

	uint8_t *ptr = (uint8_t*)element;
	
	if ((ringBuffer) && (element)) {
		uint32_t prim = __get_PRIMASK();
		if (prim) __disable_irq();
		if(ringBuffer->dataBufferLen == ringBuffer->dataBufferSize){
			if (prim) __enable_irq();
			return false;
		}
		for(size_t i = 0 ; i < ringBuffer->elementSize ; i++)
		{
			unsigned int writeIndex = (ringBuffer->tailIdx + ringBuffer->dataBufferLen) % ringBuffer->dataBufferSize;
			ringBuffer->dataBuffer[writeIndex] = *ptr;
			ringBuffer->dataBufferLen++;
			ptr++;
		}
		if (prim) __enable_irq();
		return true;
		
	}
	return false;
}

bool RingBuffer_GetElement(RingBuffer *ringBuffer, void *element)
{
	assert(ringBuffer);
	assert(element);

	uint8_t *ptr = (uint8_t*)element;
	
  	if ((ringBuffer) && (element)) {
	  	uint32_t prim = __get_PRIMASK();
		if (prim) __disable_irq();
		if(ringBuffer->dataBufferLen == 0){
			if (prim) __enable_irq();
			return false;
		}
		for(size_t i = 0 ; i < ringBuffer->elementSize ; i++)
		{
			*ptr = (uint8_t)(ringBuffer->dataBuffer[ringBuffer->tailIdx]);
			ringBuffer->dataBufferLen--;
			ringBuffer->tailIdx = (ringBuffer->tailIdx + 1) % ringBuffer->dataBufferSize;
			ptr++;
		}
		if (prim) __enable_irq();
		return true;
	}
	return false;
}
