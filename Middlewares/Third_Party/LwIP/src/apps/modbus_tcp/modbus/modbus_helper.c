#include <stm32h7xx.h>

#include <arm_math.h>
#include "modbus_helper.h"
#include "MB_HoldingReg_MAP.h"
#include "MB_InputReg_MAP.h"

///////////////////////////////////////////////

float32_t  fourByteArreyToFloat(uint16_t pos, uint8_t* array){
	uint8_t  buf4byte [4], i;
	float32_t tmpFloat=0;
	for (i = 0; i < 4 ; i++)
		buf4byte [i]= array[pos+3-i];
	tmpFloat = *(float32_t*)buf4byte;
	return tmpFloat;
}

uint32_t arrayPionterToUint32(uint8_t* array){
	uint32_t result = 0;
	result |= *array++ << 8;
	result |= *array++;
	result |= *array++ << 24;
	result |= *array++ << 16;
	return result;
}

float32_t arrayMBToFloat(uint8_t* array){
	uint32_t tmp = arrayPionterToUint32(array);
	float32_t* fPointer = (float32_t*) &tmp;
	return *fPointer;
}

void readFloatsFromMbArray(uint8_t* array, float32_t* floats, uint32_t floatCount){
	size_t i;
	for (i = 0; i < floatCount; ++i) {
		floats[i] = arrayMBToFloat(array + 4*i);
	}
}

