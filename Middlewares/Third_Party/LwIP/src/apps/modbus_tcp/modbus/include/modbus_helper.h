#ifndef __MODBUS_H
#define __MODBUS_H

float32_t  	fourByteArreyToFloat(uint16_t pos, uint8_t* array);
uint32_t	arrayPionterToUint32(uint8_t* array);
float32_t   arrayMBToFloat(uint8_t* array);
void 		readFloatsFromMbArray(uint8_t* array, float32_t* floats, uint32_t floatCount);

#endif
