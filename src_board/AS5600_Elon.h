#ifndef _AS5600_ELON_H
#define _AS5600_ELON_H


#ifdef __cplusplus
extern "C"
{
#endif
#include "driverlib.h"
#include <string.h>
#include "board.h"
#include "math.h"
#include "HAL_Elon.h"
#define MATH_RAW2DEG 0.087890625f
#define MATH_RAW2RAD 1.53398e-3f
#define POLEPAIRS 7.0f
#define MATH_7OVER2PI 1.11408460f
#define MATH_RAD2RPM 9.54929658f
#define ROTATIONDIR -1
//Code area
typedef struct _as5600_
{
    uint16_t sensorAddr;
    uint16_t regAddr;
    int16_t rawData;
    int16_t zeroPoint;
     int32_t turnCounter;
     int32_t vel_turnCounter;
    float zeroElecAngle;
     float angleMechanical;
     float angleMechanicalLast;
     float vel_angleMechanicalLast;
     float angleElectrical;


}AS5600_obj;

typedef struct _as5600_ *AS5600Handle;

 AS5600Handle HAL_AS5600_init(void *pMemory, const size_t numBytes);
 int HAL_readRaw(AS5600Handle handle);
void HAL_readRaw2(AS5600Handle handle);
void HAL_readRaw3(AS5600Handle handle);
void HAL_sensorAlignment(AS5600Handle handle);
void HAL_angleUpdate(AS5600Handle handle);
 float HAL_getElectricalAngle(AS5600Handle handle);
 float HAL_getFullAngle(AS5600Handle handle);
 float HAL_getRPM(AS5600Handle handle,float Ts);
#ifdef __cplusplus
}
#endif


#endif 

