#ifndef _INVPARK_ELON_H
#define _INVPARK_ELON_H
#include "math.h"
#ifdef __cplusplus
extern "C"
{
#endif

//Code area
typedef struct 
{
    float dInput; // rotating d axis input
    float qInput; // rotating q axis input
    float theta;// the angle between Q axis and and alpha axis in radian
    MATH_Vec2 phasor;// for sine and cosine value
    float alphaOut;// stationary coordination alpha output
    float betaOut; // stationary coordination beta ouput
}invPark_obj;


#define INVPARK_DEFAULT {\
    0.0f,\
    0.0f,\
    0.0f,\
    {0.0f},\
    0.0f,\
    0.0f,\
}

#define INVPARK_MACRO(m)\
    m.alphaOut=m.dInput*m.phasor.value[0]-m.qInput*m.phasor.value[1];\
    m.betaOut=m.qInput*m.phasor.value[0]+m.dInput*m.phasor.value[1];\

#ifdef __cplusplus
}
#endif


#endif 

