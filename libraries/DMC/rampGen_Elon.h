#ifndef _RAMPGEN_ELON_H
#define _RAMPGEN_ELON_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "math.h"
//Code area
typedef struct _ramp_gen_
{
    float freq;
    float angleFactor;
    float angle;
    float offset;
    float gain;
    float out;
}rampGen_t;

#define RAMPGEN_DEFAULT {\
    \
    0.0f,\
    3.769911e-3f,/* (ctrlPeriod_sec * MATH_TWO_PI) */\
    0.0f,\
    0.0f,\
    1.0f,\
    0.0f,\
}
//w=2*pi*f  theta=w*t;
#define RAMPGEN_MACRO(m)\
\
    m.angle+=m.freq*m.angleFactor;\
    if(m.angle>MATH_TWO_PI)\
        m.angle-=MATH_TWO_PI;\
    else if(m.angle<0)\
        m.angle+=MATH_TWO_PI;\
    m.out=m.angle*m.gain+m.offset;\
    if(m.out>MATH_TWO_PI)\
        m.out-=MATH_TWO_PI;\
    else if(m.out<0)\
        m.out+=MATH_TWO_PI;\
        m.angle=m.out;\


#ifdef __cplusplus
}
#endif


#endif 

