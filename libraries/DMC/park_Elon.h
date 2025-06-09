#ifndef _PARK_ELON_H
#define _PARK_ELON_H

#ifdef __cplusplus
extern "C"
{
#endif

//Code area
typedef struct 
{
    float alpha; // stationary coordination alpha input
    float beta; // stationary coordination beta input
    float theta;// the angle between Q axis and and alpha axis in radian
    MATH_Vec2 phasor;// for sine and cosine value
    float dOut;// rotating d axis output 
    float qOut; // rotating q axis output
}park_obj;


#define PARK_DEFAULT {\
    0.0f,\
    0.0f,\
    0.0f,\
    {0.0f},\
    0.0f,\
    0.0f,\
}

#define PARK_MACRO(m)\
    m.dOut=m.alpha*m.phasor.value[0]+m.beta*m.phasor.value[1];\
    m.qOut=m.beta*m.phasor.value[0]-m.alpha*m.phasor.value[1];\

#ifdef __cplusplus
}
#endif


#endif 

