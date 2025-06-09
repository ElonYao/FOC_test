#ifndef _CLARK_ELON_H
#define _CLARK_ELON_H

#ifdef __cplusplus
extern "C"
{
#endif

//Code area
typedef struct _clark_trans_
{
    float phaseCurrent_A[3];
    float alpha_A;
    float beta_A;
    unsigned char sensorNum; 
}clark_obj;

#define MATH_INVSQRT3 0.577350f

#define CLARK_DEFAULT {\
        {0.0f},\
        0.0f,\
        0.0f,\
        3,\
}

//clark transform macro
#define CLARK_MACRO(m)\
if(3==m.sensorNum)\
{\
    m.alpha_A=(m.phaseCurrent_A[0]*2.0f-m.phaseCurrent_A[1]-m.phaseCurrent_A[2])*MATH_ONE_OVER_THREE;\
    m.beta_A=(m.phaseCurrent_A[1]-m.phaseCurrent_A[2])*MATH_INVSQRT3;\
}\
else if(2==m.sensorNum)\
{\
    m.alpha_A=m.phaseCurrent_A[0];\
    m.beta_A=(m.phaseCurrent_A[0]+2.0f*m.phaseCurrent_A[1])*MATH_INVSQRT3;\
}


#ifdef __cplusplus
}
#endif


#endif 

