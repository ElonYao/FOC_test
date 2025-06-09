#ifndef _FILTERFO_H
#define _FILTERFO_H

#ifdef __cplusplus
extern "C"
{
#endif


typedef struct  _firstOrder_
{
    float alpha;
    float lastInput;
    float out;
}firstOrderIIR_t;

typedef struct _firstOrder_ *filterFOHandle;

//IIR filter functions
extern void firstOrderIIR_Init(filterFOHandle handle,float alpha);
extern float firstOrderIIR_Updata(filterFOHandle handle, float input);
extern float firstOrderIIR_Updata_2(filterFOHandle handle, float input);

#ifdef __cplusplus
}
#endif


#endif 

