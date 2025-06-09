#include "filter_FO.h"



void firstOrderIIR_Init(filterFOHandle handle,float alpha)
{
    firstOrderIIR_t *obj=(firstOrderIIR_t *) handle;
    //Constrain the coefficient boundary
    if(alpha<0.0f)
    {
        obj->alpha=0.0f;
    }
    else if (alpha>1.0f)
    {
        obj->alpha=1.0f;
    }
    else
    {
        obj->alpha=alpha;
    }
    //Reset filter output
    obj->out=0.0f;
}

//rc=1/freqCutoffHZ*ONE_OVER_TWO_PI
//alpha=((Ts-2*rc))/(2*rc+Ts);
float firstOrderIIR_Updata(filterFOHandle handle, float input)
{
    firstOrderIIR_t *obj=(firstOrderIIR_t *) handle;
    float inputValue=input;
    obj->out=(0.5f+0.5*obj->alpha)*(inputValue+obj->lastInput) - obj->alpha*obj->out;
    obj->lastInput=inputValue;
    return obj->out;
}

//rc=1/freqCutoffHZ*ONE_OVER_TWO_PI
//alpha=rc/(rc+Ts);
float firstOrderIIR_Updata_2(filterFOHandle handle, float input)
{
    firstOrderIIR_t *obj=(firstOrderIIR_t *) handle;
    float inputValue=input;
    obj->out=(1.0f-obj->alpha)*inputValue + obj->alpha*obj->out;
    return obj->out;
}

