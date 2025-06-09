#include "speedObserver_Elon.h"

speedObserverHandle speedObserverInit(void *pmemory,const size_t numBytes)
{
    speedObserverHandle handle;
    speedObserver_obj *obj;

    if(numBytes < sizeof(speedObserver_obj))
    {
        return((speedObserverHandle)NULL);
    }

    // assign the handle
    handle = (speedObserverHandle)pmemory;
    obj= (speedObserver_obj*) handle;
    obj->thetaStep=0.0006;// observer period
    obj->max=100.0f*MATH_TWO_PI;
    obj->min=-obj->max;
    obj->Kp=200.0f;
    obj->Ki=0.003f;// 50*observer period
    obj->fbv=0.0f;
    obj->Ui=0.0f;
    obj->speedRPM=0;
    obj->filteredAbsRPM=0;
    obj->filteredRPM=0;
    return(handle);
}

void speedObserverRun(speedObserverHandle handle,float input)
{
    speedObserver_obj *obj=(speedObserver_obj*) handle;
    obj->ref=input;
    obj->err=obj->ref-obj->fbv;
    //[-pi~pi] to ensure the continuity
    if(obj->err>=MATH_PI) obj->err-=MATH_TWO_PI;
    else if(obj->err<=-MATH_PI) obj->err+=MATH_TWO_PI;


    obj->Up=obj->err*obj->Kp;
    obj->Ui+=obj->err*obj->Ki;
    obj->Ui=(obj->Ui>obj->max)? obj->max:((obj->Ui<obj->min)? obj->min : obj->Ui);

    //obj->out=(1-0.9408826f)*(obj->Up+obj->Ui)+0.9408826f*obj->out;//first order LPF fc=1Hz
    obj->out=obj->Up+obj->Ui;
    obj->out=(obj->out>obj->max)? obj->max:((obj->out<obj->min)? obj->min : obj->out);

    obj->speedRPM=obj->out*RADS2RPM;

    // feedback signal 
    obj->fbv+=obj->out*obj->thetaStep;
    if(obj->fbv>=MATH_PI) obj->fbv-=MATH_TWO_PI;
    else if(obj->fbv<=-MATH_PI) obj->fbv+=MATH_TWO_PI;
}

void speedObserverReset(speedObserverHandle handle)
{
    speedObserver_obj *obj=(speedObserver_obj*) handle;
    obj->fbv=0.0f;
    obj->Ui=0.0f;
}

