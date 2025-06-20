#include "PID_Elon.h"

pidHandle pidControllerInit(void *pMemory, const size_t numBytes)
{
    pidHandle handle;
    PIDController_t * obj;
    
    if(numBytes < sizeof(PIDController_t))
    {
        return((pidHandle)NULL);
    }

    // assign the handle
    handle = (pidHandle)pMemory;

    // assign the object
    obj = (PIDController_t *)handle;
    //variables initialization section
    obj->N=100;//only valid for derivative term
    obj->flag_AntiWindUp=0;
    obj->kp=0.3f;
    obj->ti=0.7f;
    obj->inv_ti=1.0f/obj->ti;
    obj->td=0.0f;
    obj->ts=0.001f;
    obj->half_ts=0.5*obj->ts;
    obj->I_term=0.0f;
    obj->D_term=0.0f;
    obj->D_term_coe1=(2.0f*obj->N)/(2.0f+obj->N*obj->ts);
    obj->D_term_coe2=(2.0f-obj->N*obj->ts)/(2.0f+obj->N*obj->ts);
    obj->iterm_Max=300.0f;
    obj->iterm_Min=-300.0f;
    obj->outMax=200.0f;
    obj->outMin=-200.0f;
    obj->lastError=0.0f;
    obj->lastMeasure=0.0f;
    obj->out=0.0f;
    return handle;

}

float updatePIDcontroller(pidHandle handle)
{
   PIDController_t * obj = (PIDController_t *) handle;
    //get the error
    float error=obj->refInput-obj->fbValue;
    float result=0.0f;
    //Integrator part

     //obj->I_term=obj->half_ts*obj->inv_ti*(error+obj->lastError)+obj->I_term;

    if(obj->flag_AntiWindUp)
    {
        obj->I_term=((obj->I_term<obj->iterm_Min)? obj->iterm_Min : ((obj->I_term>obj->iterm_Max)? obj->iterm_Max : obj->I_term));
    }

    //derivative part
   // obj->D_term=(2*obj->td*N)/(2+N*Ts)*(error-obj->lastError)+(2.0f-N*Ts)/(2.0f+N*Ts)*obj->D_term;
    obj->D_term=obj->td*obj->D_term_coe1*(error-obj->lastError)+obj->D_term_coe2*obj->D_term;

    result=(obj->I_term+error+obj->D_term)*obj->kp;

    result=((result<obj->outMin)? obj->outMin : ((result>obj->outMax)? obj->outMax : result));
    obj->out=result;
    obj->lastError=error;

    return result;
}

void setKp(pidHandle handle,float kp)
{
    PIDController_t * obj = (PIDController_t *) handle;
    obj->kp=kp;
}
void setKi(pidHandle handle,float ki)
{
    PIDController_t * obj = (PIDController_t *) handle;
    obj->ti=ki;
    obj->inv_ti=1.0f/ki;
}
void setKd(pidHandle handle,float kd)
{
    PIDController_t * obj = (PIDController_t *) handle;
    obj->td=kd;
}
void setTs(pidHandle handle,float ts)
{
    PIDController_t * obj = (PIDController_t *) handle;
    obj->ts=ts;
    obj->half_ts=0.5*ts;
    obj->D_term_coe1=(2.0f*obj->N)/(2.0f+obj->N*obj->ts);
    obj->D_term_coe2=(2.0f-obj->N*obj->ts)/(2.0f+obj->N*obj->ts);
}

void setOutputLimt(pidHandle handle,float max,float min)
{
    PIDController_t * obj = (PIDController_t *) handle;
    obj->outMax=max;
    obj->outMin=min;
}
