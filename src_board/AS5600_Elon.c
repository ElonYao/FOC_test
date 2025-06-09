#include "AS5600_Elon.h"


int HAL_readRaw(AS5600Handle handle)
{
    AS5600_obj *obj= (AS5600_obj*) handle;
    volatile int16_t rawData[2];

     if(I2C_isBusBusy(AS5600_BASE))
     {
         return -1;
     }
   // I2C_clearStatus(AS5600_BASE, I2C_STS_NO_ACK);
    I2C_setTargetAddress(AS5600_BASE,obj->sensorAddr);
    I2C_setConfig(AS5600_BASE,I2C_CONTROLLER_SEND_MODE|I2C_REPEAT_MODE);
    I2C_sendStartCondition(AS5600_BASE);

    I2C_putData(AS5600_BASE,obj->regAddr);
    while(!(I2C_getStatus(AS5600_BASE) & I2C_STS_REG_ACCESS_RDY));

    //change mode to receiver and initialize a restart
    I2C_setConfig(AS5600_BASE,I2C_CONTROLLER_RECEIVE_MODE|I2C_REPEAT_MODE);
    I2C_sendStartCondition(AS5600_BASE);

    //while(!(I2C_getStatus(AS5600_BASE) & I2C_STS_REG_ACCESS_RDY));
    rawData[0]=I2C_getData(AS5600_BASE);//low byte

    while(!(I2C_getStatus(AS5600_BASE) & I2C_STS_REG_ACCESS_RDY));
    rawData[1]=I2C_getData(AS5600_BASE);//high byte

    I2C_sendStopCondition(AS5600_BASE);//stop

    obj->rawData=((rawData[1]&0x0F)<<8)|rawData[0];
    return 0;
}


void HAL_readRaw3(AS5600Handle handle)
{
    AS5600_obj *obj= (AS5600_obj*) handle;
    uint16_t rawData[2]={0};

    while(I2C_isBusBusy(AS5600_BASE));

    I2C_setConfig(AS5600_BASE,I2C_CONTROLLER_SEND_MODE|I2C_REPEAT_MODE);
    I2C_sendStartCondition(AS5600_BASE);
   // while((I2C_getStatus(I2CB_BASE)& I2C_STS_NO_ACK));

    I2C_putData(AS5600_BASE,obj->regAddr);
    while(!(I2C_getStatus(AS5600_BASE) & I2C_STS_REG_ACCESS_RDY));

    //change mode to receiver and initialize a restart
    I2C_setConfig(AS5600_BASE,I2C_CONTROLLER_RECEIVE_MODE|I2C_REPEAT_MODE);
    I2C_sendStartCondition(AS5600_BASE);
    //while((I2C_getStatus(I2CB_BASE)& I2C_STS_NO_ACK));

    //I2C_getData(AS5600_BASE); //if read multipule byte, the receiver regs may be needed to clear at the beginning

    rawData[0]=I2C_getData(AS5600_BASE);//low byte
    while(!(I2C_getStatus(AS5600_BASE) & I2C_STS_REG_ACCESS_RDY));
    rawData[1]=I2C_getData(AS5600_BASE);//high byte

    I2C_sendStopCondition(AS5600_BASE);//stop

    obj->rawData=(rawData[1]<<8)|rawData[0];
}

void HAL_readRaw2(AS5600Handle handle)
{

    AS5600_obj *obj= (AS5600_obj*) handle;
    uint16_t rawData[2]={0};
    uint16_t timeout=1000;

    while(I2C_isBusBusy(AS5600_BASE)&& --timeout);

    I2C_setTargetAddress(AS5600_BASE,obj->sensorAddr);
    I2C_setConfig(AS5600_BASE,I2C_CONTROLLER_SEND_MODE|I2C_REPEAT_MODE);
    I2C_sendStartCondition(AS5600_BASE);

    I2C_putData(AS5600_BASE,obj->regAddr);
    while((I2C_getStatus(I2CB_BASE)& I2C_STS_NO_ACK));

    //restart
    I2C_setConfig(AS5600_BASE,I2C_CONTROLLER_RECEIVE_MODE|I2C_REPEAT_MODE);
    I2C_sendStartCondition(AS5600_BASE);

    while((I2C_getStatus(I2CB_BASE)& I2C_STS_NO_ACK));

    rawData[0]=I2C_getData(AS5600_BASE);//low byte
    while(!(I2C_getStatus(AS5600_BASE) & I2C_STS_RX_DATA_RDY));
    rawData[1]=I2C_getData(AS5600_BASE);//high byte

    I2C_sendStopCondition(AS5600_BASE);//stop

    obj->rawData=((rawData[1]&0x0F)<<8)|rawData[0];


}
void HAL_sensorAlignment(AS5600Handle handle)
{
    AS5600_obj *obj= (AS5600_obj*) handle;
    HAL_readRaw(handle);
    obj->zeroPoint=obj->rawData;
    obj->zeroElecAngle=rangeNormalize(ROTATIONDIR*obj->rawData*MATH_RAW2RAD*POLEPAIRS);
}

void HAL_angleUpdate(AS5600Handle handle)
{
    AS5600_obj *obj= (AS5600_obj*) handle;
    float delta=0;
    obj->angleMechanical=obj->rawData*MATH_RAW2RAD;
    delta=obj->angleMechanical-obj->angleMechanicalLast;
    if(fabsf(delta)>0.8f*MATH_TWO_PI)
    {
        if(delta>0)
        {
            obj->turnCounter--;
        }
        else if(delta<0)
        {
            obj->turnCounter++;
        }
    }
    obj->angleMechanicalLast=obj->angleMechanical;
}

float HAL_getElectricalAngle(AS5600Handle handle)
{
    AS5600_obj *obj= (AS5600_obj*) handle;
    return rangeNormalize(ROTATIONDIR*obj->angleMechanicalLast*POLEPAIRS-obj->zeroElecAngle);
}
float HAL_getFullAngle(AS5600Handle handle)
{
    AS5600_obj *obj= (AS5600_obj*) handle;

    return (float)obj->turnCounter*MATH_TWO_PI+obj->angleMechanicalLast-obj->zeroPoint*MATH_RAW2RAD;
}

float HAL_getRPM(AS5600Handle handle,float Ts)
{
    AS5600_obj *obj= (AS5600_obj*) handle;

    float angularVelocity=((float)(obj->turnCounter-obj->vel_turnCounter)*MATH_TWO_PI+(obj->angleMechanicalLast-obj->vel_angleMechanicalLast))/Ts;
    obj->vel_turnCounter=obj->turnCounter;
    obj->vel_angleMechanicalLast=obj->angleMechanicalLast;

    return angularVelocity*MATH_RAD2RPM;

}

AS5600Handle HAL_AS5600_init(void *pMemory, const size_t numBytes)
{
    AS5600Handle handle;
    AS5600_obj *obj;
    if(numBytes<sizeof(AS5600_obj))
    {
        return ((AS5600Handle)NULL);
    }

    handle=(AS5600Handle)pMemory;
    obj = (AS5600_obj *)handle;
    obj->sensorAddr=0x36;
    obj->regAddr=0x0E;
    obj->zeroElecAngle=0;
    obj->turnCounter=0;
    obj->angleMechanicalLast=0;
    obj->vel_turnCounter=0;
    obj->vel_angleMechanicalLast=0;
    return (handle);
}

