//#############################################################################
//
// FILE:   FOCtest.c
//
// Author: Elon
// Date: 20241025
// Modified: None
//! This example is a project setup for Driverlib development.
//!
//
//#############################################################################
//
//
// $Copyright:
// Copyright (C) 2024 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//#############################################################################

//
// Included Files
//
#include "driverlib.h"
#include "device.h"
#include "board.h"
#include "c2000ware_libraries.h"


// user's header files
#include "AS5600_Elon.h"
#include "HAL_Elon.h"
#include "rmp_Elon.h"
#include "clark_Elon.h"
#include "park_Elon.h"
#include "rampGen_Elon.h"
#include "invpark_Elon.h"
#include "SVPWM_Elon.h"
#include "DLOG_4CH_F.h"
#include "pid_macro.h"
#include "filter_FO.h"
#include "comm_Elon.h"
#include "speedObserver_Elon.h"


//#pragma CODE_SECTION(INT_Motor_IS_ADC_1_ISR, ".TI.ramfunc");
#pragma INTERRUPT(INT_mainControlloop_ISR, {HP});


#define FO_ALPHA 0.5f // cutoff 15k

#define constrain(amt,low,high) ((amt<low)? low:((amt>high)? high:amt))
//Global
HAL_MOTOR_Obj Motor1;
HAL_MOTOR_Handle Motor1Handle;

AS5600_obj positionSensor;
AS5600Handle positionSensorHandle;
//as5600Obj *currentSensor=&positionSensor;


clark_obj clarkT1=CLARK_DEFAULT;
park_obj parkT1=PARK_DEFAULT;

RMPCNTL_t ramp1=RMPCNTL_DEFAULTS;
rampGen_t rampGen1=RAMPGEN_DEFAULT;

invPark_obj invParkT1=INVPARK_DEFAULT;

SVPWM_obj svpwm1=SVPWM_DEFAULT;

speedObserver_obj speedObserver;
speedObserverHandle speedObHandle;

//Speed control
PID_CONTROLLER PI_speed={
         PID_TERM_DEFAULTS,
         PID_PARAM_DEFAULTS,
         PID_DATA_DEFAULTS
};

//Iq control
PID_CONTROLLER PI_Iq={
         PID_TERM_DEFAULTS,
         PID_PARAM_DEFAULTS,
         PID_DATA_DEFAULTS
};
//Id control
PID_CONTROLLER PI_Id={
         PID_TERM_DEFAULTS,
         PID_PARAM_DEFAULTS,
         PID_DATA_DEFAULTS
};
//Position controller
PID_CONTROLLER PD_angle={
         PID_TERM_DEFAULTS,
         PID_PARAM_DEFAULTS,
         PID_DATA_DEFAULTS
};

canMsg_t motor1Message={
    .canId=0x100,
    .objID=1,
    .len=8,
    .tRate=20,
    .tRcounter=0,
    .errorCounter=0,
    .msgIDtype=CAN_MSG_FRAME_STD,
    .msgObjtype=CAN_MSG_OBJ_TYPE_TX
};

canMsg_t motorMaster={
    .canId=0x101,
    .objID=2,
    .len=8,
    .tRate=20,
    .tRcounter=0,
    .errorCounter=0,
    .msgIDtype=CAN_MSG_FRAME_STD,
    .msgObjtype=CAN_MSG_OBJ_TYPE_RX
};

//data log
// ****************************************************************************
// Variables for Datalog module
// ****************************************************************************
float DBUFF_4CH1[200],
      DBUFF_4CH2[200],
      DBUFF_4CH3[200],
      DBUFF_4CH4[200],
      DlogCh1,
      DlogCh2,
      DlogCh3,
      DlogCh4;
DLOG_4CH_F dlog_4ch1;

//__interrupt void INT_Motor_IS_ADC_1_ISR(void);//After adc converting finished
__interrupt void INT_CAN_ISR(void);

//
// Main
//
void main(void)
 {

    //
    // Initialize device clock and peripherals

    Device_init();

    //
    // Disable pin locks and enable internal pull-ups.
    //
    Device_initGPIO();

    //
    // Initialize PIE and clear PIE registers. Disables CPU interrupts.
    //
    Interrupt_initModule();

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //
    Interrupt_initVectorTable();

    //
    // PinMux and Peripheral Initialization
    //
    Board_init();
    //
    // C2000Ware Library initialization
    //
    C2000Ware_libraries_init();
    Interrupt_register(INT_CANA0, &INT_CAN_ISR);
    Interrupt_enable(INT_CANA0);

    Motor1Handle=HAL_MOTOR_init(&Motor1, sizeof(Motor1));
    positionSensorHandle=HAL_AS5600_init(&positionSensor, sizeof(positionSensor));

    speedObHandle=speedObserverInit(&speedObserver,sizeof(speedObserver));

    //angle control
    PD_angle.param.Kp=1000.0f;
    PD_angle.param.Kd=80.0f;
    PD_angle.param.Ki=0;
    PD_angle.param.Umax=200;
    PD_angle.param.Umin=-200;
    //Speed controller
    PI_speed.param.Kp=0.009f;
    PI_speed.param.Ki=0.005f;
    PI_speed.param.Umax=1.5f;
    PI_speed.param.Umin=-1.5f;

    //Iq controller
    PI_Iq.param.Kp=0.098f;
    PI_Iq.param.Ki=0.010f;
    //Id controller
    PI_Id.param.Kp=0.0f;
    PI_Id.param.Ki=0.0f;


    HAL_offsetReferenceCalibration(Motor1Handle);
    HAL_EnablePWM(Motor1Handle);

    /*For open loop test only******/

    //Motor1.speedTarget=250;
    // ****************************************************
    // Initialize DATALOG module
    // ****************************************************
        DLOG_4CH_F_init(&dlog_4ch1);
        dlog_4ch1.input_ptr1 = &DlogCh1;    //data value
        dlog_4ch1.input_ptr2 = &DlogCh2;
        dlog_4ch1.input_ptr3 = &DlogCh3;
        dlog_4ch1.input_ptr4 = &DlogCh4;
        dlog_4ch1.output_ptr1 = &DBUFF_4CH1[0];
        dlog_4ch1.output_ptr2 = &DBUFF_4CH2[0];
        dlog_4ch1.output_ptr3 = &DBUFF_4CH3[0];
        dlog_4ch1.output_ptr4 = &DBUFF_4CH4[0];
        dlog_4ch1.size = 200;
        dlog_4ch1.pre_scalar = 5;
        dlog_4ch1.trig_value = 0.01;
        dlog_4ch1.status = 2;

        //HAL_zeroCalibration(positionSensorHandle);

    //Initial position detection
    //HAL_IPD_Routine(Motor1Handle);

    //
    // Enable Global Interrupt (INTM) and real time interrupt (DBGM)
    //
    EINT;
    ERTM;


    while(1)
    {

        HAL_clearFlag_TZ(Motor1Handle);// For clearing OT trip event
        if(CPUTimer_getTimerOverflowStatus(dataPrint_BASE)){

            PD_angle.term.Fbk=ROTATIONDIR*HAL_getFullAngle(positionSensorHandle);
            PID_MACRO(PD_angle)
            PI_speed.term.Ref=PD_angle.term.Out;
            if(motor1Message.tRcounter<motor1Message.tRate)
            {
                motor1Message.tRcounter++;
            }
            else
            {
                updateCAN(&motor1Message);
            }
           // updateCAN(&motor1Message);
            CPUTimer_startTimer(dataPrint_BASE);
        }

    }
}
//Happen after ADCA SOC3 DC bus voltage conversion finish

__interrupt void INT_mainControlloop_ISR(void)
{
    Motor1.loopCounter++;
    //control code
    while(!ADC_getInterruptStatus(Motor_IS_ADC_BASE, ADC_INT_NUMBER1));
    HAL_ReadADC(&Motor1);

    if(Motor1.state==MOTOR_STOP)
    {
        invParkT1.dInput=0.0f;
        invParkT1.qInput=0.0f;
    }
    else if(Motor1.state==MOTOR_ALLIGNMENT)
    {
        //alignment process
        invParkT1.dInput=0.5f;
        invParkT1.qInput=0.0f;
        parkT1.theta=0;
        if(Motor1.stateCounterRun<Motor1.alignmentCount)
         {
            Motor1.stateCounterRun++;
            HAL_sensorAlignment(positionSensorHandle);
         }
        else
        {
                invParkT1.qInput=0.0f;
                invParkT1.dInput=0.0f;
                Motor1.state=MOTOR_READY;
                Motor1.stateCounterRun=0;
                //Control output phase voltage amplitude
               PI_Iq.param.Umax=0.35f*Motor1.VdcBus_V;
               PI_Iq.param.Umin=-PI_Iq.param.Umax;
        }
      }
    //for test only
    if(Motor1.state==MOTOR_READY) ramp1.targetValue=Motor1.speedTarget/60;
    else ramp1.targetValue=0;

    RC_MACRO(ramp1)

    //rampGen1.freq=ramp1.setPoint;
   // PI_speed.term.Ref=ramp1.setPoint*60;
    //RAMPGEN_MACRO(rampGen1)

    //clark transform to get alpha and beta current
    clarkT1.phaseCurrent_A[0]=Motor1.I_A.value[0];
    clarkT1.phaseCurrent_A[1]=Motor1.I_A.value[1];
    clarkT1.phaseCurrent_A[2]=Motor1.I_A.value[2];
    CLARK_MACRO(clarkT1)

    if(Motor1.state==MOTOR_READY)
    {
        HAL_readRaw(positionSensorHandle);
        HAL_angleUpdate(positionSensorHandle);
        //Motor1.rawSpeed=HAL_getRPM(positionSensorHandle,0.0006f);
        speedObserverRun(speedObHandle,positionSensor.angleMechanicalLast);
       // Motor1.filteredSpeed=Motor1.filteredSpeed*0.999f+Motor1.rawSpeed*0.001f;
       //PI_speed.term.Fbk= ROTATIONDIR*Motor1.filteredSpeed;
        //speedObserverRun(speedObHandle,positionSensor.angleMechanicalLast);
        Motor1.filteredSpeed=Motor1.filteredSpeed*0.99f+speedObserver.speedRPM*0.01f;
        PI_speed.term.Fbk= ROTATIONDIR*Motor1.filteredSpeed;
        PID_MACRO(PI_speed)
        PI_Iq.term.Ref=PI_speed.term.Out;
        //Dual motor control
        //PI_Iq.term.Ref=constrain(0.25f*(PD_angle.term.Ref-PD_angle.term.Fbk),-PI_Iq.param.Umax,PI_Iq.param.Umax);

    }


    if(Motor1.state==MOTOR_READY) parkT1.theta=HAL_getElectricalAngle(positionSensorHandle);
    //park and ipark share the same angle
    parkT1.phasor.value[0]=invParkT1.phasor.value[0]=__cos(parkT1.theta);
    parkT1.phasor.value[1]=invParkT1.phasor.value[1]=__sin(parkT1.theta);
    //park transform to Id and Iq
    parkT1.alpha=clarkT1.alpha_A;
    parkT1.beta=clarkT1.beta_A;
    PARK_MACRO(parkT1)

    PI_Iq.term.Fbk=0.99f*PI_Iq.term.Fbk+0.01f*parkT1.qOut;
    PI_Id.term.Fbk=0.99f*PI_Id.term.Fbk+0.01f*parkT1.dOut;
    //set the output limit of PI controller of Ud and Uq in current loop;
    Motor1.vsMax_V=VS_VOLTAGE_MAX_RATIO*Motor1.VdcBus_V;
    PI_Id.param.Umax=Motor1.vsMax_V;
    PI_Id.param.Umin=-Motor1.vsMax_V;
    PID_MACRO(PI_Id)

    //PI_Iq.param.Umax=__sqrt(Motor1.vsMax_V*Motor1.vsMax_V-PI_Id.term.Out*PI_Id.term.Out);
    //PI_Iq.param.Umin=-PI_Iq.param.Umax;
    PID_MACRO(PI_Iq)

    if(Motor1.state==MOTOR_READY)
    {
        invParkT1.qInput=PI_Iq.term.Out;
        //invParkT1.dInput=PI_Id.term.Out;
    }


    //In open-loop control, angle is generated by a ramp function,and the Uq is a preset value.
    INVPARK_MACRO(invParkT1)
    svpwm1.invDCbusVoltage=1.0f/Motor1.VdcBus_V;
    svpwm1.alpha_V=invParkT1.alphaOut;
    svpwm1.beta_V=invParkT1.betaOut;
    SVPWM_MACRO(svpwm1)

    HAL_WritePWM(Motor1Handle,&svpwm1);


    DlogCh1 =PI_speed.term.Fbk;
    DlogCh2 = invParkT1.theta;
    DlogCh3 =positionSensor.angleMechanical;
    DlogCh4 = rampGen1.out;
    DLOG_4CH_F_FUNC(&dlog_4ch1);
    //HAL_ackADCinterrupt();
    //ADC_clearInterruptStatus(Motor_IS_ADC_BASE, ADC_INT_NUMBER1);
    Interrupt_clearACKGroup(INT_mainControlloop_INTERRUPT_ACK_GROUP);
}
__interrupt void INT_Motor_IS_ADC_1_ISR(void)
{
   //blank function
}
__interrupt void INT_CAN_ISR(void)
{
    uint32_t status=CAN_getInterruptCause(myCAN0_BASE);
    uint32_t temp=0;
    if(CAN_INT_INT0ID_STATUS==status)
    {
        status=CAN_getStatus(myCAN0_BASE);
        if(((status & ~(CAN_STATUS_RXOK))!=CAN_STATUS_LEC_MSK) && ((status & ~(CAN_STATUS_RXOK))!=CAN_STATUS_LEC_NONE) )
        {
            motorMaster.errorCounter++;
        }
    }
    else if(status == motorMaster.objID)//received message object ID, not message ID
    {
        CAN_readMessage(myCAN0_BASE, motorMaster.objID, (uint16_t *)motorMaster.data);
        temp|=motorMaster.data[3];
        temp<<=8;
        temp|=motorMaster.data[2];
        PD_angle.term.Ref=(float)temp*0.0030518f-100;
        CAN_clearInterruptStatus(myCAN0_BASE, motorMaster.objID);

        if(motorMaster.errorCounter--==0)motorMaster.errorCounter=0;

    }

    CAN_clearGlobalInterruptStatus(myCAN0_BASE, CAN_GLOBAL_INT_CANINT0);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}
//
// End of File
//

