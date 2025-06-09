#include "HAL_Elon.h"

HAL_MOTOR_Handle HAL_MOTOR_init(void *pMemory, const size_t numBytes)
{
        HAL_MOTOR_Handle handle;
        HAL_MOTOR_Obj *obj;

       if(numBytes < sizeof(HAL_MOTOR_Obj))
       {
           return((HAL_MOTOR_Handle)NULL);
       }

       // assign the handle
       handle = (HAL_MOTOR_Handle)pMemory;

       // assign the object
       obj = (HAL_MOTOR_Obj *)handle;
       //variables initialization section

       obj->flag_PWM_ON=false;
       obj->flag_offsetDone=false;
       obj->flag_RevDirection=true;
       obj->flag_InitialPositionDected=false;
       obj->state=MOTOR_STOP;
       obj->pwmHandle[0]=PhaseU_BASE;
       obj->pwmHandle[1]=PhaseV_BASE;
       obj->pwmHandle[2]=PhaseW_BASE;
       obj->adcHandle[0]=Motor_IU_ADC_ADC_BASE;
       obj->adcHandle[1]=Motor_IV_ADC_ADC_BASE;
       obj->adcHandle[2]=Motor_IW_ADC_ADC_BASE;
       obj->current_sf=-0.0080566f;
       obj->voltage_sf=0.0198972f;
       obj->dcBusvoltage_sf=0.0198972f;
       obj->gateEnableGPIO=GATE_Control;
       obj->loopCounter=0;
       obj->speedTarget=0;
       obj->controllerPeriod=66.667e-6f; // =1000/15/1e6
       obj->vsMax_V=0.0f;
       obj->vQmax_V=0.0f;
       obj->idRef_A=0.0f;
       obj->iqRef_A=0.0f;
       obj->alignmentCount=2500;
       obj->stateCounterRun=0;
       obj->filteredSpeed=0;
       return handle;

}
//Enable inverter board pwm input path
void HAL_EnablePWM(HAL_MOTOR_Handle handle)
{
    HAL_MOTOR_Obj *obj=(HAL_MOTOR_Obj *)handle;

    //enable code
    obj->flag_PWM_ON=true;
    GPIO_writePin(obj->gateEnableGPIO, 0);
}
//disable inverter board pwm input path, pwm output is not influenced
void HAL_DisablePWM(HAL_MOTOR_Handle handle)
{
    HAL_MOTOR_Obj *obj=(HAL_MOTOR_Obj *)handle;

    //enable code

    obj->flag_PWM_ON=false;
    GPIO_writePin(obj->gateEnableGPIO, 1);
}

void HAL_WritePWM(HAL_MOTOR_Handle handle,SVPWM_obj *svgen)
{
    HAL_MOTOR_Obj* obj=(HAL_MOTOR_Obj*) handle;
    uint16_t period=EPWM_getTimeBasePeriod(obj->pwmHandle[0]);
    uint16_t index;
    float temp=__fsat(-svgen->Tu,0.5f,-0.5f)+0.5f; // saturate the SV module dutycycle output to 0.0~1.0
    obj->pwmCCdata[0]=(uint16_t)(temp*period);

    temp=__fsat(-svgen->Tv,0.5f,-0.5f)+0.5f; // saturate the SV module dutycycle output to 0.0~1.0
    obj->pwmCCdata[1]=(uint16_t)(temp*period);

    temp=__fsat(-svgen->Tw,0.5f,-0.5f)+0.5f; // saturate the SV module dutycycle output to 0.0~1.0
    obj->pwmCCdata[2]=(uint16_t)(temp*period);

    for(index=0;index<3;index++)
    {
        if(obj->pwmCCdata[index]<20U)
        {
            obj->pwmCCdata[index]=20U;
        }
       EPWM_setCounterCompareValue(obj->pwmHandle[index], EPWM_COUNTER_COMPARE_A, obj->pwmCCdata[index]);
       EPWM_setCounterCompareValue(obj->pwmHandle[index], EPWM_COUNTER_COMPARE_B, obj->pwmCCdata[index]);
    }

    return;
}

void HAL_ackADCinterrupt(void)
{
    ADC_clearInterruptStatus(Motor_IS_ADC_BASE, ADC_INT_NUMBER1);
    Interrupt_clearACKGroup(INT_Motor_IS_ADC_1_INTERRUPT_ACK_GROUP);
}

void HAL_ReadADC(HAL_MOTOR_Handle handle)
{
    HAL_MOTOR_Obj *obj=(HAL_MOTOR_Obj *) handle;
    float32_t value;

    //Phase U current read
    value=(float32_t)ADC_readPPBResult(Motor_IU_ADC_RESULT_BASE, Motor_IS_ADC_PPB1);
    obj->I_A.value[0]=value*obj->current_sf;

    //Phase V current read
     value=(float32_t)ADC_readPPBResult(Motor_IV_ADC_RESULT_BASE, myADC0_PPB1);
     obj->I_A.value[1]=value*obj->current_sf;

     //Phase W current read
     value=(float32_t)ADC_readPPBResult(Motor_IW_ADC_RESULT_BASE, myADC1_PPB1);
     obj->I_A.value[2]=value*obj->current_sf;

     //Phase U voltage read
      value=(float32_t)ADC_readResult(Motor_VU_ADC_RESULT_BASE, Motor_VU_ADC);
      //value=(float32_t)ADC_readPPBResult(Motor_VU_ADC_RESULT_BASE, Motor_IS_ADC_PPB2);
      obj->V_V.value[0]=value*obj->voltage_sf;

    //Phase V voltage read
      value=(float32_t)ADC_readResult(Motor_VV_ADC_RESULT_BASE, Motor_VV_ADC);
     // value=(float32_t)ADC_readPPBResult(Motor_VV_ADC_RESULT_BASE, myADC0_PPB2);
      obj->V_V.value[1]=value*obj->voltage_sf;

      //Phase W voltage read
      value=(float32_t)ADC_readResult(Motor_VW_ADC_RESULT_BASE, Motor_VW_ADC);
      //value=(float32_t)ADC_readPPBResult(Motor_VW_ADC_RESULT_BASE, myADC1_PPB2);
      obj->V_V.value[2]=value*obj->voltage_sf;

    //DC bus voltage read
     value=(float32_t)ADC_readResult(Motor_Vdc_ADC_RESULT_BASE,Motor_Vdc_ADC);
     obj->VdcBus_V=value*obj->dcBusvoltage_sf;

}

void HAL_offsetReferenceCalibration(HAL_MOTOR_Handle handle)
{
    HAL_MOTOR_Obj* obj=(HAL_MOTOR_Obj*) handle;
    obj->flag_offsetDone=false;
    uint16_t index;
    float32_t invCurrentSf = 1.0f / obj->current_sf;
   // float32_t invVoltageSf=1.0f/obj->voltage_sf;
#ifdef LPFC
    float32_t offsetK1 = 0.998001f;  // Offset filter coefficient K1: 0.05/(T+0.05);
    float32_t offsetK2 = 0.001999f;  // Offset filter coefficient K2: T/(T+0.05);
#endif
    ADC_setVREF(obj->adcHandle[0], ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);
    ADC_setVREF(obj->adcHandle[1], ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);
    ADC_setVREF(obj->adcHandle[2], ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);
    //For current measure
    ADC_setPPBReferenceOffset(Motor_IU_ADC_ADC_BASE,ADC_PPB_NUMBER1,0);
    ADC_setPPBReferenceOffset(Motor_IV_ADC_ADC_BASE,ADC_PPB_NUMBER1,0);
    ADC_setPPBReferenceOffset(Motor_IW_ADC_ADC_BASE,ADC_PPB_NUMBER1,0);

    obj->offset_I_A.value[0]=2048*obj->current_sf;
    obj->offset_I_A.value[1]=2048*obj->current_sf;
    obj->offset_I_A.value[2]=2048*obj->current_sf;

    /*
    //For voltage measure
    ADC_setPPBReferenceOffset(Motor_IU_ADC_ADC_BASE,ADC_PPB_NUMBER2,0U);
    ADC_setPPBReferenceOffset(Motor_IV_ADC_ADC_BASE,ADC_PPB_NUMBER2,0U);
    ADC_setPPBReferenceOffset(Motor_IW_ADC_ADC_BASE,ADC_PPB_NUMBER2,0U);
    */
    //use 50% dutycycle to calibrate the current offset
    obj->pwmCCdata[0]=1500;
    obj->pwmCCdata[1]=1500;
    obj->pwmCCdata[2]=1500;

    for(index=0;index<3;index++)
    {
        EPWM_setCounterCompareValue(obj->pwmHandle[index], EPWM_COUNTER_COMPARE_A, obj->pwmCCdata[index]);
        EPWM_setCounterCompareValue(obj->pwmHandle[index], EPWM_COUNTER_COMPARE_B, obj->pwmCCdata[index]);
    }
    HAL_EnablePWM(obj);

    for(index=0;index<32000;index++)
    {
        ADC_clearInterruptStatus(Motor_IS_ADC_BASE, ADC_INT_NUMBER1);
        while(!ADC_getInterruptStatus(Motor_IS_ADC_BASE, ADC_INT_NUMBER1));
        HAL_ReadADC(obj);
        if(index>2000)
        {
           /*
            //volatge offset recursive average
            obj->offset_V_V.value[0]=obj->offset_V_V.value[0]*(index-2001)/(index-2000)+obj->V_V.value[0]/(index-2000);
            obj->offset_V_V.value[1]=obj->offset_V_V.value[1]*(index-2001)/(index-2000)+obj->V_V.value[1]/(index-2000);
            obj->offset_V_V.value[2]=obj->offset_V_V.value[2]*(index-2001)/(index-2000)+obj->V_V.value[2]/(index-2000);
            */
#ifdef LPFC
            obj->offset_I_A.value[0] =
                                    offsetK1 * obj->offset_I_A.value[0] +
                                    obj->I_A.value[0] * offsetK2;
            obj->offset_I_A.value[1] =
                                    offsetK1 * obj->offset_I_A.value[1] +
                                    obj->I_A.value[1] * offsetK2;
            obj->offset_I_A.value[2] =
                                    offsetK1 * obj->offset_I_A.value[2] +
                                    obj->I_A.value[2] * offsetK2;
#else
            //current offset recursive average
                       obj->offset_I_A.value[0]=obj->offset_I_A.value[0]*(index-2001)/(index-2000)+obj->I_A.value[0]/(index-2000);
                       obj->offset_I_A.value[1]=obj->offset_I_A.value[1]*(index-2001)/(index-2000)+obj->I_A.value[1]/(index-2000);
                       obj->offset_I_A.value[2]=obj->offset_I_A.value[2]*(index-2001)/(index-2000)+obj->I_A.value[2]/(index-2000);
#endif
        }
        else if(index<1000)
        {
            HAL_EnablePWM(obj);
        }
    }
    HAL_DisablePWM(obj);
    //Change them back to raw value and write to PPB offset calibration
    obj->offset_I_A.value[0]*=invCurrentSf;
    obj->offset_I_A.value[1]*=invCurrentSf;
    obj->offset_I_A.value[2]*=invCurrentSf;


    ADC_setPPBReferenceOffset(Motor_IU_ADC_ADC_BASE,ADC_PPB_NUMBER1,(uint16_t)obj->offset_I_A.value[0]);
    ADC_setPPBReferenceOffset(Motor_IV_ADC_ADC_BASE,ADC_PPB_NUMBER1,(uint16_t)obj->offset_I_A.value[1]);
    ADC_setPPBReferenceOffset(Motor_IW_ADC_ADC_BASE,ADC_PPB_NUMBER1,(uint16_t)obj->offset_I_A.value[2]);
    /*
    obj->offset_V_V.value[0]*=invVoltageSf;
    obj->offset_V_V.value[1]*=invVoltageSf;
    obj->offset_V_V.value[2]*=invVoltageSf;
    ADC_setPPBReferenceOffset(Motor_VU_ADC_ADC_BASE,ADC_PPB_NUMBER2,(uint16_t)obj->offset_V_V.value[0]);
    ADC_setPPBReferenceOffset(Motor_VV_ADC_ADC_BASE,ADC_PPB_NUMBER2,(uint16_t)obj->offset_V_V.value[1]);
    ADC_setPPBReferenceOffset(Motor_VW_ADC_ADC_BASE,ADC_PPB_NUMBER2,(uint16_t)obj->offset_V_V.value[2]);
    */
    obj->flag_offsetDone=true;
    return;

}

void HAL_clearFlag_TZ(HAL_MOTOR_Handle handle)
{
    HAL_MOTOR_Obj *obj=(HAL_MOTOR_Obj *) handle;
    if((EPWM_getTripZoneFlagStatus(obj->pwmHandle[0]) & EPWM_TZ_FLAG_OST) && (EPWM_getTripZoneFlagStatus(obj->pwmHandle[1]) & EPWM_TZ_FLAG_OST)
            && (EPWM_getTripZoneFlagStatus(obj->pwmHandle[2]) & EPWM_TZ_FLAG_OST))
    {

        EPWM_clearTripZoneFlag(obj->pwmHandle[0], EPWM_TZ_FLAG_OST |EPWM_TZ_INTERRUPT);
        EPWM_clearTripZoneFlag(obj->pwmHandle[1], EPWM_TZ_FLAG_OST |EPWM_TZ_INTERRUPT);
        EPWM_clearTripZoneFlag(obj->pwmHandle[2], EPWM_TZ_FLAG_OST |EPWM_TZ_INTERRUPT);
    }
    return;
}

void HAL_IPD_U2VW(HAL_MOTOR_Handle handle)
{
      HAL_MOTOR_Obj *obj= (HAL_MOTOR_Obj *) handle;
      unsigned index;
      float rawValue[2]={0.0f};

               //U->V phase test
               //use TRIP ZONE one shot event to stop all pwm output
              EPWM_forceTripZoneEvent(obj->pwmHandle[0], EPWM_TZ_FORCE_EVENT_OST);
              EPWM_forceTripZoneEvent(obj->pwmHandle[1], EPWM_TZ_FORCE_EVENT_OST);
              EPWM_forceTripZoneEvent(obj->pwmHandle[2], EPWM_TZ_FORCE_EVENT_OST);

              //Disable all previous continuous software force actions
              EPWM_setActionQualifierContSWForceAction(obj->pwmHandle[0], EPWM_AQ_OUTPUT_A, EPWM_AQ_SW_DISABLED);
              EPWM_setActionQualifierContSWForceAction(obj->pwmHandle[1], EPWM_AQ_OUTPUT_A, EPWM_AQ_SW_DISABLED);
              EPWM_setActionQualifierContSWForceAction(obj->pwmHandle[2], EPWM_AQ_OUTPUT_A, EPWM_AQ_SW_DISABLED);

              //enable pwm output switch on the driver board
              HAL_EnablePWM(handle);
              //enable V phase low side first,otherwise there will be a negative impulse on the first excitation run!!!!!!
              EPWM_setActionQualifierContSWForceAction(obj->pwmHandle[1], EPWM_AQ_OUTPUT_A, EPWM_AQ_SW_OUTPUT_LOW);
              EPWM_clearTripZoneFlag(obj->pwmHandle[1], EPWM_TZ_FLAG_OST);

              //U phase compare counter setting
              EPWM_disableCounterCompareShadowLoadMode(obj->pwmHandle[0], EPWM_COUNTER_COMPARE_A);
              EPWM_setCounterCompareValue(obj->pwmHandle[0], EPWM_COUNTER_COMPARE_A, 2400U);

              //EPWM ADC trigger settings
              EPWM_enableADCTrigger(obj->pwmHandle[0], EPWM_SOC_A);
              EPWM_setADCTriggerSource(obj->pwmHandle[0], EPWM_SOC_A, EPWM_SOC_TBCTR_D_CMPC);
              EPWM_setADCTriggerEventPrescale(obj->pwmHandle[0], EPWM_SOC_A, 1);
              // add a 600ns(TBCTR number 72) delay to SOC trigger point to skip the output fluctuation at the falling edge
              EPWM_setCounterCompareValue(obj->pwmHandle[0], EPWM_COUNTER_COMPARE_C, 2328U);
              //set phase shift to rising edge when phase U enable
              EPWM_enablePhaseShiftLoad(obj->pwmHandle[0]);
              EPWM_setPhaseShift(obj->pwmHandle[0], 2400U);


              //set U phase current ADC trigger to EPWM1 trigger event
              ADC_setupSOC(obj->adcHandle[0], ADC_SOC_NUMBER1, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN11, 14U);
              // set current sampling channel as interrupt flag
              ADC_setInterruptSource(obj->adcHandle[0], ADC_INT_NUMBER1, ADC_SOC_NUMBER1);
              ADC_disableContinuousMode(obj->adcHandle[0], ADC_INT_NUMBER1);
              ADC_enableInterrupt(obj->adcHandle[0], ADC_INT_NUMBER1);


              for(index=0;index<2;index++)
              {

                  //Apply 10us high PWM waveform to U->V, and monitor the current changing on the inline shunt of U phase
                  ADC_clearInterruptStatus(obj->adcHandle[0], ADC_INT_NUMBER1);
                  EPWM_forceSyncPulse(obj->pwmHandle[0]); //force a sync event to make the pwm start with a high
                  EPWM_clearTripZoneFlag(obj->pwmHandle[0], EPWM_TZ_FLAG_OST);//enable phase U output
                  DEVICE_DELAY_US(45);
                  EPWM_forceTripZoneEvent(obj->pwmHandle[0], EPWM_TZ_FORCE_EVENT_OST);//stop phase U output

                  while(!ADC_getInterruptStatus(obj->adcHandle[0], ADC_INT_NUMBER1));
                  rawValue[index]=(float32_t)ADC_readPPBResult(Motor_IU_ADC_RESULT_BASE, ADC_PPB_NUMBER1);
                  DEVICE_DELAY_US(150);
              }
              HAL_DisablePWM(handle);
              obj->ipdCurrent[0]=0.5f*(rawValue[0]+rawValue[1])*obj->current_sf;

              //U->W phase test
              //force phase U and phase V trip zone events to stop the output on them
              EPWM_forceTripZoneEvent(obj->pwmHandle[0], EPWM_TZ_FORCE_EVENT_OST);
              EPWM_forceTripZoneEvent(obj->pwmHandle[1], EPWM_TZ_FORCE_EVENT_OST);
              //disable the phase V software force action from the previous U->V test
              EPWM_setActionQualifierContSWForceAction(obj->pwmHandle[1], EPWM_AQ_OUTPUT_A, EPWM_AQ_SW_DISABLED);

              //enable pwm output switch on the driver board
              HAL_EnablePWM(handle);
              //enable W phase low side first,otherwise there will be a negative impulse on the first excitation run!!!!!!
              EPWM_setActionQualifierContSWForceAction(obj->pwmHandle[2], EPWM_AQ_OUTPUT_A, EPWM_AQ_SW_OUTPUT_LOW);
              EPWM_clearTripZoneFlag(obj->pwmHandle[2], EPWM_TZ_FLAG_OST);
              DEVICE_DELAY_US(9);//key parameter to stabilize the first current reading!!!
              for(index=0;index<2;index++)
              {

                  //Apply 10us high PWM waveform to U->W, and monitor the current changing on the inline shunt of U phase
                  ADC_clearInterruptStatus(obj->adcHandle[0], ADC_INT_NUMBER1);
                  EPWM_forceSyncPulse(obj->pwmHandle[0]); //force a sync event to make the pwm start with a high
                  EPWM_clearTripZoneFlag(obj->pwmHandle[0], EPWM_TZ_FLAG_OST);//enable phase U output
                  DEVICE_DELAY_US(45);
                  EPWM_forceTripZoneEvent(obj->pwmHandle[0], EPWM_TZ_FORCE_EVENT_OST);//stop phase U output

                  while(!ADC_getInterruptStatus(obj->adcHandle[0], ADC_INT_NUMBER1));
                  rawValue[index]=(float32_t)ADC_readPPBResult(Motor_IU_ADC_RESULT_BASE, ADC_PPB_NUMBER1);
                  DEVICE_DELAY_US(150);
              }
              HAL_DisablePWM(handle);
              obj->ipdCurrent[1]=0.5f*(rawValue[0]+rawValue[1])*obj->current_sf;

              return;

}
void HAL_IPD_V2UW(HAL_MOTOR_Handle handle)
{
      HAL_MOTOR_Obj *obj= (HAL_MOTOR_Obj *) handle;
      unsigned index;
      float rawValue[2]={0.0f};

          //V->U phase test
          //use TRIP ZONE one shot event to stop all pwm output
         EPWM_forceTripZoneEvent(obj->pwmHandle[0], EPWM_TZ_FORCE_EVENT_OST);
         EPWM_forceTripZoneEvent(obj->pwmHandle[1], EPWM_TZ_FORCE_EVENT_OST);
         EPWM_forceTripZoneEvent(obj->pwmHandle[2], EPWM_TZ_FORCE_EVENT_OST);
         //Disable all previous continuous software force actions
         EPWM_setActionQualifierContSWForceAction(obj->pwmHandle[0], EPWM_AQ_OUTPUT_A, EPWM_AQ_SW_DISABLED);
         EPWM_setActionQualifierContSWForceAction(obj->pwmHandle[1], EPWM_AQ_OUTPUT_A, EPWM_AQ_SW_DISABLED);
         EPWM_setActionQualifierContSWForceAction(obj->pwmHandle[2], EPWM_AQ_OUTPUT_A, EPWM_AQ_SW_DISABLED);
         //enable pwm output switch on the driver board
         HAL_EnablePWM(handle);
         //enable U phase low side first,otherwise there will be a negative impulse on the first excitation run!!!!!!
         EPWM_setActionQualifierContSWForceAction(obj->pwmHandle[0], EPWM_AQ_OUTPUT_A, EPWM_AQ_SW_OUTPUT_LOW);
         EPWM_clearTripZoneFlag(obj->pwmHandle[0], EPWM_TZ_FLAG_OST);
         //V phase compare counter setting
         EPWM_disableCounterCompareShadowLoadMode(obj->pwmHandle[1], EPWM_COUNTER_COMPARE_A);
         EPWM_setCounterCompareValue(obj->pwmHandle[1], EPWM_COUNTER_COMPARE_A, 2400U);

         //EPWM ADC trigger settings
         EPWM_enableADCTrigger(obj->pwmHandle[1], EPWM_SOC_A);
         EPWM_setADCTriggerSource(obj->pwmHandle[1], EPWM_SOC_A, EPWM_SOC_TBCTR_D_CMPC);
         EPWM_setADCTriggerEventPrescale(obj->pwmHandle[1], EPWM_SOC_A, 1);
         // add a 600ns(TBCTR number 72) delay to SOC trigger point to skip the output fluctuation at the falling edge
         EPWM_setCounterCompareValue(obj->pwmHandle[1], EPWM_COUNTER_COMPARE_C, 2328U);
         //set phase shift to rising edge when phase V enable
         EPWM_enablePhaseShiftLoad(obj->pwmHandle[1]);
         EPWM_setPhaseShift(obj->pwmHandle[1], 2400U);


         //set V phase current ADC trigger to EPWM2 trigger event
         ADC_setupSOC(obj->adcHandle[1], ADC_SOC_NUMBER1, ADC_TRIGGER_EPWM2_SOCA, ADC_CH_ADCIN12, 14U);
         // set current sampling channel as interrupt flag
         ADC_setInterruptSource(obj->adcHandle[1], ADC_INT_NUMBER1, ADC_SOC_NUMBER1);
         ADC_disableContinuousMode(obj->adcHandle[1], ADC_INT_NUMBER1);
         ADC_enableInterrupt(obj->adcHandle[1], ADC_INT_NUMBER1);


         for(index=0;index<2;index++)
         {

             //Apply 10us high PWM waveform to V->U, and monitor the current changing on the inline shunt of V phase
             ADC_clearInterruptStatus(obj->adcHandle[1], ADC_INT_NUMBER1);
             EPWM_forceSyncPulse(obj->pwmHandle[1]); //force a sync event to make the pwm start with a high
             EPWM_clearTripZoneFlag(obj->pwmHandle[1], EPWM_TZ_FLAG_OST);//enable phase V output
             DEVICE_DELAY_US(45);
             EPWM_forceTripZoneEvent(obj->pwmHandle[1], EPWM_TZ_FORCE_EVENT_OST);//stop phase V output

             while(!ADC_getInterruptStatus(obj->adcHandle[1], ADC_INT_NUMBER1));
             rawValue[index]=(float32_t)ADC_readPPBResult(Motor_IV_ADC_RESULT_BASE, ADC_PPB_NUMBER1);
             DEVICE_DELAY_US(150);
         }
         HAL_DisablePWM(handle);
         obj->ipdCurrent[2]=0.5f*(rawValue[0]+rawValue[1])*obj->current_sf;

         //V->W phase test
         //force phase U and phase V trip zone events to stop the output on them
         EPWM_forceTripZoneEvent(obj->pwmHandle[0], EPWM_TZ_FORCE_EVENT_OST);
         EPWM_forceTripZoneEvent(obj->pwmHandle[1], EPWM_TZ_FORCE_EVENT_OST);
         //disable the phase U software force action from the previous V->U test
         EPWM_setActionQualifierContSWForceAction(obj->pwmHandle[0], EPWM_AQ_OUTPUT_A, EPWM_AQ_SW_DISABLED);

         //enable pwm output switch on the driver board
         HAL_EnablePWM(handle);
         //enable W phase low side first,otherwise there will be a negative impulse on the first excitation run!!!!!!
         EPWM_setActionQualifierContSWForceAction(obj->pwmHandle[2], EPWM_AQ_OUTPUT_A, EPWM_AQ_SW_OUTPUT_LOW);
         EPWM_clearTripZoneFlag(obj->pwmHandle[2], EPWM_TZ_FLAG_OST);
         DEVICE_DELAY_US(9);//key parameter to stabilize the first current reading!!!
         for(index=0;index<2;index++)
         {

             //Apply 10us high PWM waveform to V->W, and monitor the current changing on the inline shunt of V phase
             ADC_clearInterruptStatus(obj->adcHandle[1], ADC_INT_NUMBER1);
             EPWM_forceSyncPulse(obj->pwmHandle[1]); //force a sync event to make the pwm start with a high
             EPWM_clearTripZoneFlag(obj->pwmHandle[1], EPWM_TZ_FLAG_OST);//enable phase V output
             DEVICE_DELAY_US(45);
             EPWM_forceTripZoneEvent(obj->pwmHandle[1], EPWM_TZ_FORCE_EVENT_OST);//stop phase V output

             while(!ADC_getInterruptStatus(obj->adcHandle[1], ADC_INT_NUMBER1));
             rawValue[index]=(float32_t)ADC_readPPBResult(Motor_IV_ADC_RESULT_BASE, ADC_PPB_NUMBER1);
             DEVICE_DELAY_US(150);
         }
         HAL_DisablePWM(handle);
         obj->ipdCurrent[3]=0.5f*(rawValue[0]+rawValue[1])*obj->current_sf;
         return;
}

void HAL_IPD_W2UV(HAL_MOTOR_Handle handle)
{
      HAL_MOTOR_Obj *obj= (HAL_MOTOR_Obj *) handle;
      unsigned index;
      float rawValue[2]={0.0f};

         //W->U phase test
         //use TRIP ZONE one shot event to stop all pwm output
        EPWM_forceTripZoneEvent(obj->pwmHandle[0], EPWM_TZ_FORCE_EVENT_OST);
        EPWM_forceTripZoneEvent(obj->pwmHandle[1], EPWM_TZ_FORCE_EVENT_OST);
        EPWM_forceTripZoneEvent(obj->pwmHandle[2], EPWM_TZ_FORCE_EVENT_OST);
        //Disable all previous continuous software force actions
        EPWM_setActionQualifierContSWForceAction(obj->pwmHandle[0], EPWM_AQ_OUTPUT_A, EPWM_AQ_SW_DISABLED);
        EPWM_setActionQualifierContSWForceAction(obj->pwmHandle[1], EPWM_AQ_OUTPUT_A, EPWM_AQ_SW_DISABLED);
        EPWM_setActionQualifierContSWForceAction(obj->pwmHandle[2], EPWM_AQ_OUTPUT_A, EPWM_AQ_SW_DISABLED);
        //enable pwm output switch on the driver board
        HAL_EnablePWM(handle);

        //enable U phase low side first,otherwise there will be a negative impulse on the first excitation run!!!!!!
        EPWM_setActionQualifierContSWForceAction(obj->pwmHandle[0], EPWM_AQ_OUTPUT_A, EPWM_AQ_SW_OUTPUT_LOW);
        EPWM_clearTripZoneFlag(obj->pwmHandle[0], EPWM_TZ_FLAG_OST);

        //W phase compare counter setting
        EPWM_disableCounterCompareShadowLoadMode(obj->pwmHandle[2], EPWM_COUNTER_COMPARE_A);
        EPWM_setCounterCompareValue(obj->pwmHandle[2], EPWM_COUNTER_COMPARE_A, 2400U);

        //EPWM ADC trigger settings
        EPWM_enableADCTrigger(obj->pwmHandle[2], EPWM_SOC_A);
        EPWM_setADCTriggerSource(obj->pwmHandle[2], EPWM_SOC_A, EPWM_SOC_TBCTR_D_CMPC);
        EPWM_setADCTriggerEventPrescale(obj->pwmHandle[2], EPWM_SOC_A, 1);
        // add a 600ns(TBCTR number 72) delay to SOC trigger point to skip the output fluctuation at the falling edge
        EPWM_setCounterCompareValue(obj->pwmHandle[2], EPWM_COUNTER_COMPARE_C, 2328U);
        //set phase shift to rising edge when phase W enable
        EPWM_enablePhaseShiftLoad(obj->pwmHandle[2]);
        EPWM_setPhaseShift(obj->pwmHandle[2], 2400U);


        //set W phase current ADC trigger to EPWM2 trigger event
        ADC_setupSOC(obj->adcHandle[2], ADC_SOC_NUMBER1, ADC_TRIGGER_EPWM6_SOCA, ADC_CH_ADCIN3, 14U);
        // set current sampling channel as interrupt flag
        ADC_setInterruptSource(obj->adcHandle[2], ADC_INT_NUMBER1, ADC_SOC_NUMBER1);
        ADC_disableContinuousMode(obj->adcHandle[2], ADC_INT_NUMBER1);
        ADC_enableInterrupt(obj->adcHandle[2], ADC_INT_NUMBER1);


        for(index=0;index<2;index++)
        {

            //Apply 10us high PWM waveform to W->U, and monitor the current changing on the inline shunt of W phase
            ADC_clearInterruptStatus(obj->adcHandle[2], ADC_INT_NUMBER1);
            EPWM_forceSyncPulse(obj->pwmHandle[2]); //force a sync event to make the pwm start with a high
            EPWM_clearTripZoneFlag(obj->pwmHandle[2], EPWM_TZ_FLAG_OST);//enable phase W output
            DEVICE_DELAY_US(45);
            EPWM_forceTripZoneEvent(obj->pwmHandle[2], EPWM_TZ_FORCE_EVENT_OST);//stop phase W output

            while(!ADC_getInterruptStatus(obj->adcHandle[2], ADC_INT_NUMBER1));
            rawValue[index]=(float32_t)ADC_readPPBResult(Motor_IW_ADC_RESULT_BASE, ADC_PPB_NUMBER1);
            DEVICE_DELAY_US(150);
        }

        HAL_DisablePWM(handle);

        obj->ipdCurrent[4]=0.5f*(rawValue[0]+rawValue[1])*obj->current_sf;

        //W->V phase test
        //force phase W and phase U trip zone events to stop the output on them
        EPWM_forceTripZoneEvent(obj->pwmHandle[2], EPWM_TZ_FORCE_EVENT_OST);
        EPWM_forceTripZoneEvent(obj->pwmHandle[0], EPWM_TZ_FORCE_EVENT_OST);
        //disable the phase U software force action from the previous W->U test
        EPWM_setActionQualifierContSWForceAction(obj->pwmHandle[0], EPWM_AQ_OUTPUT_A, EPWM_AQ_SW_DISABLED);

        //enable pwm output switch on the driver board
        HAL_EnablePWM(handle);
        //enable V phase low side first,otherwise there will be a negative impulse on the first excitation run!!!!!!
        EPWM_setActionQualifierContSWForceAction(obj->pwmHandle[1], EPWM_AQ_OUTPUT_A, EPWM_AQ_SW_OUTPUT_LOW);
        EPWM_clearTripZoneFlag(obj->pwmHandle[1], EPWM_TZ_FLAG_OST);
        DEVICE_DELAY_US(9);//key parameter to stabilize the first current reading!!!

        for(index=0;index<2;index++)
        {

            //Apply 10us high PWM waveform to W->U, and monitor the current changing on the inline shunt of W phase
            ADC_clearInterruptStatus(obj->adcHandle[2], ADC_INT_NUMBER1);
            EPWM_forceSyncPulse(obj->pwmHandle[2]); //force a sync event to make the pwm start with a high
            EPWM_clearTripZoneFlag(obj->pwmHandle[2], EPWM_TZ_FLAG_OST);//enable phase W output
            DEVICE_DELAY_US(45);
            EPWM_forceTripZoneEvent(obj->pwmHandle[2], EPWM_TZ_FORCE_EVENT_OST);//stop phase W output

            while(!ADC_getInterruptStatus(obj->adcHandle[2], ADC_INT_NUMBER1));
            rawValue[index]=(float32_t)ADC_readPPBResult(Motor_IW_ADC_RESULT_BASE, ADC_PPB_NUMBER1);
            DEVICE_DELAY_US(150);
        }

        HAL_DisablePWM(handle);

        obj->ipdCurrent[5]=0.5f*(rawValue[0]+rawValue[1])*obj->current_sf;

        return;
}

void HAL_IPD_Routine(HAL_MOTOR_Handle handle)
{
    HAL_MOTOR_Obj *obj= (HAL_MOTOR_Obj *) handle;
    uint16_t index,n=0;
    float maxValue=0.0f;
    //ADC reference voltage setting
    ADC_setVREF(obj->adcHandle[0], ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);
    ADC_setVREF(obj->adcHandle[1], ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);
    ADC_setVREF(obj->adcHandle[2], ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);
    //ADC post processing block offset setting
    ADC_setPPBReferenceOffset(Motor_IU_ADC_ADC_BASE,ADC_PPB_NUMBER1,2042U);
    ADC_setPPBReferenceOffset(Motor_IV_ADC_ADC_BASE,ADC_PPB_NUMBER1,2044U);
    ADC_setPPBReferenceOffset(Motor_IW_ADC_ADC_BASE,ADC_PPB_NUMBER1,2045U);

    ADC_setPPBReferenceOffset(Motor_VU_ADC_ADC_BASE,ADC_PPB_NUMBER2,0U);
    ADC_setPPBReferenceOffset(Motor_VV_ADC_ADC_BASE,ADC_PPB_NUMBER2,0U);
    ADC_setPPBReferenceOffset(Motor_VW_ADC_ADC_BASE,ADC_PPB_NUMBER2,0U);

    //continuous software force mode immediate effect
    EPWM_setActionQualifierContSWForceShadowMode(obj->pwmHandle[0], EPWM_AQ_SW_IMMEDIATE_LOAD);
    EPWM_setActionQualifierContSWForceShadowMode(obj->pwmHandle[1], EPWM_AQ_SW_IMMEDIATE_LOAD);
    EPWM_setActionQualifierContSWForceShadowMode(obj->pwmHandle[2], EPWM_AQ_SW_IMMEDIATE_LOAD);

    //de-energize operation
    EPWM_setActionQualifierContSWForceAction(obj->pwmHandle[0], EPWM_AQ_OUTPUT_A, EPWM_AQ_SW_OUTPUT_LOW);
    EPWM_setActionQualifierContSWForceAction(obj->pwmHandle[1], EPWM_AQ_OUTPUT_A, EPWM_AQ_SW_OUTPUT_LOW);
    EPWM_setActionQualifierContSWForceAction(obj->pwmHandle[2], EPWM_AQ_OUTPUT_A, EPWM_AQ_SW_OUTPUT_LOW);
    HAL_EnablePWM(handle);
    DEVICE_DELAY_US(1000);//key parameter for de-energize operation,may change based on actual motor
    HAL_DisablePWM(handle);

    HAL_IPD_U2VW(handle);
    HAL_IPD_V2UW(handle);
    HAL_IPD_W2UV(handle);
    resettingAfterIPD(handle);
    for(index=0;index<6;index++)
    {
        if(MATH_abs(obj->ipdCurrent[index])>maxValue)
        {
            maxValue=MATH_abs(obj->ipdCurrent[index]);
            n=index;
        }
    }
    obj->InitialPosition=n;
    obj->flag_InitialPositionDected=true;
    return;
}
 void resettingAfterIPD(HAL_MOTOR_Handle handle)
 {
     HAL_MOTOR_Obj *obj= (HAL_MOTOR_Obj *) handle;
     if(obj->flag_PWM_ON)
     {
         HAL_DisablePWM(handle);
     }
     //Disable all previous continuous software force actions
     EPWM_setActionQualifierContSWForceAction(obj->pwmHandle[0], EPWM_AQ_OUTPUT_A, EPWM_AQ_SW_DISABLED);
     EPWM_setActionQualifierContSWForceAction(obj->pwmHandle[1], EPWM_AQ_OUTPUT_A, EPWM_AQ_SW_DISABLED);
     EPWM_setActionQualifierContSWForceAction(obj->pwmHandle[2], EPWM_AQ_OUTPUT_A, EPWM_AQ_SW_DISABLED);
     //compare shadow load enable
     EPWM_setCounterCompareShadowLoadMode(obj->pwmHandle[0], EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO);
     EPWM_setCounterCompareShadowLoadMode(obj->pwmHandle[1], EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO);
     EPWM_setCounterCompareShadowLoadMode(obj->pwmHandle[2], EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO);
     //Phase U recovery
     EPWM_setCounterCompareValue(obj->pwmHandle[0], EPWM_COUNTER_COMPARE_C, 10U);
     EPWM_disablePhaseShiftLoad(obj->pwmHandle[0]);
     EPWM_setPhaseShift(obj->pwmHandle[0], 0);
     ADC_disableInterrupt(obj->adcHandle[0], ADC_INT_NUMBER1);
     ADC_setInterruptSource(obj->adcHandle[0], ADC_INT_NUMBER1, ADC_SOC_NUMBER3);
     ADC_enableInterrupt(obj->adcHandle[0], ADC_INT_NUMBER1);
     //Phase V recovery
     EPWM_disablePhaseShiftLoad(obj->pwmHandle[1]);
     EPWM_setPhaseShift(obj->pwmHandle[1], 0);
     ADC_disableInterrupt(obj->adcHandle[1], ADC_INT_NUMBER1);
     ADC_setupSOC(obj->adcHandle[1], ADC_SOC_NUMBER1, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN12, 14U);
     //Phase W recovery
     EPWM_disablePhaseShiftLoad(obj->pwmHandle[2]);
     EPWM_setPhaseShift(obj->pwmHandle[2], 0);
     ADC_disableInterrupt(obj->adcHandle[2], ADC_INT_NUMBER1);
     ADC_setupSOC(obj->adcHandle[2], ADC_SOC_NUMBER1, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN3, 14U);
     return;
 }
uint16_t HAL_reverseMapping(HAL_MOTOR_Handle handle)
{
    HAL_MOTOR_Obj *obj= (HAL_MOTOR_Obj *) handle;
    uint16_t result;
    if(obj->InitialPosition==0) result=0;
    else if(obj->InitialPosition==1) result=5;
    else if(obj->InitialPosition==2) result=3;
    else if(obj->InitialPosition==3) result=4;
    else if(obj->InitialPosition==4) result=2;
    else if(obj->InitialPosition==5) result=1;

    return result;
}
uint16_t HAL_forwardMapping(HAL_MOTOR_Handle handle)
{
    HAL_MOTOR_Obj *obj= (HAL_MOTOR_Obj *) handle;
    uint16_t result;
    if(obj->InitialPosition==0) result=0;
    else if(obj->InitialPosition==1) result=1;
    else if(obj->InitialPosition==2) result=3;
    else if(obj->InitialPosition==3) result=2;
    else if(obj->InitialPosition==4) result=4;
    else if(obj->InitialPosition==5) result=5;

    return result;
}
float HAL_ramper(float in, float out, float rampDelta)
{
    float err;

    err = in - out;
    if (err > rampDelta)
        return(out + rampDelta);
    else if (err < -rampDelta)
        return(out - rampDelta);
    else
        return(in);
}
float rangeNormalize(float input)
{
    float a = input-((int32_t)(input*MATH_ONE_OVER_TWO_PI))*MATH_TWO_PI;
    return (a>=0)? a : (a+MATH_TWO_PI);
}

float angleGenerate(float targetFrequency,float Ts)
{
   static float angle=0;
   angle+=targetFrequency*MATH_TWO_PI*Ts;
   angle=rangeNormalize(angle);
   return angle;
}

