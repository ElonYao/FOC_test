#ifndef HAL_ELON_H
#define HAL_ELON_H
#include "driverlib.h"
#include <string.h>
#include "board.h"
#include "libraries/math/math.h"
#include "SVPWM_Elon.h"
#include <math.h>

//

#define VS_VOLTAGE_MAX_RATIO 0.35f
typedef enum
{
    MOTOR_STOP=0,
    MOTOR_ALLIGNMENT,
    MOTOR_READY,
    MOTOR_STARTUP,
    MOTOR_OFFSETCALI_A,
    MOTOR_IPD,
    MOTOR_OL_RUNNING,
    MOTOR_CL_RUNNING,
    MOTOR_RUNNING
}MotorState;

typedef struct _HAL_MOTOR_
{
    bool flag_PWM_ON;
    bool flag_offsetDone;
    bool flag_RevDirection;
    bool flag_InitialPositionDected;
    uint16_t loopSwitch;
    MotorState state;
    uint16_t InitialPosition;
    float32_t VdcBus_V;
    MATH_Vec3 I_A;
    MATH_Vec3 V_V;

    MATH_Vec3 offset_I_A;
    MATH_Vec3 offset_V_V;
    float32_t current_sf;       //!< the current scale factor, amps/cnt
    float32_t voltage_sf;       //!< the phase voltage scale factor, volts/cnt
    float32_t dcBusvoltage_sf;
    float32_t ipdCurrent[6];
    //For U V W phases pwm
    uint32_t pwmHandle[3];
    uint16_t pwmCCdata[3];
    //For ADC module
    uint32_t adcHandle[3];
    uint32_t adcResult[3];
    uint32_t gateEnableGPIO;
    uint32_t loopCounter;
    uint32_t stateCounterRun;// for counting in different states
    uint32_t alignmentCount;
    float speedTarget;
    float rawSpeed;
    float filteredSpeed;
    float controllerPeriod;
    float vsMax_V;//the maximum value of vector sum of Vd and Vq
    float vQmax_V;
    float idRef_A;
    float iqRef_A;
}HAL_MOTOR_Obj;

typedef struct _HAL_MOTOR_ *HAL_MOTOR_Handle;


//Prototypes
HAL_MOTOR_Handle HAL_MOTOR_init(void *pMemory, const size_t numBytes);
void HAL_EnablePWM(HAL_MOTOR_Handle handle);
void HAL_DisablePWM(HAL_MOTOR_Handle handle);
void HAL_WritePWM(HAL_MOTOR_Handle handle,SVPWM_obj *svgen);
void HAL_ackADCinterrupt(void);
void HAL_ReadADC(HAL_MOTOR_Handle handle);
void HAL_offsetReferenceCalibration(HAL_MOTOR_Handle handle);
void HAL_clearFlag_TZ(HAL_MOTOR_Handle handle);
float HAL_ramper(float in, float out, float rampDelta);
float rangeNormalize(float input);
float angleGenerate(float targetFrequency,float Ts);
//IPD
void HAL_IPD_Routine(HAL_MOTOR_Handle handle);
uint16_t HAL_forwardMapping(HAL_MOTOR_Handle handle);
uint16_t HAL_reverseMapping(HAL_MOTOR_Handle handle);
void resettingAfterIPD(HAL_MOTOR_Handle handle);
void HAL_IPD_U2VW(HAL_MOTOR_Handle handle);
void HAL_IPD_V2UW(HAL_MOTOR_Handle handle);
void HAL_IPD_W2UV(HAL_MOTOR_Handle handle);

#endif

