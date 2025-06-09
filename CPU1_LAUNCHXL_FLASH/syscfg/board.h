/*
 * Copyright (c) 2020 Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef BOARD_H
#define BOARD_H

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//
// Included Files
//

#include "driverlib.h"
#include "device.h"

//*****************************************************************************
//
// PinMux Configurations
//
//*****************************************************************************

//
// ANALOG -> Motor_IU_ADC_Mux Pinmux
//

//
// EPWM1 -> PhaseU Pinmux
//
//
// EPWM1_A - GPIO Settings
//
#define GPIO_PIN_EPWM1_A 0
#define PhaseU_EPWMA_GPIO 0
#define PhaseU_EPWMA_PIN_CONFIG GPIO_0_EPWM1_A
//
// EPWM1_B - GPIO Settings
//
#define GPIO_PIN_EPWM1_B 1
#define PhaseU_EPWMB_GPIO 1
#define PhaseU_EPWMB_PIN_CONFIG GPIO_1_EPWM1_B

//
// EPWM2 -> PhaseV Pinmux
//
//
// EPWM2_A - GPIO Settings
//
#define GPIO_PIN_EPWM2_A 2
#define PhaseV_EPWMA_GPIO 2
#define PhaseV_EPWMA_PIN_CONFIG GPIO_2_EPWM2_A
//
// EPWM2_B - GPIO Settings
//
#define GPIO_PIN_EPWM2_B 3
#define PhaseV_EPWMB_GPIO 3
#define PhaseV_EPWMB_PIN_CONFIG GPIO_3_EPWM2_B

//
// EPWM6 -> PhaseW Pinmux
//
//
// EPWM6_A - GPIO Settings
//
#define GPIO_PIN_EPWM6_A 10
#define PhaseW_EPWMA_GPIO 10
#define PhaseW_EPWMA_PIN_CONFIG GPIO_10_EPWM6_A
//
// EPWM6_B - GPIO Settings
//
#define GPIO_PIN_EPWM6_B 11
#define PhaseW_EPWMB_GPIO 11
#define PhaseW_EPWMB_PIN_CONFIG GPIO_11_EPWM6_B
//
// GPIO37 - GPIO Settings
//
#define GATE_Control_GPIO_PIN_CONFIG GPIO_37_GPIO37
//
// GPIO33 - GPIO Settings
//
#define OT_INPUT_GPIO_PIN_CONFIG GPIO_33_GPIO33

//
// I2CB -> AS5600 Pinmux
//
//
// I2CB_SDA - GPIO Settings
//
#define GPIO_PIN_I2CB_SDA 34
#define AS5600_I2CSDA_GPIO 34
#define AS5600_I2CSDA_PIN_CONFIG GPIO_34_I2CB_SDA
//
// I2CB_SCL - GPIO Settings
//
#define GPIO_PIN_I2CB_SCL 51
#define AS5600_I2CSCL_GPIO 51
#define AS5600_I2CSCL_PIN_CONFIG GPIO_51_I2CB_SCL

//
// SCIB -> PC_SCI Pinmux
//
//
// SCIB_RX - GPIO Settings
//
#define GPIO_PIN_SCIB_RX 15
#define PC_SCI_SCIRX_GPIO 15
#define PC_SCI_SCIRX_PIN_CONFIG GPIO_15_SCIB_RX
//
// SCIB_TX - GPIO Settings
//
#define GPIO_PIN_SCIB_TX 56
#define PC_SCI_SCITX_GPIO 56
#define PC_SCI_SCITX_PIN_CONFIG GPIO_56_SCIB_TX

//*****************************************************************************
//
// ADC Configurations
//
//*****************************************************************************
#define Motor_IS_ADC_BASE ADCA_BASE
#define Motor_IS_ADC_RESULT_BASE ADCARESULT_BASE
#define Motor_IU_ADC ADC_SOC_NUMBER1
#define Motor_IU_ADC_FORCE ADC_FORCE_SOC1
#define Motor_IU_ADC_ADC_BASE ADCA_BASE
#define Motor_IU_ADC_RESULT_BASE ADCARESULT_BASE
#define Motor_IU_ADC_SAMPLE_WINDOW 116.66666666666667
#define Motor_IU_ADC_TRIGGER_SOURCE ADC_TRIGGER_EPWM1_SOCA
#define Motor_IU_ADC_CHANNEL ADC_CH_ADCIN11
#define Motor_VU_ADC ADC_SOC_NUMBER2
#define Motor_VU_ADC_FORCE ADC_FORCE_SOC2
#define Motor_VU_ADC_ADC_BASE ADCA_BASE
#define Motor_VU_ADC_RESULT_BASE ADCARESULT_BASE
#define Motor_VU_ADC_SAMPLE_WINDOW 166.66666666666669
#define Motor_VU_ADC_TRIGGER_SOURCE ADC_TRIGGER_EPWM1_SOCA
#define Motor_VU_ADC_CHANNEL ADC_CH_ADCIN2
#define Motor_Vdc_ADC ADC_SOC_NUMBER3
#define Motor_Vdc_ADC_FORCE ADC_FORCE_SOC3
#define Motor_Vdc_ADC_ADC_BASE ADCA_BASE
#define Motor_Vdc_ADC_RESULT_BASE ADCARESULT_BASE
#define Motor_Vdc_ADC_SAMPLE_WINDOW 166.66666666666669
#define Motor_Vdc_ADC_TRIGGER_SOURCE ADC_TRIGGER_EPWM1_SOCA
#define Motor_Vdc_ADC_CHANNEL ADC_CH_ADCIN6
#define Motor_IS_ADC_PPB1 ADC_PPB_NUMBER1
#define Motor_IS_ADC_SOC_PPB1 ADC_SOC_NUMBER1
void Motor_IS_ADC_init();

#define myADC0_BASE ADCB_BASE
#define myADC0_RESULT_BASE ADCBRESULT_BASE
#define Motor_IV_ADC ADC_SOC_NUMBER1
#define Motor_IV_ADC_FORCE ADC_FORCE_SOC1
#define Motor_IV_ADC_ADC_BASE ADCB_BASE
#define Motor_IV_ADC_RESULT_BASE ADCBRESULT_BASE
#define Motor_IV_ADC_SAMPLE_WINDOW 116.66666666666667
#define Motor_IV_ADC_TRIGGER_SOURCE ADC_TRIGGER_EPWM1_SOCA
#define Motor_IV_ADC_CHANNEL ADC_CH_ADCIN12
#define Motor_VV_ADC ADC_SOC_NUMBER2
#define Motor_VV_ADC_FORCE ADC_FORCE_SOC2
#define Motor_VV_ADC_ADC_BASE ADCB_BASE
#define Motor_VV_ADC_RESULT_BASE ADCBRESULT_BASE
#define Motor_VV_ADC_SAMPLE_WINDOW 166.66666666666669
#define Motor_VV_ADC_TRIGGER_SOURCE ADC_TRIGGER_EPWM1_SOCA
#define Motor_VV_ADC_CHANNEL ADC_CH_ADCIN9
#define myADC0_PPB1 ADC_PPB_NUMBER1
#define myADC0_SOC_PPB1 ADC_SOC_NUMBER1
void myADC0_init();

#define myADC1_BASE ADCC_BASE
#define myADC1_RESULT_BASE ADCCRESULT_BASE
#define Motor_IW_ADC ADC_SOC_NUMBER1
#define Motor_IW_ADC_FORCE ADC_FORCE_SOC1
#define Motor_IW_ADC_ADC_BASE ADCC_BASE
#define Motor_IW_ADC_RESULT_BASE ADCCRESULT_BASE
#define Motor_IW_ADC_SAMPLE_WINDOW 116.66666666666667
#define Motor_IW_ADC_TRIGGER_SOURCE ADC_TRIGGER_EPWM1_SOCA
#define Motor_IW_ADC_CHANNEL ADC_CH_ADCIN3
#define Motor_VW_ADC ADC_SOC_NUMBER2
#define Motor_VW_ADC_FORCE ADC_FORCE_SOC2
#define Motor_VW_ADC_ADC_BASE ADCC_BASE
#define Motor_VW_ADC_RESULT_BASE ADCCRESULT_BASE
#define Motor_VW_ADC_SAMPLE_WINDOW 166.66666666666669
#define Motor_VW_ADC_TRIGGER_SOURCE ADC_TRIGGER_EPWM1_SOCA
#define Motor_VW_ADC_CHANNEL ADC_CH_ADCIN4
#define myADC1_PPB1 ADC_PPB_NUMBER1
#define myADC1_SOC_PPB1 ADC_SOC_NUMBER1
void myADC1_init();


//*****************************************************************************
//
// ASYSCTL Configurations
//
//*****************************************************************************

//*****************************************************************************
//
// CMPSS Configurations
//
//*****************************************************************************
#define CMPSS_U_BASE CMPSS1_BASE
#define CMPSS_U_HIGH_COMP_BASE CMPSS1_BASE    
#define CMPSS_U_LOW_COMP_BASE CMPSS1_BASE    
void CMPSS_U_init();
#define CMPSS_V_BASE CMPSS3_BASE
#define CMPSS_V_HIGH_COMP_BASE CMPSS3_BASE    
#define CMPSS_V_LOW_COMP_BASE CMPSS3_BASE    
void CMPSS_V_init();
#define CMPSS_W_BASE CMPSS4_BASE
#define CMPSS_W_HIGH_COMP_BASE CMPSS4_BASE    
#define CMPSS_W_LOW_COMP_BASE CMPSS4_BASE    
void CMPSS_W_init();

//*****************************************************************************
//
// CPUTIMER Configurations
//
//*****************************************************************************
#define dataPrint_BASE CPUTIMER0_BASE
void dataPrint_init();

//*****************************************************************************
//
// EPWM Configurations
//
//*****************************************************************************
#define PhaseU_BASE EPWM1_BASE
#define PhaseU_TBPRD 3000
#define PhaseU_COUNTER_MODE EPWM_COUNTER_MODE_UP_DOWN
#define PhaseU_TBPHS 0
#define PhaseU_CMPA 500
#define PhaseU_CMPB 500
#define PhaseU_CMPC 10
#define PhaseU_CMPD 0
#define PhaseU_DBRED 5
#define PhaseU_DBFED 5
#define PhaseU_TZA_ACTION EPWM_TZ_ACTION_LOW
#define PhaseU_TZB_ACTION EPWM_TZ_ACTION_LOW
#define PhaseU_OSHT_SOURCES EPWM_TZ_SIGNAL_OSHT1
#define PhaseU_INTERRUPT_SOURCE EPWM_INT_TBCTR_DISABLED
#define PhaseV_BASE EPWM2_BASE
#define PhaseV_TBPRD 3000
#define PhaseV_COUNTER_MODE EPWM_COUNTER_MODE_UP_DOWN
#define PhaseV_TBPHS 0
#define PhaseV_CMPA 500
#define PhaseV_CMPB 100
#define PhaseV_CMPC 0
#define PhaseV_CMPD 0
#define PhaseV_DBRED 5
#define PhaseV_DBFED 5
#define PhaseV_TZA_ACTION EPWM_TZ_ACTION_LOW
#define PhaseV_TZB_ACTION EPWM_TZ_ACTION_LOW
#define PhaseV_OSHT_SOURCES EPWM_TZ_SIGNAL_OSHT1
#define PhaseV_INTERRUPT_SOURCE EPWM_INT_TBCTR_DISABLED
#define PhaseW_BASE EPWM6_BASE
#define PhaseW_TBPRD 3000
#define PhaseW_COUNTER_MODE EPWM_COUNTER_MODE_UP_DOWN
#define PhaseW_TBPHS 0
#define PhaseW_CMPA 500
#define PhaseW_CMPB 500
#define PhaseW_CMPC 0
#define PhaseW_CMPD 0
#define PhaseW_DBRED 5
#define PhaseW_DBFED 5
#define PhaseW_TZA_ACTION EPWM_TZ_ACTION_LOW
#define PhaseW_TZB_ACTION EPWM_TZ_ACTION_LOW
#define PhaseW_OSHT_SOURCES EPWM_TZ_SIGNAL_OSHT1
#define PhaseW_INTERRUPT_SOURCE EPWM_INT_TBCTR_DISABLED

//*****************************************************************************
//
// GPIO Configurations
//
//*****************************************************************************
#define GATE_Control 37
void GATE_Control_init();
#define OT_INPUT 33
void OT_INPUT_init();

//*****************************************************************************
//
// I2C Configurations
//
//*****************************************************************************
#define AS5600_BASE I2CB_BASE
#define AS5600_BITRATE 400000
#define AS5600_TARGET_ADDRESS 54
#define AS5600_OWN_ADDRESS 0
#define AS5600_MODULE_CLOCK_FREQUENCY 10000000
void AS5600_init();

//*****************************************************************************
//
// INPUTXBAR Configurations
//
//*****************************************************************************
#define OT_inputX_SOURCE 33
#define OT_inputX_INPUT XBAR_INPUT1
void OT_inputX_init();

//*****************************************************************************
//
// INTERRUPT Configurations
//
//*****************************************************************************

// Interrupt Settings for INT_Motor_IS_ADC_1
// ISR need to be defined for the registered interrupts
#define INT_Motor_IS_ADC_1 INT_ADCA1
#define INT_Motor_IS_ADC_1_INTERRUPT_ACK_GROUP INTERRUPT_ACK_GROUP1
extern __interrupt void INT_Motor_IS_ADC_1_ISR(void);

//*****************************************************************************
//
// SCI Configurations
//
//*****************************************************************************
#define PC_SCI_BASE SCIB_BASE
#define PC_SCI_BAUDRATE 115200
#define PC_SCI_CONFIG_WLEN SCI_CONFIG_WLEN_8
#define PC_SCI_CONFIG_STOP SCI_CONFIG_STOP_ONE
#define PC_SCI_CONFIG_PAR SCI_CONFIG_PAR_NONE
void PC_SCI_init();

//*****************************************************************************
//
// SYNC Scheme Configurations
//
//*****************************************************************************

//*****************************************************************************
//
// Board Configurations
//
//*****************************************************************************
void	Board_init();
void	ADC_init();
void	ASYSCTL_init();
void	CMPSS_init();
void	CPUTIMER_init();
void	EPWM_init();
void	GPIO_init();
void	I2C_init();
void	INPUTXBAR_init();
void	INTERRUPT_init();
void	SCI_init();
void	SYNC_init();
void	PinMux_init();

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif  // end of BOARD_H definition
