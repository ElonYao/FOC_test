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

#include "board.h"

//*****************************************************************************
//
// Board Configurations
// Initializes the rest of the modules. 
// Call this function in your application if you wish to do all module 
// initialization.
// If you wish to not use some of the initializations, instead of the 
// Board_init use the individual Module_inits
//
//*****************************************************************************
void Board_init()
{
	EALLOW;

	PinMux_init();
	INPUTXBAR_init();
	SYNC_init();
	ASYSCTL_init();
	ADC_init();
	CMPSS_init();
	CPUTIMER_init();
	EPWM_init();
	GPIO_init();
	I2C_init();
	SCI_init();
	INTERRUPT_init();

	EDIS;
}

//*****************************************************************************
//
// PINMUX Configurations
//
//*****************************************************************************
void PinMux_init()
{
	//
	// PinMux for modules assigned to CPU1
	//
	
	//
	// ANALOG -> Motor_IU_ADC_Mux Pinmux
	//
	// Analog PinMux for A11/B10/C0
	GPIO_setPinConfig(GPIO_237_GPIO237);
	// AIO -> Analog mode selected
	GPIO_setAnalogMode(237, GPIO_ANALOG_ENABLED);
	// Analog PinMux for A14/B14/C4
	GPIO_setPinConfig(GPIO_239_GPIO239);
	// AIO -> Analog mode selected
	GPIO_setAnalogMode(239, GPIO_ANALOG_ENABLED);
	// Analog PinMux for A2/B6/C9
	GPIO_setPinConfig(GPIO_224_GPIO224);
	// AIO -> Analog mode selected
	GPIO_setAnalogMode(224, GPIO_ANALOG_ENABLED);
	// Analog PinMux for A3, C7/B9
	GPIO_setPinConfig(GPIO_229_GPIO229);
	// AIO -> Analog mode selected
	GPIO_setAnalogMode(229, GPIO_ANALOG_ENABLED);
	// Analog PinMux for A6
	GPIO_setPinConfig(GPIO_228_GPIO228);
	// AIO -> Analog mode selected
	GPIO_setAnalogMode(228, GPIO_ANALOG_ENABLED);
	// Analog PinMux for C2/B12
	GPIO_setPinConfig(GPIO_244_GPIO244);
	// AIO -> Analog mode selected
	GPIO_setAnalogMode(244, GPIO_ANALOG_ENABLED);
	// Analog PinMux for C3/A7
	GPIO_setPinConfig(GPIO_245_GPIO245);
	// AIO -> Analog mode selected
	GPIO_setAnalogMode(245, GPIO_ANALOG_ENABLED);
	//
	// EPWM1 -> PhaseU Pinmux
	//
	GPIO_setPinConfig(PhaseU_EPWMA_PIN_CONFIG);
	GPIO_setPadConfig(PhaseU_EPWMA_GPIO, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(PhaseU_EPWMA_GPIO, GPIO_QUAL_SYNC);

	GPIO_setPinConfig(PhaseU_EPWMB_PIN_CONFIG);
	GPIO_setPadConfig(PhaseU_EPWMB_GPIO, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(PhaseU_EPWMB_GPIO, GPIO_QUAL_SYNC);

	//
	// EPWM2 -> PhaseV Pinmux
	//
	GPIO_setPinConfig(PhaseV_EPWMA_PIN_CONFIG);
	GPIO_setPadConfig(PhaseV_EPWMA_GPIO, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(PhaseV_EPWMA_GPIO, GPIO_QUAL_SYNC);

	GPIO_setPinConfig(PhaseV_EPWMB_PIN_CONFIG);
	GPIO_setPadConfig(PhaseV_EPWMB_GPIO, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(PhaseV_EPWMB_GPIO, GPIO_QUAL_SYNC);

	//
	// EPWM6 -> PhaseW Pinmux
	//
	GPIO_setPinConfig(PhaseW_EPWMA_PIN_CONFIG);
	GPIO_setPadConfig(PhaseW_EPWMA_GPIO, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(PhaseW_EPWMA_GPIO, GPIO_QUAL_SYNC);

	GPIO_setPinConfig(PhaseW_EPWMB_PIN_CONFIG);
	GPIO_setPadConfig(PhaseW_EPWMB_GPIO, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(PhaseW_EPWMB_GPIO, GPIO_QUAL_SYNC);

	// GPIO37/TDO -> GATE_Control Pinmux
	GPIO_setPinConfig(GPIO_37_GPIO37);
	// GPIO33 -> OT_INPUT Pinmux
	GPIO_setPinConfig(GPIO_33_GPIO33);
	//
	// I2CB -> AS5600 Pinmux
	//
	GPIO_setPinConfig(AS5600_I2CSDA_PIN_CONFIG);
	GPIO_setPadConfig(AS5600_I2CSDA_GPIO, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(AS5600_I2CSDA_GPIO, GPIO_QUAL_ASYNC);

	GPIO_setPinConfig(AS5600_I2CSCL_PIN_CONFIG);
	GPIO_setPadConfig(AS5600_I2CSCL_GPIO, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(AS5600_I2CSCL_GPIO, GPIO_QUAL_ASYNC);

	//
	// SCIB -> PC_SCI Pinmux
	//
	GPIO_setPinConfig(PC_SCI_SCIRX_PIN_CONFIG);
	GPIO_setPadConfig(PC_SCI_SCIRX_GPIO, GPIO_PIN_TYPE_STD | GPIO_PIN_TYPE_PULLUP);
	GPIO_setQualificationMode(PC_SCI_SCIRX_GPIO, GPIO_QUAL_ASYNC);

	GPIO_setPinConfig(PC_SCI_SCITX_PIN_CONFIG);
	GPIO_setPadConfig(PC_SCI_SCITX_GPIO, GPIO_PIN_TYPE_STD | GPIO_PIN_TYPE_PULLUP);
	GPIO_setQualificationMode(PC_SCI_SCITX_GPIO, GPIO_QUAL_ASYNC);


}

//*****************************************************************************
//
// ADC Configurations
//
//*****************************************************************************
void ADC_init(){
	Motor_IS_ADC_init();
	myADC0_init();
	myADC1_init();
}

void Motor_IS_ADC_init(){
	//
	// ADC Initialization: Write ADC configurations and power up the ADC
	//
	// Set the analog voltage reference selection and ADC module's offset trims.
	// This function sets the analog voltage reference to internal (with the reference voltage of 1.65V or 2.5V) or external for ADC
	// which is same as ASysCtl APIs.
	//
	ADC_setVREF(Motor_IS_ADC_BASE, ADC_REFERENCE_EXTERNAL, ADC_REFERENCE_2_5V);
	//
	// Configures the analog-to-digital converter module prescaler.
	//
	ADC_setPrescaler(Motor_IS_ADC_BASE, ADC_CLK_DIV_2_0);
	//
	// Sets the timing of the end-of-conversion pulse
	//
	ADC_setInterruptPulseMode(Motor_IS_ADC_BASE, ADC_PULSE_END_OF_CONV);
	//
	// Powers up the analog-to-digital converter core.
	//
	ADC_enableConverter(Motor_IS_ADC_BASE);
	//
	// Delay for 1ms to allow ADC time to power up
	//
	DEVICE_DELAY_US(500);
	//
	// SOC Configuration: Setup ADC EPWM channel and trigger settings
	//
	// Disables SOC burst mode.
	//
	ADC_disableBurstMode(Motor_IS_ADC_BASE);
	//
	// Sets the priority mode of the SOCs.
	//
	ADC_setSOCPriority(Motor_IS_ADC_BASE, ADC_PRI_ALL_HIPRI);
	//
	// Start of Conversion 1 Configuration
	//
	//
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 1
	//	  	Trigger			: ADC_TRIGGER_EPWM1_SOCA
	//	  	Channel			: ADC_CH_ADCIN11
	//	 	Sample Window	: 14 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	//
	ADC_setupSOC(Motor_IS_ADC_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN11, 14U);
	ADC_setInterruptSOCTrigger(Motor_IS_ADC_BASE, ADC_SOC_NUMBER1, ADC_INT_SOC_TRIGGER_NONE);
	//
	// Start of Conversion 2 Configuration
	//
	//
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 2
	//	  	Trigger			: ADC_TRIGGER_EPWM1_SOCA
	//	  	Channel			: ADC_CH_ADCIN2
	//	 	Sample Window	: 20 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	//
	ADC_setupSOC(Motor_IS_ADC_BASE, ADC_SOC_NUMBER2, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN2, 20U);
	ADC_setInterruptSOCTrigger(Motor_IS_ADC_BASE, ADC_SOC_NUMBER2, ADC_INT_SOC_TRIGGER_NONE);
	//
	// Start of Conversion 3 Configuration
	//
	//
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 3
	//	  	Trigger			: ADC_TRIGGER_EPWM1_SOCA
	//	  	Channel			: ADC_CH_ADCIN6
	//	 	Sample Window	: 20 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	//
	ADC_setupSOC(Motor_IS_ADC_BASE, ADC_SOC_NUMBER3, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN6, 20U);
	ADC_setInterruptSOCTrigger(Motor_IS_ADC_BASE, ADC_SOC_NUMBER3, ADC_INT_SOC_TRIGGER_NONE);
	//
	// ADC Interrupt 1 Configuration
	// 		Source	: ADC_SOC_NUMBER3
	// 		Interrupt Source: enabled
	// 		Continuous Mode	: disabled
	//
	//
	ADC_setInterruptSource(Motor_IS_ADC_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER3);
	ADC_clearInterruptStatus(Motor_IS_ADC_BASE, ADC_INT_NUMBER1);
	ADC_disableContinuousMode(Motor_IS_ADC_BASE, ADC_INT_NUMBER1);
	ADC_enableInterrupt(Motor_IS_ADC_BASE, ADC_INT_NUMBER1);
			
	//
	// PPB Configuration: Configure high and low limits detection for ADCPPB
	//
	// Post Processing Block 1 Configuration
	// 		Configures a post-processing block (PPB) in the ADC.
	// 		PPB Number				: 1
	// 		SOC/EOC number			: 1
	// 		Calibration Offset		: 0
	// 		Reference Offset		: 2042
	// 		Two's Complement		: Disabled
	// 		Trip High Limit			: 0
	// 		Trip Low Limit			: 0
	// 		Clear PPB Event Flags	: Disabled
	//
	ADC_setupPPB(Motor_IS_ADC_BASE, ADC_PPB_NUMBER1, ADC_SOC_NUMBER1);
	ADC_disablePPBEvent(Motor_IS_ADC_BASE, ADC_PPB_NUMBER1, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
	ADC_disablePPBEventInterrupt(Motor_IS_ADC_BASE, ADC_PPB_NUMBER1, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
	ADC_setPPBCalibrationOffset(Motor_IS_ADC_BASE, ADC_PPB_NUMBER1, 0);
	ADC_setPPBReferenceOffset(Motor_IS_ADC_BASE, ADC_PPB_NUMBER1, 2042);
	ADC_disablePPBTwosComplement(Motor_IS_ADC_BASE, ADC_PPB_NUMBER1);
	ADC_setPPBTripLimits(Motor_IS_ADC_BASE, ADC_PPB_NUMBER1, 0, 0);
	ADC_disablePPBEventCBCClear(Motor_IS_ADC_BASE, ADC_PPB_NUMBER1);
}

void myADC0_init(){
	//
	// ADC Initialization: Write ADC configurations and power up the ADC
	//
	// Set the analog voltage reference selection and ADC module's offset trims.
	// This function sets the analog voltage reference to internal (with the reference voltage of 1.65V or 2.5V) or external for ADC
	// which is same as ASysCtl APIs.
	//
	ADC_setVREF(myADC0_BASE, ADC_REFERENCE_EXTERNAL, ADC_REFERENCE_2_5V);
	//
	// Configures the analog-to-digital converter module prescaler.
	//
	ADC_setPrescaler(myADC0_BASE, ADC_CLK_DIV_2_0);
	//
	// Sets the timing of the end-of-conversion pulse
	//
	ADC_setInterruptPulseMode(myADC0_BASE, ADC_PULSE_END_OF_CONV);
	//
	// Powers up the analog-to-digital converter core.
	//
	ADC_enableConverter(myADC0_BASE);
	//
	// Delay for 1ms to allow ADC time to power up
	//
	DEVICE_DELAY_US(500);
	//
	// SOC Configuration: Setup ADC EPWM channel and trigger settings
	//
	// Disables SOC burst mode.
	//
	ADC_disableBurstMode(myADC0_BASE);
	//
	// Sets the priority mode of the SOCs.
	//
	ADC_setSOCPriority(myADC0_BASE, ADC_PRI_ALL_HIPRI);
	//
	// Start of Conversion 1 Configuration
	//
	//
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 1
	//	  	Trigger			: ADC_TRIGGER_EPWM1_SOCA
	//	  	Channel			: ADC_CH_ADCIN12
	//	 	Sample Window	: 14 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	//
	ADC_setupSOC(myADC0_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN12, 14U);
	ADC_setInterruptSOCTrigger(myADC0_BASE, ADC_SOC_NUMBER1, ADC_INT_SOC_TRIGGER_NONE);
	//
	// Start of Conversion 2 Configuration
	//
	//
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 2
	//	  	Trigger			: ADC_TRIGGER_EPWM1_SOCA
	//	  	Channel			: ADC_CH_ADCIN9
	//	 	Sample Window	: 20 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	//
	ADC_setupSOC(myADC0_BASE, ADC_SOC_NUMBER2, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN9, 20U);
	ADC_setInterruptSOCTrigger(myADC0_BASE, ADC_SOC_NUMBER2, ADC_INT_SOC_TRIGGER_NONE);
			
	//
	// PPB Configuration: Configure high and low limits detection for ADCPPB
	//
	// Post Processing Block 1 Configuration
	// 		Configures a post-processing block (PPB) in the ADC.
	// 		PPB Number				: 1
	// 		SOC/EOC number			: 1
	// 		Calibration Offset		: 0
	// 		Reference Offset		: 2044
	// 		Two's Complement		: Disabled
	// 		Trip High Limit			: 0
	// 		Trip Low Limit			: 0
	// 		Clear PPB Event Flags	: Disabled
	//
	ADC_setupPPB(myADC0_BASE, ADC_PPB_NUMBER1, ADC_SOC_NUMBER1);
	ADC_disablePPBEvent(myADC0_BASE, ADC_PPB_NUMBER1, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
	ADC_disablePPBEventInterrupt(myADC0_BASE, ADC_PPB_NUMBER1, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
	ADC_setPPBCalibrationOffset(myADC0_BASE, ADC_PPB_NUMBER1, 0);
	ADC_setPPBReferenceOffset(myADC0_BASE, ADC_PPB_NUMBER1, 2044);
	ADC_disablePPBTwosComplement(myADC0_BASE, ADC_PPB_NUMBER1);
	ADC_setPPBTripLimits(myADC0_BASE, ADC_PPB_NUMBER1, 0, 0);
	ADC_disablePPBEventCBCClear(myADC0_BASE, ADC_PPB_NUMBER1);
}

void myADC1_init(){
	//
	// ADC Initialization: Write ADC configurations and power up the ADC
	//
	// Set the analog voltage reference selection and ADC module's offset trims.
	// This function sets the analog voltage reference to internal (with the reference voltage of 1.65V or 2.5V) or external for ADC
	// which is same as ASysCtl APIs.
	//
	ADC_setVREF(myADC1_BASE, ADC_REFERENCE_EXTERNAL, ADC_REFERENCE_2_5V);
	//
	// Configures the analog-to-digital converter module prescaler.
	//
	ADC_setPrescaler(myADC1_BASE, ADC_CLK_DIV_2_0);
	//
	// Sets the timing of the end-of-conversion pulse
	//
	ADC_setInterruptPulseMode(myADC1_BASE, ADC_PULSE_END_OF_CONV);
	//
	// Powers up the analog-to-digital converter core.
	//
	ADC_enableConverter(myADC1_BASE);
	//
	// Delay for 1ms to allow ADC time to power up
	//
	DEVICE_DELAY_US(500);
	//
	// SOC Configuration: Setup ADC EPWM channel and trigger settings
	//
	// Disables SOC burst mode.
	//
	ADC_disableBurstMode(myADC1_BASE);
	//
	// Sets the priority mode of the SOCs.
	//
	ADC_setSOCPriority(myADC1_BASE, ADC_PRI_ALL_HIPRI);
	//
	// Start of Conversion 1 Configuration
	//
	//
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 1
	//	  	Trigger			: ADC_TRIGGER_EPWM1_SOCA
	//	  	Channel			: ADC_CH_ADCIN3
	//	 	Sample Window	: 14 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	//
	ADC_setupSOC(myADC1_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN3, 14U);
	ADC_setInterruptSOCTrigger(myADC1_BASE, ADC_SOC_NUMBER1, ADC_INT_SOC_TRIGGER_NONE);
	//
	// Start of Conversion 2 Configuration
	//
	//
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 2
	//	  	Trigger			: ADC_TRIGGER_EPWM1_SOCA
	//	  	Channel			: ADC_CH_ADCIN4
	//	 	Sample Window	: 20 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	//
	ADC_setupSOC(myADC1_BASE, ADC_SOC_NUMBER2, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN4, 20U);
	ADC_setInterruptSOCTrigger(myADC1_BASE, ADC_SOC_NUMBER2, ADC_INT_SOC_TRIGGER_NONE);
			
	//
	// PPB Configuration: Configure high and low limits detection for ADCPPB
	//
	// Post Processing Block 1 Configuration
	// 		Configures a post-processing block (PPB) in the ADC.
	// 		PPB Number				: 1
	// 		SOC/EOC number			: 1
	// 		Calibration Offset		: 0
	// 		Reference Offset		: 2045
	// 		Two's Complement		: Disabled
	// 		Trip High Limit			: 0
	// 		Trip Low Limit			: 0
	// 		Clear PPB Event Flags	: Disabled
	//
	ADC_setupPPB(myADC1_BASE, ADC_PPB_NUMBER1, ADC_SOC_NUMBER1);
	ADC_disablePPBEvent(myADC1_BASE, ADC_PPB_NUMBER1, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
	ADC_disablePPBEventInterrupt(myADC1_BASE, ADC_PPB_NUMBER1, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
	ADC_setPPBCalibrationOffset(myADC1_BASE, ADC_PPB_NUMBER1, 0);
	ADC_setPPBReferenceOffset(myADC1_BASE, ADC_PPB_NUMBER1, 2045);
	ADC_disablePPBTwosComplement(myADC1_BASE, ADC_PPB_NUMBER1);
	ADC_setPPBTripLimits(myADC1_BASE, ADC_PPB_NUMBER1, 0, 0);
	ADC_disablePPBEventCBCClear(myADC1_BASE, ADC_PPB_NUMBER1);
}


//*****************************************************************************
//
// ASYSCTL Configurations
//
//*****************************************************************************
void ASYSCTL_init(){
	//
	// asysctl initialization
	//
	// Disables the temperature sensor output to the ADC.
	//
	ASysCtl_disableTemperatureSensor();
	//
	// Set the analog voltage reference selection to external.
	//
	ASysCtl_setAnalogReferenceExternal( ASYSCTL_VREFHI );
}

//*****************************************************************************
//
// CMPSS Configurations
//
//*****************************************************************************
void CMPSS_init(){
	CMPSS_U_init();
	CMPSS_V_init();
	CMPSS_W_init();
}

void CMPSS_U_init(){
    //
    // Select the value for CMP1HPMXSEL.
    //
    ASysCtl_selectCMPHPMux(ASYSCTL_CMPHPMUX_SELECT_1,1U);
    //
    // Select the value for CMP1LPMXSEL.
    //
    ASysCtl_selectCMPLPMux(ASYSCTL_CMPLPMUX_SELECT_1,1U);
    //
    // Sets the configuration for the high comparator.
    //
    CMPSS_configHighComparator(CMPSS_U_BASE,(CMPSS_INSRC_DAC));
    //
    // Sets the configuration for the low comparator.
    //
    CMPSS_configLowComparator(CMPSS_U_BASE,(CMPSS_INSRC_DAC | CMPSS_INV_INVERTED));
    //
    // Sets the configuration for the internal comparator DACs.
    //
    CMPSS_configDAC(CMPSS_U_BASE,(CMPSS_DACVAL_SYSCLK | CMPSS_DACREF_VDDA | CMPSS_DACSRC_SHDW));
    //
    // Sets the value of the internal DAC of the high comparator.
    //
    CMPSS_setDACValueHigh(CMPSS_U_BASE,3584U);
    //
    // Sets the value of the internal DAC of the low comparator.
    //
    CMPSS_setDACValueLow(CMPSS_U_BASE,512U);
    //
    //  Configures the digital filter of the high comparator.
    //
    CMPSS_configFilterHigh(CMPSS_U_BASE, 32U, 32U, 30U);
    //
    // Configures the digital filter of the low comparator.
    //
    CMPSS_configFilterLow(CMPSS_U_BASE, 32U, 32U, 30U);
    //
    // Initializes the digital filter of the high comparator.
    //
    CMPSS_initFilterHigh(CMPSS_U_BASE);
    //
    // Initializes the digital filter of the low comparator.
    //
    CMPSS_initFilterLow(CMPSS_U_BASE);
    //
    // Sets the output signal configuration for the high comparator.
    //
    CMPSS_configOutputsHigh(CMPSS_U_BASE,(CMPSS_TRIPOUT_FILTER | CMPSS_TRIP_FILTER));
    //
    // Sets the output signal configuration for the low comparator.
    //
    CMPSS_configOutputsLow(CMPSS_U_BASE,(CMPSS_TRIPOUT_FILTER | CMPSS_TRIP_FILTER));
    //
    // Sets the comparator hysteresis settings.
    //
    CMPSS_setHysteresis(CMPSS_U_BASE,2U);
    //
    // Configures the comparator subsystem's ramp generator.
    //
    CMPSS_configRamp(CMPSS_U_BASE,0U,0U,0U,1U,true);
    //
    // Disables reset of HIGH comparator digital filter output latch on PWMSYNC
    //
    CMPSS_disableLatchResetOnPWMSYNCHigh(CMPSS_U_BASE);
    //
    // Disables reset of LOW comparator digital filter output latch on PWMSYNC
    //
    CMPSS_disableLatchResetOnPWMSYNCLow(CMPSS_U_BASE);
    //
    // Sets the ePWM module blanking signal that holds trip in reset.
    //
    CMPSS_configBlanking(CMPSS_U_BASE,1U);
    //
    // Disables an ePWM blanking signal from holding trip in reset.
    //
    CMPSS_disableBlanking(CMPSS_U_BASE);
    //
    // Configures whether or not the digital filter latches are reset by PWMSYNC
    //
    CMPSS_configLatchOnPWMSYNC(CMPSS_U_BASE,false,false);
    //
    // Enables the CMPSS module.
    //
    CMPSS_enableModule(CMPSS_U_BASE);
    //
    // Delay for CMPSS DAC to power up.
    //
    DEVICE_DELAY_US(500);
    //
    // Causes a software reset of the high comparator digital filter output latch.
    //
    CMPSS_clearFilterLatchHigh(CMPSS_U_BASE);
    //
    // Causes a software reset of the low comparator digital filter output latch.
    //
    CMPSS_clearFilterLatchLow(CMPSS_U_BASE);
}
void CMPSS_V_init(){
    //
    // Select the value for CMP3HPMXSEL.
    //
    ASysCtl_selectCMPHPMux(ASYSCTL_CMPHPMUX_SELECT_3,1U);
    //
    // Select the value for CMP3LPMXSEL.
    //
    ASysCtl_selectCMPLPMux(ASYSCTL_CMPLPMUX_SELECT_3,1U);
    //
    // Sets the configuration for the high comparator.
    //
    CMPSS_configHighComparator(CMPSS_V_BASE,(CMPSS_INSRC_DAC));
    //
    // Sets the configuration for the low comparator.
    //
    CMPSS_configLowComparator(CMPSS_V_BASE,(CMPSS_INSRC_DAC | CMPSS_INV_INVERTED));
    //
    // Sets the configuration for the internal comparator DACs.
    //
    CMPSS_configDAC(CMPSS_V_BASE,(CMPSS_DACVAL_SYSCLK | CMPSS_DACREF_VDDA | CMPSS_DACSRC_SHDW));
    //
    // Sets the value of the internal DAC of the high comparator.
    //
    CMPSS_setDACValueHigh(CMPSS_V_BASE,3584U);
    //
    // Sets the value of the internal DAC of the low comparator.
    //
    CMPSS_setDACValueLow(CMPSS_V_BASE,512U);
    //
    //  Configures the digital filter of the high comparator.
    //
    CMPSS_configFilterHigh(CMPSS_V_BASE, 32U, 32U, 30U);
    //
    // Configures the digital filter of the low comparator.
    //
    CMPSS_configFilterLow(CMPSS_V_BASE, 32U, 32U, 30U);
    //
    // Initializes the digital filter of the high comparator.
    //
    CMPSS_initFilterHigh(CMPSS_V_BASE);
    //
    // Initializes the digital filter of the low comparator.
    //
    CMPSS_initFilterLow(CMPSS_V_BASE);
    //
    // Sets the output signal configuration for the high comparator.
    //
    CMPSS_configOutputsHigh(CMPSS_V_BASE,(CMPSS_TRIPOUT_FILTER | CMPSS_TRIP_FILTER));
    //
    // Sets the output signal configuration for the low comparator.
    //
    CMPSS_configOutputsLow(CMPSS_V_BASE,(CMPSS_TRIPOUT_FILTER | CMPSS_TRIP_FILTER));
    //
    // Sets the comparator hysteresis settings.
    //
    CMPSS_setHysteresis(CMPSS_V_BASE,2U);
    //
    // Configures the comparator subsystem's ramp generator.
    //
    CMPSS_configRamp(CMPSS_V_BASE,0U,0U,0U,1U,true);
    //
    // Disables reset of HIGH comparator digital filter output latch on PWMSYNC
    //
    CMPSS_disableLatchResetOnPWMSYNCHigh(CMPSS_V_BASE);
    //
    // Disables reset of LOW comparator digital filter output latch on PWMSYNC
    //
    CMPSS_disableLatchResetOnPWMSYNCLow(CMPSS_V_BASE);
    //
    // Sets the ePWM module blanking signal that holds trip in reset.
    //
    CMPSS_configBlanking(CMPSS_V_BASE,1U);
    //
    // Disables an ePWM blanking signal from holding trip in reset.
    //
    CMPSS_disableBlanking(CMPSS_V_BASE);
    //
    // Configures whether or not the digital filter latches are reset by PWMSYNC
    //
    CMPSS_configLatchOnPWMSYNC(CMPSS_V_BASE,false,false);
    //
    // Enables the CMPSS module.
    //
    CMPSS_enableModule(CMPSS_V_BASE);
    //
    // Delay for CMPSS DAC to power up.
    //
    DEVICE_DELAY_US(500);
    //
    // Causes a software reset of the high comparator digital filter output latch.
    //
    CMPSS_clearFilterLatchHigh(CMPSS_V_BASE);
    //
    // Causes a software reset of the low comparator digital filter output latch.
    //
    CMPSS_clearFilterLatchLow(CMPSS_V_BASE);
}
void CMPSS_W_init(){
    //
    // Select the value for CMP4HPMXSEL.
    //
    ASysCtl_selectCMPHPMux(ASYSCTL_CMPHPMUX_SELECT_4,1U);
    //
    // Select the value for CMP4LPMXSEL.
    //
    ASysCtl_selectCMPLPMux(ASYSCTL_CMPLPMUX_SELECT_4,1U);
    //
    // Sets the configuration for the high comparator.
    //
    CMPSS_configHighComparator(CMPSS_W_BASE,(CMPSS_INSRC_DAC));
    //
    // Sets the configuration for the low comparator.
    //
    CMPSS_configLowComparator(CMPSS_W_BASE,(CMPSS_INSRC_DAC | CMPSS_INV_INVERTED));
    //
    // Sets the configuration for the internal comparator DACs.
    //
    CMPSS_configDAC(CMPSS_W_BASE,(CMPSS_DACVAL_SYSCLK | CMPSS_DACREF_VDDA | CMPSS_DACSRC_SHDW));
    //
    // Sets the value of the internal DAC of the high comparator.
    //
    CMPSS_setDACValueHigh(CMPSS_W_BASE,3584U);
    //
    // Sets the value of the internal DAC of the low comparator.
    //
    CMPSS_setDACValueLow(CMPSS_W_BASE,512U);
    //
    //  Configures the digital filter of the high comparator.
    //
    CMPSS_configFilterHigh(CMPSS_W_BASE, 32U, 32U, 30U);
    //
    // Configures the digital filter of the low comparator.
    //
    CMPSS_configFilterLow(CMPSS_W_BASE, 32U, 32U, 30U);
    //
    // Initializes the digital filter of the high comparator.
    //
    CMPSS_initFilterHigh(CMPSS_W_BASE);
    //
    // Initializes the digital filter of the low comparator.
    //
    CMPSS_initFilterLow(CMPSS_W_BASE);
    //
    // Sets the output signal configuration for the high comparator.
    //
    CMPSS_configOutputsHigh(CMPSS_W_BASE,(CMPSS_TRIPOUT_FILTER | CMPSS_TRIP_FILTER));
    //
    // Sets the output signal configuration for the low comparator.
    //
    CMPSS_configOutputsLow(CMPSS_W_BASE,(CMPSS_TRIPOUT_FILTER | CMPSS_TRIP_FILTER));
    //
    // Sets the comparator hysteresis settings.
    //
    CMPSS_setHysteresis(CMPSS_W_BASE,2U);
    //
    // Configures the comparator subsystem's ramp generator.
    //
    CMPSS_configRamp(CMPSS_W_BASE,0U,0U,0U,1U,true);
    //
    // Disables reset of HIGH comparator digital filter output latch on PWMSYNC
    //
    CMPSS_disableLatchResetOnPWMSYNCHigh(CMPSS_W_BASE);
    //
    // Disables reset of LOW comparator digital filter output latch on PWMSYNC
    //
    CMPSS_disableLatchResetOnPWMSYNCLow(CMPSS_W_BASE);
    //
    // Sets the ePWM module blanking signal that holds trip in reset.
    //
    CMPSS_configBlanking(CMPSS_W_BASE,1U);
    //
    // Disables an ePWM blanking signal from holding trip in reset.
    //
    CMPSS_disableBlanking(CMPSS_W_BASE);
    //
    // Configures whether or not the digital filter latches are reset by PWMSYNC
    //
    CMPSS_configLatchOnPWMSYNC(CMPSS_W_BASE,false,false);
    //
    // Enables the CMPSS module.
    //
    CMPSS_enableModule(CMPSS_W_BASE);
    //
    // Delay for CMPSS DAC to power up.
    //
    DEVICE_DELAY_US(500);
    //
    // Causes a software reset of the high comparator digital filter output latch.
    //
    CMPSS_clearFilterLatchHigh(CMPSS_W_BASE);
}

//*****************************************************************************
//
// CPUTIMER Configurations
//
//*****************************************************************************
void CPUTIMER_init(){
	dataPrint_init();
}

void dataPrint_init(){
	CPUTimer_setEmulationMode(dataPrint_BASE, CPUTIMER_EMULATIONMODE_STOPAFTERNEXTDECREMENT);
	CPUTimer_setPreScaler(dataPrint_BASE, 0U);
	CPUTimer_setPeriod(dataPrint_BASE, 1200000U);
	CPUTimer_disableInterrupt(dataPrint_BASE);
	CPUTimer_stopTimer(dataPrint_BASE);

	CPUTimer_reloadTimerCounter(dataPrint_BASE);
	CPUTimer_startTimer(dataPrint_BASE);
}

//*****************************************************************************
//
// EPWM Configurations
//
//*****************************************************************************
void EPWM_init(){
    EPWM_setClockPrescaler(PhaseU_BASE, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);	
    EPWM_setPeriodLoadMode(PhaseU_BASE, EPWM_PERIOD_DIRECT_LOAD);	
    EPWM_setTimeBasePeriod(PhaseU_BASE, 3000);	
    EPWM_setTimeBaseCounter(PhaseU_BASE, 0);	
    EPWM_setTimeBaseCounterMode(PhaseU_BASE, EPWM_COUNTER_MODE_UP_DOWN);	
    EPWM_setCountModeAfterSync(PhaseU_BASE, EPWM_COUNT_MODE_UP_AFTER_SYNC);	
    EPWM_disablePhaseShiftLoad(PhaseU_BASE);	
    EPWM_setPhaseShift(PhaseU_BASE, 0);	
    EPWM_setSyncInPulseSource(PhaseU_BASE, EPWM_SYNC_IN_PULSE_SRC_DISABLE);	
    EPWM_enableSyncOutPulseSource(PhaseU_BASE, EPWM_SYNC_OUT_PULSE_ON_CNTR_ZERO);	
    EPWM_setCounterCompareValue(PhaseU_BASE, EPWM_COUNTER_COMPARE_A, 500);	
    EPWM_setCounterCompareShadowLoadMode(PhaseU_BASE, EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO);	
    EPWM_setCounterCompareValue(PhaseU_BASE, EPWM_COUNTER_COMPARE_B, 500);	
    EPWM_setCounterCompareShadowLoadMode(PhaseU_BASE, EPWM_COUNTER_COMPARE_B, EPWM_COMP_LOAD_ON_CNTR_ZERO);	
    EPWM_setCounterCompareValue(PhaseU_BASE, EPWM_COUNTER_COMPARE_C, 10);	
    EPWM_setActionQualifierAction(PhaseU_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);	
    EPWM_setActionQualifierAction(PhaseU_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);	
    EPWM_setActionQualifierAction(PhaseU_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
    EPWM_setActionQualifierAction(PhaseU_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);	
    EPWM_setActionQualifierAction(PhaseU_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
    EPWM_setActionQualifierAction(PhaseU_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);	
    EPWM_setActionQualifierAction(PhaseU_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);	
    EPWM_setActionQualifierAction(PhaseU_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);	
    EPWM_setActionQualifierAction(PhaseU_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
    EPWM_setActionQualifierAction(PhaseU_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);	
    EPWM_setActionQualifierAction(PhaseU_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
    EPWM_setActionQualifierAction(PhaseU_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);	
    EPWM_setDeadBandDelayPolarity(PhaseU_BASE, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);	
    EPWM_setDeadBandDelayMode(PhaseU_BASE, EPWM_DB_RED, true);	
    EPWM_setRisingEdgeDelayCountShadowLoadMode(PhaseU_BASE, EPWM_RED_LOAD_ON_CNTR_ZERO);	
    EPWM_disableRisingEdgeDelayCountShadowLoadMode(PhaseU_BASE);	
    EPWM_setRisingEdgeDelayCount(PhaseU_BASE, 5);	
    EPWM_setDeadBandDelayMode(PhaseU_BASE, EPWM_DB_FED, true);	
    EPWM_setFallingEdgeDelayCountShadowLoadMode(PhaseU_BASE, EPWM_FED_LOAD_ON_CNTR_ZERO);	
    EPWM_disableFallingEdgeDelayCountShadowLoadMode(PhaseU_BASE);	
    EPWM_setFallingEdgeDelayCount(PhaseU_BASE, 5);	
    EPWM_setTripZoneAction(PhaseU_BASE, EPWM_TZ_ACTION_EVENT_TZA, EPWM_TZ_ACTION_LOW);	
    EPWM_setTripZoneAction(PhaseU_BASE, EPWM_TZ_ACTION_EVENT_TZB, EPWM_TZ_ACTION_LOW);	
    EPWM_enableTripZoneSignals(PhaseU_BASE, EPWM_TZ_SIGNAL_OSHT1);	
    EPWM_enableADCTrigger(PhaseU_BASE, EPWM_SOC_A);	
    EPWM_setADCTriggerSource(PhaseU_BASE, EPWM_SOC_A, EPWM_SOC_TBCTR_D_CMPC);	
    EPWM_setADCTriggerEventPrescale(PhaseU_BASE, EPWM_SOC_A, 1);	
    EPWM_setClockPrescaler(PhaseV_BASE, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);	
    EPWM_setPeriodLoadMode(PhaseV_BASE, EPWM_PERIOD_DIRECT_LOAD);	
    EPWM_setTimeBasePeriod(PhaseV_BASE, 3000);	
    EPWM_setupEPWMLinks(PhaseV_BASE, EPWM_LINK_WITH_EPWM_1, EPWM_LINK_TBPRD);	
    EPWM_setTimeBaseCounter(PhaseV_BASE, 0);	
    EPWM_setTimeBaseCounterMode(PhaseV_BASE, EPWM_COUNTER_MODE_UP_DOWN);	
    EPWM_setCountModeAfterSync(PhaseV_BASE, EPWM_COUNT_MODE_UP_AFTER_SYNC);	
    EPWM_disablePhaseShiftLoad(PhaseV_BASE);	
    EPWM_setPhaseShift(PhaseV_BASE, 0);	
    EPWM_setSyncInPulseSource(PhaseV_BASE, EPWM_SYNC_IN_PULSE_SRC_DISABLE);	
    EPWM_setCounterCompareValue(PhaseV_BASE, EPWM_COUNTER_COMPARE_A, 500);	
    EPWM_setCounterCompareShadowLoadMode(PhaseV_BASE, EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO);	
    EPWM_setCounterCompareValue(PhaseV_BASE, EPWM_COUNTER_COMPARE_B, 100);	
    EPWM_setCounterCompareShadowLoadMode(PhaseV_BASE, EPWM_COUNTER_COMPARE_B, EPWM_COMP_LOAD_ON_CNTR_ZERO);	
    EPWM_setActionQualifierAction(PhaseV_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);	
    EPWM_setActionQualifierAction(PhaseV_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);	
    EPWM_setActionQualifierAction(PhaseV_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
    EPWM_setActionQualifierAction(PhaseV_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);	
    EPWM_setActionQualifierAction(PhaseV_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
    EPWM_setActionQualifierAction(PhaseV_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);	
    EPWM_setActionQualifierAction(PhaseV_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);	
    EPWM_setActionQualifierAction(PhaseV_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);	
    EPWM_setActionQualifierAction(PhaseV_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
    EPWM_setActionQualifierAction(PhaseV_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);	
    EPWM_setActionQualifierAction(PhaseV_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
    EPWM_setActionQualifierAction(PhaseV_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);	
    EPWM_setDeadBandDelayPolarity(PhaseV_BASE, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);	
    EPWM_setDeadBandDelayMode(PhaseV_BASE, EPWM_DB_RED, true);	
    EPWM_setRisingEdgeDelayCountShadowLoadMode(PhaseV_BASE, EPWM_RED_LOAD_ON_CNTR_ZERO);	
    EPWM_disableRisingEdgeDelayCountShadowLoadMode(PhaseV_BASE);	
    EPWM_setRisingEdgeDelayCount(PhaseV_BASE, 5);	
    EPWM_setDeadBandDelayMode(PhaseV_BASE, EPWM_DB_FED, true);	
    EPWM_setFallingEdgeDelayCountShadowLoadMode(PhaseV_BASE, EPWM_FED_LOAD_ON_CNTR_ZERO);	
    EPWM_disableFallingEdgeDelayCountShadowLoadMode(PhaseV_BASE);	
    EPWM_setFallingEdgeDelayCount(PhaseV_BASE, 5);	
    EPWM_setTripZoneAction(PhaseV_BASE, EPWM_TZ_ACTION_EVENT_TZA, EPWM_TZ_ACTION_LOW);	
    EPWM_setTripZoneAction(PhaseV_BASE, EPWM_TZ_ACTION_EVENT_TZB, EPWM_TZ_ACTION_LOW);	
    EPWM_enableTripZoneSignals(PhaseV_BASE, EPWM_TZ_SIGNAL_OSHT1);	
    EPWM_setClockPrescaler(PhaseW_BASE, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);	
    EPWM_setPeriodLoadMode(PhaseW_BASE, EPWM_PERIOD_DIRECT_LOAD);	
    EPWM_setTimeBasePeriod(PhaseW_BASE, 3000);	
    EPWM_setupEPWMLinks(PhaseW_BASE, EPWM_LINK_WITH_EPWM_1, EPWM_LINK_TBPRD);	
    EPWM_setTimeBaseCounter(PhaseW_BASE, 0);	
    EPWM_setTimeBaseCounterMode(PhaseW_BASE, EPWM_COUNTER_MODE_UP_DOWN);	
    EPWM_setCountModeAfterSync(PhaseW_BASE, EPWM_COUNT_MODE_UP_AFTER_SYNC);	
    EPWM_disablePhaseShiftLoad(PhaseW_BASE);	
    EPWM_setPhaseShift(PhaseW_BASE, 0);	
    EPWM_setSyncInPulseSource(PhaseW_BASE, EPWM_SYNC_IN_PULSE_SRC_DISABLE);	
    EPWM_setCounterCompareValue(PhaseW_BASE, EPWM_COUNTER_COMPARE_A, 500);	
    EPWM_setCounterCompareShadowLoadMode(PhaseW_BASE, EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO);	
    EPWM_setCounterCompareValue(PhaseW_BASE, EPWM_COUNTER_COMPARE_B, 500);	
    EPWM_setCounterCompareShadowLoadMode(PhaseW_BASE, EPWM_COUNTER_COMPARE_B, EPWM_COMP_LOAD_ON_CNTR_ZERO);	
    EPWM_setActionQualifierAction(PhaseW_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);	
    EPWM_setActionQualifierAction(PhaseW_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);	
    EPWM_setActionQualifierAction(PhaseW_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
    EPWM_setActionQualifierAction(PhaseW_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);	
    EPWM_setActionQualifierAction(PhaseW_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
    EPWM_setActionQualifierAction(PhaseW_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);	
    EPWM_setActionQualifierAction(PhaseW_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);	
    EPWM_setActionQualifierAction(PhaseW_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);	
    EPWM_setActionQualifierAction(PhaseW_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
    EPWM_setActionQualifierAction(PhaseW_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);	
    EPWM_setActionQualifierAction(PhaseW_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
    EPWM_setActionQualifierAction(PhaseW_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);	
    EPWM_setDeadBandDelayPolarity(PhaseW_BASE, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);	
    EPWM_setDeadBandDelayMode(PhaseW_BASE, EPWM_DB_RED, true);	
    EPWM_setRisingEdgeDelayCountShadowLoadMode(PhaseW_BASE, EPWM_RED_LOAD_ON_CNTR_ZERO);	
    EPWM_disableRisingEdgeDelayCountShadowLoadMode(PhaseW_BASE);	
    EPWM_setRisingEdgeDelayCount(PhaseW_BASE, 5);	
    EPWM_setDeadBandDelayMode(PhaseW_BASE, EPWM_DB_FED, true);	
    EPWM_setFallingEdgeDelayCountShadowLoadMode(PhaseW_BASE, EPWM_FED_LOAD_ON_CNTR_ZERO);	
    EPWM_disableFallingEdgeDelayCountShadowLoadMode(PhaseW_BASE);	
    EPWM_setFallingEdgeDelayCount(PhaseW_BASE, 5);	
    EPWM_setTripZoneAction(PhaseW_BASE, EPWM_TZ_ACTION_EVENT_TZA, EPWM_TZ_ACTION_LOW);	
    EPWM_setTripZoneAction(PhaseW_BASE, EPWM_TZ_ACTION_EVENT_TZB, EPWM_TZ_ACTION_LOW);	
    EPWM_enableTripZoneSignals(PhaseW_BASE, EPWM_TZ_SIGNAL_OSHT1);	
}

//*****************************************************************************
//
// GPIO Configurations
//
//*****************************************************************************
void GPIO_init(){
	GATE_Control_init();
	OT_INPUT_init();
}

void GATE_Control_init(){
	GPIO_writePin(GATE_Control, 1);
	GPIO_setPadConfig(GATE_Control, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(GATE_Control, GPIO_QUAL_SYNC);
	GPIO_setDirectionMode(GATE_Control, GPIO_DIR_MODE_OUT);
	GPIO_setControllerCore(GATE_Control, GPIO_CORE_CPU1);
}
void OT_INPUT_init(){
	GPIO_setPadConfig(OT_INPUT, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(OT_INPUT, GPIO_QUAL_3SAMPLE);
	GPIO_setDirectionMode(OT_INPUT, GPIO_DIR_MODE_IN);
	GPIO_setControllerCore(OT_INPUT, GPIO_CORE_CPU1);
}

//*****************************************************************************
//
// I2C Configurations
//
//*****************************************************************************
void I2C_init(){
	AS5600_init();
}

void AS5600_init(){
	I2C_disableModule(AS5600_BASE);
	I2C_initController(AS5600_BASE, DEVICE_SYSCLK_FREQ, AS5600_BITRATE, I2C_DUTYCYCLE_50);
	I2C_setConfig(AS5600_BASE, I2C_CONTROLLER_SEND_MODE);
	I2C_disableLoopback(AS5600_BASE);
	I2C_setOwnAddress(AS5600_BASE, AS5600_OWN_ADDRESS);
	I2C_setTargetAddress(AS5600_BASE, AS5600_TARGET_ADDRESS);
	I2C_setBitCount(AS5600_BASE, I2C_BITCOUNT_8);
	I2C_setDataCount(AS5600_BASE, 0);
	I2C_setAddressMode(AS5600_BASE, I2C_ADDR_MODE_7BITS);
	I2C_disableFIFO(AS5600_BASE);
	I2C_setEmulationMode(AS5600_BASE, I2C_EMULATION_FREE_RUN);
	I2C_enableModule(AS5600_BASE);
}

//*****************************************************************************
//
// INPUTXBAR Configurations
//
//*****************************************************************************
void INPUTXBAR_init(){
	OT_inputX_init();
}

void OT_inputX_init(){
	XBAR_setInputPin(INPUTXBAR_BASE, OT_inputX_INPUT, OT_inputX_SOURCE);
	XBAR_lockInput(INPUTXBAR_BASE, OT_inputX_INPUT);
}

//*****************************************************************************
//
// INTERRUPT Configurations
//
//*****************************************************************************
void INTERRUPT_init(){
	
	// Interrupt Settings for INT_Motor_IS_ADC_1
	// ISR need to be defined for the registered interrupts
	Interrupt_register(INT_Motor_IS_ADC_1, &INT_Motor_IS_ADC_1_ISR);
	Interrupt_enable(INT_Motor_IS_ADC_1);
}
//*****************************************************************************
//
// SCI Configurations
//
//*****************************************************************************
void SCI_init(){
	PC_SCI_init();
}

void PC_SCI_init(){
	SCI_clearInterruptStatus(PC_SCI_BASE, SCI_INT_RXFF | SCI_INT_TXFF | SCI_INT_FE | SCI_INT_OE | SCI_INT_PE | SCI_INT_RXERR | SCI_INT_RXRDY_BRKDT | SCI_INT_TXRDY);
	SCI_clearOverflowStatus(PC_SCI_BASE);
	SCI_resetTxFIFO(PC_SCI_BASE);
	SCI_resetRxFIFO(PC_SCI_BASE);
	SCI_resetChannels(PC_SCI_BASE);
	SCI_setConfig(PC_SCI_BASE, DEVICE_LSPCLK_FREQ, PC_SCI_BAUDRATE, (SCI_CONFIG_WLEN_8|SCI_CONFIG_STOP_ONE|SCI_CONFIG_PAR_NONE));
	SCI_disableLoopback(PC_SCI_BASE);
	SCI_performSoftwareReset(PC_SCI_BASE);
	SCI_enableFIFO(PC_SCI_BASE);
	SCI_enableModule(PC_SCI_BASE);
}

//*****************************************************************************
//
// SYNC Scheme Configurations
//
//*****************************************************************************
void SYNC_init(){
	SysCtl_setSyncOutputConfig(SYSCTL_SYNC_OUT_SRC_EPWM1SYNCOUT);
	//
	// SOCA
	//
	SysCtl_enableExtADCSOCSource(0);
	//
	// SOCB
	//
	SysCtl_enableExtADCSOCSource(0);
}
