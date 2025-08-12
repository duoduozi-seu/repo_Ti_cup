/*
 * Copyright (c) 2023, Texas Instruments Incorporated
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
 */

/*
 *  ============ ti_msp_dl_config.c =============
 *  Configured MSPM0 DriverLib module definitions
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */

#include "ti_msp_dl_config.h"

DL_TimerA_backupConfig gPWM_MOTOR_1Backup;
DL_TimerA_backupConfig gENCODER1ABackup;
DL_TimerG_backupConfig gENCODER2ABackup;
DL_TimerG_backupConfig gCLOCK_ZYDBackup;
DL_UART_Main_backupConfig gUART_FORWORDBackup;

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform any initialization needed before using any board APIs
 */
SYSCONFIG_WEAK void SYSCFG_DL_init(void)
{
    SYSCFG_DL_initPower();
    SYSCFG_DL_GPIO_init();
    /* Module-Specific Initializations*/
    SYSCFG_DL_SYSCTL_init();
    SYSCFG_DL_PWM_MOTOR_1_init();
    SYSCFG_DL_ENCODER1A_init();
    SYSCFG_DL_ENCODER2A_init();
    SYSCFG_DL_CLOCK_ZYD_init();
    SYSCFG_DL_CLOCK_PTZ_init();
    SYSCFG_DL_I2C_OLED_init();
    SYSCFG_DL_I2C_LSM6DSV16X_init();
    SYSCFG_DL_UART_K230_init();
    SYSCFG_DL_UART_FORWORD_init();
    SYSCFG_DL_UART_BACKWARD_init();
    SYSCFG_DL_DMA_init();
    /* Ensure backup structures have no valid state */
	gPWM_MOTOR_1Backup.backupRdy 	= false;
	gENCODER1ABackup.backupRdy 	= false;
	gENCODER2ABackup.backupRdy 	= false;
	gCLOCK_ZYDBackup.backupRdy 	= false;
	gUART_FORWORDBackup.backupRdy 	= false;

}
/*
 * User should take care to save and restore register configuration in application.
 * See Retention Configuration section for more details.
 */
SYSCONFIG_WEAK bool SYSCFG_DL_saveConfiguration(void)
{
    bool retStatus = true;

	retStatus &= DL_TimerA_saveConfiguration(PWM_MOTOR_1_INST, &gPWM_MOTOR_1Backup);
	retStatus &= DL_TimerA_saveConfiguration(ENCODER1A_INST, &gENCODER1ABackup);
	retStatus &= DL_TimerG_saveConfiguration(ENCODER2A_INST, &gENCODER2ABackup);
	retStatus &= DL_TimerG_saveConfiguration(CLOCK_ZYD_INST, &gCLOCK_ZYDBackup);
	retStatus &= DL_UART_Main_saveConfiguration(UART_FORWORD_INST, &gUART_FORWORDBackup);

    return retStatus;
}


SYSCONFIG_WEAK bool SYSCFG_DL_restoreConfiguration(void)
{
    bool retStatus = true;

	retStatus &= DL_TimerA_restoreConfiguration(PWM_MOTOR_1_INST, &gPWM_MOTOR_1Backup, false);
	retStatus &= DL_TimerA_restoreConfiguration(ENCODER1A_INST, &gENCODER1ABackup, false);
	retStatus &= DL_TimerG_restoreConfiguration(ENCODER2A_INST, &gENCODER2ABackup, false);
	retStatus &= DL_TimerG_restoreConfiguration(CLOCK_ZYD_INST, &gCLOCK_ZYDBackup, false);
	retStatus &= DL_UART_Main_restoreConfiguration(UART_FORWORD_INST, &gUART_FORWORDBackup);

    return retStatus;
}

SYSCONFIG_WEAK void SYSCFG_DL_initPower(void)
{
    DL_GPIO_reset(GPIOA);
    DL_GPIO_reset(GPIOB);
    DL_TimerA_reset(PWM_MOTOR_1_INST);
    DL_TimerA_reset(ENCODER1A_INST);
    DL_TimerG_reset(ENCODER2A_INST);
    DL_TimerG_reset(CLOCK_ZYD_INST);
    DL_TimerG_reset(CLOCK_PTZ_INST);
    DL_I2C_reset(I2C_OLED_INST);
    DL_I2C_reset(I2C_LSM6DSV16X_INST);
    DL_UART_Main_reset(UART_K230_INST);
    DL_UART_Main_reset(UART_FORWORD_INST);
    DL_UART_Main_reset(UART_BACKWARD_INST);


    DL_GPIO_enablePower(GPIOA);
    DL_GPIO_enablePower(GPIOB);
    DL_TimerA_enablePower(PWM_MOTOR_1_INST);
    DL_TimerA_enablePower(ENCODER1A_INST);
    DL_TimerG_enablePower(ENCODER2A_INST);
    DL_TimerG_enablePower(CLOCK_ZYD_INST);
    DL_TimerG_enablePower(CLOCK_PTZ_INST);
    DL_I2C_enablePower(I2C_OLED_INST);
    DL_I2C_enablePower(I2C_LSM6DSV16X_INST);
    DL_UART_Main_enablePower(UART_K230_INST);
    DL_UART_Main_enablePower(UART_FORWORD_INST);
    DL_UART_Main_enablePower(UART_BACKWARD_INST);

    delay_cycles(POWER_STARTUP_DELAY);
}

SYSCONFIG_WEAK void SYSCFG_DL_GPIO_init(void)
{

    DL_GPIO_initPeripheralOutputFunction(GPIO_PWM_MOTOR_1_C0_IOMUX,GPIO_PWM_MOTOR_1_C0_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_PWM_MOTOR_1_C0_PORT, GPIO_PWM_MOTOR_1_C0_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_PWM_MOTOR_1_C1_IOMUX,GPIO_PWM_MOTOR_1_C1_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_PWM_MOTOR_1_C1_PORT, GPIO_PWM_MOTOR_1_C1_PIN);

    DL_GPIO_initPeripheralInputFunction(GPIO_ENCODER1A_C0_IOMUX,GPIO_ENCODER1A_C0_IOMUX_FUNC);
    DL_GPIO_initPeripheralInputFunction(GPIO_ENCODER2A_C0_IOMUX,GPIO_ENCODER2A_C0_IOMUX_FUNC);

    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_I2C_OLED_IOMUX_SDA,
        GPIO_I2C_OLED_IOMUX_SDA_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_I2C_OLED_IOMUX_SCL,
        GPIO_I2C_OLED_IOMUX_SCL_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(GPIO_I2C_OLED_IOMUX_SDA);
    DL_GPIO_enableHiZ(GPIO_I2C_OLED_IOMUX_SCL);
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_I2C_LSM6DSV16X_IOMUX_SDA,
        GPIO_I2C_LSM6DSV16X_IOMUX_SDA_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_I2C_LSM6DSV16X_IOMUX_SCL,
        GPIO_I2C_LSM6DSV16X_IOMUX_SCL_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(GPIO_I2C_LSM6DSV16X_IOMUX_SDA);
    DL_GPIO_enableHiZ(GPIO_I2C_LSM6DSV16X_IOMUX_SCL);

    DL_GPIO_initPeripheralOutputFunction(
        GPIO_UART_K230_IOMUX_TX, GPIO_UART_K230_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_UART_K230_IOMUX_RX, GPIO_UART_K230_IOMUX_RX_FUNC);
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_UART_FORWORD_IOMUX_TX, GPIO_UART_FORWORD_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_UART_FORWORD_IOMUX_RX, GPIO_UART_FORWORD_IOMUX_RX_FUNC);
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_UART_BACKWARD_IOMUX_TX, GPIO_UART_BACKWARD_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_UART_BACKWARD_IOMUX_RX, GPIO_UART_BACKWARD_IOMUX_RX_FUNC);

    DL_GPIO_initDigitalInputFeatures(GPIO_LSM6DSV16X_PIN_LSM6DSV16X_INT_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(GPIO_KEY_PIN_C3_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(GPIO_KEY_PIN_C4_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalOutput(GPIO_KEY_PIN_R1_IOMUX);

    DL_GPIO_initDigitalOutput(GPIO_KEY_PIN_R2_IOMUX);

    DL_GPIO_initDigitalOutput(GPIO_KEY_PIN_R3_IOMUX);

    DL_GPIO_initDigitalOutput(GPIO_KEY_PIN_R4_IOMUX);

    DL_GPIO_initDigitalInputFeatures(GPIO_KEY_PIN_C1_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(GPIO_KEY_PIN_C2_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalOutput(GPIO_MOTOR_PIN_MOTOR_AIN1_IOMUX);

    DL_GPIO_initDigitalOutput(GPIO_MOTOR_PIN_MOTOR_AIN2_IOMUX);

    DL_GPIO_initDigitalOutput(GPIO_MOTOR_PIN_MOTOR_BIN1_IOMUX);

    DL_GPIO_initDigitalOutput(GPIO_MOTOR_PIN_MOTOR_BIN2_IOMUX);

    DL_GPIO_initDigitalInputFeatures(GPIO_ENCODER_PIN_ENCODER1B_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(GPIO_ENCODER_PIN_ENCODER2B_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_clearPins(GPIOA, GPIO_MOTOR_PIN_MOTOR_AIN2_PIN |
		GPIO_MOTOR_PIN_MOTOR_BIN1_PIN |
		GPIO_MOTOR_PIN_MOTOR_BIN2_PIN);
    DL_GPIO_enableOutput(GPIOA, GPIO_MOTOR_PIN_MOTOR_AIN2_PIN |
		GPIO_MOTOR_PIN_MOTOR_BIN1_PIN |
		GPIO_MOTOR_PIN_MOTOR_BIN2_PIN);
    DL_GPIO_clearPins(GPIOB, GPIO_MOTOR_PIN_MOTOR_AIN1_PIN);
    DL_GPIO_setPins(GPIOB, GPIO_KEY_PIN_R1_PIN |
		GPIO_KEY_PIN_R2_PIN |
		GPIO_KEY_PIN_R3_PIN |
		GPIO_KEY_PIN_R4_PIN);
    DL_GPIO_enableOutput(GPIOB, GPIO_KEY_PIN_R1_PIN |
		GPIO_KEY_PIN_R2_PIN |
		GPIO_KEY_PIN_R3_PIN |
		GPIO_KEY_PIN_R4_PIN |
		GPIO_MOTOR_PIN_MOTOR_AIN1_PIN);
    DL_GPIO_setLowerPinsPolarity(GPIOB, DL_GPIO_PIN_12_EDGE_RISE |
		DL_GPIO_PIN_6_EDGE_RISE);
    DL_GPIO_clearInterruptStatus(GPIOB, GPIO_LSM6DSV16X_PIN_LSM6DSV16X_INT_PIN);
    DL_GPIO_enableInterrupt(GPIOB, GPIO_LSM6DSV16X_PIN_LSM6DSV16X_INT_PIN);

}


static const DL_SYSCTL_SYSPLLConfig gSYSPLLConfig = {
    .inputFreq              = DL_SYSCTL_SYSPLL_INPUT_FREQ_16_32_MHZ,
	.rDivClk2x              = 1,
	.rDivClk1               = 0,
	.rDivClk0               = 0,
	.enableCLK2x            = DL_SYSCTL_SYSPLL_CLK2X_ENABLE,
	.enableCLK1             = DL_SYSCTL_SYSPLL_CLK1_DISABLE,
	.enableCLK0             = DL_SYSCTL_SYSPLL_CLK0_DISABLE,
	.sysPLLMCLK             = DL_SYSCTL_SYSPLL_MCLK_CLK2X,
	.sysPLLRef              = DL_SYSCTL_SYSPLL_REF_SYSOSC,
	.qDiv                   = 4,
	.pDiv                   = DL_SYSCTL_SYSPLL_PDIV_2
};
SYSCONFIG_WEAK void SYSCFG_DL_SYSCTL_init(void)
{

	//Low Power Mode is configured to be SLEEP0
    DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL_0);
    DL_SYSCTL_setFlashWaitState(DL_SYSCTL_FLASH_WAIT_STATE_2);

    
	DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);
	/* Set default configuration */
	DL_SYSCTL_disableHFXT();
	DL_SYSCTL_disableSYSPLL();
    DL_SYSCTL_configSYSPLL((DL_SYSCTL_SYSPLLConfig *) &gSYSPLLConfig);
    DL_SYSCTL_setULPCLKDivider(DL_SYSCTL_ULPCLK_DIV_2);
    DL_SYSCTL_setMCLKSource(SYSOSC, HSCLK, DL_SYSCTL_HSCLK_SOURCE_SYSPLL);
    /* INT_GROUP1 Priority */
    NVIC_SetPriority(GPIOB_INT_IRQn, 3);

}


/*
 * Timer clock configuration to be sourced by  / 2 (40000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   40000000 Hz = 40000000 Hz / (2 * (0 + 1))
 */
static const DL_TimerA_ClockConfig gPWM_MOTOR_1ClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_2,
    .prescale = 0U
};

static const DL_TimerA_PWMConfig gPWM_MOTOR_1Config = {
    .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN_UP,
    .period = 4000,
    .isTimerWithFourCC = true,
    .startTimer = DL_TIMER_START,
};

SYSCONFIG_WEAK void SYSCFG_DL_PWM_MOTOR_1_init(void) {

    DL_TimerA_setClockConfig(
        PWM_MOTOR_1_INST, (DL_TimerA_ClockConfig *) &gPWM_MOTOR_1ClockConfig);

    DL_TimerA_initPWMMode(
        PWM_MOTOR_1_INST, (DL_TimerA_PWMConfig *) &gPWM_MOTOR_1Config);

    // Set Counter control to the smallest CC index being used
    DL_TimerA_setCounterControl(PWM_MOTOR_1_INST,DL_TIMER_CZC_CCCTL0_ZCOND,DL_TIMER_CAC_CCCTL0_ACOND,DL_TIMER_CLC_CCCTL0_LCOND);

    DL_TimerA_setCaptureCompareOutCtl(PWM_MOTOR_1_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERA_CAPTURE_COMPARE_0_INDEX);

    DL_TimerA_setCaptCompUpdateMethod(PWM_MOTOR_1_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERA_CAPTURE_COMPARE_0_INDEX);
    DL_TimerA_setCaptureCompareValue(PWM_MOTOR_1_INST, 0, DL_TIMER_CC_0_INDEX);

    DL_TimerA_setCaptureCompareOutCtl(PWM_MOTOR_1_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERA_CAPTURE_COMPARE_1_INDEX);

    DL_TimerA_setCaptCompUpdateMethod(PWM_MOTOR_1_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERA_CAPTURE_COMPARE_1_INDEX);
    DL_TimerA_setCaptureCompareValue(PWM_MOTOR_1_INST, 0, DL_TIMER_CC_1_INDEX);

    DL_TimerA_enableClock(PWM_MOTOR_1_INST);


    
    DL_TimerA_setCCPDirection(PWM_MOTOR_1_INST , DL_TIMER_CC0_OUTPUT | DL_TIMER_CC1_OUTPUT );


}



/*
 * Timer clock configuration to be sourced by BUSCLK /  (80000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   40000000 Hz = 80000000 Hz / (1 * (1 + 1))
 */
static const DL_TimerA_ClockConfig gENCODER1AClockConfig = {
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale = 1U
};

/*
 * Timer load value (where the counter starts from) is calculated as (timerPeriod * timerClockFreq) - 1
 * ENCODER1A_INST_LOAD_VALUE = (1 ms * 40000000 Hz) - 1
 */
static const DL_TimerA_CaptureConfig gENCODER1ACaptureConfig = {
    .captureMode    = DL_TIMER_CAPTURE_MODE_EDGE_TIME,
    .period         = ENCODER1A_INST_LOAD_VALUE,
    .startTimer     = DL_TIMER_START,
    .edgeCaptMode   = DL_TIMER_CAPTURE_EDGE_DETECTION_MODE_RISING,
    .inputChan      = DL_TIMER_INPUT_CHAN_0,
    .inputInvMode   = DL_TIMER_CC_INPUT_INV_NOINVERT,
};

SYSCONFIG_WEAK void SYSCFG_DL_ENCODER1A_init(void) {

    DL_TimerA_setClockConfig(ENCODER1A_INST,
        (DL_TimerA_ClockConfig *) &gENCODER1AClockConfig);

    DL_TimerA_initCaptureMode(ENCODER1A_INST,
        (DL_TimerA_CaptureConfig *) &gENCODER1ACaptureConfig);
    DL_TimerA_enableInterrupt(ENCODER1A_INST , DL_TIMERA_INTERRUPT_CC0_DN_EVENT);

    DL_TimerA_enableClock(ENCODER1A_INST);

}

/*
 * Timer clock configuration to be sourced by BUSCLK /  (80000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   40000000 Hz = 80000000 Hz / (1 * (1 + 1))
 */
static const DL_TimerG_ClockConfig gENCODER2AClockConfig = {
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale = 1U
};

/*
 * Timer load value (where the counter starts from) is calculated as (timerPeriod * timerClockFreq) - 1
 * ENCODER2A_INST_LOAD_VALUE = (1 ms * 40000000 Hz) - 1
 */
static const DL_TimerG_CaptureConfig gENCODER2ACaptureConfig = {
    .captureMode    = DL_TIMER_CAPTURE_MODE_EDGE_TIME,
    .period         = ENCODER2A_INST_LOAD_VALUE,
    .startTimer     = DL_TIMER_START,
    .edgeCaptMode   = DL_TIMER_CAPTURE_EDGE_DETECTION_MODE_RISING,
    .inputChan      = DL_TIMER_INPUT_CHAN_0,
    .inputInvMode   = DL_TIMER_CC_INPUT_INV_NOINVERT,
};

SYSCONFIG_WEAK void SYSCFG_DL_ENCODER2A_init(void) {

    DL_TimerG_setClockConfig(ENCODER2A_INST,
        (DL_TimerG_ClockConfig *) &gENCODER2AClockConfig);

    DL_TimerG_initCaptureMode(ENCODER2A_INST,
        (DL_TimerG_CaptureConfig *) &gENCODER2ACaptureConfig);
    DL_TimerG_enableInterrupt(ENCODER2A_INST , DL_TIMERG_INTERRUPT_CC0_DN_EVENT);

    DL_TimerG_enableClock(ENCODER2A_INST);

}


/*
 * Timer clock configuration to be sourced by BUSCLK /  (20000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   400000 Hz = 20000000 Hz / (4 * (49 + 1))
 */
static const DL_TimerG_ClockConfig gCLOCK_ZYDClockConfig = {
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_4,
    .prescale    = 49U,
};

/*
 * Timer load value (where the counter starts from) is calculated as (timerPeriod * timerClockFreq) - 1
 * CLOCK_ZYD_INST_LOAD_VALUE = (10 ms * 400000 Hz) - 1
 */
static const DL_TimerG_TimerConfig gCLOCK_ZYDTimerConfig = {
    .period     = CLOCK_ZYD_INST_LOAD_VALUE,
    .timerMode  = DL_TIMER_TIMER_MODE_PERIODIC,
    .startTimer = DL_TIMER_START,
};

SYSCONFIG_WEAK void SYSCFG_DL_CLOCK_ZYD_init(void) {

    DL_TimerG_setClockConfig(CLOCK_ZYD_INST,
        (DL_TimerG_ClockConfig *) &gCLOCK_ZYDClockConfig);

    DL_TimerG_initTimerMode(CLOCK_ZYD_INST,
        (DL_TimerG_TimerConfig *) &gCLOCK_ZYDTimerConfig);
    DL_TimerG_enableInterrupt(CLOCK_ZYD_INST , DL_TIMERG_INTERRUPT_ZERO_EVENT);
    DL_TimerG_enableClock(CLOCK_ZYD_INST);





}

/*
 * Timer clock configuration to be sourced by BUSCLK /  (5000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   500000 Hz = 5000000 Hz / (8 * (9 + 1))
 */
static const DL_TimerG_ClockConfig gCLOCK_PTZClockConfig = {
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_8,
    .prescale    = 9U,
};

/*
 * Timer load value (where the counter starts from) is calculated as (timerPeriod * timerClockFreq) - 1
 * CLOCK_PTZ_INST_LOAD_VALUE = (80 ms * 500000 Hz) - 1
 */
static const DL_TimerG_TimerConfig gCLOCK_PTZTimerConfig = {
    .period     = CLOCK_PTZ_INST_LOAD_VALUE,
    .timerMode  = DL_TIMER_TIMER_MODE_PERIODIC,
    .startTimer = DL_TIMER_START,
};

SYSCONFIG_WEAK void SYSCFG_DL_CLOCK_PTZ_init(void) {

    DL_TimerG_setClockConfig(CLOCK_PTZ_INST,
        (DL_TimerG_ClockConfig *) &gCLOCK_PTZClockConfig);

    DL_TimerG_initTimerMode(CLOCK_PTZ_INST,
        (DL_TimerG_TimerConfig *) &gCLOCK_PTZTimerConfig);
    DL_TimerG_enableInterrupt(CLOCK_PTZ_INST , DL_TIMERG_INTERRUPT_ZERO_EVENT);
    DL_TimerG_enableClock(CLOCK_PTZ_INST);





}


static const DL_I2C_ClockConfig gI2C_OLEDClockConfig = {
    .clockSel = DL_I2C_CLOCK_BUSCLK,
    .divideRatio = DL_I2C_CLOCK_DIVIDE_1,
};

SYSCONFIG_WEAK void SYSCFG_DL_I2C_OLED_init(void) {

    DL_I2C_setClockConfig(I2C_OLED_INST,
        (DL_I2C_ClockConfig *) &gI2C_OLEDClockConfig);
    DL_I2C_setAnalogGlitchFilterPulseWidth(I2C_OLED_INST,
        DL_I2C_ANALOG_GLITCH_FILTER_WIDTH_50NS);
    DL_I2C_enableAnalogGlitchFilter(I2C_OLED_INST);

    /* Configure Controller Mode */
    DL_I2C_resetControllerTransfer(I2C_OLED_INST);
    /* Set frequency to 400000 Hz*/
    DL_I2C_setTimerPeriod(I2C_OLED_INST, 9);
    DL_I2C_setControllerTXFIFOThreshold(I2C_OLED_INST, DL_I2C_TX_FIFO_LEVEL_EMPTY);
    DL_I2C_setControllerRXFIFOThreshold(I2C_OLED_INST, DL_I2C_RX_FIFO_LEVEL_BYTES_1);
    DL_I2C_enableControllerClockStretching(I2C_OLED_INST);


    /* Enable module */
    DL_I2C_enableController(I2C_OLED_INST);


}
static const DL_I2C_ClockConfig gI2C_LSM6DSV16XClockConfig = {
    .clockSel = DL_I2C_CLOCK_BUSCLK,
    .divideRatio = DL_I2C_CLOCK_DIVIDE_1,
};

SYSCONFIG_WEAK void SYSCFG_DL_I2C_LSM6DSV16X_init(void) {

    DL_I2C_setClockConfig(I2C_LSM6DSV16X_INST,
        (DL_I2C_ClockConfig *) &gI2C_LSM6DSV16XClockConfig);
    DL_I2C_setAnalogGlitchFilterPulseWidth(I2C_LSM6DSV16X_INST,
        DL_I2C_ANALOG_GLITCH_FILTER_WIDTH_50NS);
    DL_I2C_enableAnalogGlitchFilter(I2C_LSM6DSV16X_INST);

    /* Configure Controller Mode */
    DL_I2C_resetControllerTransfer(I2C_LSM6DSV16X_INST);
    /* Set frequency to 100000 Hz*/
    DL_I2C_setTimerPeriod(I2C_LSM6DSV16X_INST, 39);
    DL_I2C_setControllerTXFIFOThreshold(I2C_LSM6DSV16X_INST, DL_I2C_TX_FIFO_LEVEL_EMPTY);
    DL_I2C_setControllerRXFIFOThreshold(I2C_LSM6DSV16X_INST, DL_I2C_RX_FIFO_LEVEL_BYTES_1);
    DL_I2C_enableControllerClockStretching(I2C_LSM6DSV16X_INST);


    /* Enable module */
    DL_I2C_enableController(I2C_LSM6DSV16X_INST);


}

static const DL_UART_Main_ClockConfig gUART_K230ClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gUART_K230Config = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_UART_K230_init(void)
{
    DL_UART_Main_setClockConfig(UART_K230_INST, (DL_UART_Main_ClockConfig *) &gUART_K230ClockConfig);

    DL_UART_Main_init(UART_K230_INST, (DL_UART_Main_Config *) &gUART_K230Config);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 115200
     *  Actual baud rate: 115190.78
     */
    DL_UART_Main_setOversampling(UART_K230_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(UART_K230_INST, UART_K230_IBRD_40_MHZ_115200_BAUD, UART_K230_FBRD_40_MHZ_115200_BAUD);


    /* Configure Interrupts */
    DL_UART_Main_enableInterrupt(UART_K230_INST,
                                 DL_UART_MAIN_INTERRUPT_RX);

    /* Configure DMA Receive Event */
    DL_UART_Main_enableDMAReceiveEvent(UART_K230_INST, DL_UART_DMA_INTERRUPT_RX);
    /* Configure FIFOs */
    DL_UART_Main_enableFIFOs(UART_K230_INST);
    DL_UART_Main_setRXFIFOThreshold(UART_K230_INST, DL_UART_RX_FIFO_LEVEL_FULL);
    DL_UART_Main_setTXFIFOThreshold(UART_K230_INST, DL_UART_TX_FIFO_LEVEL_EMPTY);

    DL_UART_Main_enable(UART_K230_INST);
}
static const DL_UART_Main_ClockConfig gUART_FORWORDClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gUART_FORWORDConfig = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_UART_FORWORD_init(void)
{
    DL_UART_Main_setClockConfig(UART_FORWORD_INST, (DL_UART_Main_ClockConfig *) &gUART_FORWORDClockConfig);

    DL_UART_Main_init(UART_FORWORD_INST, (DL_UART_Main_Config *) &gUART_FORWORDConfig);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 115200
     *  Actual baud rate: 115190.78
     */
    DL_UART_Main_setOversampling(UART_FORWORD_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(UART_FORWORD_INST, UART_FORWORD_IBRD_80_MHZ_115200_BAUD, UART_FORWORD_FBRD_80_MHZ_115200_BAUD);


    /* Configure Interrupts */
    DL_UART_Main_enableInterrupt(UART_FORWORD_INST,
                                 DL_UART_MAIN_INTERRUPT_DMA_DONE_RX |
                                 DL_UART_MAIN_INTERRUPT_EOT_DONE);

    /* Configure DMA Receive Event */
    DL_UART_Main_enableDMAReceiveEvent(UART_FORWORD_INST, DL_UART_DMA_INTERRUPT_RX);
    /* Configure DMA Transmit Event */
    DL_UART_Main_enableDMATransmitEvent(UART_FORWORD_INST);

    DL_UART_Main_enable(UART_FORWORD_INST);
}
static const DL_UART_Main_ClockConfig gUART_BACKWARDClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gUART_BACKWARDConfig = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_UART_BACKWARD_init(void)
{
    DL_UART_Main_setClockConfig(UART_BACKWARD_INST, (DL_UART_Main_ClockConfig *) &gUART_BACKWARDClockConfig);

    DL_UART_Main_init(UART_BACKWARD_INST, (DL_UART_Main_Config *) &gUART_BACKWARDConfig);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 115200
     *  Actual baud rate: 115190.78
     */
    DL_UART_Main_setOversampling(UART_BACKWARD_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(UART_BACKWARD_INST, UART_BACKWARD_IBRD_40_MHZ_115200_BAUD, UART_BACKWARD_FBRD_40_MHZ_115200_BAUD);


    /* Configure Interrupts */
    DL_UART_Main_enableInterrupt(UART_BACKWARD_INST,
                                 DL_UART_MAIN_INTERRUPT_DMA_DONE_TX |
                                 DL_UART_MAIN_INTERRUPT_EOT_DONE);

    /* Configure DMA Transmit Event */
    DL_UART_Main_enableDMATransmitEvent(UART_BACKWARD_INST);

    DL_UART_Main_enable(UART_BACKWARD_INST);
}

static const DL_DMA_Config gDMA_CH0Config = {
    .transferMode   = DL_DMA_SINGLE_TRANSFER_MODE,
    .extendedMode   = DL_DMA_NORMAL_MODE,
    .destIncrement  = DL_DMA_ADDR_INCREMENT,
    .srcIncrement   = DL_DMA_ADDR_UNCHANGED,
    .destWidth      = DL_DMA_WIDTH_BYTE,
    .srcWidth       = DL_DMA_WIDTH_BYTE,
    .trigger        = UART_K230_INST_DMA_TRIGGER,
    .triggerType    = DL_DMA_TRIGGER_TYPE_EXTERNAL,
};

SYSCONFIG_WEAK void SYSCFG_DL_DMA_CH0_init(void)
{
    DL_DMA_clearInterruptStatus(DMA, DL_DMA_INTERRUPT_CHANNEL0);
    DL_DMA_enableInterrupt(DMA, DL_DMA_INTERRUPT_CHANNEL0);
    DL_DMA_setTransferSize(DMA, DMA_CH0_CHAN_ID, 7);
    DL_DMA_initChannel(DMA, DMA_CH0_CHAN_ID , (DL_DMA_Config *) &gDMA_CH0Config);
}
static const DL_DMA_Config gDMA_CH1Config = {
    .transferMode   = DL_DMA_FULL_CH_REPEAT_SINGLE_TRANSFER_MODE,
    .extendedMode   = DL_DMA_NORMAL_MODE,
    .destIncrement  = DL_DMA_ADDR_INCREMENT,
    .srcIncrement   = DL_DMA_ADDR_UNCHANGED,
    .destWidth      = DL_DMA_WIDTH_BYTE,
    .srcWidth       = DL_DMA_WIDTH_BYTE,
    .trigger        = UART_FORWORD_INST_DMA_TRIGGER_0,
    .triggerType    = DL_DMA_TRIGGER_TYPE_EXTERNAL,
};

SYSCONFIG_WEAK void SYSCFG_DL_DMA_CH1_init(void)
{
    DL_DMA_clearInterruptStatus(DMA, DL_DMA_INTERRUPT_CHANNEL1);
    DL_DMA_enableInterrupt(DMA, DL_DMA_INTERRUPT_CHANNEL1);
    DL_DMA_setTransferSize(DMA, DMA_CH1_CHAN_ID, 30);
    DL_DMA_initChannel(DMA, DMA_CH1_CHAN_ID , (DL_DMA_Config *) &gDMA_CH1Config);
}
static const DL_DMA_Config gDMA_CH2Config = {
    .transferMode   = DL_DMA_SINGLE_TRANSFER_MODE,
    .extendedMode   = DL_DMA_NORMAL_MODE,
    .destIncrement  = DL_DMA_ADDR_UNCHANGED,
    .srcIncrement   = DL_DMA_ADDR_INCREMENT,
    .destWidth      = DL_DMA_WIDTH_BYTE,
    .srcWidth       = DL_DMA_WIDTH_BYTE,
    .trigger        = UART_FORWORD_INST_DMA_TRIGGER_1,
    .triggerType    = DL_DMA_TRIGGER_TYPE_EXTERNAL,
};

SYSCONFIG_WEAK void SYSCFG_DL_DMA_CH2_init(void)
{
    DL_DMA_clearInterruptStatus(DMA, DL_DMA_INTERRUPT_CHANNEL2);
    DL_DMA_enableInterrupt(DMA, DL_DMA_INTERRUPT_CHANNEL2);
    DL_DMA_setTransferSize(DMA, DMA_CH2_CHAN_ID, 29);
    DL_DMA_initChannel(DMA, DMA_CH2_CHAN_ID , (DL_DMA_Config *) &gDMA_CH2Config);
}
static const DL_DMA_Config gDMA_CH3Config = {
    .transferMode   = DL_DMA_SINGLE_TRANSFER_MODE,
    .extendedMode   = DL_DMA_NORMAL_MODE,
    .destIncrement  = DL_DMA_ADDR_UNCHANGED,
    .srcIncrement   = DL_DMA_ADDR_INCREMENT,
    .destWidth      = DL_DMA_WIDTH_BYTE,
    .srcWidth       = DL_DMA_WIDTH_BYTE,
    .trigger        = UART_BACKWARD_INST_DMA_TRIGGER,
    .triggerType    = DL_DMA_TRIGGER_TYPE_EXTERNAL,
};

SYSCONFIG_WEAK void SYSCFG_DL_DMA_CH3_init(void)
{
    DL_DMA_clearInterruptStatus(DMA, DL_DMA_INTERRUPT_CHANNEL3);
    DL_DMA_enableInterrupt(DMA, DL_DMA_INTERRUPT_CHANNEL3);
    DL_DMA_setTransferSize(DMA, DMA_CH3_CHAN_ID, 16);
    DL_DMA_initChannel(DMA, DMA_CH3_CHAN_ID , (DL_DMA_Config *) &gDMA_CH3Config);
}
SYSCONFIG_WEAK void SYSCFG_DL_DMA_init(void){
    SYSCFG_DL_DMA_CH0_init();
    SYSCFG_DL_DMA_CH1_init();
    SYSCFG_DL_DMA_CH2_init();
    SYSCFG_DL_DMA_CH3_init();
}


