/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
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
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X
#define CONFIG_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)



#define CPUCLK_FREQ                                                     80000000



/* Defines for PWM_MOTOR_1 */
#define PWM_MOTOR_1_INST                                                   TIMA1
#define PWM_MOTOR_1_INST_IRQHandler                             TIMA1_IRQHandler
#define PWM_MOTOR_1_INST_INT_IRQN                               (TIMA1_INT_IRQn)
#define PWM_MOTOR_1_INST_CLK_FREQ                                       40000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_MOTOR_1_C0_PORT                                           GPIOB
#define GPIO_PWM_MOTOR_1_C0_PIN                                    DL_GPIO_PIN_4
#define GPIO_PWM_MOTOR_1_C0_IOMUX                                (IOMUX_PINCM17)
#define GPIO_PWM_MOTOR_1_C0_IOMUX_FUNC               IOMUX_PINCM17_PF_TIMA1_CCP0
#define GPIO_PWM_MOTOR_1_C0_IDX                              DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_MOTOR_1_C1_PORT                                           GPIOB
#define GPIO_PWM_MOTOR_1_C1_PIN                                    DL_GPIO_PIN_1
#define GPIO_PWM_MOTOR_1_C1_IOMUX                                (IOMUX_PINCM13)
#define GPIO_PWM_MOTOR_1_C1_IOMUX_FUNC               IOMUX_PINCM13_PF_TIMA1_CCP1
#define GPIO_PWM_MOTOR_1_C1_IDX                              DL_TIMER_CC_1_INDEX



/* Defines for ENCODER1A */
#define ENCODER1A_INST                                                   (TIMA0)
#define ENCODER1A_INST_IRQHandler                               TIMA0_IRQHandler
#define ENCODER1A_INST_INT_IRQN                                 (TIMA0_INT_IRQn)
#define ENCODER1A_INST_LOAD_VALUE                                       (39999U)
/* GPIO defines for channel 0 */
#define GPIO_ENCODER1A_C0_PORT                                             GPIOA
#define GPIO_ENCODER1A_C0_PIN                                      DL_GPIO_PIN_8
#define GPIO_ENCODER1A_C0_IOMUX                                  (IOMUX_PINCM19)
#define GPIO_ENCODER1A_C0_IOMUX_FUNC                 IOMUX_PINCM19_PF_TIMA0_CCP0

/* Defines for ENCODER2A */
#define ENCODER2A_INST                                                   (TIMG7)
#define ENCODER2A_INST_IRQHandler                               TIMG7_IRQHandler
#define ENCODER2A_INST_INT_IRQN                                 (TIMG7_INT_IRQn)
#define ENCODER2A_INST_LOAD_VALUE                                       (39999U)
/* GPIO defines for channel 0 */
#define GPIO_ENCODER2A_C0_PORT                                             GPIOB
#define GPIO_ENCODER2A_C0_PIN                                     DL_GPIO_PIN_15
#define GPIO_ENCODER2A_C0_IOMUX                                  (IOMUX_PINCM32)
#define GPIO_ENCODER2A_C0_IOMUX_FUNC                 IOMUX_PINCM32_PF_TIMG7_CCP0





/* Defines for CLOCK_ZYD */
#define CLOCK_ZYD_INST                                                   (TIMG6)
#define CLOCK_ZYD_INST_IRQHandler                               TIMG6_IRQHandler
#define CLOCK_ZYD_INST_INT_IRQN                                 (TIMG6_INT_IRQn)
#define CLOCK_ZYD_INST_LOAD_VALUE                                        (3999U)
/* Defines for CLOCK_PTZ */
#define CLOCK_PTZ_INST                                                   (TIMG0)
#define CLOCK_PTZ_INST_IRQHandler                               TIMG0_IRQHandler
#define CLOCK_PTZ_INST_INT_IRQN                                 (TIMG0_INT_IRQn)
#define CLOCK_PTZ_INST_LOAD_VALUE                                       (39999U)




/* Defines for I2C_OLED */
#define I2C_OLED_INST                                                       I2C1
#define I2C_OLED_INST_IRQHandler                                 I2C1_IRQHandler
#define I2C_OLED_INST_INT_IRQN                                     I2C1_INT_IRQn
#define I2C_OLED_BUS_SPEED_HZ                                             400000
#define GPIO_I2C_OLED_SDA_PORT                                             GPIOB
#define GPIO_I2C_OLED_SDA_PIN                                      DL_GPIO_PIN_3
#define GPIO_I2C_OLED_IOMUX_SDA                                  (IOMUX_PINCM16)
#define GPIO_I2C_OLED_IOMUX_SDA_FUNC                   IOMUX_PINCM16_PF_I2C1_SDA
#define GPIO_I2C_OLED_SCL_PORT                                             GPIOB
#define GPIO_I2C_OLED_SCL_PIN                                      DL_GPIO_PIN_2
#define GPIO_I2C_OLED_IOMUX_SCL                                  (IOMUX_PINCM15)
#define GPIO_I2C_OLED_IOMUX_SCL_FUNC                   IOMUX_PINCM15_PF_I2C1_SCL

/* Defines for I2C_LSM6DSV16X */
#define I2C_LSM6DSV16X_INST                                                 I2C0
#define I2C_LSM6DSV16X_INST_IRQHandler                           I2C0_IRQHandler
#define I2C_LSM6DSV16X_INST_INT_IRQN                               I2C0_INT_IRQn
#define I2C_LSM6DSV16X_BUS_SPEED_HZ                                       100000
#define GPIO_I2C_LSM6DSV16X_SDA_PORT                                       GPIOA
#define GPIO_I2C_LSM6DSV16X_SDA_PIN                               DL_GPIO_PIN_28
#define GPIO_I2C_LSM6DSV16X_IOMUX_SDA                             (IOMUX_PINCM3)
#define GPIO_I2C_LSM6DSV16X_IOMUX_SDA_FUNC                IOMUX_PINCM3_PF_I2C0_SDA
#define GPIO_I2C_LSM6DSV16X_SCL_PORT                                       GPIOA
#define GPIO_I2C_LSM6DSV16X_SCL_PIN                               DL_GPIO_PIN_31
#define GPIO_I2C_LSM6DSV16X_IOMUX_SCL                             (IOMUX_PINCM6)
#define GPIO_I2C_LSM6DSV16X_IOMUX_SCL_FUNC                IOMUX_PINCM6_PF_I2C0_SCL


/* Defines for UART_K230 */
#define UART_K230_INST                                                     UART0
#define UART_K230_INST_FREQUENCY                                        40000000
#define UART_K230_INST_IRQHandler                               UART0_IRQHandler
#define UART_K230_INST_INT_IRQN                                   UART0_INT_IRQn
#define GPIO_UART_K230_RX_PORT                                             GPIOA
#define GPIO_UART_K230_TX_PORT                                             GPIOA
#define GPIO_UART_K230_RX_PIN                                     DL_GPIO_PIN_11
#define GPIO_UART_K230_TX_PIN                                     DL_GPIO_PIN_10
#define GPIO_UART_K230_IOMUX_RX                                  (IOMUX_PINCM22)
#define GPIO_UART_K230_IOMUX_TX                                  (IOMUX_PINCM21)
#define GPIO_UART_K230_IOMUX_RX_FUNC                   IOMUX_PINCM22_PF_UART0_RX
#define GPIO_UART_K230_IOMUX_TX_FUNC                   IOMUX_PINCM21_PF_UART0_TX
#define UART_K230_BAUD_RATE                                             (115200)
#define UART_K230_IBRD_40_MHZ_115200_BAUD                                   (21)
#define UART_K230_FBRD_40_MHZ_115200_BAUD                                   (45)
/* Defines for UART_FORWORD */
#define UART_FORWORD_INST                                                  UART3
#define UART_FORWORD_INST_FREQUENCY                                     80000000
#define UART_FORWORD_INST_IRQHandler                            UART3_IRQHandler
#define UART_FORWORD_INST_INT_IRQN                                UART3_INT_IRQn
#define GPIO_UART_FORWORD_RX_PORT                                          GPIOB
#define GPIO_UART_FORWORD_TX_PORT                                          GPIOA
#define GPIO_UART_FORWORD_RX_PIN                                  DL_GPIO_PIN_13
#define GPIO_UART_FORWORD_TX_PIN                                  DL_GPIO_PIN_26
#define GPIO_UART_FORWORD_IOMUX_RX                               (IOMUX_PINCM30)
#define GPIO_UART_FORWORD_IOMUX_TX                               (IOMUX_PINCM59)
#define GPIO_UART_FORWORD_IOMUX_RX_FUNC                IOMUX_PINCM30_PF_UART3_RX
#define GPIO_UART_FORWORD_IOMUX_TX_FUNC                IOMUX_PINCM59_PF_UART3_TX
#define UART_FORWORD_BAUD_RATE                                          (115200)
#define UART_FORWORD_IBRD_80_MHZ_115200_BAUD                                (43)
#define UART_FORWORD_FBRD_80_MHZ_115200_BAUD                                (26)
/* Defines for UART_BACKWARD */
#define UART_BACKWARD_INST                                                 UART1
#define UART_BACKWARD_INST_FREQUENCY                                    40000000
#define UART_BACKWARD_INST_IRQHandler                           UART1_IRQHandler
#define UART_BACKWARD_INST_INT_IRQN                               UART1_INT_IRQn
#define GPIO_UART_BACKWARD_RX_PORT                                         GPIOA
#define GPIO_UART_BACKWARD_TX_PORT                                         GPIOA
#define GPIO_UART_BACKWARD_RX_PIN                                  DL_GPIO_PIN_9
#define GPIO_UART_BACKWARD_TX_PIN                                 DL_GPIO_PIN_17
#define GPIO_UART_BACKWARD_IOMUX_RX                              (IOMUX_PINCM20)
#define GPIO_UART_BACKWARD_IOMUX_TX                              (IOMUX_PINCM39)
#define GPIO_UART_BACKWARD_IOMUX_RX_FUNC               IOMUX_PINCM20_PF_UART1_RX
#define GPIO_UART_BACKWARD_IOMUX_TX_FUNC               IOMUX_PINCM39_PF_UART1_TX
#define UART_BACKWARD_BAUD_RATE                                         (115200)
#define UART_BACKWARD_IBRD_40_MHZ_115200_BAUD                               (21)
#define UART_BACKWARD_FBRD_40_MHZ_115200_BAUD                               (45)





/* Defines for DMA_CH0 */
#define DMA_CH0_CHAN_ID                                                      (0)
#define UART_K230_INST_DMA_TRIGGER                           (DMA_UART0_RX_TRIG)
/* Defines for DMA_CH1 */
#define DMA_CH1_CHAN_ID                                                      (1)
#define UART_FORWORD_INST_DMA_TRIGGER_0                      (DMA_UART3_RX_TRIG)
/* Defines for DMA_CH2 */
#define DMA_CH2_CHAN_ID                                                      (2)
#define UART_FORWORD_INST_DMA_TRIGGER_1                      (DMA_UART3_TX_TRIG)
/* Defines for DMA_CH3 */
#define DMA_CH3_CHAN_ID                                                      (3)
#define UART_BACKWARD_INST_DMA_TRIGGER                       (DMA_UART1_TX_TRIG)


/* Port definition for Pin Group GPIO_LSM6DSV16X */
#define GPIO_LSM6DSV16X_PORT                                             (GPIOB)

/* Defines for PIN_LSM6DSV16X_INT: GPIOB.12 with pinCMx 29 on package pin 64 */
// pins affected by this interrupt request:["PIN_LSM6DSV16X_INT"]
#define GPIO_LSM6DSV16X_INT_IRQN                                (GPIOB_INT_IRQn)
#define GPIO_LSM6DSV16X_INT_IIDX                (DL_INTERRUPT_GROUP1_IIDX_GPIOB)
#define GPIO_LSM6DSV16X_PIN_LSM6DSV16X_INT_IIDX             (DL_GPIO_IIDX_DIO12)
#define GPIO_LSM6DSV16X_PIN_LSM6DSV16X_INT_PIN                  (DL_GPIO_PIN_12)
#define GPIO_LSM6DSV16X_PIN_LSM6DSV16X_INT_IOMUX                 (IOMUX_PINCM29)
/* Defines for PIN_C3: GPIOB.10 with pinCMx 27 on package pin 62 */
#define GPIO_KEY_PIN_C3_PORT                                             (GPIOB)
#define GPIO_KEY_PIN_C3_PIN                                     (DL_GPIO_PIN_10)
#define GPIO_KEY_PIN_C3_IOMUX                                    (IOMUX_PINCM27)
/* Defines for PIN_C4: GPIOA.14 with pinCMx 36 on package pin 7 */
#define GPIO_KEY_PIN_C4_PORT                                             (GPIOA)
#define GPIO_KEY_PIN_C4_PIN                                     (DL_GPIO_PIN_14)
#define GPIO_KEY_PIN_C4_IOMUX                                    (IOMUX_PINCM36)
/* Defines for PIN_R1: GPIOB.22 with pinCMx 50 on package pin 21 */
#define GPIO_KEY_PIN_R1_PORT                                             (GPIOB)
#define GPIO_KEY_PIN_R1_PIN                                     (DL_GPIO_PIN_22)
#define GPIO_KEY_PIN_R1_IOMUX                                    (IOMUX_PINCM50)
/* Defines for PIN_R2: GPIOB.21 with pinCMx 49 on package pin 20 */
#define GPIO_KEY_PIN_R2_PORT                                             (GPIOB)
#define GPIO_KEY_PIN_R2_PIN                                     (DL_GPIO_PIN_21)
#define GPIO_KEY_PIN_R2_IOMUX                                    (IOMUX_PINCM49)
/* Defines for PIN_R3: GPIOB.26 with pinCMx 57 on package pin 28 */
#define GPIO_KEY_PIN_R3_PORT                                             (GPIOB)
#define GPIO_KEY_PIN_R3_PIN                                     (DL_GPIO_PIN_26)
#define GPIO_KEY_PIN_R3_IOMUX                                    (IOMUX_PINCM57)
/* Defines for PIN_R4: GPIOB.23 with pinCMx 51 on package pin 22 */
#define GPIO_KEY_PIN_R4_PORT                                             (GPIOB)
#define GPIO_KEY_PIN_R4_PIN                                     (DL_GPIO_PIN_23)
#define GPIO_KEY_PIN_R4_IOMUX                                    (IOMUX_PINCM51)
/* Defines for PIN_C1: GPIOB.11 with pinCMx 28 on package pin 63 */
#define GPIO_KEY_PIN_C1_PORT                                             (GPIOB)
#define GPIO_KEY_PIN_C1_PIN                                     (DL_GPIO_PIN_11)
#define GPIO_KEY_PIN_C1_IOMUX                                    (IOMUX_PINCM28)
/* Defines for PIN_C2: GPIOB.5 with pinCMx 18 on package pin 53 */
#define GPIO_KEY_PIN_C2_PORT                                             (GPIOB)
#define GPIO_KEY_PIN_C2_PIN                                      (DL_GPIO_PIN_5)
#define GPIO_KEY_PIN_C2_IOMUX                                    (IOMUX_PINCM18)
/* Defines for PIN_MOTOR_AIN1: GPIOB.9 with pinCMx 26 on package pin 61 */
#define GPIO_MOTOR_PIN_MOTOR_AIN1_PORT                                   (GPIOB)
#define GPIO_MOTOR_PIN_MOTOR_AIN1_PIN                            (DL_GPIO_PIN_9)
#define GPIO_MOTOR_PIN_MOTOR_AIN1_IOMUX                          (IOMUX_PINCM26)
/* Defines for PIN_MOTOR_AIN2: GPIOA.27 with pinCMx 60 on package pin 31 */
#define GPIO_MOTOR_PIN_MOTOR_AIN2_PORT                                   (GPIOA)
#define GPIO_MOTOR_PIN_MOTOR_AIN2_PIN                           (DL_GPIO_PIN_27)
#define GPIO_MOTOR_PIN_MOTOR_AIN2_IOMUX                          (IOMUX_PINCM60)
/* Defines for PIN_MOTOR_BIN1: GPIOA.12 with pinCMx 34 on package pin 5 */
#define GPIO_MOTOR_PIN_MOTOR_BIN1_PORT                                   (GPIOA)
#define GPIO_MOTOR_PIN_MOTOR_BIN1_PIN                           (DL_GPIO_PIN_12)
#define GPIO_MOTOR_PIN_MOTOR_BIN1_IOMUX                          (IOMUX_PINCM34)
/* Defines for PIN_MOTOR_BIN2: GPIOA.13 with pinCMx 35 on package pin 6 */
#define GPIO_MOTOR_PIN_MOTOR_BIN2_PORT                                   (GPIOA)
#define GPIO_MOTOR_PIN_MOTOR_BIN2_PIN                           (DL_GPIO_PIN_13)
#define GPIO_MOTOR_PIN_MOTOR_BIN2_IOMUX                          (IOMUX_PINCM35)
/* Defines for PIN_ENCODER1B: GPIOA.25 with pinCMx 55 on package pin 26 */
#define GPIO_ENCODER_PIN_ENCODER1B_PORT                                  (GPIOA)
#define GPIO_ENCODER_PIN_ENCODER1B_PIN                          (DL_GPIO_PIN_25)
#define GPIO_ENCODER_PIN_ENCODER1B_IOMUX                         (IOMUX_PINCM55)
/* Defines for PIN_ENCODER2B: GPIOB.6 with pinCMx 23 on package pin 58 */
#define GPIO_ENCODER_PIN_ENCODER2B_PORT                                  (GPIOB)
#define GPIO_ENCODER_PIN_ENCODER2B_PIN                           (DL_GPIO_PIN_6)
#define GPIO_ENCODER_PIN_ENCODER2B_IOMUX                         (IOMUX_PINCM23)

/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_PWM_MOTOR_1_init(void);
void SYSCFG_DL_ENCODER1A_init(void);
void SYSCFG_DL_ENCODER2A_init(void);
void SYSCFG_DL_CLOCK_ZYD_init(void);
void SYSCFG_DL_CLOCK_PTZ_init(void);
void SYSCFG_DL_I2C_OLED_init(void);
void SYSCFG_DL_I2C_LSM6DSV16X_init(void);
void SYSCFG_DL_UART_K230_init(void);
void SYSCFG_DL_UART_FORWORD_init(void);
void SYSCFG_DL_UART_BACKWARD_init(void);
void SYSCFG_DL_DMA_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
