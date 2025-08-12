#ifndef __HW_key_H__
#define __HW_key_H__


#include <string.h>
#include "ti_msp_dl_config.h"

char KEY_SCAN(void);
char KEY_ROW_SCAN(void);


uint8_t HAL_GPIO_ReadPin(GPIO_Regs *gpio, uint32_t pins);

#define KEY_CLO3_OUT_LOW   DL_GPIO_clearPins(GPIOB,DL_GPIO_PIN_21)
#define KEY_CLO2_OUT_LOW   DL_GPIO_clearPins(GPIOB,DL_GPIO_PIN_22)
#define KEY_CLO1_OUT_LOW   DL_GPIO_clearPins(GPIOB,DL_GPIO_PIN_26)
#define KEY_CLO0_OUT_LOW   DL_GPIO_clearPins(GPIOB,DL_GPIO_PIN_23)

#define KEY_CLO3_OUT_HIGH   DL_GPIO_setPins(GPIOB,DL_GPIO_PIN_21)
#define KEY_CLO2_OUT_HIGH   DL_GPIO_setPins(GPIOB,DL_GPIO_PIN_22)
#define KEY_CLO1_OUT_HIGH   DL_GPIO_setPins(GPIOB,DL_GPIO_PIN_26)
#define KEY_CLO0_OUT_HIGH   DL_GPIO_setPins(GPIOB,DL_GPIO_PIN_23)

#endif
