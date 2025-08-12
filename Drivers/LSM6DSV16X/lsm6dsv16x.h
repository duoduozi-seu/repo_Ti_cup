/*
 * SysConfig Configuration Steps:
 *   I2C:
 *     1. Add an I2C module.
 *     2. Name it as "I2C_LSM6DSV16X".
 *     3. Check the box "Enable Controller Mode".
 *     4. Set Standard Bus Speed to "Fast Mode Plus (1MHz)". (optional)
 *     5. Set the pins according to your needs.
 *   GPIO:
 *     1. Add a GPIO module.
 *     2. Name the group as "GPIO_LSM6DSV16X".
 *     3. Name the pin as "PIN_LSM6DSV16X_INT".
 *     4. Set Direction to "Input".
 *     5. Check the box "Enable Interrupts".
 *     6. Set "Interrupt Priority" to "Level 3 - Lowest".
 *     7. Set "Trigger Polarity" to "Trigger on Rising Edge".
 *     8. Set the pin according to your needs.
 */

#ifndef _LSM6DSV16X_H_
#define _LSM6DSV16X_H_
#include <stdbool.h>
extern short gyro[3], accel[3];
extern float pitch, roll, yaw;
extern float prev_yaw;
extern float target_yaw;

void LSM6DSV16X_Init(void);
void Read_LSM6DSV16X(void);
void init_yaw_detector(float start_yaw);
bool check_yaw_change(float current_yaw, float threshold_deg);
#endif  /* #ifndef _LSM6DSV16X_H_ */