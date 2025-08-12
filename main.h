#ifndef _MAIN_H_
#define _MAIN_H_

#include "clock.h"
#include "interrupt.h"

#include "mpu6050.h"
#include "oled_software_i2c.h"
#include "oled_hardware_i2c.h"
#include "oled_software_spi.h"
#include "oled_hardware_spi.h"
#include "ultrasonic_capture.h"
#include "ultrasonic_gpio.h"
#include "bno08x_uart_rvc.h"
#include "wit.h"
#include "vl53l0x.h"
#include "lsm6dsv16x.h"
#include "imu660rb.h"
#include "motor.h"
#include "PTZ_controller.h"
#include <string.h>
#include "uart_handler.h"
#include <math.h>
#include "HW_key.h"

extern volatile int circle_num_curr;
extern volatile int32_t encoder_recoder;
extern volatile int in_line_x;
extern volatile bool perf_4_flag;
extern volatile bool perf_4_first_time;
float InputFloat(int max_digits, int row, int col);
void OLED_Show_Init();
void stop();
void Perform1();
void Perform2();
void Perform3();
void Perform4();
void adj_line_pid();
void test();
void adj_turn_pid();
void start_cam_k230();
void test_turn_angle_send();
void adj_perf4_pid();
#endif  /* #ifndef _MAIN_H_ */
