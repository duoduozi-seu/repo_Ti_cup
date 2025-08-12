#pragma once
#include "ti_msp_dl_config.h"
#include "motor.h"
#include <math.h>
#include "oled_hardware_i2c.h"
#include "main.h"
float get_y_coordinate();
float get_x_coordinate();
void calculate_angles(float x, float y, float* theta_x, float* theta_y);
void move_to_position(float target_x, float target_y);
void draw_two_sine_waves_step();
void calculate_angles_for_perf4(float x, float y, float height, int car_direction, 
                      float target_x, float target_y, float target_height,
                      float *yaw, float *pitch) ;
double radians_to_degrees(double radians);
void get_target_yaw_pitch(float *yaw, float *pitch);
void contrl_ptz();
