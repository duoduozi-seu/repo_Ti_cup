/*
 * Copyright (c) 2021, Texas Instruments Incorporated
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

#include "ti_msp_dl_config.h"
#include "main.h"
#include <string.h>
#include "uart_handler.h"
#include "oled_hardware_i2c.h"
#include <math.h>
#include "ultrasonic_capture.h"
#include "HW_key.h"

void stop()
{
    set_left_speed(0);
    set_right_speed(0);
    uint32_t tick = tick_zydtime;
    while(tick_zydtime - tick <= 200)
    {
        update_motor_speed();
    }
    left_motor.pwm_duty = 0;
    right_motor.pwm_duty = 0;
    set_motor_duty(&left_motor);
    set_motor_duty(&right_motor);
}

void Perform1()
{
    /* 重启DMA */
    uart_init();
    /* 输入圈数 */
    int circle_num = InputFloat(6, 0, 0);

    /* 全局变量初始化 */
    perfmode = 1;
    is_stage_2_detected = false;
    big_turn_counter = 0;
    big_turn_counter_stop_target = circle_num * 4;
    big_turn = false;

    OLED_Clear();
    OLED_ShowString(0, 7, (uint8_t *)"Perf 1", 8);

    /* 行驶状态初始化 */
    car_stage = 'm';

    /* 发送巡线启动指令 */
    char chr[17] = "<pattern_1_1>\n";
    for(int i = 0;i < 15; i++)
    {
        uart0_send_char(UART_K230_INST, chr[i]);
    }

    //delay_ms(2000);

    while(car_stage != 's'){
        go_on_line(0.30);
        update_motor_speed();
    }

    /* 停车 */
    stop();

    OLED_ShowString(0, 6, (uint8_t *)"STOP!", 16);
    delay_ms(2000);

    return;
}

void Perform2()
{
    /* 重启DMA */
    uart_init();
    
    /* 全局变量初始化 */
    int circle_num = 1;
    big_turn_counter = 0;
    big_turn_counter_stop_target = circle_num * 4;
    big_turn = false;



    OLED_Clear();
    OLED_ShowString(0, 7, (uint8_t *)"Perf 2", 8);
    

    /* 发送打靶启动指令 */
    char chr[18] = "<pattern_2_0>";
    for(int i = 0;i < 15; i++)
    {
        uart0_send_char(UART_BACKWARD_INST, chr[i]);
    }

    return;
}

void Perform3()
{
    /* 重启DMA */
    uart_init();
    float yaw_init = yaw;
    /* 全局变量初始化 */
    int circle_num = 1;
    big_turn_counter = 0;
    big_turn_counter_stop_target = circle_num * 4;
    big_turn = false;

    OLED_ShowString(0, 0, (uint8_t *)"Press to act 3", 8);
    KEY_SCAN();
    float offset = - calculate_yaw_error(yaw_init, yaw);
    char chr[20];
    sprintf(chr, "<pattern_3_%.1f>\0", offset);

    OLED_Clear();
    OLED_ShowString(0, 7, (uint8_t *)"Perf 3", 8);
    

    /* 发送打靶启动指令 */
    
    for(int i = 0; chr[i] != 0; i++)
    {
        uart0_send_char(UART_BACKWARD_INST, chr[i]);
    }

    return;
}

volatile int32_t encoder_recoder = 0;
volatile int in_line_x = 0;
volatile bool perf_4_flag = false;
volatile bool perf_4_first_time = true;
volatile int circle_num_curr = 0;
void Perform4()
{
    float oldKP = KP;
    float oldKD = KD;
    float old_turn_kp = kp_for_turn;

    KP = KP_p4;
    KD = KD_p4;
    kp_for_turn = kp_for_turn_p4;


    perf_4_flag = true;
    /* 重启DMA */
    uart_init();

    OLED_Clear();
    OLED_ShowString(0, 7, (uint8_t *)"Perf 4", 8);

    /* 全局变量初始化 */
    perfmode = 2;
    is_stage_2_detected = false;
    int circle_num = 1;//圈数
    big_turn_counter = 0;
    big_turn_counter_stop_target = circle_num * 4 + 1;
    big_turn = true;
    
    //里程计初始化
    encoder_recoder = left_motor.encoder.total_count;
    in_line_x = 4;
    prev_line_index = 4;
    loop_count = 0;
    perf_4_first_time = true;
    circle_num_curr = 0;
    /* 行驶状态初始化 */
    car_stage = 'm';


// ==============test==============

    // char test[16];
    // float yaw1 = 0, pitch1 = 0;
    // calculate_angles_for_perf4(-50, 50, 17.7, 4, 
    //                     0, 100, 50,
    //                     &yaw1, &pitch1);
    // sprintf(test,"yaw:%.2f, pitch:%.2f", yaw1, pitch1);
    // OLED_ShowString(6, 1, (uint8_t *)test, 8);
    // char wait = KEY_SCAN();
    // while(1)
    // {
    //     float x, y;
    //     x = InputFloat(6, 0, 0);
    //     y = InputFloat(6, 0, 0);
    //         calculate_angles_for_perf4(x, y, 17.7, 3, 
    //                     0, 100, 20,
    //                     &yaw1, &pitch1);
    //     sprintf(test,"y:%.2f, p:%.2f", yaw1, pitch1);
    //     OLED_ShowString(6, 1, (uint8_t *)test, 8);
    //     wait = KEY_SCAN();
    // }

//==============test end==============
    /* 发送打靶启动指令 */
    char chr[18] = "<pattern_4_0>\n";
    for(int i = 0;i < 15; i++)
    {
        uart0_send_char(UART_BACKWARD_INST, chr[i]);
    }

    /* 发送巡线启动指令 */
    char str[14] = "<pattern_1>\n"; //复用 pattern_1 视觉巡线
    for(int i = 0;i < 13; i++)
    {
        uart0_send_char(UART_K230_INST, str[i]);
    }

    //delay_ms(2500);

    while(car_stage != 's'){
        go_on_line(0.25);
        update_motor_speed();
    }

    /* 停车 */
    stop();

    OLED_ShowString(0, 6, (uint8_t *)"STOP!", 16);
    delay_ms(2000);
    perf_4_flag = false;
    KP = oldKP;
    kp_for_turn = old_turn_kp;
    KD = oldKD;
    return;
}

void Perform5()
{
    /* 重启DMA */
    uart_init();

    OLED_Clear();
    OLED_ShowString(0, 7, (uint8_t *)"Perf 5", 8);

    /* 全局变量初始化 */
    perfmode = 2;
    is_stage_2_detected = false;
    int circle_num = 2; //圈数
    big_turn_counter = 0;
    big_turn_counter_stop_target = circle_num * 4;
    big_turn = true;

    /* 行驶状态初始化 */
    car_stage = 'm';

    /* 发送打靶启动指令 */
    char chr[18] = "<pattern_5_0>\n";
    for(int i = 0;i < 15; i++)
    {
        uart0_send_char(UART_BACKWARD_INST, chr[i]);
    }

    /* 发送巡线启动指令 */
    char str[17] = "<pattern_1_1>\n"; //复用 pattern_1 视觉巡线
    for(int i = 0;i < 15; i++)
    {
        uart0_send_char(UART_K230_INST, str[i]);
    }

    //delay_ms(2500);

    while(car_stage != 's'){
        go_on_line(0.3);
        update_motor_speed();
    }

    /* 停车 */
    stop();

    OLED_ShowString(0, 6, (uint8_t *)"STOP!", 16);
    delay_ms(2000);
    return;
}

void Perform6()
{
    /* 重启DMA */
    uart_init();

    OLED_Clear();
    OLED_ShowString(0, 7, (uint8_t *)"Perf 6", 8);

    /* 全局变量初始化 */
    perfmode = 2;
    is_stage_2_detected = false;
    int circle_num = 1;//圈数
    big_turn_counter = 0;
    big_turn_counter_stop_target = circle_num * 4;
    big_turn = true;

    /* 行驶状态初始化 */
    car_stage = 'm';

    /* 发送打靶启动指令 */
    char chr[18] = "<pattern_6_0>\n";
    for(int i = 0;i < 15; i++)
    {
        uart0_send_char(UART_BACKWARD_INST, chr[i]);
    }

    /* 发送巡线启动指令 */
    char str[14] = "<pattern_1_1>\n"; //复用 pattern_1 视觉巡线
    for(int i = 0;i < 15; i++)
    {
        uart0_send_char(UART_K230_INST, str[i]);
    }

    //delay_ms(2500);

    while(car_stage != 's'){
        go_on_line(0.3);
        update_motor_speed();
    }

    /* 停车 */
    stop();

    OLED_ShowString(0, 6, (uint8_t *)"STOP!", 16);
    delay_ms(2000);
    return;
}

int main(void)
{
    SYSCFG_DL_init();
    SysTick_Init();

    // MPU6050_Init();
    OLED_Init();
    // Ultrasonic_Init();
    // BNO08X_Init();
    // WIT_Init();
    // VL53L0X_Init();
    LSM6DSV16X_Init();
    // IMU660RB_Init();
    Interrupt_Init();
    /* Don't remove this! */
    
    uart_init(); // 初始化dma及串口K230
    motors_init();
    
    char str1[3];
    char trans[16];
    sprintf(trans, "initing...");
    uart0_send_string(UART_BACKWARD_INST, trans);
    /* 陀螺仪初始化 */
    // char str[16];
    // uint32_t tick = tick_zydtime;
    // while(tick_zydtime - tick <= 1000)
    // {
    //     sprintf(str, "yaw %.2f", yaw);
    //     OLED_ShowString(0, 6, (uint8_t *)str, 16);
    //     OLED_ShowString(0, 5, (uint8_t *)"initing", 8);        
    // }
    //OLED_Clear();
    // while(1)
    // {
    //     //draw_two_sine_waves_step();
    //     sprintf(str, "error %.2f", line_error);
    //     OLED_ShowString(0, 5, (uint8_t *)str, 8);
    // }
    OLED_Clear();
    while(1){
        car_stage = 'i';
        OLED_Show_Init();
        char key = KEY_SCAN();
        switch (key) {
            case 1:
                Perform1();
                break;
            case 2:
                Perform2();
                break;
            case 3:
                Perform3();
                break;
            case 4:
                Perform4();
                break;
            case 5:
                Perform5();
                break;
            case 6:
                Perform6();
                break;
            case 7:
                contrl_ptz();
                break;
            case 11:
                adj_line_pid();
                break;
            case 12:
                adj_turn_pid();
                break;
            case 13:
                adj_perf4_pid();
                break;
            case 16:
                start_cam_k230();
                break;
            default:
                break;
        }
    }

    //set_left_speed(0.2);
    //set_right_speed(0.2);
    // while (1) {
    //     sprintf(str, "yaw %.2f", yaw);
    //     OLED_ShowString(0, 6, (uint8_t *)str, 8);
    //     sprintf(str, "curspd %.2f", right_motor.encoder.current_speed);
    //     OLED_ShowString(0, 2, (uint8_t *)str, 8);
    //     sprintf(str, "totle %d", right_motor.encoder.total_count);
    //     OLED_ShowString(0, 4, (uint8_t *)str, 8);
    //     update_motor_speed();
    // }

}

float InputFloat(int max_digits, int row, int col) {
    char input_buf[16];  // 输入缓冲区（包含小数点和结束符）
    int buf_index = 0;
    int has_decimal = 0; // 是否已输入小数点
    int KeyConfirm;
    
    // 清空输入行区域
    OLED_Clear();
    OLED_ShowString(col, row, (uint8_t *)"Input:", 16);
    
    // 显示初始输入位置
    int display_x = col + 6 * 8;
    OLED_ShowChar(display_x, row, '_', 16);  // 显示输入光标

    // 输入循环
    while(1) {
        KeyConfirm = KEY_SCAN();
        
        if(KeyConfirm == 16) {  // 确认键
            break;
        }
        else if(KeyConfirm == 15) {  // 退出键
            OLED_Clear();
            OLED_ShowString(col, row, (uint8_t *)"Exiting...", 16);
            delay_ms(500);
            OLED_Clear();
            return 0.0;  // 返回退出代码
        }
        else if(KeyConfirm == 14) {  // 小数点
            if(!has_decimal && buf_index < max_digits - 1) {
                input_buf[buf_index++] = '.';
                has_decimal = 1;
                
                // 更新显示
                OLED_ShowChar(display_x, row, '.', 16);
                display_x += 8;
                OLED_ShowChar(display_x, row, '_', 16);  // 更新光标
            }
        }
        // 处理数字键：0-9和13(作为0)
        else if((KeyConfirm >= 0 && KeyConfirm <= 9) || KeyConfirm == 13) {
            if(buf_index < max_digits - 1) {
                // 13键作为0处理
                char digit = (KeyConfirm == 13) ? '0' : ('0' + KeyConfirm);
                
                input_buf[buf_index++] = digit;
                
                // 更新显示
                OLED_ShowChar(display_x, row, digit, 16);
                display_x += 8;
                OLED_ShowChar(display_x, row, '_', 16);  // 更新光标
            }
        }
    }
    
    // 结束输入
    input_buf[buf_index] = '\0';  // 添加字符串结束符
    OLED_ShowChar(display_x, row, ' ', 16);  // 清除光标
    
    // 检查有效输入
    if(buf_index == 0) {
        return 0.0f;  // 无有效输入
    }
    
    // 转换字符串为浮点数
    float result = 0.0f;
    float decimal_factor = 1.0f;
    int in_decimal = 0;  // 是否在小数部分
    
    for(int i = 0; i < buf_index; i++) {
        if(input_buf[i] == '.') {
            in_decimal = 1;
            decimal_factor = 0.1f;
            continue;
        }
        
        if(!in_decimal) {
            result = result * 10.0f + (input_buf[i] - '0');
        } else {
            result += (input_buf[i] - '0') * decimal_factor;
            decimal_factor *= 0.1f;
        }
    }
    
    OLED_Clear();
    return result;
}

void OLED_Show_Init()
{
    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t *)"1-6:Perf 1-6", 8);
    OLED_ShowString(0, 2, (uint8_t *)"7:contrl_ptz", 8);
    OLED_ShowString(0, 3, (uint8_t *)"11:lin-pid 12:tur-pid", 8);
    OLED_ShowString(0, 4, (uint8_t *)"13:adj_perf4_pid", 8);
    OLED_ShowString(0, 7, (uint8_t *)"16:startcam", 8);
}

void adj_line_pid()
{
    OLED_Clear();
    char str[20];
    sprintf(str, "kp %.4f kd %.4f", KP, KD);

    OLED_ShowString(0, 0, (uint8_t *)str, 8);

    char mode = 0;
    while(!(mode == 1 || mode == 2 || mode == 15))mode = KEY_SCAN();
    
    OLED_Clear();
    if(mode == 15)return;
    float input = InputFloat(8, 2, 0);
    if(mode == 1){KP = input;}
    else{KD = input;}
}

void adj_turn_pid()
{
    OLED_Clear();
    char str[20];
    sprintf(str, "kp %.6f", kp_for_turn);

    OLED_ShowString(0, 0, (uint8_t *)str, 16);


    char mode = 0;
    while(mode != 1 && mode != 15)mode = KEY_SCAN();
    OLED_Clear();
    if(mode == 15) return;
    kp_for_turn = InputFloat(8, 2, 0);
    return;
}
void test()
{        
    float yaw_now = yaw;
    float yaw_target = yaw + 90;
    if(yaw_target > 180){yaw_target -= 360;}
    turn_pid(0,         // 基础速度0.2m/s
             yaw_target,   // 目标角度
             20.0,         // 角度阈值设为3度
             0.002);       // 比例系数Kp=0.01
        
    /* 转弯结束恢复巡线 */
    set_left_speed(0);
    set_right_speed(0);
    uint32_t tick = tick_zydtime;
    while(tick_zydtime - tick <= 300){
        update_motor_speed();
    }
}

void start_cam_k230()
{
    /* 发送巡线启动指令 */
    char chr[17] = "<pattern_1_1>\n"; //复用 pattern_1 视觉巡线
    for(int i = 0;i < 15; i++)
    {
        uart0_send_char(UART_K230_INST, chr[i]);
    }
}
void test_turn_angle_send()
{
    OLED_Clear();
    float previous_yaw = yaw, init_yaw = yaw;
    float change_angle = 0;
    float change_total = 0;
    char test[10];
    uint32_t tick = tick_zydtime;
    while(tick_zydtime - tick <= 1500)
    {
        delay_ms(100);
        change_angle = calculate_yaw_error(yaw, previous_yaw);
        change_total = calculate_yaw_error(yaw,init_yaw);
        sprintf(test, "%.2f\0", change_total);
        OLED_ShowString(0, 0, (uint8_t *)test, 8);
        sprintf(test, "<%.2f_0>\0", change_angle);
        uart0_send_string(UART_BACKWARD_INST, test);
        previous_yaw = yaw;
    }
    return ;
}
void adj_perf4_pid()
{
    OLED_Clear();
    char str[20];
    OLED_ShowString(0, 0, (uint8_t *)"line_pid", 8);
    sprintf(str, "kp %.4f kd %.4f", KP_p4, KD_p4);

    OLED_ShowString(0, 2, (uint8_t *)str, 8);

    OLED_ShowString(0, 4, (uint8_t *)"turn_pid", 8);
    sprintf(str, "kp %.4f", kp_for_turn_p4);

    OLED_ShowString(0, 6, (uint8_t *)str, 8);
    char mode = 0;
    while(!(mode == 1 || mode == 2 || mode == 3 || mode == 15))mode = KEY_SCAN();
    
    OLED_Clear();
    if(mode == 15)return;
    float input = InputFloat(8, 2, 0);
    if(mode == 1){KP_p4 = input;}
    else if(mode == 2){KD_p4 = input;}
    else{kp_for_turn_p4 = input;}
}