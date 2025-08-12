// ========================= 电机控制实现(motor.c) =========================
#include "motor.h"
#include "ti_msp_dl_config.h"
#include "uart_handler.h"
#include "ultrasonic_capture.h"
#include "oled_hardware_i2c.h"
#include "main.h"
// 全局电机控制器实例
motor_controller_t left_motor;
motor_controller_t right_motor;
volatile uint8_t speed_update_flag = 0;  // 速度更新标志
volatile bool big_turn = false;
volatile int big_turn_counter = 0;
volatile int big_turn_counter_stop_target = 0;
volatile int perfmode = 0;
// ====================== 系统初始化函数 ======================
void motors_init(void) {
    // 启动PWM定时器
    memset(left_motor.encoder.speed_records, 0, sizeof(left_motor.encoder.speed_records));
    memset(right_motor.encoder.speed_records, 0, sizeof(right_motor.encoder.speed_records));
    DL_TimerG_startCounter(PWM_MOTOR_1_INST);
    
    // 配置编码器中断

    NVIC_EnableIRQ(ENCODER2A_INST_INT_IRQN);
    NVIC_EnableIRQ(ENCODER1A_INST_INT_IRQN);
    NVIC_EnableIRQ(GPIO_LSM6DSV16X_INT_IRQN);
    NVIC_EnableIRQ(CLOCK_PTZ_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_K230_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_FORWORD_INST_INT_IRQN);
    // 配置速度更新定时器(100Hz)
    NVIC_EnableIRQ(CLOCK_ZYD_INST_INT_IRQN);
    DL_TimerG_startCounter(CLOCK_ZYD_INST);
    DL_TimerG_startCounter(CLOCK_PTZ_INST);
    // 初始化左电机参数
    left_motor.pwm_duty = 0;
    left_motor.target_speed = 0;
    left_motor.pwm_channel = GPIO_PWM_MOTOR_1_C0_IDX;
    left_motor.forward_port = GPIO_MOTOR_PIN_MOTOR_AIN1_PORT;
    left_motor.reverse_port = GPIO_MOTOR_PIN_MOTOR_AIN2_PORT;
    left_motor.forward_pin = GPIO_MOTOR_PIN_MOTOR_AIN1_PIN;
    left_motor.reverse_pin = GPIO_MOTOR_PIN_MOTOR_AIN2_PIN;
    
    // 初始化PID参数
    left_motor.speed_pid = (pid_controller_t){
        .kp = 0.0014655, .ki = 0.0005, .kd = 0.00001,
        .max_integral = 2000, .max_output = 1, .dead_zone = 0
    };
    
    // 初始化右电机参数
    right_motor.pwm_duty = 0;
    right_motor.target_speed = 0;
    right_motor.pwm_channel = GPIO_PWM_MOTOR_1_C1_IDX;
    right_motor.forward_port = GPIO_MOTOR_PIN_MOTOR_BIN1_PORT;
    right_motor.reverse_port = GPIO_MOTOR_PIN_MOTOR_BIN2_PORT;
    right_motor.forward_pin = GPIO_MOTOR_PIN_MOTOR_BIN1_PIN;
    right_motor.reverse_pin = GPIO_MOTOR_PIN_MOTOR_BIN2_PIN;
    
    // 初始化PID参数
    right_motor.speed_pid = (pid_controller_t){
        .kp = 0.0014655, .ki = 0.0005, .kd = 0.00001,
        .max_integral = 2000, .max_output = 1, .dead_zone = 0
    };
}

// ====================== 编码器中断处理 ======================
void ENCODER1A_INST_IRQHandler(void) {
    
    switch (DL_TimerA_getPendingInterrupt(ENCODER1A_INST)) {
        case DL_TIMERA_IIDX_CC0_DN:
            DL_TimerA_clearInterruptStatus(ENCODER1A_INST, DL_TIMERG_IIDX_CC0_DN);
            // 读取编码器方向引脚
            left_motor.encoder.direction = DL_GPIO_readPins(
                GPIO_ENCODER_PIN_ENCODER1B_PORT, 
                GPIO_ENCODER_PIN_ENCODER1B_PIN
            );
            
            // 更新编码器计数值
            left_motor.encoder.total_count += 
                (left_motor.encoder.direction) ? -1 : 1;
            break;
        default:
            break;
    }
}

void ENCODER2A_INST_IRQHandler(void) {
    
    switch (DL_TimerG_getPendingInterrupt(ENCODER2A_INST)) {
        case DL_TIMERG_IIDX_CC0_DN:
            // 读取编码器方向引脚
            right_motor.encoder.direction = DL_GPIO_readPins(
                GPIO_ENCODER_PIN_ENCODER2B_PORT, 
                GPIO_ENCODER_PIN_ENCODER2B_PIN
            );
            
            // 更新编码器计数值
            right_motor.encoder.total_count += 
                (right_motor.encoder.direction) ? 1 : -1;
            break;
        default:
            break;
    }
}

//见 interrupt.c
uint32_t tick_zydtime = 0; 
bool zyd_tick_25_flag = false;
bool zyd_tick_8_flag = false;
void CLOCK_PTZ_INST_IRQHandler(void)
{
    zyd_tick_8_flag = true;

    if(perf_4_flag && circle_num_curr >= 1 && (!is_turning))
    {
        float total_dist = (circle_num_curr - 1) * 100 + (left_motor.encoder.total_count-encoder_recoder) * 0.05869 ;
            
        char trans[16];
        sprintf(trans, "<%.2f>\0", total_dist);
        // OLED_ShowString(0, 2, (uint8_t *)trans, 8);
        backward_uart_send((uint8_t*)trans,strlen(trans));
        //uart0_send_string(UART_BACKWARD_INST, trans);
        /* turning_yaw turning_pitch*/
        
        
    }
}
// ====================== 速度计算中断(100Hz) ======================
void CLOCK_ZYD_INST_IRQHandler(void) {
    tick_zydtime++;
    switch (DL_TimerA_getPendingInterrupt(CLOCK_ZYD_INST)) {
        case DL_TIMER_IIDX_ZERO:
            DL_TimerA_clearInterruptStatus(CLOCK_ZYD_INST, DL_TIMER_IIDX_ZERO);
            
            // 计算左电机速度 (RPM = 脉冲变化量 * 60000ms / 每圈脉冲数 / 采样周期100ms)
            float delta = left_motor.encoder.total_count - left_motor.encoder.last_count;
            left_motor.encoder.current_speed = (delta * 6000) / PULSE_PER_CYCLE;
            left_motor.encoder.current_speed = speed_filter(
                left_motor.encoder.current_speed, 
                &left_motor.encoder
            );
            left_motor.encoder.last_count = left_motor.encoder.total_count;
            //speed_update_flag ++;
            
            // 计算右电机速度
            delta = right_motor.encoder.total_count - right_motor.encoder.last_count;
            right_motor.encoder.current_speed = (delta * 6000) / PULSE_PER_CYCLE;
            right_motor.encoder.current_speed = speed_filter(
                right_motor.encoder.current_speed, 
                &right_motor.encoder
            );
            right_motor.encoder.last_count = right_motor.encoder.total_count;
            break;
        default:
            break;
    }
    if(tick_zydtime % 5 == 0)
    {
        speed_update_flag = 1;

        //接收消息
        // 当接收到完整消息
        // if(rx_flag == 1) {
        //     // 启动下一次接收
        //     uart_rx_DMA_config(gRxBuffer, sizeof(gRxBuffer));
        // }
        if(fabsf(line_error - 0) < 0.0001f){reset_uart_reception();}
    }
    if(tick_zydtime % 25 == 0)
    {
        zyd_tick_25_flag = true;
    }
    // if(tick_zydtime % 15 == 0)
    // {
    //     zyd_tick_8_flag = true;
    //     if(perf_4_flag && circle_num_curr >= 1)
    //     {
    //         float total_dist = (circle_num_curr - 1) * 100 + (left_motor.encoder.total_count-encoder_recoder) * 0.05869 + 13;
            
    //         char trans[16];
    //         sprintf(trans, "<%.2f>\0", total_dist);
    //         OLED_ShowString(0, 2, (uint8_t *)trans, 8);
    //         uart0_send_string(UART_BACKWARD_INST, trans);
    //         /* turning_yaw turning_pitch*/
    //         zyd_tick_8_flag = false;
        
    //     }
    // }
    
}


// ====================== 速度滤波函数 ======================
float speed_filter(float new_speed, encoder_t *encoder) {
    float sum = 0.0f;
    
    // 滑动窗口更新
    for(uint8_t i = SPEED_RECORD_NUM - 1; i > 0; i--) {
        encoder->speed_records[i] = encoder->speed_records[i - 1];
        sum += encoder->speed_records[i];
    }
    
    // 添加新采样值
    encoder->speed_records[0] = new_speed;
    sum += new_speed;
    
    // 返回平均值
    return sum / SPEED_RECORD_NUM;
}

// ====================== PID控制算法 ======================
float calculate_pid(pid_controller_t *pid, float target, float actual) {
    // 计算误差
    pid->error = target - actual;
    
    // 死区处理
    if(fabs(pid->error) < pid->dead_zone) {
        pid->error = 0;
    }
    
    // 积分项计算与限幅
    pid->integral += pid->error;
    if(pid->integral > pid->max_integral) pid->integral = pid->max_integral;
    else if(pid->integral < -pid->max_integral) pid->integral = -pid->max_integral;
    
    // 微分项计算
    float derivative = pid->error - pid->last_error;
    
    // PID输出计算
    /////////////////////////////
    pid->output = (pid->kp * pid->error) 
                + (pid->ki * pid->integral) 
                + (pid->kd * derivative);
    // 输出限幅
    if(pid->output > pid->max_output) pid->output = pid->max_output;
    else if(pid->output < -pid->max_output) pid->output = -pid->max_output;
    
    // 保存当前误差
    pid->last_error = pid->error;
    
    return pid->output;
}


// ====================== 电机控制函数 ======================
void set_motor_duty(motor_controller_t *motor) {
    if (motor->pwm_duty >= 0) { // 正转
        DL_TimerA_setCaptureCompareValue(
            PWM_MOTOR_1_INST,
            motor->pwm_duty*4000,
            motor->pwm_channel
        );
        DL_GPIO_setPins(motor->forward_port, motor->forward_pin);
        DL_GPIO_clearPins(motor->reverse_port, motor->reverse_pin);
    } else { // 反转
        DL_TimerA_setCaptureCompareValue(
            PWM_MOTOR_1_INST,
            (-motor->pwm_duty)*4000,
            motor->pwm_channel
        );
        DL_GPIO_setPins(motor->reverse_port, motor->reverse_pin);
        DL_GPIO_clearPins(motor->forward_port, motor->forward_pin);
    }
}

void set_motor_speed(motor_controller_t *motor) {
    motor->pwm_duty = calculate_pid(
        &motor->speed_pid,
        motor->target_speed,
        motor->encoder.current_speed
    );
    
    
    set_motor_duty(motor);
}

void set_motor_position(motor_controller_t *motor) {
    // 将位置(cm)转换为脉冲数
    int32_t target_pulses = (int32_t)(
        (motor->target_position * PULSE_PER_CYCLE) / LINE_SPEED_C
    );
    
    // 位置环PID计算得到目标速度
    motor->target_speed = calculate_pid(
        &motor->pos_pid,
        target_pulses,
        motor->encoder.total_count
    );
    
    // 进入速度环控制
    set_motor_speed(motor);
}

void stop_motor(motor_controller_t *motor) {
    motor->target_speed = 0;
    set_motor_speed(motor);
    
    // 完全停止后重置控制状态
    if(motor->encoder.current_speed == 0) {
        motor->encoder.total_count = 0;
        motor->encoder.last_count = 0;
        motor->speed_pid.integral = 0;
    }
}
void set_left_speed(float speed_meter_per_s)
{
    float rpm = speed_meter_per_s * 60 * 100 / LINE_SPEED_C;
    left_motor.target_speed = rpm;
}
void set_right_speed(float speed_meter_per_s)
{
    float rpm = speed_meter_per_s * 60 * 100 / LINE_SPEED_C;
    right_motor.target_speed = rpm;
}
void update_motor_speed()
{
    if(speed_update_flag == 0)return;
    set_motor_speed(&left_motor);
    set_motor_speed(&right_motor);
    speed_update_flag = 0;
}

int tmpstop_flag = 0;

// PD控制器参数（根据实际调试调整）
float KP = 0.0021;    // 比例系数
float KD = 0.00021f;   // 微分系数
float KP_p4 = 0.00102;    // 比例系数
float KD_p4 = 0.00012f;   // 微分系数
#define MIN_SPEED 0.05f  // 防止负速度导致电机反转
//return 0 正常返回 return 1 检测到停止 return 2 暂停
float kp_for_turn = 0.0014;
float kp_for_turn_p4 = 0.0012;
// ============== 全局变量定义 ==============
 // 起始绝对角度135度
int prev_line_index = 4;   // 上一次的边索引
int loop_count = 0;        // 完成的圈数
float threshold = 20.0;
int go_on_line(float speed_m_per_s) {
    static float last_error = 0.0f;  // 保存上一次误差值
    char str[16];

        // ============== 新增：维护绝对角度 curr_ang ==============
        // 获取当前边索引
    // if(perf_4_flag && zyd_tick_25_flag)
    // {
    //     int curr_line_index = in_line_x;
        
    //     // 检查是否完成一圈（从边4到边1）
    //     if (prev_line_index == 4 && curr_line_index == 1) {
    //         loop_count++;
    //     }
        
    //     // 计算当前帧的yaw
    //     int32_t yaw_temp, curr_pitch;
    //     float x,y;
    //     if(perf_4_first_time)
    //     {
    //         x = -45;
    //         y = 50;
    //         calculate_angles_for_perf4(
    //             -45000, 
    //             50000, 
    //             17600,  // 17.6 * 1000
    //             in_line_x, 
    //             0, 
    //             100000, // 100.0 * 1000
    //             20000,  // 20.0 * 1000
    //             &yaw_temp, 
    //             &curr_pitch 
    //         );       
    //     }
    //     else
    //     {
    //         x = get_x_coordinate();
    //         y = get_y_coordinate();
    //         get_target_yaw_pitch_fixed(&yaw_temp, &curr_pitch);
    //         perf_4_first_time = false;
    //     }
        
    //     // 更新绝对角度（考虑圈数）
    //     curr_ang = yaw_temp + 360000 * loop_count;
    //     /* curr_ang curr_pitch*/

    //     // 更新prev_line_index为当前的curr_line_index
    //     prev_line_index = curr_line_index;
    //     char trans[16];
    //     sprintf(trans, "(%.2f,%.2f) in_line: %d, curr_yaw: %d, curr_pitch: %d \n", x, y, in_line_x, curr_ang, curr_pitch);
    //     uart0_send_string(UART_BACKWARD_INST, trans);

    //     zyd_tick_25_flag = false;
    // }
    // ============== 原有代码保持不变 ==============

    /* 检测到直角转弯 */
    if(big_turn == true)
    {
        big_turn_counter ++;

        sprintf(str, "in Turn %d",big_turn_counter);
        OLED_ShowString(0, 0, (uint8_t *)str, 8);
        
        /* 检查是否到达停车点 */
        if(big_turn_counter == big_turn_counter_stop_target && perfmode == 1)
        {
            car_stage = 's';
            //OLED_ShowString(0, 6, (uint8_t *)"<S>", 16);

            /* 计算目标角度 */
            float yaw_now = yaw;
            float yaw_target = yaw + 90;
            if(yaw_target > 180){yaw_target -= 360;}
            // 调用阻塞式转弯PID
            delay_ms(50);
            if(is_stage_2_detected){delay_ms(20);}
            turn_pid(0.1,         // 基础速度0.2m/s
                    yaw_target,   // 目标角度
                    threshold,         // 角度阈值设为3度
                    kp_for_turn);       // 比例系数Kp=0.01
            set_left_speed(0);
            set_right_speed(0);
            big_turn = false;
            is_stage_2_detected = false;
            return 1;
        }
        else if(big_turn_counter == big_turn_counter_stop_target && perfmode == 2)
        {
            car_stage = 's';
            //OLED_ShowString(0, 6, (uint8_t *)"<S>", 16);

            set_left_speed(0);
            set_right_speed(0);
            big_turn = false;
            is_stage_2_detected = false;
            return 1;
        }


        /* 计算目标角度 */
        float yaw_now = yaw;
        float yaw_target = yaw + 90;
        if(yaw_target > 180){yaw_target -= 360;}
        


        // 调用阻塞式转弯PID
        delay_ms(50);
        if(is_stage_2_detected){delay_ms(20);}
        turn_pid(0.1,         // 基础速度0.2m/s
                 yaw_target,   // 目标角度
                 threshold,         // 角度阈值设为3度
                 kp_for_turn);       // 比例系数Kp=0.01
        
        //
        circle_num_curr ++;
        in_line_x ++;
        in_line_x = in_line_x % 5 + 1;
        encoder_recoder = left_motor.encoder.total_count;
        //
        /* 转弯结束恢复巡线 */
        set_left_speed(speed_m_per_s);
        set_right_speed(speed_m_per_s);
        big_turn = false;
        is_stage_2_detected = false;
    }

    // 1. 获取当前巡线误差值
    float current_error = line_error;
    
    // sprintf(str, "err %.2f", current_error);
    // OLED_ShowString(50, 7, (uint8_t *)str, 8);


    // 2. 计算PD控制输出
    float p_term = KP * current_error;              // 比例项
    float d_term = KD * (current_error - last_error); // 微分项
    float offset = p_term + d_term;                  // 总调整量
    
    // 3. 保存当前误差供下次使用
    last_error = current_error;
    
    // 4. 计算左右电机速度
    float left_speed, right_speed;
    
    if(offset >= 0) {
        // 正偏移：向右调整，左轮加速，右轮减速
        left_speed = speed_m_per_s + fabs(offset)/2;
        right_speed = speed_m_per_s - fabs(offset)/2;
    } else {
        // 负偏移：向左调整，左轮减速，右轮加速
        left_speed = speed_m_per_s - fabs(offset)/2;
        right_speed = speed_m_per_s + fabs(offset)/2;
    }
    
    // 新增速度限幅（核心改进点）
    left_speed = fmaxf(left_speed, MIN_SPEED);
    right_speed = fmaxf(right_speed, MIN_SPEED);

    // 5. 应用速度设置
    set_left_speed(left_speed);
    set_right_speed(right_speed);
    
//

    // char test[16];
    // float yaw_ = 0, pitch_ = 0;
    // get_target_yaw_pitch(&yaw_, &pitch_);
    // sprintf(test, "%.2f", yaw_);
    // OLED_ShowString(0, 2, (uint8_t *)test, 8);
//

    // 6. 返回状态（0表示正常运行）
    return 0;
}


// PID控制器参数（根据实际调试调整）
float KP_DISTANCE = 0.68f;     // 距离比例系数 (速度/m)
float KI_DISTANCE = 0.0f;     // 距离积分系数 (速度/(m·s))
float KD_DISTANCE = 0.1f;     // 距离微分系数 (速度/(m/s))
#define MIN_DISTANCE 0.05f   // 最小安全距离(m)
#define MIN_VALID_DISTANCE 0.03f
#define MAX_VALID_DISTANCE 0.50f // 最大有效距离(m)
#define MAX_INTEGRAL 6.0f    // 积分限幅值 (满足MAX_INTEGRAL * KI_DISTANCE ≥ 1)
#define MAX_SPEED_ADJUST 0.1f // 最大速度调整量(m/s)

// int follow_front(float dis, float max_speed, float initspeed) {

//     static float last_valid_speed = 0.0f;  // 初始速度值
//     static float last_error = 0.0f;        // 上一次距离误差
//     static float integral = 0.0f;          // 积分项
//     static int flag = 0;
//     if(flag == 0){last_valid_speed = initspeed;flag ++;}
//     // 1. 获取当前超声波距离
//     float current_distance = get_ultrasonic_distance();
    
//     // 2. 检查距离有效性
//     if(current_distance > MAX_VALID_DISTANCE || current_distance < MIN_VALID_DISTANCE) {
//         // 无效距离：init speed
//         integral = 0;  // 重置积分项
//         go_on_line(initspeed);
//         return -1;  // 返回错误码表示使用无效距离
//     }
    
//     // 3. 计算距离误差 (m)
//     float error = current_distance - dis;
    
//     // 4. 计算PID控制输出
//     float p_term = KP_DISTANCE * error;           // 比例项
//     float i_term = KI_DISTANCE * integral;        // 积分项
//     float d_term = KD_DISTANCE * (error - last_error); // 微分项
    
//     // 积分抗饱和：仅在误差较大时累积
//     if(fabs(error) < 0.1f) {  // 当误差小于10cm时累积积分
//         integral += error;
//         // 积分限幅
//         if(integral > MAX_INTEGRAL) integral = MAX_INTEGRAL;
//         else if(integral < -MAX_INTEGRAL) integral = -MAX_INTEGRAL;
//     }
    
//     float speed_adjust = p_term + i_term + d_term; // PID输出
    
//     // 输出限幅
//     if(speed_adjust > MAX_SPEED_ADJUST) speed_adjust = MAX_SPEED_ADJUST;
//     else if(speed_adjust < -MAX_SPEED_ADJUST) speed_adjust = -MAX_SPEED_ADJUST;
    
//     // 5. 计算目标速度
//     float target_speed = last_valid_speed + speed_adjust;
    
//     // 6. 应用速度限制
//     // 确保不低于最小安全速度
//     if(target_speed < -0.3f) target_speed = -0.3f;
//     // 确保不超过最大速度
//     if(target_speed > max_speed) target_speed = max_speed;
    
//     // 紧急制动：当距离小于最小安全距离时
//     if(current_distance < MIN_DISTANCE) {
//         target_speed = 0.0f;  // 完全停止
//         integral = 0;         // 重置积分项
//     }
    
//     // 7. 保存状态
//     last_error = error;
//     last_valid_speed = target_speed;
    
//     // 8. 应用速度控制
//     go_on_line(last_valid_speed);

//     return 0;  // 成功执行
// }

/*=============================== 云台电机控制 ===============================*/

// 全局电机状态变量
MotorState motor_x = {0.0f, -180.0f, 180.0f};  // X轴电机初始位置0°
MotorState motor_y = {0.0f, -90.0f, 90.0f};  // Y轴电机初始位置0°

// 生成电机命令帧 (12字节)
void generate_motor_frame(uint8_t* frame, MotorState* motor, 
                         float offset, uint8_t address) {
    uint16_t rpm = 300;
    float angle_delta = 0.0f;
    
    // 计算角度变化量
    angle_delta = offset;
    
    // 更新并限制角度范围
    motor->current_angle = angle_delta;
    motor->current_angle = fmaxf(motor->min_angle, 
                                fminf(motor->current_angle, motor->max_angle));
    if(fabsf(angle_delta) >= 10){
        rpm = 300;
    }
    // 计算脉冲数 (角度×10)
    int32_t pulse_count = (int32_t)(motor->current_angle * 10.0f);
    uint8_t direction = (pulse_count >= 0) ? DIR_CW : DIR_CCW;
    pulse_count = abs(pulse_count);
    
    // 构建命令帧
    frame[0] = address;          // 电机地址
    frame[1] = COMMAND_CODE;     // 命令码
    frame[2] = direction;        // 方向
    frame[3] = (rpm >> 8) & 0xFF; // 速度高字节
    frame[4] = rpm & 0xFF;       // 速度低字节
    frame[5] = (pulse_count >> 24) & 0xFF;  // 脉冲数字节3
    frame[6] = (pulse_count >> 16) & 0xFF;  // 脉冲数字节2
    frame[7] = (pulse_count >> 8) & 0xFF;   // 脉冲数字节1
    frame[8] = pulse_count & 0xFF;          // 脉冲数字节0
    frame[9] = ABSOLUTE_MODE;    // 绝对位置模式
    frame[10] = SYNC_DISABLED;   // 多机同步标志
    frame[11] = 0x6B;            // 固定校验值
}

// 设置XY轴偏移并发送控制帧
void set_xy_offsets(float x_offset, float y_offset) {
    uint8_t full_frame[29] = {0};
    uint8_t x_frame[12] = {0};
    uint8_t y_frame[12] = {0};
    
    // 1. 生成X轴命令帧
    generate_motor_frame(x_frame, &motor_x, x_offset, MOTOR_ADDRESS_1);
    
    // 2. 生成Y轴命令帧
    generate_motor_frame(y_frame, &motor_y, y_offset, MOTOR_ADDRESS_2);
    
    // 3. 构建29字节完整帧
    full_frame[0] = 0x00;  // 帧头高字节
    full_frame[1] = 0xAA;  // 帧头低字节
    full_frame[2] = 0x00;  // 长度高字节
    full_frame[3] = 0x1D;  // 长度低字节 (29 = 0x1D)
    
    // 复制X/Y轴命令帧
    memcpy(&full_frame[4], x_frame, 12);  // X轴命令
    memcpy(&full_frame[16], y_frame, 12); // Y轴命令
    
    full_frame[28] = 0x6B;  // 结束校验码
    
    // for(int i = 0; i < 29; i++)
    // {
    //     uart0_send_char(UART_FORWORD_INST, full_frame[i]);
    // }
    // 4. 通过UART发送完整帧
    memcpy(gForwardBuffer, full_frame, 29);
    uart_forward_start_send();
}

float max(float num1, float num2)
{
    return (num1 > num2 ? num1: num2);
}

// 函数：计算两个角度之间的最小误差（考虑-180/180边界）
float calculate_yaw_error(float target, float current) {
    float error = target - current;
    if (error > 180.0f) {
        error -= 360.0f;
    } else if (error < -180.0f) {
        error += 360.0f;
    }
    return error;
}
volatile bool is_turning = false;

// 函数：阻塞式转弯PID控制
void turn_pid(float base_speed, float target_angle, float threshold, float Kp) {
    if(Kp >= 0.0011 && Kp <= 0.0013)
    {
        threshold = 20;
    }
    else if(Kp < 0.0009)
    {
        threshold = 15;
    }
    is_turning = true;
    float current_error;
    float output;
    float previous_turned_yaw = 0;
    // 初始更新误差
    current_error = calculate_yaw_error(target_angle, yaw);


    // 循环直到角度误差小于阈值
    while (fabsf(current_error) >= threshold) {
        // 计算P项输出（只需比例控制）
        output = Kp * current_error;
        
        // 设置左右轮速度（左轮减速，右轮加速）
        set_left_speed(base_speed - output);
        set_right_speed(base_speed + output);
        
        // 必须调用此函数更新实际电机速度
        update_motor_speed();  

        //debug
        if(perf_4_flag && zyd_tick_8_flag)
        {
            float turned_angle = 90 - fabsf(current_error);
            // turning_yaw = curr_ang + turned_angle, turning_pitch = curr_pitch;
            char trans[16];
            sprintf(trans, "<%.2f_0>\0", turned_angle - previous_turned_yaw);
            //uart0_send_string(UART_BACKWARD_INST, trans);
            backward_uart_send((uint8_t*)trans,strlen(trans));
            previous_turned_yaw = turned_angle;
            /* turning_yaw turning_pitch*/
            zyd_tick_8_flag = false;
        }
        //debug end

        // 更新当前角度误差
        current_error = calculate_yaw_error(target_angle, yaw);
    }
    is_turning = false;
}


