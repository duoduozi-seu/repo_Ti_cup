#ifndef __MOTOR_H_
#define __MOTOR_H_

#include "stdlib.h"
#include "ti_msp_dl_config.h"
#include "uart_handler.h"
// ========================= 硬件参数宏定义 =========================
#define MOTOR_SPEED_RERATIO  28      // 电机减速比
#define PULSE_PRE_ROUND      13      // 电机每圈脉冲数(编码器分辨率)
#define ENCODER_MULTIPLE     1.0     // 编码器倍频系数
#define PULSE_PER_CYCLE      (MOTOR_SPEED_RERATIO * PULSE_PRE_ROUND * ENCODER_MULTIPLE) // 电机轴每圈总脉冲数
#define RADIUS_OF_TYRE       3.4     // 轮子半径(cm)
#define LINE_SPEED_C         (RADIUS_OF_TYRE * 2 * 3.14) // 轮子周长(cm)
#define SPEED_RECORD_NUM     20      // 速度滤波采样窗口大小

// ========================= 数据结构定义 =========================
typedef struct {
    int direction;               // 旋转方向
    int32_t total_count;             // 编码器累计计数值
    int32_t last_count;              // 上次计数值(用于速度计算)
    float   current_speed;           // 当前计算速度(RPM)
    float   speed_records[SPEED_RECORD_NUM]; // 速度记录数组(用于滤波)
} encoder_t;

typedef struct {
    float kp, ki, kd;        // PID参数
    float error;              // 当前误差
    float last_error;         // 上次误差
    float integral;           // 积分项
    float max_integral;       // 积分限幅
    float output;             // PID输出
    float max_output;         // 输出限幅
    float dead_zone;          // 死区范围
} pid_controller_t;

typedef struct {
    // 电机控制参数
    int direction;        // 当前转向
    uint8_t pwm_channel;      // PWM定时器通道
    float   pwm_duty;         // PWM占空比(0~100%)
    float   target_speed;     // 目标速度(RPM)
    float   target_position;  // 目标位置(cm)
    int32_t position_pulses;  // 位置环目标脉冲数
    
    // 硬件接口
    GPIO_Regs *forward_port;  // 前进方向GPIO端口
    GPIO_Regs *reverse_port;  // 后退方向GPIO端口
    uint32_t forward_pin;     // 前进方向引脚
    uint32_t reverse_pin;     // 后退方向引脚
    
    // 组件
    encoder_t encoder;        // 编码器数据
    pid_controller_t speed_pid; // 速度环PID控制器
    pid_controller_t pos_pid;   // 位置环PID控制器
} motor_controller_t;



// ========================= 全局变量声明 =========================
extern motor_controller_t left_motor;
extern motor_controller_t right_motor;
extern volatile int perfmode;
extern volatile uint8_t speed_update_flag;  // 100Hz速度更新标志
extern uint32_t tick_zydtime;
extern volatile bool big_turn;
extern volatile int big_turn_counter;
extern volatile int big_turn_counter_stop_target;
// PD控制器参数（根据实际调试调整）
extern float KP;    // 比例系数
extern float KD;   // 微分系数
// ========================= 云台控制 =========================
// 电机控制常量
#define DIR_CW 0x01
#define DIR_CCW 0x00
#define MOTOR_ADDRESS_1 0x02
#define MOTOR_ADDRESS_2 0x01
#define COMMAND_CODE 0xFB
#define ABSOLUTE_MODE 0x01
#define SYNC_DISABLED 0x00

// 电机全局状态
typedef struct {
    float current_angle;  // 当前角度（度）
    float min_angle;      // 最小角度限制
    float max_angle;      // 最大角度限制
} MotorState;

extern MotorState motor_x;
extern MotorState motor_y;
// ========================= 函数声明 =========================



void motors_init(void);                   // 初始化双电机系统
void update_motor_speed(void);            // 更新电机速度计算(定时调用)
void set_motor_duty(motor_controller_t *motor); // 设置电机PWM占空比
void set_motor_speed(motor_controller_t *motor); // 速度环控制
void set_motor_position(motor_controller_t *motor); // 位置环控制
void stop_motor(motor_controller_t *motor);       // 停止电机

// PID相关函数
float calculate_pid(pid_controller_t *pid, float target, float actual); // PID计算
float speed_filter(float new_speed, encoder_t *encoder); // 速度滤波

void set_left_speed(float speed_meter_per_s);
void set_right_speed(float speed_meter_per_s);
void update_motor_speed();
int go_on_line(float speed_m_per_s);
// int follow_front(float dis, float max_speed, float initspeed);


void generate_motor_frame(uint8_t* frame, MotorState* motor, float offset, uint8_t address);
void set_xy_offsets(float x_offset, float y_offset);
float max(float num1, float num2);
void turn_pid(float base_speed, float target_angle, float threshold, float Kp);
float calculate_yaw_error(float target, float current);
extern float kp_for_turn;
extern int32_t curr_ang;
extern int32_t curr_pitch; 
extern int prev_line_index;   // 上一次的边索引
extern int loop_count;        // 完成的圈数
extern volatile bool is_turning;
extern float KP_p4;    // 比例系数
extern float KD_p4;   // 微分系数
extern float kp_for_turn_p4;
extern float threshold;
#endif /* __MOTOR_H_ */