#include "ti_msp_dl_config.h"
#include "PTZ_controller.h"


// 全局参数
float dist_to_canvas = 1300.0f; // 云台到画布的距离(mm)
float pen_height = 0.0f;       // 激光笔高度(mm)
float amplitude = 100.0f;      // 正弦波振幅(mm)
float wavelength = 100.0f;     // 正弦波波长(mm)
int points_per_wave = 100;     // 每个正弦波的点数
float move_delay = 20.0f;     // 点间移动延时(ms)

// 舵机控制相关参数
static float theta = 0.0f;         // 当前角度参数
static const float theta_step = 0.02f; // 每步的角度增量
static int mode = 0;               // 状态机模式
static int total_steps = 0;         // 总步数
static float last_theta_x = 0.0f;   // 上一次X轴角度
static float last_theta_y = 0.0f;   // 上一次Y轴角度
static float step_x = 0.0f;         // X轴角度步进
static float step_y = 0.0f;         // Y轴角度步进
char chr[32];                       // OLED显示缓冲区

// 计算点在画布上的坐标对应的云台绝对角度
void calculate_angles(float x, float y, float* theta_x, float* theta_y) {
    // 计算水平角度 (θ_x)
    *theta_x = atan2f(x, dist_to_canvas) * (180.0f / M_PI); // 弧度转角度
    
    // 计算垂直角度 (θ_y)
    // 考虑激光笔高度对垂直角度的影响
    float effective_dist = sqrtf(x*x + dist_to_canvas*dist_to_canvas);
    *theta_y = atan2f(y - pen_height, effective_dist) * (180.0f / M_PI); // 弧度转角度
}

// 移动到指定位置
void move_to_position(float target_x, float target_y) {
    // 计算目标角度
    float target_theta_x, target_theta_y;
    calculate_angles(target_x, target_y, &target_theta_x, &target_theta_y);
    
    // 限制角度在电机允许范围内
    target_theta_x = fmaxf(motor_x.min_angle, fminf(target_theta_x, motor_x.max_angle));
    target_theta_y = fmaxf(motor_y.min_angle, fminf(target_theta_y, motor_y.max_angle));
    
    // 计算每步的增量
    step_x = (target_theta_x - last_theta_x) / 100.0f;
    step_y = (target_theta_y - last_theta_y) / 100.0f;
    
    // 更新上一次的角度
    last_theta_x = target_theta_x;
    last_theta_y = target_theta_y;
    
    // 计算实际角度（插值）
    float real_theta_x = last_theta_x + step_x;
    float real_theta_y = last_theta_y + step_y;
    
    // 发送绝对角度控制命令
    set_xy_offsets(real_theta_x, real_theta_y);
    
    // 更新电机当前角度状态
    motor_x.current_angle = real_theta_x;
    motor_y.current_angle = real_theta_y;
    
    // 等待电机移动到目标位置
    delay_ms(move_delay);
}

// 分步绘制两个正弦波函数
void draw_two_sine_waves_step() {
    // 状态机控制
    switch (mode) {
        case 0: // 移动到第一个正弦波的起始位置
            move_to_position(-wavelength, 0);
            if (fabsf(motor_x.current_angle - last_theta_x) < 0.1f && 
                fabsf(motor_y.current_angle - last_theta_y) < 0.1f) {
                mode = 1; // 进入绘制第一个正弦波状态
                theta = 0.0f; // 重置角度参数
            }
            break;
            
        case 1: // 绘制第一个正弦波
        case 2: // 绘制第二个正弦波
            // 计算当前点的X坐标
            float wave_offset = (mode == 1) ? -wavelength : 0.0f;
            float x = wave_offset + theta * wavelength / (2 * M_PI);
            
            // 计算正弦波Y值
            float y = amplitude * sinf(theta);
            
            // 计算需要的云台绝对角度
            float target_theta_x, target_theta_y;
            calculate_angles(x, y, &target_theta_x, &target_theta_y);
            
            // 限制角度在电机允许范围内
            target_theta_x = fmaxf(motor_x.min_angle, fminf(target_theta_x, motor_x.max_angle));
            target_theta_y = fmaxf(motor_y.min_angle, fminf(target_theta_y, motor_y.max_angle));
            
            // 显示调试信息
            sprintf(chr, "x:%.1f y:%.1f", target_theta_x, target_theta_y);
            OLED_ShowString(0, 2, (uint8_t *)chr, 8);
            
            // 计算每步的增量
            step_x = (target_theta_x - last_theta_x) / 100.0f;
            step_y = (target_theta_y - last_theta_y) / 100.0f;
            
            // 更新上一次的角度
            last_theta_x = target_theta_x;
            last_theta_y = target_theta_y;
            
            // 增加角度参数
            theta += theta_step;
            
            // 检查是否完成当前正弦波
            if (theta >= 2 * M_PI) {
                theta = 0.0f; // 重置角度
                mode++; // 进入下一个状态
                
                // 如果完成第二个正弦波，进入返回状态
                if (mode > 2) {
                    mode = 3; // 进入返回中点状态
                }
            }
            break;
            
        case 3: // 返回中点(0,0)
            move_to_position(0, 0);
            if (fabsf(motor_x.current_angle) < 0.1f && 
                fabsf(motor_y.current_angle) < 0.1f) {
                mode = 0; // 重置状态机
            }
            break;
    }
    
    // 计算实际角度（插值）
    float real_theta_x = last_theta_x + step_x;
    float real_theta_y = last_theta_y + step_y;
    
    // 发送绝对角度控制命令
    set_xy_offsets(real_theta_x, real_theta_y);
    
    // 更新电机当前角度状态
    motor_x.current_angle = real_theta_x;
    motor_y.current_angle = real_theta_y;
    
    // 等待电机移动到目标位置
    delay_ms(move_delay);
}

#include <math.h>
// 定义常量
#define PI 3.1416

// // 角度转换函数
// double radians_to_degrees(double radians) {
//     return radians * (180.0 / PI);
// }

// // 主计算函数
// void calculate_angles_for_perf4(float x, float y, float height, int car_direction, 
//                       float target_x, float target_y, float target_height,
//                       float *yaw, float *pitch) {
//     // 1. 计算从云台到靶点的向量
//     float dx = target_x - x;
//     float dy = target_y - y;
//     float dz = target_height - height;
    
//     // 2. 计算水平距离（忽略高度）
//     float horizontal_distance = sqrtf(dx * dx + dy * dy);
    
//     // 3. 计算俯仰角（pitch）
//     *pitch = radians_to_degrees(atan2f(dz, horizontal_distance));
    
//     // 4. 计算绝对水平角度（相对于世界坐标系）
//     float absolute_yaw = radians_to_degrees(atan2f(dy, dx));
    
//     // 5. 根据车头朝向计算相对偏移角
//     float reference_angle = 0.0f;
    
//     switch (car_direction) {
//         case 1: // y轴负方向
//             reference_angle = -90.0f;
//             break;
//         case 2: // x轴正方向
//             reference_angle = 0.0f;
//             break;
//         case 3: // y轴正方向
//             reference_angle = 90.0f;
//             break;
//         case 4: // x轴负方向
//             reference_angle = 180.0f;
//             break;
//     }
    
//     // 6. 计算相对于车头朝向的偏移角
//     *yaw = absolute_yaw - reference_angle;
    
//     // 7. 将偏移角规范到[-180, 180]范围
//     if (*yaw > 180.0f) {
//         *yaw -= 360.0f;
//     } else if (*yaw < -180.0f) {
//         *yaw += 360.0f;
//     }
//     *yaw = -*yaw;
// }

// float get_x_coordinate()
// {
//     switch (in_line_x) {
//         case 1:
//         {
//             return -50.0;
//             break;
//         }
//         case 2:
//         {
//             return -50.0 + ((left_motor.encoder.total_count - encoder_recoder)/17.0);
//             break;
//         }
//         case 3:
//         {
//             return 50.0;
//             break;
//         }
//         case 4:
//         {
//             return 50.0 - ((left_motor.encoder.total_count - encoder_recoder)/17.0);
//         }
//     }
// }
// float get_y_coordinate()
// {
//     switch (in_line_x) {
//         case 1:
//         {
//             return 50.0 - ((left_motor.encoder.total_count - encoder_recoder)/17.0);
//             break;
//         }
//         case 2:
//         {
//             return -50.0;
//             break;
//         }
//         case 3:
//         {
//             return - 50.0 + ((left_motor.encoder.total_count - encoder_recoder)/17.0);
//             break;
//         }
//         case 4:
//         {
//             return 50.0;
//         }
//     }
// }
// void get_target_yaw_pitch(float *yaw, float *pitch)
// {
//     calculate_angles_for_perf4(get_x_coordinate(), get_y_coordinate(), 17.6, in_line_x, 0, 100, 20, yaw, pitch);
//     return;
// }


// #include <math.h>

// // 预定义常量（使用整数表示）
// #define PI_FIXED 3141592      // π * 10^6
// #define RAD_TO_DEG_FIXED 57295780  // (180/π) * 10^6
// #define DEG_TO_RAD_FIXED 17453     // (π/180) * 10^6
// #define SCALE_FACTOR 1000000  // 缩放因子

// // 快速整数平方根近似（使用巴比伦方法）
// int32_t fast_isqrt(int64_t num) {
//     if (num <= 0) return 0;
    
//     int64_t res = num;
//     int64_t last;
    
//     do {
//         last = res;
//         res = (res + num / res) >> 1;  // 使用移位代替除法
//     } while (last != res && (last - res) > 1);
    
//     return (int32_t)res;
// }

// // 快速整数atan2近似（使用多项式近似）
// int32_t fast_atan2(int32_t y, int32_t x) {
//     if (x == 0) {
//         return y > 0 ? 1570796 : (y < 0 ? -1570796 : 0);  // π/2 * 10^6
//     }
    
//     int32_t abs_y = y > 0 ? y : -y;
//     int32_t r;
//     int32_t angle;
    
//     if (x > 0) {
//         r = (x - abs_y) * SCALE_FACTOR / (x + abs_y);
//         angle = 785398 - r * (2447 - r * 66) / 10000;  // π/4 * 10^6
//     } else {
//         r = (x + abs_y) * SCALE_FACTOR / (abs_y - x);
//         angle = 2356194 - r * (2447 - r * 66) / 10000;  // 3π/4 * 10^6
//     }
    
//     return y < 0 ? -angle : angle;
// }

// // 主计算函数（优化版）
// void calculate_angles_for_perf4(int32_t x, int32_t y, int32_t height, int car_direction, 
//                       int32_t target_x, int32_t target_y, int32_t target_height,
//                       int32_t *yaw, int32_t *pitch) {
//     // 1. 计算从云台到靶点的向量（使用整数）
//     int32_t dx = target_x - x;
//     int32_t dy = target_y - y;
//     int32_t dz = target_height - height;
    
//     // 2. 计算水平距离（使用快速整数平方根）
//     int64_t dx_sq = (int64_t)dx * dx;
//     int64_t dy_sq = (int64_t)dy * dy;
//     int32_t horizontal_distance = fast_isqrt(dx_sq + dy_sq);
    
//     // 3. 计算俯仰角（pitch）
//     int32_t pitch_rad = fast_atan2(dz, horizontal_distance);
//     *pitch = (pitch_rad * RAD_TO_DEG_FIXED) / SCALE_FACTOR;
    
//     // 4. 计算绝对水平角度（相对于世界坐标系）
//     int32_t absolute_yaw_rad = fast_atan2(dy, dx);
//     int32_t absolute_yaw = (absolute_yaw_rad * RAD_TO_DEG_FIXED) / SCALE_FACTOR;
    
//     // 5. 根据车头朝向计算相对偏移角
//     int32_t reference_angle = 0;
    
//     switch (car_direction) {
//         case 1: // y轴负方向
//             reference_angle = -90000000;  // -90度 * 10^6
//             break;
//         case 2: // x轴正方向
//             reference_angle = 0;
//             break;
//         case 3: // y轴正方向
//             reference_angle = 90000000;   // 90度 * 10^6
//             break;
//         case 4: // x轴负方向
//             reference_angle = 180000000;  // 180度 * 10^6
//             break;
//     }
    
//     // 6. 计算相对于车头朝向的偏移角
//     *yaw = absolute_yaw - reference_angle;
    
//     // 7. 将偏移角规范到[-180, 180]范围
//     if (*yaw > 180000000) {
//         *yaw -= 360000000;
//     } else if (*yaw < -180000000) {
//         *yaw += 360000000;
//     }
    
//     // 8. 转换为最终输出格式（整数度 * 1000）
//     *yaw = -*yaw / 1000;  // 转换为整数度 * 1000
//     *pitch = *pitch / 1000;  // 转换为整数度 * 1000
// }

// #define ENCODER_SCALE_FACTOR 17000  // 17.0 * 1000
// #define FIXED_POINT_SCALE 1000      // 定点数缩放因子




// // 获取X坐标（整型计算）
// int32_t get_x_coordinate_fixed() {
//     int32_t encoder_diff = left_motor.encoder.total_count - encoder_recoder;
    
//     switch (in_line_x) {
//         case 1: return -50000; // -50.0 * 1000
//         case 2: return -50000 + (encoder_diff * FIXED_POINT_SCALE) / ENCODER_SCALE_FACTOR;
//         case 3: return 50000;  // 50.0 * 1000
//         case 4: return 50000 - (encoder_diff * FIXED_POINT_SCALE) / ENCODER_SCALE_FACTOR;
//         default: return 0;
//     }
// }

// // 获取Y坐标（整型计算）
// int32_t get_y_coordinate_fixed() {
//     int32_t encoder_diff = left_motor.encoder.total_count - encoder_recoder;
    
//     switch (in_line_x) {
//         case 1: return 50000 - (encoder_diff * FIXED_POINT_SCALE) / ENCODER_SCALE_FACTOR;
//         case 2: return -50000; // -50.0 * 1000
//         case 3: return -50000 + (encoder_diff * FIXED_POINT_SCALE) / ENCODER_SCALE_FACTOR;
//         case 4: return 50000;  // 50.0 * 1000
//         default: return 0;
//     }
// }

// // 获取目标偏航角和俯仰角（整型计算）
// void get_target_yaw_pitch_fixed(int32_t *yaw, int32_t *pitch) {
//     // 调用优化后的角度计算函数
//     calculate_angles_for_perf4(
//         get_x_coordinate_fixed(), 
//         get_y_coordinate_fixed(), 
//         17600,  // 17.6 * 1000
//         in_line_x, 
//         0, 
//         100000, // 100.0 * 1000
//         20000,  // 20.0 * 1000
//         yaw, 
//         pitch
//     );
// }

// 浮点接口（保持原接口不变）
// float get_x_coordinate() {
//     return (float)get_x_coordinate_fixed() / FIXED_POINT_SCALE;
// }

// float get_y_coordinate() {
//     return (float)get_y_coordinate_fixed() / FIXED_POINT_SCALE;
// }

// void get_target_yaw_pitch(float *yaw, float *pitch) {
//     int32_t yaw_fixed, pitch_fixed;
//     get_target_yaw_pitch_fixed(&yaw_fixed, &pitch_fixed);
//     *yaw = (float)yaw_fixed / 1000.0f;   // 转换为浮点角度
//     *pitch = (float)pitch_fixed / 1000.0f;
// }

void contrl_ptz()
{
    OLED_Clear();
    int x = (int)InputFloat(5, 0, 0);
    int y = (int)InputFloat(5, 0, 0);
    set_xy_offsets(x,y);
    OLED_Clear();
}