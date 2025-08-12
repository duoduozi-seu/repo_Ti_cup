#include "ti_msp_dl_config.h"
#include "uart_handler.h"
#include <string.h>
#include "oled_hardware_i2c.h"
#include "uart_handler.h"
volatile float line_error = 0;
volatile char car_stage = 'i';
volatile bool is_stage_2_detected = false;
void uart0_send_char(UART_Regs *const uart, char ch)
{
    //当串口0忙的时候等待，不忙的时候再发送传进来的字符
    DL_UART_Main_transmitDataBlocking(uart, ch);
}

//串口发送字符串
void uart0_send_string(UART_Regs *const uart, char* str)
{
    //当前字符串地址不在结尾 并且 字符串首地址不为空
    while(*str!=0 && str!=0)
    {
        //发送字符串首地址中的字符，并且在发送完成之后首地址自增
        uart0_send_char(uart, *str++);
    }
}

char nums[10] = {0}; // 存储匹配结果
__IO uint8_t gRxBuffer[8]; // 固定长度7字节的DMA接收缓冲区（<+/-xx.xx>格式）
__IO uint8_t rx_flag = 1;   // DMA接收完成标志
__IO uint8_t uart_tx_complete_flag = 1; // 发送完成标志
volatile uint8_t frame_buffer[8]; // 完整帧缓冲区
volatile uint8_t frame_index = 0; // 当前帧位置
volatile bool is_frame_started = false; // 帧开始标志

void uart_init(void) {
    
    // 初始配置DMA接收
    rx_flag = 1;
    uart_forward_init();
    uart_init_k230();
    backward_uart_init();
    // 初始状态设置为等待帧尾
    reset_uart_reception();
}

// void uart_send(uint8_t buff[], uint16_t size) {
//     // 等待上次发送完成
//     while(!uart_tx_complete_flag);
    
//     // 设置源地址
//     DL_DMA_setSrcAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t)(buff));
    
//     // 设置目标地址
//     DL_DMA_setDestAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t)(&UART_K230_INST->TXDATA));
    
//     // 设置要搬运的字节数
//     DL_DMA_setTransferSize(DMA, DMA_CH0_CHAN_ID, size);
    
//     // 使能DMA通道
//     DL_DMA_enableChannel(DMA, DMA_CH0_CHAN_ID);
    
//     uart_tx_complete_flag = 0;
// }

// void uart_rx_DMA_config(uint8_t *buff, uint8_t size) {
//     // 等待当前DMA接收完成
//     while(!rx_flag);
    
//     if(rx_flag) {
//         // 设置源地址（串口接收寄存器）
//         DL_DMA_setSrcAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t)(&UART_K230_INST->RXDATA));
        
//         // 设置目标地址（接收缓冲区）
//         DL_DMA_setDestAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t)(buff));
        
//         // 设置要搬运的字节数（固定长度）
//         DL_DMA_setTransferSize(DMA, DMA_CH0_CHAN_ID, size);
        
//         // 使能DMA通道
//         DL_DMA_enableChannel(DMA, DMA_CH0_CHAN_ID);
        
//         // 等待数据接收完成
//         rx_flag = 0;
//     }
// }

// 改进的模板匹配函数
void extract_template(uint8_t *data, uint8_t size) {
    // 检查是否为正确的格式：< +/- xx.xx> (8字节)
    if(size >= 8 && 
       data[0] == '<' && 
       (data[1] == '+' || data[1] == '-') &&
       isdigit(data[2]) && isdigit(data[3]) && 
       data[4] == '.' &&
       isdigit(data[5]) && isdigit(data[6]) &&
       data[7] == '>') {
        
        // 提取完整的符号+数字格式：+xx.xx 或 -xx.xx
        nums[0] = data[1]; // 符号
        nums[1] = data[2]; // 十位
        nums[2] = data[3]; // 个位
        nums[3] = '.';     // 小数点
        nums[4] = data[5]; // 十分位
        nums[5] = data[6]; // 百分位
        nums[6] = '\0';    // 结束符
        if(is_string_eq(nums, "+99.98", 6))
        {
            is_stage_2_detected = true;
            big_turn = true;
            line_error = 0.0f;
            return;
        }
        else if(is_string_eq(nums, "+99.99", 6))
        {
            big_turn = true;
            line_error = 0.0f;
            return;
        }
        // 转换为浮点数
        convert_to_float(nums);
    }
    // 格式错误时处理
    else {
        nums[0] = '\0';
        // 可选：清除旧的line_error或设置默认值
        line_error = 0.0f;
    }
}

bool is_string_eq(char* string_1, char* string_2, int lenth)
{
    for(int i = 0; i < lenth; i++)
    {
        if(string_1[i] != string_2[i])
        {
            return false;
        }
    }
    return true;
}

// 将字符串转换为浮点数
void convert_to_float(char *str) {
    // 跳过符号位先处理数值部分
    float value = 0.0f;
    uint8_t index = 1; // 从第二个字符开始（跳过符号）
    uint8_t decimal_place = 0;
    bool after_decimal = false;
    
    // 解析数字部分
    for(; index < strlen(str); index++) {
        if(str[index] == '.') {
            after_decimal = true;
            decimal_place = 1;
            continue;
        }
        
        if(isdigit(str[index])) {
            if(after_decimal) {
                value += (str[index] - '0') * (1.0f / pow(10, decimal_place));
                decimal_place++;
            } else {
                value = value * 10 + (str[index] - '0');
            }
        }
    }
    
    // 应用符号
    if(str[0] == '-') {
        line_error = -value;
    } else {
        line_error = value;
    }
}


// 串口初始化函数（K230专用）
void uart_init_k230(void) {
    NVIC_EnableIRQ(UART_K230_INST_INT_IRQN);
    
    // 配置DMA接收事件
    DL_UART_Main_enableDMAReceiveEvent(UART_K230_INST, DL_UART_DMA_INTERRUPT_RX);
    
    rx_flag = 1;
    is_frame_started = false;
    frame_index = 0;
    
    // 初始配置DMA
    DL_DMA_setSrcAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t)(&UART_K230_INST->RXDATA));
    DL_DMA_setDestAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t)(gRxBuffer));
    DL_DMA_setTransferSize(DMA, DMA_CH0_CHAN_ID, 8); // 修改为接收8字节
}

// UART重置函数
void reset_uart_reception(void) {
    // 禁用DMA相关中断和通道
    DL_UART_Main_disableInterrupt(UART_K230_INST, DL_UART_MAIN_INTERRUPT_DMA_DONE_RX);
    DL_DMA_disableChannel(DMA, DMA_CH0_CHAN_ID);
    
    // 清除任何挂起的DMA中断
    DL_DMA_clearInterruptStatus(DMA, DL_DMA_INTERRUPT_CHANNEL0);
    
    // 启用UART接收中断
    DL_UART_Main_enableInterrupt(UART_K230_INST, DL_UART_MAIN_INTERRUPT_RX);
    
    // 重置状态变量
    rx_flag = 1;
    is_frame_started = false;
    frame_index = 0;
    
    // 设置为等待帧尾模式
    DL_UART_Main_enableInterrupt(UART_K230_INST, DL_UART_MAIN_INTERRUPT_RX);
    DL_UART_Main_disableInterrupt(UART_K230_INST, DL_UART_MAIN_INTERRUPT_DMA_DONE_RX);
}


// UART中断服务函数
void UART_K230_INST_IRQHandler(void) {
    volatile uint32_t res = DL_UART_Main_getPendingInterrupt(UART_K230_INST);
    
    // 关键：清除中断标志 - 必须在处理前完成
    DL_UART_Main_clearInterruptStatus(UART_K230_INST, res);
    
    // RX中断（等待帧尾模式）
    if (res & DL_UART_MAIN_IIDX_RX) {
        // 接收单个字符
        uint8_t ch = DL_UART_Main_receiveData(UART_K230_INST);
        
        // 检查是否收到帧尾 '>'
        if (ch == '>') {
            // 找到帧尾，现在可以开启DMA接收完整帧
            
            // 禁用UART接收中断
            DL_UART_Main_disableInterrupt(UART_K230_INST, DL_UART_MAIN_INTERRUPT_RX);
            
            // 启用DMA接收中断
            DL_UART_Main_enableInterrupt(UART_K230_INST, DL_UART_MAIN_INTERRUPT_DMA_DONE_RX);
            
            // 配置并启动DMA接收8字节完整帧
            DL_DMA_setTransferSize(DMA, DMA_CH0_CHAN_ID, 8);
            DL_DMA_setDestAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t)(gRxBuffer));
            DL_DMA_enableChannel(DMA, DMA_CH0_CHAN_ID);
            
            rx_flag = 0;
            is_frame_started = true;
        }
    }
    
    // DMA接收完成中断
    if (res & DL_UART_MAIN_IIDX_DMA_DONE_RX) {
        // 处理接收到的完整帧
        extract_template(gRxBuffer, 8);
        
        // 重新启动DMA接收下一个8字节帧
        DL_DMA_setTransferSize(DMA, DMA_CH0_CHAN_ID, 8);
        DL_DMA_setDestAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t)(gRxBuffer));
        DL_DMA_enableChannel(DMA, DMA_CH0_CHAN_ID);
        
        // 清除DMA通道中断标志
        DL_DMA_clearInterruptStatus(DMA, DL_DMA_INTERRUPT_CHANNEL0);
    }
    
    // 传输完成中断
    if (res & DL_UART_IIDX_EOT_DONE) {
        uart_tx_complete_flag = 1;
    }
}


/* FORWORD HANDLER PART - 单缓冲区Single模式修正版 */

// // ==== UART_FORWARD 配置 ====
// #define FORWARD_RX_SIZE 29
// __IO uint8_t gForwardBuffer[29];  // 单缓冲区：接收和发送共用
// __IO uint8_t forward_tx_complete = 1; // 发送完成标志
// volatile bool is_receiving = false;   // 接收状态标志

// // 初始化UART_FORWARD的DMA - 单缓冲区模式
// void uart_forward_init(void) {
//     // 配置接收通道（DMA通道1）
//     DL_DMA_setSrcAddr(DMA, DMA_CH1_CHAN_ID, (uint32_t)(&UART_FORWORD_INST->RXDATA));
//     DL_DMA_setDestAddr(DMA, DMA_CH1_CHAN_ID, (uint32_t)gForwardBuffer);
    
//     // 配置发送通道（DMA通道2）
//     DL_DMA_setSrcAddr(DMA, DMA_CH2_CHAN_ID, (uint32_t)gForwardBuffer);
//     DL_DMA_setDestAddr(DMA, DMA_CH2_CHAN_ID, (uint32_t)(&UART_FORWORD_INST->TXDATA));
    
//     // 启用中断
//     NVIC_EnableIRQ(UART_FORWORD_INST_INT_IRQN);
//     forward_tx_complete = 1; // 发送完成标志
//     is_receiving = false;   // 接收状态标志
//     // 启动第一次接收
//     uart_forward_start_receive();
// }

// // 启动接收 - Single模式
// void uart_forward_start_receive(void) {
//     if (!is_receiving) {
//         DL_DMA_setSrcAddr(DMA, DMA_CH1_CHAN_ID, (uint32_t)(&UART_FORWORD_INST->RXDATA));
//         DL_DMA_setDestAddr(DMA, DMA_CH1_CHAN_ID, (uint32_t)gForwardBuffer);
//         DL_DMA_setTransferSize(DMA, DMA_CH1_CHAN_ID, FORWARD_RX_SIZE);
//         DL_DMA_enableChannel(DMA, DMA_CH1_CHAN_ID);
//         is_receiving = true;
//     }
// }

// // 启动发送 - Single模式
// void uart_forward_start_send(void) {
//     if (forward_tx_complete) {
//         DL_DMA_setSrcAddr(DMA, DMA_CH2_CHAN_ID, (uint32_t)gForwardBuffer);
//         DL_DMA_setDestAddr(DMA, DMA_CH2_CHAN_ID, (uint32_t)(&UART_FORWORD_INST->TXDATA));
//         DL_DMA_setTransferSize(DMA, DMA_CH2_CHAN_ID, FORWARD_RX_SIZE);
//         DL_DMA_enableChannel(DMA, DMA_CH2_CHAN_ID);
//         forward_tx_complete = 0;
//     }
// }


// // UART_FORWARD中断处理函数 - 单缓冲区优化
// void UART_FORWORD_INST_IRQHandler(void) {
//     volatile uint32_t res = DL_UART_Main_getPendingInterrupt(UART_FORWORD_INST);
    
//     // 处理接收完成中断
//     if (res & DL_UART_MAIN_IIDX_DMA_DONE_RX) {
//         // counter++;
//         // sprintf(chr, "FORWORD %d", counter);
//         OLED_ShowString(0, 4, (uint8_t *)"FORWORD", 8);
        
//         // 标记接收完成
//         is_receiving = false;
        
//         // 立即启动发送
//         uart_forward_start_send();
//     }
    
//     // 处理发送完成中断
//     if (res & DL_UART_IIDX_EOT_DONE) {
//         forward_tx_complete = 1;
        
//         // 发送完成后重启接收
//         uart_forward_start_receive();
//     }
// }

/* Rubost Forward */
/* FORWORD HANDLER PART - 单缓冲区Single模式修正版 */

// ==== UART_FORWARD 配置 ====
#define FORWARD_RX_SIZE 30  // 修改为30字节
__IO uint8_t gForwardBuffer[30];  // 单缓冲区：接收和发送共用
__IO uint8_t forward_tx_complete = 1; // 发送完成标志
volatile bool is_receiving = false;   // 接收状态标志
volatile bool forward_reset_requested = false; // 重置请求标志

volatile bool is_in_dma_mode = true;
// 重置函数 - 用于清理无效消息
void reset_forward_reception(void) {
    // 禁用DMA接收
    DL_DMA_disableChannel(DMA, DMA_CH1_CHAN_ID);
    
    // 清除任何挂起的DMA中断
    DL_DMA_clearInterruptStatus(DMA, DL_DMA_INTERRUPT_CHANNEL1);
    
    // 启用UART接收中断
    DL_UART_Main_enableInterrupt(UART_FORWORD_INST, DL_UART_MAIN_INTERRUPT_RX);
    DL_UART_Main_disableInterrupt(UART_FORWORD_INST, DL_UART_MAIN_INTERRUPT_DMA_DONE_RX);
    // 重置状态标志
    is_receiving = false;
    forward_reset_requested = false;
    
    OLED_ShowString(0, 6, (uint8_t *)"FWD RESET", 8); // 显示重置状态
}

// 初始化UART_FORWARD的DMA - 单缓冲区模式
void uart_forward_init(void) {
    // 配置接收通道（DMA通道1）
    DL_DMA_setSrcAddr(DMA, DMA_CH1_CHAN_ID, (uint32_t)(&UART_FORWORD_INST->RXDATA));
    DL_DMA_setDestAddr(DMA, DMA_CH1_CHAN_ID, (uint32_t)gForwardBuffer);
    
    // 配置发送通道（DMA通道2）
    DL_DMA_setSrcAddr(DMA, DMA_CH2_CHAN_ID, (uint32_t)gForwardBuffer);
    DL_DMA_setDestAddr(DMA, DMA_CH2_CHAN_ID, (uint32_t)(&UART_FORWORD_INST->TXDATA));
    
    // 启用中断
    NVIC_EnableIRQ(UART_FORWORD_INST_INT_IRQN);
    DL_UART_Main_disableInterrupt(UART_FORWORD_INST, DL_UART_MAIN_INTERRUPT_RX);
    forward_tx_complete = 1; // 发送完成标志
    is_receiving = false;   // 接收状态标志
    
    // 初始状态设置为等待帧尾
    uart_forward_start_receive();
}

// 启动接收 - Single模式
void uart_forward_start_receive(void) {
    if (!is_receiving) {
        DL_DMA_setSrcAddr(DMA, DMA_CH1_CHAN_ID, (uint32_t)(&UART_FORWORD_INST->RXDATA));
        DL_DMA_setDestAddr(DMA, DMA_CH1_CHAN_ID, (uint32_t)gForwardBuffer);
        DL_DMA_setTransferSize(DMA, DMA_CH1_CHAN_ID, FORWARD_RX_SIZE);
        DL_DMA_enableChannel(DMA, DMA_CH1_CHAN_ID);
        is_receiving = true;
    }
}

// 启动发送 - Single模式
void uart_forward_start_send(void) {
    // if (forward_tx_complete) {
        // 只发送前29字节
        DL_DMA_setSrcAddr(DMA, DMA_CH2_CHAN_ID, (uint32_t)gForwardBuffer);
        DL_DMA_setDestAddr(DMA, DMA_CH2_CHAN_ID, (uint32_t)(&UART_FORWORD_INST->TXDATA));
        DL_DMA_setTransferSize(DMA, DMA_CH2_CHAN_ID, 29); // 只发送29字节
        DL_DMA_enableChannel(DMA, DMA_CH2_CHAN_ID);
        
    //}
}

// UART_FORWARD中断处理函数 - 单缓冲区优化
void UART_FORWORD_INST_IRQHandler(void) {
    volatile uint32_t res = DL_UART_Main_getPendingInterrupt(UART_FORWORD_INST);
    
    // 清除中断标志
    DL_UART_Main_clearInterruptStatus(UART_FORWORD_INST, res);
    
    // RX中断（等待帧尾模式）
    if (res & DL_UART_MAIN_IIDX_RX) {
        //counter++;
        // 接收单个字符
        uint8_t ch = DL_UART_Main_receiveData(UART_FORWORD_INST);
        
        // 检查是否收到帧尾 '>'
        if (ch == '>') {
            
            // 找到帧尾，切换到DMA接收模式
            DL_UART_Main_disableInterrupt(UART_FORWORD_INST, DL_UART_MAIN_INTERRUPT_RX);
            DL_UART_Main_enableInterrupt(UART_FORWORD_INST, DL_UART_MAIN_INTERRUPT_DMA_DONE_RX);
            DL_DMA_enableChannel(DMA, DMA_CH1_CHAN_ID);
            uart_forward_start_receive();
        }
    }
    
    // 处理接收完成中断
    if (res & DL_UART_MAIN_IIDX_DMA_DONE_RX) {
        // 检查帧尾是否正确
        if (gForwardBuffer[29] != '>') {
            // 帧尾错误，重置接收
            reset_forward_reception();
            return;
        }
        
        //OLED_ShowString(0, 4, (uint8_t *)"FORWORD", 8);
        
        // 标记接收完成
        is_receiving = false;
        forward_tx_complete = 1;
        // 立即启动发送（只发送前29字节）
        uart_forward_start_send();
    }
    
    // 处理发送完成中断
    if (res & DL_UART_IIDX_EOT_DONE && !(res & DL_UART_MAIN_IIDX_RX)) {
        forward_tx_complete = 1;
        
        // 发送完成后重启接收
        uart_forward_start_receive();
    }
}
/* FORWORD HANDLER PART - 单缓冲区Single模式修正版 */

// // ==== UART_FORWARD 配置 ====
// #define FORWARD_RX_SIZE 30  // 修改为30字节
// __IO uint8_t gForwardBuffer[30];  // 单缓冲区：接收和发送共用
// __IO uint8_t forward_tx_complete = 1; // 发送完成标志
// volatile bool is_receiving = false;   // 接收状态标志
// volatile bool is_waiting_for_end = false; // 帧尾等待模式标志

// // 重置函数 - 用于清理无效消息
// void reset_forward_reception(void) {
//     // 禁用所有 DMA 通道
//     DL_DMA_disableChannel(DMA, DMA_CH1_CHAN_ID); // 接收通道
//     DL_DMA_disableChannel(DMA, DMA_CH2_CHAN_ID); // 发送通道
    
//     // 清除所有挂起的 DMA 中断
//     DL_DMA_clearInterruptStatus(DMA, DL_DMA_INTERRUPT_CHANNEL1);
//     DL_DMA_clearInterruptStatus(DMA, DL_DMA_INTERRUPT_CHANNEL2);
    
//     // 禁用所有 UART 中断
//     DL_UART_Main_disableInterrupt(UART_FORWORD_INST, DL_UART_MAIN_INTERRUPT_RX);
//     DL_UART_Main_disableInterrupt(UART_FORWORD_INST, DL_UART_MAIN_INTERRUPT_DMA_DONE_RX);
//     DL_UART_Main_disableInterrupt(UART_FORWORD_INST, DL_UART_IIDX_EOT_DONE);
    
//     // 清除所有 UART 中断标志
//     //DL_UART_Main_clearInterruptsStatus(UART_FORWORD_INST);
    
//     // 启用帧尾等待模式
//     is_waiting_for_end = true;
//     DL_UART_Main_enableInterrupt(UART_FORWORD_INST, DL_UART_MAIN_INTERRUPT_RX);
    
//     // 重置状态标志
//     is_receiving = false;
//     forward_tx_complete = 1;
    
//     OLED_ShowString(0, 6, (uint8_t *)"FWD RESET", 8); // 显示重置状态
// }

// // 初始化UART_FORWARD的DMA - 单缓冲区模式
// void uart_forward_init(void) {
//     // 配置接收通道（DMA通道1）
//     DL_DMA_setSrcAddr(DMA, DMA_CH1_CHAN_ID, (uint32_t)(&UART_FORWORD_INST->RXDATA));
//     DL_DMA_setDestAddr(DMA, DMA_CH1_CHAN_ID, (uint32_t)gForwardBuffer);
    
//     // 配置发送通道（DMA通道2）
//     DL_DMA_setSrcAddr(DMA, DMA_CH2_CHAN_ID, (uint32_t)gForwardBuffer);
//     DL_DMA_setDestAddr(DMA, DMA_CH2_CHAN_ID, (uint32_t)(&UART_FORWORD_INST->TXDATA));
    
//     // 启用中断
//     NVIC_EnableIRQ(UART_FORWORD_INST_INT_IRQN);
//     forward_tx_complete = 1; // 发送完成标志
//     is_receiving = false;   // 接收状态标志
    
//     uart_forward_start_receive();
// }

// // 启动接收 - Single模式
// void uart_forward_start_receive(void) {
//     if (!is_receiving) {
//         // 禁用帧尾等待模式
//         is_waiting_for_end = false;
        
//         // 禁用 RX 中断，启用 DMA 完成中断
//         DL_UART_Main_disableInterrupt(UART_FORWORD_INST, DL_UART_MAIN_INTERRUPT_RX);
//         DL_UART_Main_enableInterrupt(UART_FORWORD_INST, DL_UART_MAIN_INTERRUPT_DMA_DONE_RX);
        
//         // 清除所有挂起的 DMA 中断
//         DL_DMA_clearInterruptStatus(DMA, DL_DMA_INTERRUPT_CHANNEL1);
        
//         // 配置并启动 DMA 接收
//         DL_DMA_setSrcAddr(DMA, DMA_CH1_CHAN_ID, (uint32_t)(&UART_FORWORD_INST->RXDATA));
//         DL_DMA_setDestAddr(DMA, DMA_CH1_CHAN_ID, (uint32_t)gForwardBuffer);
//         DL_DMA_setTransferSize(DMA, DMA_CH1_CHAN_ID, FORWARD_RX_SIZE);
//         DL_DMA_enableChannel(DMA, DMA_CH1_CHAN_ID);
        
//         is_receiving = true;
//     }
// }

// // 启动发送 - Single模式
// void uart_forward_start_send(void) {
//     if (forward_tx_complete) {
//         // 清除所有挂起的 DMA 中断
//         DL_DMA_clearInterruptStatus(DMA, DL_DMA_INTERRUPT_CHANNEL2);
        
//         // 只发送前29字节
//         DL_DMA_setSrcAddr(DMA, DMA_CH2_CHAN_ID, (uint32_t)gForwardBuffer);
//         DL_DMA_setDestAddr(DMA, DMA_CH2_CHAN_ID, (uint32_t)(&UART_FORWORD_INST->TXDATA));
//         DL_DMA_setTransferSize(DMA, DMA_CH2_CHAN_ID, 29); // 只发送29字节
//         DL_DMA_enableChannel(DMA, DMA_CH2_CHAN_ID);
        
//         forward_tx_complete = 0;
//     }
// }

// // UART_FORWARD中断处理函数 - 单缓冲区优化
// void UART_FORWORD_INST_IRQHandler(void) {
//     volatile uint32_t res = DL_UART_Main_getPendingInterrupt(UART_FORWORD_INST);
    
//     // 清除中断标志
//     DL_UART_Main_clearInterruptStatus(UART_FORWORD_INST, res);
    
//     // 只在帧尾等待模式下处理 RX 中断
//     if ((res & DL_UART_MAIN_IIDX_RX) && is_waiting_for_end) {
//         // 接收单个字符
//         uint8_t ch = DL_UART_Main_receiveDataBlocking(UART_FORWORD_INST);
        
//         // 检查是否收到帧尾 '>'
//         if (ch == '>') {
//             // 找到帧尾，切换到DMA接收模式
//             uart_forward_start_receive();
//         }
//     }
    
//     // 处理接收完成中断
//     if (res & DL_UART_MAIN_IIDX_DMA_DONE_RX) {
//         // 检查帧尾是否正确
//         if (gForwardBuffer[29] != '>') {
//             // 帧尾错误，重置接收
//             reset_forward_reception();
//             return;
//         }
        
//         OLED_ShowString(0, 4, (uint8_t *)"FORWORD", 8);
        
//         // 标记接收完成
//         is_receiving = false;
        
//         // 立即启动发送（只发送前29字节）
//         uart_forward_start_send();
//     }
    
//     // 处理发送完成中断
//     if (res & DL_UART_IIDX_EOT_DONE) {
//         forward_tx_complete = 1;
        
//         // 发送完成后重启接收
//         uart_forward_start_receive();
//     }
// }

// ==== 新串口配置 ====
#define BACKWARD_TX_SIZE 16  // 固定发送16字节
__IO uint8_t gBackwardTxBuffer[BACKWARD_TX_SIZE];  // 发送专用缓冲区
volatile bool backward_tx_complete = true;         // 发送完成标志

// 初始化新串口的DMA发送
void backward_uart_init(void) {
    // 1. 配置DMA通道3
    DL_DMA_setSrcAddr(DMA, DMA_CH3_CHAN_ID, (uint32_t)gBackwardTxBuffer);
    DL_DMA_setDestAddr(DMA, DMA_CH3_CHAN_ID, (uint32_t)(&UART_BACKWARD_INST->TXDATA));
    
    // 2. 启用发送完成中断
    DL_UART_Main_enableInterrupt(UART_BACKWARD_INST, DL_UART_IIDX_EOT_DONE);
    
    // 3. 启用UART中断
    NVIC_EnableIRQ(UART_BACKWARD_INST_INT_IRQN);
    
    // 4. 初始化缓冲区为全0
    memset((void*)gBackwardTxBuffer, 0, BACKWARD_TX_SIZE);
    
    // 5. 设置发送完成标志
    backward_tx_complete = true;
}

// 新串口发送函数
void backward_uart_send(const uint8_t* data, uint8_t size) {
    // 1. 等待上次发送完成
    while(!backward_tx_complete);
    DL_DMA_setSrcAddr(DMA, DMA_CH3_CHAN_ID, (uint32_t)gBackwardTxBuffer);
    DL_DMA_setDestAddr(DMA, DMA_CH3_CHAN_ID, (uint32_t)(&UART_BACKWARD_INST->TXDATA));
    // 2. 初始化缓冲区为全0
    memset((void*)gBackwardTxBuffer, 0, BACKWARD_TX_SIZE);
    
    // 3. 复制有效数据
    uint8_t copy_size = (size <= BACKWARD_TX_SIZE) ? size : BACKWARD_TX_SIZE;
    memcpy((void*)gBackwardTxBuffer, data, copy_size);
    
    // 4. 配置并启动DMA发送
    DL_DMA_setTransferSize(DMA, DMA_CH3_CHAN_ID, BACKWARD_TX_SIZE);
    DL_DMA_enableChannel(DMA, DMA_CH3_CHAN_ID);
    
    // 5. 设置发送中标志
    backward_tx_complete = false;
}

// 新串口中断处理函数
void UART_BACKWARD_INST_IRQHandler(void) {
    volatile uint32_t res = DL_UART_Main_getPendingInterrupt(UART_BACKWARD_INST);
    
    // 处理发送完成中断
    if (res & DL_UART_IIDX_EOT_DONE) {
        // 清除中断标志
        DL_UART_Main_clearInterruptStatus(UART_BACKWARD_INST, DL_UART_IIDX_EOT_DONE);
        
        // 设置发送完成标志
        backward_tx_complete = true;
    }
}