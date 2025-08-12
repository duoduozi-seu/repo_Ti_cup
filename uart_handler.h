#include "ti_msp_dl_config.h"
#include "motor.h"
void uart0_send_char(UART_Regs *const uart, char ch);
void uart0_send_string(UART_Regs *const uart, char* str);

int get_k230_error(void);
float safe_atof(const char *str);

extern char nums[10]; // 存储匹配结果
extern __IO uint8_t gRxBuffer[8]; // 固定长度7字节的DMA接收缓冲区（<xx.xx>格式）
extern __IO uint8_t rx_flag;   // DMA接收完成标志
extern __IO uint8_t uart_tx_complete_flag; // 发送完成标志
extern volatile float line_error;
extern volatile char car_stage;
extern volatile bool is_stage_2_detected;
// extern __IO uint8_t gForwardRxBuffer[29];  // 接收缓冲区
// extern __IO uint8_t gForwardTxBuffer[29];  // 发送缓冲区
// extern __IO uint8_t forward_rx_flag;                // 接收完成标志
// extern __IO uint8_t forward_tx_complete;             // 发送完成标志
void uart_init(void);
void uart_send(uint8_t buff[], uint16_t size); 
void uart_rx_DMA_config(uint8_t *buff, uint8_t size);
void extract_template(uint8_t *data, uint8_t size);
void convert_to_float(char *str);
bool is_string_eq(char* string_1, char* string_2, int lenth);
void uart_init_k230(void);
extern __IO uint8_t gForwardBuffer[30];  // 单缓冲区：接收和发送共用
extern __IO uint8_t forward_tx_complete; // 发送完成标志
extern volatile bool is_receiving;   // 接收状态标志
void uart_forward_init(void);
// void uart_forward_rx_config(void);
// void uart_forward_send(uint8_t *data, uint16_t size);
// void UART_FORWORD_INST_IRQHandler(void);
void uart_forward_start_receive(void);
void uart_forward_start_send(void);
void reset_uart_reception(void);
void backward_uart_init(void);
void backward_uart_send(const uint8_t* data, uint8_t size);
