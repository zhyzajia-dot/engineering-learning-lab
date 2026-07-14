/*
 * serial.h - 与上位机通信的串口收发接口
 *
 * 收发都使用环形缓冲区：
 *   - 接收：UART 中断把字节塞进 RX 环形缓冲区，任务上下文按行取出
 *   - 发送：SERIAL_SendString/SendInt32 把字节塞进 TX 环形缓冲区，
 *           SERIAL_Task() 在主循环里搬运到 UART FIFO
 *
 * 这样的设计可以让“打印日志”几乎不阻塞控制循环。
 */

#ifndef SERIAL_H
#define SERIAL_H

#include <stdint.h>

/*
 * 使用约束：
 *   - UART 中断只把字节放进 RX 环形缓冲区；
 *   - LAB_Task() 通过 SERIAL_ReadLine() 在主循环解析完整命令；
 *   - 发送接口只负责入队，SERIAL_Task() 才把数据搬入 UART FIFO；
 *   - 缓冲区满时选择丢弃数据，避免日志阻塞电机闭环控制。
 */

/* 初始化串口：清空收发环形缓冲区、使能 UART 中断 */
void SERIAL_Init(void);

/* 把 TX 缓冲区里的数据搬运到 UART FIFO，应在主循环高频调用 */
void SERIAL_Task(void);

/* 若还能原子容纳至少 bytes 个日志字节，返回 1；否则返回 0。 */
uint8_t SERIAL_TxCanAccept(uint16_t bytes);

/*
 * 尝试从 RX 缓冲区读出一行（以 \r 或 \n 结尾）。
 * 成功返回 1，并把去掉行尾换行的字符串写入 line；否则返回 0。
 */
uint8_t SERIAL_ReadLine(char *line, uint16_t capacity);

/* 往 TX 缓冲区写入一段字符串（不会立即发出） */
void SERIAL_SendString(const char *text);

/* 往 TX 缓冲区写入一个 int32 整数的十进制表示 */
void SERIAL_SendInt32(int32_t value);

#endif
