#include "serial.h"
#include "ti_msp_dl_config.h"

#include <stdint.h>

/* 接收 / 发送环形缓冲区大小。发送缓冲设大一点，方便一次打印多行日志 */
#define SERIAL_RX_BUFFER_SIZE 128U
#define SERIAL_TX_BUFFER_SIZE 1024U

/* 解析一行命令的最大长度（不含 \0） */
#define SERIAL_LINE_SIZE       64U

/* 接收环形缓冲区（UART 中断写入，任务上下文读取） */
static volatile uint8_t g_rxBuffer[SERIAL_RX_BUFFER_SIZE];
static volatile uint16_t g_rxHead = 0U;
static volatile uint16_t g_rxTail = 0U;

/* 发送环形缓冲区（任务上下文写入，任务上下文搬运到 UART FIFO） */
static uint8_t g_txBuffer[SERIAL_TX_BUFFER_SIZE];
static uint16_t g_txHead = 0U;
static uint16_t g_txTail = 0U;

/* 解析时使用的行缓冲区；超过 SERIAL_LINE_SIZE 时会直接丢弃整行 */
static char g_lineBuffer[SERIAL_LINE_SIZE];
static uint16_t g_lineLength = 0U;
/* 上一个字节是不是 \r/\n，用于把 "\r\n" 视作一次换行 */
static uint8_t g_lastWasDelimiter = 0U;

/* 环形缓冲区下标 +1，溢出时回到 0 */
/* 环形缓冲区始终空出一个槽位：
 * head 的下一个位置等于 tail 表示已满，head == tail 表示为空。 */
static uint16_t next_index(uint16_t index, uint16_t size)
{
    index++;
    return (index >= size) ? 0U : index;
}

/* 把一字节写入 TX 环形缓冲区，缓冲区满时直接丢弃（避免阻塞） */
static void tx_push(uint8_t value)
{
    uint16_t next = next_index(g_txHead, SERIAL_TX_BUFFER_SIZE);

    if (next == g_txTail) {
        /* 日志拥塞时宁可丢字节，也不能等待串口并拖慢闭环控制。 */
        return;
    }

    g_txBuffer[g_txHead] = value;
    g_txHead = next;
}

void SERIAL_Init(void)
{
    /* 清空所有缓冲区状态 */
    g_rxHead = 0U;
    g_rxTail = 0U;
    g_txHead = 0U;
    g_txTail = 0U;
    g_lineLength = 0U;
    g_lastWasDelimiter = 0U;

    /* 打开 UART 接收中断 */
    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
}

void SERIAL_Task(void)
{
    /* 在 FIFO 没满的情况下持续搬运 TX 数据，调用频率越高日志越流畅 */
    while ((g_txTail != g_txHead) &&
           (!DL_UART_Main_isTXFIFOFull(UART_0_INST))) {
        DL_UART_Main_transmitData(UART_0_INST, g_txBuffer[g_txTail]);
        g_txTail = next_index(g_txTail, SERIAL_TX_BUFFER_SIZE);
    }
}

uint8_t SERIAL_ReadLine(char *line, uint16_t capacity)
{
    if ((line == 0) || (capacity < 2U)) {
        return 0U;
    }

    /* 一直从 RX 环形缓冲区里取字节，直到取到换行或队列空 */
    while (g_rxTail != g_rxHead) {
        uint8_t value = g_rxBuffer[g_rxTail];
        g_rxTail = next_index(g_rxTail, SERIAL_RX_BUFFER_SIZE);

        if ((value == '\r') || (value == '\n')) {
            /* 把连续的 \r/\n 视作一次分隔，避免空行重复 */
            if (g_lastWasDelimiter != 0U) {
                continue;
            }

            g_lastWasDelimiter = 1U;
            if (g_lineLength != 0U) {
                uint16_t i;
                uint16_t copyLength = g_lineLength;

                /* 拷贝时不要超过调用方提供的容量 */
                if (copyLength >= capacity) {
                    copyLength = (uint16_t)(capacity - 1U);
                }

                for (i = 0U; i < copyLength; i++) {
                    line[i] = g_lineBuffer[i];
                }
                line[copyLength] = '\0';
                g_lineLength = 0U;
                return 1U;
            }
        } else {
            g_lastWasDelimiter = 0U;
            if (g_lineLength < (SERIAL_LINE_SIZE - 1U)) {
                g_lineBuffer[g_lineLength] = (char)value;
                g_lineLength++;
            } else {
                /* 行太长：直接丢弃，重新开始累积，防止缓冲区卡死 */
                g_lineLength = 0U;
            }
        }
    }

    return 0U;
}

void SERIAL_SendString(const char *text)
{
    if (text == 0) {
        return;
    }

    while (*text != '\0') {
        tx_push((uint8_t)*text);
        text++;
    }
}

void SERIAL_SendInt32(int32_t value)
{
    char digits[12];
    uint8_t count = 0U;
    uint32_t magnitude;

    /* 处理负数：先输出 '-'，再把补码转换回正数绝对值 */
    if (value < 0) {
        tx_push((uint8_t)'-');
        magnitude = (uint32_t)(-(value + 1));
        magnitude++;
    } else {
        magnitude = (uint32_t)value;
    }

    /* 先低位再高位写入临时数组，再反向发送 */
    do {
        digits[count] = (char)('0' + (magnitude % 10U));
        magnitude /= 10U;
        count++;
    } while ((magnitude != 0U) && (count < sizeof(digits)));

    while (count != 0U) {
        count--;
        tx_push((uint8_t)digits[count]);
    }
}

void UART_0_INST_IRQHandler(void)
{
    /* 中断中不解析命令，只做最小量的数据搬运。 */
    switch (DL_UART_Main_getPendingInterrupt(UART_0_INST)) {
    case DL_UART_MAIN_IIDX_RX:
    {
        uint16_t next;
        uint8_t value = DL_UART_Main_receiveData(UART_0_INST);

        /* 把收到的字节塞进 RX 环形缓冲区，满了就丢弃 */
        next = next_index(g_rxHead, SERIAL_RX_BUFFER_SIZE);
        if (next != g_rxTail) {
            g_rxBuffer[g_rxHead] = value;
            g_rxHead = next;
        }
        break;
    }
    default:
        break;
    }
}
