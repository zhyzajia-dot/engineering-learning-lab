/*
 * 文件：ui.h
 * 用途：声明独立小车的按键交互接口。
 * 说明：三个物理按键当前使用相同语义；1ms扫描只产生动作，真正的
 *       启动、停止和圈数切换由主循环中的 UI_Process() 执行。
 */

#ifndef UI_H
#define UI_H

/* 清空消抖、长按和待处理动作状态。 */
void UI_Init(void);
/* 1ms周期调用；建议只在定时中断中调用。 */
void UI_Task1ms(void);
/* 主循环高频调用，执行中断侧已经确认的按键动作。 */
void UI_Process(void);

#endif
