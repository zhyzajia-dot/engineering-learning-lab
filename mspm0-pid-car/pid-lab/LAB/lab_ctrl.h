/*
 * lab_ctrl.h - 上位机控制 & 闭环算法统一入口
 *
 * 模块职责：
 *   1. 串口命令解析（PWM / STEP / RUN / LINE / SQUARE / SET / SAVE / LOAD ...）
 *   2. SW1/SW2 选圈、SW3 启停与 ARM 启动
 *   3. 车轮 PI 闭环控制
 *   4. 直线同步外环 / 循迹 PD 外环
 *   5. 逆时针正方形跑图状态机
 *   6. 参数 Flash 持久化
 *   7. 实时数据流式日志与离线日志重发
 */

#ifndef LAB_CTRL_H
#define LAB_CTRL_H

#include <stdint.h>

/*
 * 参数持久化说明：
 *   SAVE 写入 Flash V2，并校验轮速、循迹和正方形转弯的全部参数；
 *   LOAD 仍兼容早期 V1，下一次 SAVE 会自动迁移到 V2。
 * 本模块是控制策略唯一入口，其他模块不应直接修改这些运行参数。
 */

/* 初始化参数、加载 Flash 默认值、打印欢迎信息 */
void LAB_Init(void);

/* 主循环里周期调用：解析串口 / 处理按键 / 执行闭环控制 / 推送日志 */
void LAB_Task(uint32_t nowMs);

#endif
