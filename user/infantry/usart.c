/**
 * @file    usart.c
 * @brief   调试串口(USART6)模块实现
 *
 * 说明:
 *   1. printf 重定向(fputc -> USART6)已经在 library/atom/usart/usart.c 中完成,
 *      该文件已加入 Keil 工程。因此本文件绝不能再定义 fputc,
 *      否则链接时报错(L6200E: 符号重复定义)。
 *   2. 本文件负责把 USART6 外设初始化好: 在 main 初始化阶段调用一次
 *      Debug_USART_Init(), 之后任意位置调用 printf() 即可从 USART6 的 TX(PG9) 输出。
 *   3. USART6 引脚分配(由 BSP_USART6_Init 实现):
 *          TX -> PG9  (复用 AF_USART6)
 *          RX -> PG14 (复用 AF_USART6)
 *      外设时钟线: APB2
 */

#include "usart.h"
#include "Driver_BSP.h"   // 提供 BSP_USART6_Init() 与 USART_IT_IDLE 等串口相关定义

/**
 * @brief  初始化调试串口 USART6
 *
 * @note   BSP_USART6_Init(baudRate, interruptFlag) 内部完成:
 *          - GPIO 复用配置(PG9/PG14 -> AF_USART6), 上拉、推挽、50MHz
 *          - 使能 GPIO 与 USART6 外设时钟(APB2)
 *          - 配置: 波特率 / 8位数据 / 1停止位 / 无校验 / 无流控 / 收发模式
 *          - 使能 USART 外设
 *          - interruptFlag 非 0 时: 配置 NVIC 并使能对应中断
 *
 * @note   中断标志(interruptFlag)可选值:
 *          0             -> 不开启中断(纯调试输出用)
 *          USART_IT_RXNE -> 接收中断(每收到一个字节触发一次)
 *          USART_IT_IDLE -> 空闲中断(收完一帧数据后触发)
 *
 * @note   开启 USART_IT_IDLE 后, 收到的数据会进入 interrupt.c 中的
 *         USART6_IRQHandler, 其内部调用 Bridge_Receive_USART() 将数据按
 *         裁判系统协议解析。若仅想用 printf 打印调试信息、不接收数据,
 *         请把下面的中断参数改为 0。
 * @note   函数名不能叫 USART_Init: ST 标准库 stm32f4xx_usart.h 里已有同名函数,
 *         同名会报 #147-D: declaration is incompatible。
 */
void Debug_USART_Init(void) {
    // 调试串口: 波特率 115200, 开启空闲中断(与工程原有裁判系统接收逻辑一致)
    // 若只用于 printf 调试输出, 可改为: BSP_USART6_Init(115200, 0);
    BSP_USART6_Init(115200, USART_IT_IDLE);
}

