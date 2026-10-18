/**
 * @file    usart.h
 * @brief   调试串口(USART6)模块头文件
 *
 * 说明:
 *   - printf 的重定向(fputc -> USART6)已经在 library/atom/usart/usart.c 中完成,
 *     因此本模块只需把 USART6 外设初始化好, 之后即可直接使用 printf 输出。
 *   - USART6 引脚分配: TX -> PG9, RX -> PG14(复用功能 AF_USART6, 时钟线 APB2)。
 */

#ifndef __MDK_USART_H
#define __MDK_USART_H

/**
 * @brief  初始化调试串口 USART6
 *
 * @note   在 main 初始化阶段调用一次后, 即可在任意位置使用 printf()
 *         从 USART6 输出调试信息。
 *
 * @note   函数名故意不叫 USART_Init: ST 标准库 stm32f4xx_usart.h 里已经有
 *         "void USART_Init(USART_TypeDef*, USART_InitTypeDef*)",
 *         同名会报 #147-D: declaration is incompatible。
 */
void Debug_USART_Init(void);

#endif /* __MDK_USART_H */