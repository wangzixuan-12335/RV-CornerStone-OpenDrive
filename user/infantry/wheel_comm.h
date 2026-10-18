/**
 * @file    wheel_comm.h
 * @brief   方向盘: 下位机(本板) <-> PC 上位机 串口通信  (UART7, PE7=TX / PE8=RX)
 *
 * 协议与上位机 opendrive-upper/tasks/task_Serial.h 逐字节对齐:
 *   - 1152000 bps, 8 数据位 / 1 停止位 / 无校验 / 无硬件流控
 *   - 双向都是 16 字节定长帧, 原生小端(低字节在前)
 *   - 下行(PC->板)帧头 0xAA 0x55 ; 上行(板->PC)帧头 0x55 0xAA (不可颠倒)
 *   - CRC16-CCITT: 多项式 0x1021, 初值 0xFFFF, 输入输出不反转, 只校验前 14 字节
 *
 * 下行控制帧 (PC -> 板, 16 字节):
 *   [0-1]  0xAA 0x55        帧头
 *   [2-3]  target_torque    ±10000 对应盘面 ±7.2 N·m (小端 int16)
 *   [4]    damper_gain      动态阻尼增益 (目前板子用固定系数, 收到仅缓存)
 *   [5]    friction_gain    摩擦补偿增益 (目前板子用固定系数, 收到仅缓存)
 *   [6-7]  soft_stop_limit  软限位单边阈值, 0.1° (目前板子用固定 450.0°)
 *   [8]    control_flags    bit0 电机使能 / bit1 触发寻零 / bit2 软限位 (收到仅缓存)
 *   [9]    seq_num          帧序号, 0~255 循环
 *   [10-13] reserved        预留
 *   [14-15] crc16           小端
 *
 * 上行状态帧 (板 -> PC, 16 字节):
 *   [0-1]  0x55 0xAA        帧头
 *   [2-5]  wheel_angle      主轴连续多圈角度, 0.01° (小端 int32)
 *   [6-7]  wheel_speed      主轴角速度, 0.1°/s (小端 int16)
 *   [8-9]  throttle_raw     油门 ADC (踏板未接, 暂发 0)
 *   [10-11] brake_load      刹车称重 (踏板未接, 暂发 0)
 *   [12]   button_states    拨片/按键 (未接, 暂发 0)
 *   [13]   motor_temp_status bit0 故障 / bit1 软限位介入 / bit2~7 温度(0~63℃)
 *   [14-15] crc16           小端
 *
 * @note    本模块只负责"和电脑说话", 不碰电机。
 *          电机出力在 Driver_MySteeringWheel.c(力矩合成) 与 Driver_CAN.c(CAN 发送) 里。
 */

#ifndef __WHEEL_COMM_H
#define __WHEEL_COMM_H

#include <stdint.h>

/**
 * 串口波特率。必须与上位机 task_Serial.c 里 init_serial_nonblocking() 的
 * dcb.BaudRate 一致。上位机现在是 1152000, 所以这里也用 1152000
 * (UART7 在 APB1=45MHz 下 BRR=0x271, 分频恰好整除, 无误差)。
 * 若上位机改成 921600, 这里同步改成 921600 即可。
 */
#define WHEEL_COMM_BAUD 1152000U

/**
 * 通信看门狗: 连续这么久没收到合法下行帧, 就认为上位机挂了/线掉了,
 * 目标力矩强制归零(电机自由滑行), 防止方向盘保持最后一帧的力疯转。
 */
#define WHEEL_COMM_TIMEOUT_MS 50U

#define WHEEL_COMM_FRAME_SIZE 16U

#define WHEEL_COMM_HEAD_DOWN_0 0xAAU /* PC -> 板 */
#define WHEEL_COMM_HEAD_DOWN_1 0x55U
#define WHEEL_COMM_HEAD_UP_0   0x55U /* 板 -> PC */
#define WHEEL_COMM_HEAD_UP_1   0xAAU

/* target_torque 满量程: ±10000 -> 盘面 ±7.2 N·m */
#define WHEEL_COMM_TORQUE_FULL   10000.0f
#define WHEEL_COMM_TORQUE_MAX_NM 7.2f

/* control_flags 位定义(上位机补全后才会用上) */
#define WHEEL_FLAG_OUTPUT_ENABLE (1U << 0) /* 1=电机使能 0=自由滑行 */
#define WHEEL_FLAG_HOMING        (1U << 1) /* 触发自动对中寻零 */
#define WHEEL_FLAG_SOFTSTOP_ON   (1U << 2) /* 虚拟软限位强制开启 */

/** @brief 初始化串口 UART7 与接收状态机(在 main 里调用一次) */
void WheelComm_Init(void);

/** @brief 串口接收中断处理: 在 interrupt.c 的 UART7_IRQHandler 里调用 */
void WheelComm_RxHandler(void);

/** @brief 通信心跳轮询, 在 1ms 任务里每周期调用一次(超时判定在这里做) */
void WheelComm_Poll(void);

/**
 * @brief 取上位机下发的盘面目标力矩
 * @return 单位 N·m; 未连接或通信超时返回 0
 */
float WheelComm_Get_TargetTorque(void);

/** @brief 上位机链路是否正常(在收帧) */
uint8_t WheelComm_Is_Online(void);

/**
 * @brief 回一帧上行状态帧给上位机(1ms 周期调用)
 * @param wheel_angle_deg    主轴连续多圈角度, 单位 度
 * @param wheel_speed_dps    主轴角速度, 单位 度/秒
 * @param motor_temp_c       电机温度, 单位 摄氏度
 * @param fault              故障标志, 0=正常 1=有故障
 * @param soft_limit_active  软限位是否正在介入, 0/1
 */
void WheelComm_SendStatus(float   wheel_angle_deg,
                          float   wheel_speed_dps,
                          float   motor_temp_c,
                          uint8_t fault,
                          uint8_t soft_limit_active);

#endif /* __WHEEL_COMM_H */
