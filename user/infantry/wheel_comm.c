/**
 * @file    wheel_comm.c
 * @brief   方向盘上位机通信(UART7)的实现 —— 收目标力矩, 回状态帧
 *
 * 收帧方式: 每个字节进一次接收中断(RXNE), 用一个小状态机找帧头 0xAA 0x55,
 * 收满 16 字节后算 CRC 校验, 过了才算一帧。
 * 之所以不用"空闲中断(IDLE)整帧收": 上位机是 1ms 一帧连续发的,
 * 靠"线上空闲"判断帧结束不可靠, 而且丢一个字节整帧就错位;
 * 现在靠定长 + CRC 对齐, 丢字节最多损失一帧, 下一帧自动重新同步。
 */

#include "wheel_comm.h"
#include "Driver_BSP.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>

/* 下行帧里要用到的字段偏移(与 wheel_comm.h 的表逐字节对应) */
#define DOWN_OFF_TORQUE 2  /* [2-3] target_torque */
#define DOWN_OFF_CRC    14 /* [14-15] crc16 */

/* ---------------- 接收侧状态 ---------------- */
static uint8_t s_rx_buf[WHEEL_COMM_FRAME_SIZE]; /* 组帧缓冲 */
static uint8_t s_rx_len;                        /* 0 = 正在找帧头 */

/* 中断里从合法帧里取出来的数据(volatile: 中断写, 任务读) */
static volatile uint8_t  s_rx_new;           /* 1 = 本周期收到过合法帧 */
static volatile int16_t  s_target_torque_raw;/* target_torque 原始值 */
static volatile uint8_t  s_damper_gain;      /* 缓存, 暂未使用 */
static volatile uint8_t  s_friction_gain;    /* 缓存, 暂未使用 */
static volatile uint16_t s_soft_stop_limit;  /* 缓存, 暂未使用 */
static volatile uint8_t  s_control_flags;    /* 缓存, 暂未使用 */
static volatile uint8_t  s_seq_num;          /* 上位机帧序号 */
static volatile uint32_t s_crc_err_count;    /* CRC 失败计数(诊断用) */

/* ---------------- 链路状态(只在任务里动) ---------------- */
static uint8_t    s_online;        /* 1 = 上位机在正常发帧 */
static TickType_t s_last_rx_tick;  /* 最近一次收到合法帧的时刻 */

/**
 * @brief CRC16-CCITT (poly 0x1021, init 0xFFFF, 不反转)
 * @note  与上位机 task_Serial.c 里的查表法逐位等价(已验证), 这里用逐位算法省 512 字节表
 */
static uint16_t WheelComm_CRC16(const uint8_t *data, uint16_t length) {
    uint16_t crc = 0xFFFF;
    uint8_t  i;

    while (length--) {
        crc ^= (uint16_t)(*data++) << 8;
        for (i = 0; i < 8; i++) {
            if (crc & 0x8000) {
                crc = (uint16_t)((crc << 1) ^ 0x1021);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

void WheelComm_Init(void) {
    uint8_t i;

    for (i = 0; i < WHEEL_COMM_FRAME_SIZE; i++) {
        s_rx_buf[i] = 0;
    }
    s_rx_len            = 0;
    s_rx_new            = 0;
    s_target_torque_raw = 0;
    s_damper_gain       = 0;
    s_friction_gain     = 0;
    s_soft_stop_limit   = 0;
    s_control_flags     = 0;
    s_seq_num           = 0;
    s_crc_err_count     = 0;
    s_online            = 0;
    s_last_rx_tick      = 0;

    /* 只开 RXNE(收一个字节进一次中断), NVIC 优先级 2 */
    BSP_UART7_Init(WHEEL_COMM_BAUD, USART_IT_RXNE);

    printf("[WHEEL] UART7 ready: %u bps, 16B frame, CRC16-CCITT\r\n",
           (unsigned)WHEEL_COMM_BAUD);
}

void WheelComm_RxHandler(void) {
    uint8_t  byte;
    uint16_t crc;

    if (USART_GetITStatus(UART7, USART_IT_RXNE) == RESET) {
        return;
    }
    byte = (uint8_t)USART_ReceiveData(UART7); /* 读 DR 同时清 RXNE 标志 */

    if (s_rx_len == 0) { /* 第 1 个字节: 必须是 0xAA */
        if (byte == WHEEL_COMM_HEAD_DOWN_0) {
            s_rx_buf[0] = byte;
            s_rx_len    = 1;
        }
        return;
    }

    if (s_rx_len == 1) { /* 第 2 个字节: 必须是 0x55 */
        if (byte == WHEEL_COMM_HEAD_DOWN_1) {
            s_rx_buf[1] = byte;
            s_rx_len    = 2;
        } else if (byte != WHEEL_COMM_HEAD_DOWN_0) {
            s_rx_len = 0; /* 不是 0x55 也不是 0xAA, 重新找; 是 0xAA 就当新的帧头第 1 字节 */
        }
        return;
    }

    s_rx_buf[s_rx_len++] = byte; /* 第 3~16 个字节: 直接收 */
    if (s_rx_len < WHEEL_COMM_FRAME_SIZE) {
        return;
    }
    s_rx_len = 0; /* 一帧收满, 不管对错都重新开始找帧头 */

    crc = (uint16_t)s_rx_buf[14] | ((uint16_t)s_rx_buf[15] << 8);
    if (WheelComm_CRC16(s_rx_buf, DOWN_OFF_CRC) != crc) {
        s_crc_err_count++;
        return; /* 校验不过: 整帧丢掉, 等下一帧 */
    }

    /* 校验通过。先填数据, 最后才置 s_rx_new, 保证任务侧看到标志时数据已经完整 */
    s_target_torque_raw = (int16_t)((uint16_t)s_rx_buf[DOWN_OFF_TORQUE] |
                                    ((uint16_t)s_rx_buf[DOWN_OFF_TORQUE + 1] << 8));
    s_damper_gain       = s_rx_buf[4];
    s_friction_gain     = s_rx_buf[5];
    s_soft_stop_limit   = (uint16_t)s_rx_buf[6] | ((uint16_t)s_rx_buf[7] << 8);
    s_control_flags     = s_rx_buf[8];
    s_seq_num           = s_rx_buf[9];
    s_rx_new            = 1;
}

void WheelComm_Poll(void) {
    TickType_t now = xTaskGetTickCount();

    if (s_rx_new) { /* 这一毫秒收到了合法帧: 喂狗 */
        s_rx_new       = 0;
        s_last_rx_tick = now;
        if (!s_online) {
            s_online = 1;
            printf("[WHEEL] PC link up\r\n");
        }
        return;
    }

    if (s_online && (TickType_t)(now - s_last_rx_tick) >= WHEEL_COMM_TIMEOUT_MS) {
        s_online = 0; /* 熔断: 力矩归零, 电机切自由滑行 */
        printf("[WHEEL] PC link timeout > %ums, torque -> 0\r\n",
               (unsigned)WHEEL_COMM_TIMEOUT_MS);
    }
}

float WheelComm_Get_TargetTorque(void) {
    if (!s_online) {
        return 0.0f; /* 没连上 / 超时: 不给力 */
    }
    /* ±10000 -> ±7.2 N·m */
    return (float)s_target_torque_raw * (WHEEL_COMM_TORQUE_MAX_NM / WHEEL_COMM_TORQUE_FULL);
}

uint8_t WheelComm_Is_Online(void) {
    return s_online;
}

void WheelComm_SendStatus(float   wheel_angle_deg,
                          float   wheel_speed_dps,
                          float   motor_temp_c,
                          uint8_t fault,
                          uint8_t soft_limit_active) {
    uint8_t  buf[WHEEL_COMM_FRAME_SIZE];
    uint16_t crc;
    uint8_t  i;
    int32_t  angle = (int32_t)(wheel_angle_deg * 100.0f); /* 0.01° */
    int16_t  speed = (int16_t)(wheel_speed_dps * 10.0f);  /* 0.1°/s */
    float    temp_f;
    uint8_t  temp;

    temp_f = motor_temp_c;
    if (temp_f < 0.0f) {
        temp_f = 0.0f;
    }
    if (temp_f > 63.0f) {
        temp_f = 63.0f;
    }
    temp = (uint8_t)temp_f;

    buf[0] = WHEEL_COMM_HEAD_UP_0; /* [0-1] 帧头 0x55 0xAA */
    buf[1] = WHEEL_COMM_HEAD_UP_1;

    buf[2] = (uint8_t)((uint32_t)angle & 0xFFu); /* [2-5] wheel_angle, 小端 int32 */
    buf[3] = (uint8_t)(((uint32_t)angle >> 8) & 0xFFu);
    buf[4] = (uint8_t)(((uint32_t)angle >> 16) & 0xFFu);
    buf[5] = (uint8_t)(((uint32_t)angle >> 24) & 0xFFu);

    buf[6] = (uint8_t)((uint16_t)speed & 0xFFu); /* [6-7] wheel_speed, 小端 int16 */
    buf[7] = (uint8_t)(((uint16_t)speed >> 8) & 0xFFu);

    buf[8]  = 0; /* [8-9]  油门 ADC 未接, 暂发 0 */
    buf[9]  = 0;
    buf[10] = 0; /* [10-11] 刹车称重未接, 暂发 0 */
    buf[11] = 0;
    buf[12] = 0; /* [12] 拨片/按键未接, 暂发 0 */

    /* [13] bit0 故障 / bit1 软限位介入 / bit2~7 温度(0~63℃) */
    buf[13] = (uint8_t)((fault ? 1u : 0u) | (soft_limit_active ? 2u : 0u) | ((uint32_t)temp << 2));

    crc     = WheelComm_CRC16(buf, 14);
    buf[14] = (uint8_t)(crc & 0xFFu); /* [14-15] crc16, 小端 */
    buf[15] = (uint8_t)(crc >> 8);

    for (i = 0; i < WHEEL_COMM_FRAME_SIZE; i++) {
        while (USART_GetFlagStatus(UART7, USART_FLAG_TXE) == RESET) {
        }
        USART_SendData(UART7, buf[i]);
    }
}
