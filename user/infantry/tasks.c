/* @brief 任务*/
#include "tasks.h"
#include "config.h"
#include "macro.h"
#include "handle.h"
#include "math.h"
#include "wheel_comm.h"

/**
 * @brief 方向盘主任务, 1ms 一个周期
 * @note  流程: 通信心跳轮询 -> 取上位机目标力矩 -> 力矩合成 -> CAN 下发 -> 回状态帧
 *        全程非阻塞(串口收发都是轮询 + 中断), 不会把 1ms 周期拖长
 */
void Task_SteeringWheel(void *Parameters){
    TickType_t xLastWakeTime=xTaskGetTickCount();
    const TickType_t xFrequency=pdMS_TO_TICKS(1);
    float target_torque;

    while (1)
    {
        // 1) 通信心跳: 超时/未连接会自动让下面的力矩变成 0 (电机自由滑行)
        WheelComm_Poll();

        // 2) 上位机下发的盘面目标力矩, 单位 N·m
        target_torque=WheelComm_Get_TargetTorque();

        // 3) 力矩合成: 低通 -> 动态阻尼 -> 软限位 -> 死区前馈 -> CAN 电流值
        General_Control(&Steering_Wheel,target_torque);

        // 4) 通过 CAN1 发给 GM6020
        SteeringWheel_Motor_Send(&Steering_Wheel,CAN1);

        // 5) 回一帧状态给上位机
        //    Motor_Update 里 speed 已折算到主轴(rpm), ×6 换算成 °/s
        WheelComm_SendStatus(Steering_Wheel.angle,
                             (float)Steering_Wheel.speed*6.0f,
                             (float)Steering_Wheel.temperature,
                             WheelComm_Is_Online()?0:1,
                             g_soft_limit_active);

        vTaskDelayUntil(&xLastWakeTime,xFrequency);
    }
}