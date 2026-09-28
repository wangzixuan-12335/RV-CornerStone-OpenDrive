/* @brief 任务*/
#include "tasks.h"
#include "config.h"
#include "macro.h"
#include "handle.h"
#include "math.h"

void Task_SteeringWheel(void *Parameters){
    TickType_t xLastWakeTime=xTaskGetTickCount();
    const TickType_t xFrequency=pdMS_TO_TICKS(1);

    while (1)
    {
        General_Control(&Steering_Wheel,0.0f);
        SteeringWheel_Motor_Send(&Steering_Wheel,CAN1);
        vTaskDelayUntil(&xLastWakeTime,xFrequency);
    }
}