#define __HANDLE_GLOBALS

#include "config.h"
#include "macro.h"
#include "handle.h"
#include "FreeRTOS.h"
#include "task.h"
#include "tasks.h"

int main(void) {
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    
    Delay_Init(180);
    Motor_Init(&Steering_Wheel,6,1,1,1);
    //zerooffset未设置
    Motor_Set_Position_Bias(&Steering_Wheel,0);
    LPF_Init(&Steering_Wheel_Filter,0.15f);
    
    BSP_CAN_Init();

    //创建SteeringWheel任务
    xTaskCreate(
        Task_SteeringWheel,
        "Task_SteeringWheel",
        256,
        NULL,
        5,
        &SteeringWheelTask_Handler
    );

    vTaskStartScheduler();

    while (1){
		
    }
    
}
