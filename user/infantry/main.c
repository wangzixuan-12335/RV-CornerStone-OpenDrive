#define __HANDLE_GLOBALS

#include "config.h"
#include "macro.h"
#include "handle.h"
#include "FreeRTOS.h"
#include "task.h"
#include "tasks.h"
#include "usart.h"      // 调试串口模块(USART6)
#include "wheel_comm.h" // 上位机通信模块(UART7)

int main(void) {
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
   
    Delay_Init(180);
    Motor_Init(&Steering_Wheel,6,1,1,1);
    //zerooffset未设置
    Motor_Set_Position_Bias(&Steering_Wheel,0);
    LPF_Init(&Steering_Wheel_Filter,0.15f); // 力矩低通系数: 越小越顺滑但越滞后, 想更跟手就往 1.0 调
    
    BSP_CAN_Init();
    Debug_USART_Init(); // 初始化调试串口 USART6, 之后可使用 printf() 输出调试信息
    WheelComm_Init(); // 初始化与上位机的通信口 UART7(16 字节定长帧 + CRC16)

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
