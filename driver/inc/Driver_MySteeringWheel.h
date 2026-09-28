#ifndef __MY_STEERINGWHEEL_H
#define __MY_STEERINGWHEEL_H

// 定义低通滤波器结构体
typedef struct {
    float alpha;       // 滤波系数 (0.0 ~ 1.0)
    float last_output; // 上一次的滤波结果 Y(n-1)
} LowPassFilter_t;

typedef struct {
    float max_angle_deg;   // 盘面单边最大角度 (例如 450.0 度)
    float K_wall;          // 撞墙刚度 (N·m / deg)
    float B_wall;          // 撞墙阻尼 (N·m / (rad/s))
} SoftLimit_Controller_t;

void LPF_Init(LowPassFilter_t *lpf, float alpha);
float LPF_Update(LowPassFilter_t *lpf, float raw_input);
float Proportional_Scaling_torque(float target_game_torque);
float Damping_Torque(float speed);
float Apply_Wheel_Soft_Limits(float raw_wheel_torque, float motor_angle_deg, float motor_rpm);
float Apply_Gearbox_Deadzone(float target_motor_torque);
int16_t Wheel_Torque_To_CAN(Motor_Type *motor,float target_wheel_torque);
void General_Control(Motor_Type *motor,float target_torque);
void SteeringWheel_Motor_Send(Motor_Type *motor,CAN_TypeDef *CANx);


#endif
