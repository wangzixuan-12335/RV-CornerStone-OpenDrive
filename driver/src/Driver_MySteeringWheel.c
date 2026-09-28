#include "handle.h"

SoftLimit_Controller_t g_soft_limit = {
    .max_angle_deg = 450.0f, // 左右各 450 度
    .K_wall = 0.20f,         // 刚度：超界 1 度产生 0.2 N·m 反力
    .B_wall = 0.08f          // 墙壁阻尼：防止撞墙发抖
};

/**
 * @brief 初始化低通滤波器
 * @param lpf 滤波器结构体指针
 * @param alpha 滤波系数 (例如 0.15)
 */
void LPF_Init(LowPassFilter_t *lpf, float alpha) {
    lpf->alpha = alpha;
    lpf->last_output = 0.0f;
}

/**
 * @brief 执行低通滤波计算
 * @param lpf 滤波器结构体指针
 * @param raw_input 当前时刻的原始输入值 X(n)
 * @return float 滤波后的平滑输出值 Y(n)
 */
float LPF_Update(LowPassFilter_t *lpf, float raw_input) {
    // Y(n) = (1 - alpha) * Y(n-1) + alpha * X(n)
    lpf->last_output = (1.0f - lpf->alpha) * lpf->last_output + lpf->alpha * raw_input;
    return lpf->last_output;
}

float Proportional_Scaling_torque(float target_game_torque){
    float T_GameMax=10.0f;
    float T_WheelMax=5.0f;
    float K=T_WheelMax/T_GameMax;
    return (float)(target_game_torque*K);
}

float Damping_Torque(float speed){
    float DampingCoefficient=0.04;
    float w=speed*2.0f*PI/60;
    float damping_torque=-1.0f*DampingCoefficient*w;
    if (damping_torque > 1.5f)  damping_torque = 1.5f;
    if (damping_torque < -1.5f) damping_torque = -1.5f;
    return (damping_torque);
}

/**
 * @brief 软限位计算函数
 * @param raw_wheel_torque 当前经过缩放和阻尼后的目标盘面力矩 (N·m)
 * @param motor_angle_deg 电机反馈的连续角度 (单位：度)
 * @param motor_rpm 电机反馈的转速 (RPM)
 * @return float 叠加软限位反力后的盘面目标力矩 (N·m)
 */
float Apply_Wheel_Soft_Limits(float raw_wheel_torque, float motor_angle_deg, float motor_rpm) {
    // 1. 计算盘面实际连续角度与转速 (1:6 减速比)
    float wheel_angle_deg = motor_angle_deg;
    float wheel_rpm       = motor_rpm;
    float wheel_omega     = wheel_rpm * 0.10472f; // 转为 rad/s

    float wall_torque = 0.0f; // 软限位产生的额外反力矩

    // 2. 右转超界判断 (超过 +450 度)
    if (wheel_angle_deg > g_soft_limit.max_angle_deg) {
        float delta_angle = wheel_angle_deg - g_soft_limit.max_angle_deg; // 越界度数 (> 0)

        // 产生向左(负方向)的拉力：- (刚度反力 + 墙壁阻尼)
        wall_torque = - (g_soft_limit.K_wall * delta_angle + g_soft_limit.B_wall * wheel_omega);
        
        // 【核心体验优化】：如果手正在往回（向左）往安全区域拉，取消墙壁阻尼，避免往回拉时有“卡壳感”
        if (wheel_omega < 0.0f) {
            wall_torque = - (g_soft_limit.K_wall * delta_angle);
        }
    } 
    // 3. 左转超界判断 (低于 -450 度)
    else if (wheel_angle_deg < -g_soft_limit.max_angle_deg) {
        float delta_angle = wheel_angle_deg - (-g_soft_limit.max_angle_deg); // 越界度数 (< 0)

        // 产生向右(正方向)的拉力
        wall_torque = - (g_soft_limit.K_wall * delta_angle + g_soft_limit.B_wall * wheel_omega);

        // 如果手正在往回（向右）往安全区域拉，取消墙壁阻尼
        if (wheel_omega > 0.0f) {
            wall_torque = - (g_soft_limit.K_wall * delta_angle);
        }
    }

    // 4. 将软限位力矩叠加到游戏原始力矩上
    float final_torque = raw_wheel_torque + wall_torque;

    return final_torque;
}


/**
 * @brief 动态双向死区前馈补偿（左右绝对对称公平）
 * @param target_motor_torque 经过阻尼和滤波后的目标电机轴扭矩
 * @return float 叠加死区前馈后的电机轴扭矩
 */
float Apply_Gearbox_Deadzone(float target_motor_torque) {
    // 实测得到的正向与反向死区电流对应的电机扭矩 (N·m)
    const float MOTOR_DEADZONE_POS =  0.05f; // 正向(CCW)需叠加 +0.35
    const float MOTOR_DEADZONE_NEG = -0.08f; // 反向(CW)需叠加  -0.38 (注意是负数)
    
    // 零点保护带：当期望力矩极小时不给偏置，防止静止时高频发震或啸叫
    const float Epsilon_door = 0.005f; 

    if (target_motor_torque > Epsilon_door) {
        // 【向左用力时】：给正向偏置，帮助向左克服克服静摩擦
        return target_motor_torque + MOTOR_DEADZONE_POS;
        
    } else if (target_motor_torque < -Epsilon_door) {
        // 【向右用力时】：给负向偏置，帮助向右克服静摩擦
        return target_motor_torque + MOTOR_DEADZONE_NEG;
        
    } else {
        // 【没有用力时】：不给偏置，保持绝对静止
        return 0.0f;
    }
}

int16_t Wheel_Torque_To_CAN(Motor_Type *motor,float target_wheel_torque) {
    const float REDUCTION_RATIO = 6.0f;  // 减速比 1:6
    const float WHEEL_MAX_TORQUE = 5.0f; // 盘面最大手感力矩 (N·m)
    const float KT = 0.741f;             // GM6020 扭矩常数
    const float MAX_CURRENT_A = 3.0f;

    // 1. 盘面手感软限幅
    if (target_wheel_torque > WHEEL_MAX_TORQUE)  target_wheel_torque = WHEEL_MAX_TORQUE;
    if (target_wheel_torque < -WHEEL_MAX_TORQUE) target_wheel_torque = -WHEEL_MAX_TORQUE;

    // 2. 折算到电机轴的期望力矩
    float target_motor_torque = target_wheel_torque / REDUCTION_RATIO;

    // 3. 叠加电机+减速器的静摩擦死区前馈
    target_motor_torque = Apply_Gearbox_Deadzone(target_motor_torque);

    // 4. 换算为电机电流 (A) 并转换为 CAN 发送值 (-16384 ~ 16384)
    float current_A = target_motor_torque / KT;
    int32_t can_value = (int32_t)(current_A * (16384.0f / MAX_CURRENT_A));

    // 5. 驱动器绝对硬件安全限幅
    if (can_value > 16384)  can_value = 16384;
    if (can_value < -16384) can_value = -16384;

    motor->input=can_value*motor->direction;
    return (int16_t)(can_value*motor->direction);
}

void General_Control(Motor_Type *motor,float target_torque){
    
    float controlTorque=Proportional_Scaling_torque(target_torque);
    controlTorque=LPF_Update(&Steering_Wheel_Filter,controlTorque);
    controlTorque+=Damping_Torque(motor->speed);
    controlTorque=Apply_Wheel_Soft_Limits(controlTorque,motor->angle,motor->speed);
    Wheel_Torque_To_CAN(motor,controlTorque);
}

void SteeringWheel_Motor_Send(Motor_Type *motor,CAN_TypeDef *CANx){
    uint8_t offset = (STEERINGWHEEL_CAN_Rx-0x205)*2;
    uint8_t buf[8]={0};
    buf[offset]=motor->input >> 8;
    buf[offset+1]=motor->input & 0x00FF;
    Can_Send_Msg(CANx,STEERINGWHEEL_CAN_Tx,buf,8);
}