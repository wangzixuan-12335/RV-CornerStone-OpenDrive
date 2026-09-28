#include "Driver_Motor.h"
#include "macro.h"

#define PI 3.1415926

void Motor_Init(Motor_Type *motor, float reductionRate, int8_t angleEnabled, int8_t inputEnabled,int8_t direction) {
    motor->positionBias  = -1; // -1为未赋值状态
    motor->angleBias     = -1; // -1为未赋值状态
    motor->lastPosition  = -1; // -1为未赋值状态
    motor->reductionRate = reductionRate;
    motor->angleEnabled  = angleEnabled;
    motor->inputEnabled  = inputEnabled;
    motor->direction     = direction;
}

//修改过，转矩与电流计算仅适配6020
void Motor_Update(Motor_Type *motor, uint8_t data[8], uint8_t type) {
    int16_t position;
    int16_t speed;
    int16_t actualCurrent;
    int16_t temperature;
    float angle;
    switch (type)
    {
    //DJI
    case 1:
        position      = data[0] << 8 | data[1];
        speed         = data[2] << 8 | data[3];
        actualCurrent = data[4] << 8 | data[5];
        temperature   = data[6];
        angle         = 0;
        motor->updatedAt      = xTaskGetTickCount();
        motor->online         = 1;

        // 更新转子信息
        motor->position      = position;
        motor->speed         = speed * motor->direction / motor->reductionRate;
        motor->actualCurrent = (actualCurrent / 16384.0) * 3.0 * motor->direction;
        // 基于官方 I-T 曲线连续性的 GM6020 转矩计算
        float calc_current = motor->actualCurrent; // 已经带符号 (+/ -)

        if (calc_current > 0.325f) { // 超过正向克服摩擦电流门槛 (0.241 / 0.741 ≈ 0.325A)
            motor->torque = (0.741f * calc_current - 0.241f) * motor->reductionRate;
        } else if (calc_current < -0.325f) { // 超过反向克服摩擦电流门槛
            motor->torque = (0.741f * calc_current + 0.241f) * motor->reductionRate;
        } else {
            // 在静摩擦死区范围内，输出力矩归零（平滑过原点，不发生突变）
            motor->torque = 0.0f;
        }
        motor->temperature   = temperature;

        //如果启用了连续角度计算
        if (motor->angleEnabled) {
            if (motor->lastPosition != -1) {
                //两次编码器的反馈值差别太大,表示圈数发生了改变
                motor->positionDiff = motor->position - motor->lastPosition;
                if (motor->positionDiff < -4096) {
                    motor->round++;
                } else if (motor->positionDiff > 4096) {
                    motor->round--;
                }
            }
            //计算得到连续角度值,范围正负无穷大
            angle = motor->direction * (motor->position/8192.0f*360.0f + motor->round*360.0f) / motor->reductionRate;

            // 更新连续角度初始位置
            if (motor->angleBiasInit == 0) {
                if (motor->positionBias != -1) {
                    motor->angleBias = motor->positionBias / 8192.0f * 360; // 向下兼容
                } else {
                    motor->angleBias = angle; // 将当前位置设为
                }
                motor->angleBiasInit = 1;
            }
            motor->angle = angle - motor->angleBias;

            // 更新前转子位置
            motor->lastPosition = motor->position;
        }
        break;

    //LK
    case 2:
        temperature = data[1];
        actualCurrent = data[2] | data[3] << 8;
        speed = data[4] | data[5] << 8;
        position = data[6] | data[7] << 8;
        angle         = 0;
        motor->updatedAt      = xTaskGetTickCount();
        motor->online         = 1;

        // 更新转子信息
        motor->position      = position;
        motor->speed         = speed / 60.0; //LK是dps输出，统一到rpm；
        motor->actualCurrent = actualCurrent; //MS系列没有电流反馈，以输出功率替代；
        motor->torque        = 0; //无法计算
        motor->temperature   = temperature;

        if (motor->angleEnabled) {
            if (motor->lastPosition != -1) {
                //两次编码器的反馈值差别太大,表示圈数发生了改变
                motor->positionDiff = motor->position - motor->lastPosition;
                if (motor->positionDiff < -32768) {
                    motor->round++;
                } else if (motor->positionDiff > 32768) {
                    motor->round--;
                }
            }
            //计算得到连续角度值,范围正负无穷大
            angle = (motor->position / 65536.0f * 360 + motor->round * 360) / motor->reductionRate;

            // 更新连续角度初始位置
            if (motor->angleBiasInit == 0) {
                if (motor->positionBias != -1) {
                    motor->angleBias = motor->direction * (motor->positionBias / 8192.0f * 360.0f) / motor->reductionRate; // 向下兼容
                } else {
                    motor->angleBias = angle; // 将当前位置设为
                }
                motor->angleBiasInit = 1;
            }
            motor->angle = angle - motor->angleBias;

            // 更新前转子位置
            motor->lastPosition = motor->position;
        }
    break;

    default:
    break;
}
}

void Motor_Set_Angle_Bias(Motor_Type *motor, float angleBias) {
    motor->angleBias     = angleBias;
    motor->angleBiasInit = 1;
    motor->lastPosition  = -1;
}

void Motor_Set_Position_Bias(Motor_Type *motor, float positionBias) {
    motor->positionBias     = positionBias;
    motor->angleBiasInit = 0;
    motor->lastPosition  = -1;
}