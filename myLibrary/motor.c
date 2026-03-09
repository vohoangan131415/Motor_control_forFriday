#include "motor.h"
extern TIM_HandleTypeDef htim2; // Lấy timer từ main.c

void Init_Motor() {
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
}

void Motor_Forward(uint16_t speed) {
    HAL_GPIO_WritePin(EN_PORT, R_EN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(EN_PORT, L_EN, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, speed);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
}

void Motor_Backward(uint16_t speed) {
    HAL_GPIO_WritePin(EN_PORT, R_EN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(EN_PORT, L_EN, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, speed);
}

void Motor_Stop() {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
}