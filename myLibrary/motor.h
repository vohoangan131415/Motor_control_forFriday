#ifndef MOTOR_H
#define MOTOR_H
#include "main.h"

// Giữ nguyên define của Ân
#define R_EN GPIO_PIN_14
#define L_EN GPIO_PIN_15
#define EN_PORT GPIOB

void Init_Motor(void);
void Motor_Forward(uint16_t speed);
void Motor_Backward(uint16_t speed);
void Motor_Stop(void);

#endif