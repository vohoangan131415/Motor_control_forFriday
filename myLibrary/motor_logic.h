#ifndef MOTOR_LOGIC_H
#define MOTOR_LOGIC_H
#include "motor.h"
#include "stm32f1xx.h"
extern TIM_HandleTypeDef htim2; // L?y timer t? main.c
typedef enum {
    MOTOR_NONE = 0,
    MOTOR_DIRECTION_CHANGE,
    MOTOR_SPEED_UP, 
    MOTOR_SPEED_DOWN,
    MOTOR_STOP, 
		
} MOTOR_STATUS;
extern uint8_t is_waiting;

// Khai báo extern d? main.c và các file khác có th? dùng chung
extern volatile uint8_t current_motor_status;
extern uint8_t direction ;
extern uint16_t current_speed;
void Motor_handle(void);

#endif