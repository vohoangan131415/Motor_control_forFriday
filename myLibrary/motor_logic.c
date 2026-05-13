#include "motor_logic.h"
void Motor_handle()
{
	if (is_waiting == 1) 
    {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
        return; // Thoát luôn, không ch?y xu?ng switch-case bên du?i
    }
	switch(current_motor_status)
	{
		
		case MOTOR_SPEED_UP:
			Motor_Forward(current_speed);
			break;
		case MOTOR_SPEED_DOWN:
			Motor_Backward(current_speed);
			break;
		case MOTOR_STOP:
			Motor_Stop();
			break;
		default:
			break;
	}
	//current_motor_status = MOTOR_NONE;
}
