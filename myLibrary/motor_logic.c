#include "motor_logic.h"

void Motor_handle()
{
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
