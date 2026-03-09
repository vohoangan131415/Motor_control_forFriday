#include "motor_logic.h"
volatile uint8_t motor_speed_increase =0;
volatile uint8_t motor_speed_decrease =0;
volatile uint8_t current_motor_mode = MOTOR_MANUAL;
void Motor_handle()
{
	switch(current_motor_status)
	{
		case MOTOR_DIRECTION_CHANGE:
			direction *= -1;
			break;
		case MOTOR_SPEED_UP:
			motor_speed_increase = 1;
			motor_speed_decrease = 0;
			break;
		case MOTOR_SPEED_DOWN:
			motor_speed_increase = 0;
			motor_speed_decrease =1;
			break;
		case MOTOR_STOP:
			current_speed =0;
			motor_speed_increase = 0;
			motor_speed_decrease =0;
			if(current_motor_mode == MOTOR_MANUAL) current_motor_mode = MOTOR_AUTO;
      else current_motor_mode = MOTOR_MANUAL;
			break;
		default:
			break;
	}
	current_motor_status = MOTOR_NONE;
}
void Motor_Control()

{

	//Direction

	if(current_speed == 0)
	{
			Motor_Stop(); // Stop motor 
	}
	else 
	{
		if(direction == 1) Motor_Forward(current_speed); // notor CW
		else Motor_Backward(current_speed); // motor CWW
	}
	//Speed
	if(current_motor_mode == MOTOR_AUTO) // auto len toi max
	{
			if(motor_speed_increase == 1) // increase speed
			{
				if(current_speed < 99) 
				current_speed++;
				else motor_speed_increase = 0;
				HAL_Delay(30);
			}
		if(motor_speed_decrease ==1) // decrease speed
			{
				if (current_speed > 0) 
				current_speed--;
				else motor_speed_decrease =0;
				HAL_Delay(30);
			}
	}
	else   // giu toi dau thi speed toi do
	{
		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET ) {

					if (current_speed < 99) {
							current_speed++; 
					}
					HAL_Delay(30); 
			}
			if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14) == GPIO_PIN_SET ) {
					if (current_speed > 0) {
							current_speed--; // Gi?m d?n khi còn gi? nút
					}
					HAL_Delay(30);
				}
	}
}