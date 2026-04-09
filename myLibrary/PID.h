#ifndef __PID_H
#define __PID_H

#include "main.h"

//typedef struct {
//    float Kp, Ki, Kd;
//    float target_velocity; // V?n t?c d?t (mm/s ho?c xung/s)
//    float current_velocity;
//    float error, prev_error;
//    float integral;
//    float out_max;         // Ph?i là 999 d? kh?p v?i ARR c?a Ân
//    float i_limit;         // Ch?ng v?t l? (Anti-windup)
//} Velocity_PID;
typedef struct{
	float Kp, Ki, Kd;
	float error, previous_error;
	float integral_Stored;
	float integral_limit;
	float output_limit;
	float previous_actual;
	float target_velocity;
	float current_pos;
	float last_pos;
}PID_typedef;
void PID_Init(PID_typedef *pid, float kp, float ki, float kd, float limit);
float PID_Compute(PID_typedef *pid, float SetPoint, float MeasuredValue, float dt);
void PID_Reset(PID_typedef *pid);

#endif