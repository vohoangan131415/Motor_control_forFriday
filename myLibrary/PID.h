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
	volatile float Kp, Ki, Kd;
	volatile float error, previous_error;
	volatile float integral_Stored;
	volatile float integral_limit;
	volatile float output_limit;
	volatile float previous_actual;
	volatile float target_velocity;
	volatile float target_mm;
	volatile float current_pos;
	volatile float last_pos;
	volatile float current_velocity;
	volatile float intergral_vel_limit; // custom
}PID_typedef;
void PID_Init(PID_typedef *pid, float kp, float ki, float kd, float limit);
float PID_Compute(PID_typedef *pid, float SetPoint, float MeasuredValue, float dt);
float PID_ComputeVel(PID_typedef *pid, float SetPoint, float MeasuredValue, float dt); //custom
void PID_Reset(PID_typedef *pid);

#endif