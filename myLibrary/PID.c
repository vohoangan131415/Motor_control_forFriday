#include "PID.h"

void PID_Init(PID_typedef *pid, float kp, float ki, float kd, float limit) {
    pid->Kp = kp; pid->Ki = ki; pid->Kd = kd;
    pid->output_limit = limit;
    pid->integral_limit = limit * 0.5f; // Ch?n tích phân ? 50% công su?t
		pid->last_pos = 0;
    PID_Reset(pid);
}

void PID_Reset(PID_typedef *pid) {
    pid->error = 0; pid->previous_error = 0;
    pid->integral_Stored = 0;
}

float PID_Compute(PID_typedef *pid, float SetPoint, float MeasuredValue, float dt) {
    pid->error = SetPoint - MeasuredValue;
    
    // Thành ph?n P
    float P = pid->Kp * pid->error;
    
    // Thành ph?n I (Quan tr?ng nh?t d? gi? v?n t?c ?n d?nh khi có t?i)
    pid->integral_Stored += pid->error * dt;
    if (pid->integral_Stored > pid->integral_limit) pid->integral_Stored = pid->integral_limit;
    else if (pid->integral_Stored < -pid->integral_limit) pid->integral_Stored = -pid->integral_limit;
    float I = pid->Ki * pid->integral_Stored;
    
    // Thành ph?n D
    float D = pid->Kd * (pid->error - pid->previous_error) / dt;
    pid->previous_error = pid->error;
  
    float output = P + I + D;
    
    // Gi?i h?n d?u ra PWM (Saturation) [cite: 33, 730]
    if (output > pid->output_limit) output = pid->output_limit;
    else if (output < -pid->output_limit) output = -pid->output_limit;
    return output;
}