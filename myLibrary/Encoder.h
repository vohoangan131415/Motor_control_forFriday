#ifndef __ENCODER_H
#define __ENCODER_H

#include "main.h"

// Thông số cơ khí của Ân
#define LEAD_SCREW_PITCH  8.0f    // 8mm per revolution
#define ENCODER_CPR       422.4f  // Counts per revolution
#define PULSE_TO_MM  (8.0f / 422.4f)
// Khai báo các hàm quản lý Encoder
void Encoder_Init(TIM_HandleTypeDef *htim);
void Encoder_Update(void);
void Encoder_SetZero(void);

// Các hàm lấy giá trị (Getter)
float Encoder_GetDistance(void);
int32_t Encoder_GetPulses(void);

#endif