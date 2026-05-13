#include "encoder.h"
#include "encoder.h"

// Bi?n n?i b? (ch? file này th?y)
static TIM_HandleTypeDef *encoder_htim;
static int32_t total_pulses = 0;
static int16_t last_counter = 0;
static float current_mm = 0;

void Encoder_Init(TIM_HandleTypeDef *htim) {
    encoder_htim = htim;
    total_pulses = 0;
    last_counter = 0;
    current_mm = 0;
    HAL_TIM_Encoder_Start(encoder_htim, TIM_CHANNEL_ALL);
}

void Encoder_Update(void) {
    // 1. Ð?c giá tr? hi?n t?i t? thanh ghi Timer
    int16_t current_counter = (int16_t)__HAL_TIM_GET_COUNTER(encoder_htim);
    
    // 2. Tính toán d? chênh l?ch (Delta) x? lý du?c c? tru?ng h?p tràn timer
    int16_t delta = current_counter - last_counter;
    
    // 3. C?ng d?n vào t?ng s? xung
    total_pulses += delta;
    
    // 4. C?p nh?t l?i counter cu cho l?n tính sau
    last_counter = current_counter;

    // 5. Quy d?i ra mm (Distance = Pulses * Pitch / CPR)
    current_mm = (float)total_pulses * (LEAD_SCREW_PITCH / ENCODER_CPR);
}

void Encoder_SetZero(PID_typedef *pidPointer) {
    total_pulses = 0;
    __HAL_TIM_SET_COUNTER(encoder_htim, 0);
    last_counter = 0;
    pidPointer->current_pos = 0;
}

float Encoder_GetDistance(void) {
    return current_mm;
}

int32_t Encoder_GetPulses(void) {
    return total_pulses;
}