/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "motor_logic.h"
#include "stdint.h"
#include "stdio.h"
#include "stdlib.h"
#include "Encoder.h"
#include "PID.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define VREF            3.3f    // Điện áp tham chiếu STM32 (V)
#define ADC_MAX         4095.0f // Độ phân giải 12-bit
#define R_SHUNT         1000.0f // Điện trở trên chân IS (thường là 1k Ohm = 1000)
#define K_ILIS          1950.0f // Hệ số của chip BTS7960 (tra datasheet)
#define PULSE_TO_MM_RATIO (8.0f / 422.4f)
uint8_t direction = 1;       
uint16_t rx_buffer[8] = {0}; 
uint8_t rx_data;
uint8_t speed_mode = 0;
uint16_t current_speed;
uint8_t flag = 0;
volatile uint8_t current_motor_status;
//buffer
char buffer[60]; // gửi dữ liệu lên matlab
char MatlabBufferEncoder[8]; // receive Encoder data from matlab
char MatlabBufferPID1[64]; // receive PID data from matlab
//========
//float current_mm = 0;
uint32_t last_send = 0;
uint8_t isMoving = 0;
uint8_t isEncoderReceiving = 0; //no PID
uint8_t rx_index;
float target_mm = 0;
// pid variable ============================
float output;
uint8_t PIDinUSED = 0;
uint8_t isPIDReceiving = 0;
PID_typedef myPID;
uint32_t last_send_time =0;
uint32_t mode = 0;
float start_pos;
	float v_acc = 0;
	float acc_step = 2.0f;
	float current_vel;
           // Hệ số lọc (càng nhỏ càng mượt nhưng càng trễ)
	static uint32_t last_pid_time = 0;
// custom
uint8_t isCustomReceiving = 0;
uint8_t isPIDCustomReceiving = 0;
// Dung bien tro de dieu chinh toc do 
uint8_t isManual = 0;
/// các che do hinh thang 
typedef struct{
	float accel_dist;
	float maintain_dist;
	float decel_dist;
}Mode_Profile;
uint8_t EnNum;
uint8_t is_waiting = 0;
Mode_Profile profiles[4] = {
    {5000.0f, 2000.0f, 4000.0f}, // Mode 1: Cân bằng
    {2000.0f, 5000.0f, 3000.0f}, // Mode 2: Mượt mà
    {20000.0f,  2000.0f, 8000},  // Mode 3: Gắt/Nhanh
    {0.0f, 0.0f, 0.0f}  // Mode 4: Khởi động chậm, dừng nhanh
};
Mode_Profile profiles_mm[4]; // Mảng này dùng để chạy thực tế
typedef enum{
	VELOCITY_MODE = 0,
	POSITION_CONTROL,
	MANUAL_MODE,
	VELOCITY_POT, // CHANGE SETPOINT BY PO
	ENCODER_MODE,
	RESET_MODE,
}PID_Mode;
 volatile PID_Mode pid_mode;

typedef enum{
	MODE_E1,
	MODE_E2,
	MODE_E3,
	MODE_E4,
	MODE_RESET
}Encoder_Mode;
 volatile Encoder_Mode ModeE;

void Convert_Pulse_to_MM_Profiles(Mode_Profile p_pulse[]) {
    for(int i = 0; i < 4; i++) {
        p_pulse[i].accel_dist    = profiles[i].accel_dist    * PULSE_TO_MM_RATIO;
        p_pulse[i].maintain_dist = profiles[i].maintain_dist * PULSE_TO_MM_RATIO;
        p_pulse[i].decel_dist    = profiles[i].decel_dist    * PULSE_TO_MM_RATIO;
    }
}
float Encoder_mode_Handle(PID_typedef *pid, Mode_Profile p[EnNum])
{
		
		pid->current_pos = Encoder_GetDistance() ; // dua ve don vi xung
		pid->current_velocity = (pid->current_pos - pid->last_pos)/0.02f;
		float v_max = 91; 
		float total_pulses = p[EnNum].accel_dist + p[EnNum].maintain_dist + p[EnNum].decel_dist;
		 
		if(pid->current_pos < p[EnNum].accel_dist) // Accel phase 
		{
			pid->target_velocity = (pid->current_pos / p[EnNum].accel_dist) * 91.0f;
			if (pid->target_velocity < 10.0f) pid->target_velocity = 10.0f; // min vel
		}
		else if(pid->current_pos < (p[EnNum].accel_dist + p[EnNum].maintain_dist)) // maintain phase
		{
			pid->target_velocity = v_max;
		}
		else if(pid->current_pos < total_pulses) // decel phase
		{
			float dist_left = total_pulses - pid->current_pos;
			pid->target_velocity = (dist_left/ p[EnNum].decel_dist) * 91.0f;
		}
		output = PID_Compute(&myPID, pid->target_velocity, pid->current_velocity, 0.02f);
		pid->last_pos = pid->current_pos;
		if (pid->current_pos >= total_pulses) {
				current_speed = 0;
				current_motor_status = MOTOR_STOP;
				return 0; // Thoát hàm luôn, không cho PID_Compute chạy nữa
		}
		if(output > 0)
		{
			 current_motor_status = MOTOR_SPEED_UP;
			 current_speed = (uint16_t)output;
			}
			else
			{
				current_motor_status = MOTOR_SPEED_DOWN;
				myPID.integral_Stored = 0;
				current_speed= 0;
		}
}


//Interrupt for receiving data

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  if(huart->Instance == USART1)
	{
		flag = 1;
		PID_Reset(&myPID);
		
		HAL_UART_Receive_IT(&huart1, &rx_data, 1);
	}
}
volatile uint8_t limit_left = 0;
volatile uint8_t limit_right = 0;
void Motor_State_Control_With_Reset(uint8_t target_state, uint16_t speed) 
{
    static uint8_t last_state = MOTOR_STOP;
    static uint32_t stop_time_start = 0;

    // 1. Phát hiện sự kiện đảo chiều
    if ((target_state == MOTOR_SPEED_UP && last_state == MOTOR_SPEED_DOWN) ||
        (target_state == MOTOR_SPEED_DOWN && last_state == MOTOR_SPEED_UP)) 
    {
        is_waiting = 1;
        stop_time_start = HAL_GetTick();
        
        PID_Reset(&myPID); // Reset I và các thông số PID
        
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
    }

    // 2. Chờ bằng Timer (Trạm gác nội bộ)
    if (is_waiting) 
    {
        if (HAL_GetTick() - stop_time_start < 1000) // Để 10s cho Ân dễ quan sát
        {
            // Ép dừng và THOÁT HÀM, không cho Bước 3 chạy
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
            
            // Cập nhật last_state ở đây để tránh lặp lại if ở Bước 1
            last_state = target_state; 
            return; 
        }
        else 
        {
            is_waiting = 0; // Hết thời gian chờ, cho phép xuống Bước 3
        }
    }

    // 3. Thực thi điều khiển (Chỉ chạy khi ĐÃ CHỜ XONG)
    if (target_state == MOTOR_SPEED_UP) 
    {
        current_motor_status = MOTOR_SPEED_UP;
    } 
    else if (target_state == MOTOR_SPEED_DOWN) 
    {
        current_motor_status = MOTOR_SPEED_DOWN;
    }
    else 
    {
        // Trường hợp Stop
        current_motor_status = MOTOR_STOP;
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
    }

    last_state = target_state;
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_10) { // TRÁI
        limit_left = 1;
        Motor_Stop();	
    }
    else if (GPIO_Pin == GPIO_PIN_11) { // PHẢI
        limit_right = 1;
        Motor_Stop();
    }
}

uint32_t last_ramp_time = 0;
void handle_simple_command(uint8_t *command)
{
	switch(*command)
			{
				case 'V':
					speed_mode = 1;
				break;
				// Trong handle_simple_command:
				case 'f': // Forward
            // CHỈ cho phép cập nhật hướng nếu đang ở MANUAL_MODE
            if(pid_mode == MANUAL_MODE) {
                current_motor_status = MOTOR_SPEED_UP;
            }
            break;

        case 'r': // Backward
            if(pid_mode == MANUAL_MODE) {
                current_motor_status = MOTOR_SPEED_DOWN;
            }
            break;
		
				case 's': // Stop
          current_motor_status = MOTOR_STOP;
					isMoving = 0;
					
					v_acc =0;
				isManual =0;
					break;
				case 'h':  //Home
						//current_speed = 30;
						pid_mode = POSITION_CONTROL; // Ép hệ thống chuyển sang điều khiển vị trí
						myPID.target_mm = 0;         // Đặt đích đến là gốc 0     
						//myPID.integral_Stored  = 0;
            break;
				case 'z': //Set Zero
						current_motor_status = MOTOR_STOP;
						Encoder_SetZero(&myPID);	
						break;
				case 'P':	
					isPIDReceiving = 1;
					//PID_Reset(&myPID);
					//myPID.integral_Stored = 0;
					//rx_index = 0;
					break;
				case 'p':
					pid_mode = VELOCITY_POT;
				isPIDReceiving = 1;
				break;
				case 'J': 
					pid_mode = RESET_MODE;
					
				
				break;
//				case 'm':
//						//PIDinUSED = 0;        
//						//isMoving = 0;         // Dừng motor cho an toàn khi chuyển chế độ
//						//current_motor_status = MOTOR_STOP;
//						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
//						// Reset các biến tích phân của PID để lần sau bật lại không bị giật
//						//myPID.integral_Stored = 0;
//						//myPID.previous_error = 0;
//						isPIDReceiving =0;
//				
//					isManual =1;
//					break;
				case 'l':
						HAL_GPIO_WritePin(GPIOA, R_EN | L_EN, GPIO_PIN_RESET); 
            
            // 2. Đợi ít nhất 3us (ở đây để 10ms cho chắc chắn và xả nhiệt) 
            HAL_Delay(10); 
            
            // 3. Kéo chân Enable lên HIGH để kích hoạt lại Driver [cite: 608]
            HAL_GPIO_WritePin(GPIOA, R_EN | L_EN, GPIO_PIN_SET);
					break;
				case 'e': 
					pid_mode = ENCODER_MODE;
				Convert_Pulse_to_MM_Profiles(profiles_mm);
					break;
				case 'b':
					pid_mode = VELOCITY_MODE;
					PID_Reset(&myPID);
				break;
				case 'a':
						pid_mode = POSITION_CONTROL;
					PID_Reset(&myPID);
					break;
				case 'c':
						pid_mode = MANUAL_MODE;
					PID_Reset(&myPID);
				break;
				case 'g':
					ModeE = MODE_E1;
					myPID.integral_Stored = 0;
				break;
				case 'x':
					ModeE = MODE_E2;
					myPID.integral_Stored = 0;
					Encoder_SetZero(&myPID); // Đưa xung về 0 trước khi chạy
					PID_Reset(&myPID);
					pid_mode = ENCODER_MODE;
				break;
				case 'v':
					ModeE = MODE_E3;
					Encoder_SetZero(&myPID); // Đưa xung về 0 trước khi chạy
					PID_Reset(&myPID);
					pid_mode = ENCODER_MODE;
					myPID.integral_Stored = 0;
				break;
				case 'n':
					ModeE = MODE_E4;
					Encoder_SetZero(&myPID); // Đưa xung về 0 trước khi chạy
					PID_Reset(&myPID);
					pid_mode = ENCODER_MODE;
					myPID.integral_Stored = 0;
				break;
				case 'j': 
					ModeE = MODE_RESET;
					myPID.integral_Stored = 0;
					Encoder_SetZero(&myPID); // Đưa xung về 0 trước khi chạy
					PID_Reset(&myPID);
					pid_mode = ENCODER_MODE;
					myPID.current_pos = 0;
					myPID.last_pos = 0;
				case 'C':
					isCustomReceiving = 1;  
				break;
				case 'u':
					isPIDCustomReceiving = 1;
				break;
			}
}

/**
 * @brief  Hàm đọc dòng điện thực tế của Motor từ chân PA6
 * @param  hadc_ptr: Con trỏ tới bộ ADC đang dùng (ví dụ &hadc1)
 * @retval Giá trị dòng điện đơn vị Amper (float)
 */
float RAW = 0;
uint32_t adc_raw = 0;
float Get_Motor_Current(ADC_HandleTypeDef *hadc_ptr) {
    uint32_t adc_val = 0;
    float voltage = 0.0f;
    float current_A = 0.0f;

    // 1. Bắt đầu chuyển đổi ADC
    HAL_ADC_Start(hadc_ptr);
    
    // 2. Chờ ADC chuyển đổi xong (timeout 10ms)
    if (HAL_ADC_PollForConversion(hadc_ptr, 10) == HAL_OK) {
        adc_val = HAL_ADC_GetValue(hadc_ptr);
        
        // 3. Công thức quy đổi:
        // (adc_val / 4095.0f) * 3.3f  --> Đổi ra Volt
        // Volt / 1000.0f              --> Dòng IS (vì Ân dùng trở 1k)
        // IS * 1950.0f                --> Dòng Motor thực tế (hệ số chip)
        
        current_A = ((float)adc_val * 3.3f / 4095.0f / 1000.0f) * 1950.0f;
        
        // 4. Bộ lọc nhiễu "về 0" (Dead-band)
        // Nếu dòng nhỏ hơn 0.05A (nhiễu nhẹ), ép về 0 cho đẹp Matlab
        if (current_A < 0.05f) {
            current_A = 0.0f;
        }
    }
    
    HAL_ADC_Stop(hadc_ptr);
    return current_A;
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
	// init Module
	Init_Motor();
	Encoder_Init(&htim4);
	//Calibrate_Current_System();
	PID_Init(&myPID, myPID.Kp, myPID.Ki, myPID.Kd, 799);
	//Start Encoder, Timer and Interrupt UART
	HAL_UART_Receive_IT(&huart1, &rx_data, 1); 
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	HAL_TIM_Base_Start_IT(&htim4);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_Start(&hadc2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	
		
		if(flag)
		{
			// UART Handle
			flag = 0;
			//Encoder Mode
//			if(isEncoderReceiving)
//			{
//					if(rx_data == '\n' || rx_data == '\r')
//					{
//							MatlabBufferEncoder[rx_index] = '\0';
//							target_mm = atof(MatlabBufferEncoder);
//							isEncoderReceiving = 0;
//							isMoving = 1;
//							rx_index = 0;
//							
//							PID_Reset(&myPID);    // X�a s?ch Integral v� Error cu
//					}
//					else if (rx_index < 8) // Tránh lưu đè chữ G vào buffer
//					{
//							MatlabBufferEncoder[rx_index++] = rx_data;
//					}
//			}
			//speed mode
					if(speed_mode == 1)
					{
						current_speed = rx_data;
						speed_mode = 0;
					}
				// PID mode 
					else if(isPIDReceiving)
						{
								
								if(rx_data == '\n' || rx_data == '\r')
								{
										MatlabBufferPID1[rx_index] = '\0';
										sscanf(MatlabBufferPID1, "%f I %f D %f M %f O%f", &myPID.Kp, &myPID.Ki, &myPID.Kd, &myPID.target_velocity, &myPID.target_mm);
									//sscanf(MatlabBufferPID1, "%f I %f D %f S %f M %f K %d", &myPID.Kp, &myPID.Ki, &myPID.Kd, &target_mm, &myPID.target_velocity, &mode);
										isPIDReceiving = 0;
										isMoving = 1;
										PIDinUSED = 1;
										rx_index = 0;
										PID_Reset(&myPID);    // X�a s?ch Integral v� Error cu
								}
								else if(rx_index < 40) // Tránh lưu đè chữ G vào buffer
								{
								
										MatlabBufferPID1[rx_index++] = rx_data;
								}
						}
					else if(isCustomReceiving && ModeE == MODE_E4)
						{
							if(rx_data == '\n' || rx_data == '\r')
								{
										MatlabBufferPID1[rx_index] = '\0';
										sscanf(MatlabBufferPID1, "%f I %f D %f A %f T %f d %f", &myPID.Kp, &myPID.Ki, &myPID.Kd, &profiles[3].accel_dist, &profiles[3].maintain_dist, &profiles[3].decel_dist);
										Convert_Pulse_to_MM_Profiles(profiles_mm);
									isCustomReceiving = 0;
										rx_index = 0;
									
								}
								else if(rx_index < 64) {
										MatlabBufferPID1[rx_index++] = rx_data;
								}
							
							}
					else if(isPIDCustomReceiving)
						{
							if(rx_data == '\n' || rx_data == '\r')
								{
										MatlabBufferPID1[rx_index] = '\0';
										sscanf(MatlabBufferPID1, "%f i %f o %f ", &myPID.Kp, &myPID.Ki, &myPID.Kd);
										isPIDCustomReceiving = 0;
										rx_index = 0;
								}
								else if(rx_index < 20) {
										MatlabBufferPID1[rx_index++] = rx_data;
								}
						}
		}
		
		handle_simple_command(&rx_data);
		Encoder_Update();	
		

			if(HAL_GetTick() - last_pid_time > 20)
			{
				

				if(pid_mode == VELOCITY_MODE || pid_mode == VELOCITY_POT)
					{
						if(pid_mode == VELOCITY_POT)
						{
							HAL_ADC_Start(&hadc1);
							if(HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
								{
										uint32_t pot_raw = HAL_ADC_GetValue(&hadc1);
										// Quy đổi: (pot_raw / 4095) * 799
										myPID.target_velocity = (uint16_t)((pot_raw / 4095.0f) * 91.0f);
								}
						}
						myPID.current_pos =  Encoder_GetDistance() ; // lay vi tri hien tai
						myPID.current_velocity = (myPID.current_pos - myPID.last_pos)/0.02f;
						if(myPID.target_velocity >= 91)
							{
								myPID.target_velocity = 91;
							}
						float error_vel = myPID.target_velocity - myPID.current_velocity ;
						output = PID_Compute(&myPID, myPID.target_velocity, myPID.current_velocity , 0.02f);
						myPID.last_pos = myPID.current_pos;
						if(output > 0)
						{
							current_motor_status = MOTOR_SPEED_UP;
							current_speed = (uint16_t)output;
						}
						else
						{
							current_motor_status = MOTOR_STOP;
							//myPID.integral_Stored = 0;
							current_speed= 0;
						}
						
					}
				else if(pid_mode == MANUAL_MODE)
					{
						myPID.current_pos =  Encoder_GetDistance() ;
						HAL_ADC_Start(&hadc1); 
						Motor_State_Control_With_Reset(current_motor_status, current_speed);
					// Chờ cho đến khi ADC chuyển đổi xong (Timeout 10ms)
					if(HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
						{
								uint32_t pot_raw = HAL_ADC_GetValue(&hadc1);

								//current_motor_status = MOTOR_SPEED_UP;
								// Quy đổi: (pot_raw / 4095) * 799
								current_speed = (uint16_t)((pot_raw / 4095.0f) * 799.0f);
						}
					
					

					myPID.last_pos = myPID.current_pos;
					}
				else if(pid_mode == POSITION_CONTROL)
					{
						myPID.current_pos =  Encoder_GetDistance() ; // lay vi tri hien tai
						myPID.current_velocity = (myPID.current_pos - myPID.last_pos)/0.02f;
						float error_pos = myPID.target_mm - myPID.current_pos ;
						output = PID_Compute(&myPID, myPID.target_mm , myPID.current_pos , 0.02f);
						if(output > 0)
						{
							current_motor_status = MOTOR_SPEED_UP;
							current_speed = (uint16_t)output;
						}
						else
						{
							current_motor_status = MOTOR_SPEED_DOWN;
							current_speed= (uint16_t)__fabs(output);
						}
						if (__fabs(myPID.target_mm  - myPID.current_pos) < 0.5f) {
									current_motor_status = MOTOR_STOP;
									current_speed = 0;
									
							}
						myPID.current_velocity = (myPID.current_pos - myPID.last_pos) / 0.02f;
						myPID.last_pos = myPID.current_pos;
						
					}
				else if(pid_mode == ENCODER_MODE)
					{
						Convert_Pulse_to_MM_Profiles(profiles_mm);
						 if(ModeE == MODE_E1) Encoder_mode_Handle(&myPID, &profiles_mm[0]);
						 else if (ModeE == MODE_E2) Encoder_mode_Handle(&myPID, &profiles_mm[1]);
						 else if (ModeE == MODE_E3) Encoder_mode_Handle(&myPID, &profiles_mm[2]);
						 else if (ModeE == MODE_E4) Encoder_mode_Handle(&myPID, &profiles_mm[3]);
						 else if (ModeE == MODE_RESET)
							{
								current_motor_status = MOTOR_STOP;
								myPID.integral_Stored = 0;
								Encoder_SetZero(&myPID); // Đưa xung về 0 trước khi chạy
								PID_Reset(&myPID);
								myPID.current_velocity= 0;
								myPID.current_pos = 0;
								myPID.last_pos = 0;
								myPID.target_mm =0;
								myPID.target_velocity = 0;
							}
						 }
					else if(pid_mode == RESET_MODE)
					{
								current_motor_status = MOTOR_STOP;
								Encoder_SetZero(&myPID); // Đưa xung về 0 trước khi chạy
								myPID.integral_Stored = 0;
								myPID.current_velocity= 0;
								PID_Reset(&myPID);
								myPID.current_pos = 0;
								myPID.last_pos = 0;
								myPID.target_mm =0;
								myPID.target_velocity = 0;
								output = 0;
					}
					Motor_handle();
				last_pid_time = HAL_GetTick();
					}
		if (HAL_GetTick() - last_send_time >= 100) 
		{
				
				// Định dạng: V[số],E[số]\n  (Ví dụ: V150,E12.50\n)
			memset(buffer, 0, sizeof(buffer));
			int len = sprintf(buffer, "V%d E%.2f M%d S%d T%.2f Y%.2f\n", current_speed, myPID.current_pos, pid_mode, current_motor_status, myPID.target_mm, myPID.target_velocity);
				HAL_UART_Transmit(&huart1, (uint8_t*)buffer, len, 100);
				last_send_time = HAL_GetTick(); 
				
				
		
		}
	}
  
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 799;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
