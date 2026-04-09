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
uint8_t direction = 1;       
uint16_t rx_buffer[8] = {0}; 
uint8_t rx_data;
uint8_t speed_mode = 0;
uint16_t current_speed;
uint8_t flag = 0;
volatile uint8_t current_motor_status;
//buffer
char buffer[21]; // gửi dữ liệu lên matlab
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
	float v_acc = 0;
	float acc_step = 2.0f;
	float current_vel;
           // Hệ số lọc (càng nhỏ càng mượt nhưng càng trễ)
	static uint32_t last_pid_time = 0;
//Interrupt for receiving data

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  if(huart->Instance == USART1)
	{
		flag = 1;
		HAL_UART_Receive_IT(&huart1, &rx_data, 1);
	}
}

void handle_simple_command(uint8_t *command)
{
	switch(*command)
			{
				case 'V':
					speed_mode = 1;
				
				case 'f': //Forward
					current_motor_status = MOTOR_SPEED_UP;
					
					break;
				case 'r': //Backward
					current_motor_status = MOTOR_SPEED_DOWN;
				
					break;
				case 's': // Stop
          current_motor_status = MOTOR_STOP;
					isMoving = 0;
					break;
				case 'h':  //Home
						//current_speed = 30;
						target_mm = 0;
						isMoving = 1;      // PHẢI CÓ DÒNG NÀY để kích hoạt bộ điều khiển
						isEncoderReceiving = 0;
				
					PIDinUSED =1;
            break;
				case 'g': // Go
							isEncoderReceiving = 1;
							rx_index = 0;
							PIDinUSED =1;
				
							//current_speed = 30;
						break;
				case 'z': //Set Zero
						Encoder_SetZero();
						break;
				case 'P':	
					isPIDReceiving = 1;
					//rx_index = 0;
					break;
				case 'm':
						PIDinUSED = 0;        
						isMoving = 0;         // Dừng motor cho an toàn khi chuyển chế độ
						current_motor_status = MOTOR_STOP;
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
						// Reset các biến tích phân của PID để lần sau bật lại không bị giật
						myPID.integral_Stored = 0;
						myPID.previous_error = 0;
						isPIDReceiving =0;
					break;
				case 'l':
						HAL_GPIO_WritePin(GPIOA, R_EN | L_EN, GPIO_PIN_RESET); 
            
            // 2. Đợi ít nhất 3us (ở đây để 10ms cho chắc chắn và xả nhiệt) 
            HAL_Delay(10); 
            
            // 3. Kéo chân Enable lên HIGH để kích hoạt lại Driver [cite: 608]
            HAL_GPIO_WritePin(GPIOA, R_EN | L_EN, GPIO_PIN_SET);
					break;
					
			}
}
//uint16_t ADC_Value[2];
//uint16_t current_Threshold = 200; //mA
//uint8_t isSafetoReverse = 0;
//uint16_t L_ISValue = 0;
//uint16_t R_ISValue = 0;
//// Cap nhat current feedback 
float current_L = 0, current_R = 0;
float total_current = 0;
float R_Ampe = 0;
float L_Ampe = 0;
uint32_t raw_L;
uint32_t raw_R;
// Hàm tính dòng điện từ giá trị ADC Raw dựa trên thông số phần cứng
float Convert_ADC_to_Amper(uint32_t adc_raw) {
    // 1. Tính điện áp thực tế tại chân IS của STM32 (V)
    // Công thức: (Giá trị ADC / 4095) * 3.3V
    float V_is = (adc_raw * VREF) / ADC_MAX;
    
    // 2. Tính dòng điện chạy ra từ chân IS của Driver (I_is)
    // Theo định luật Ohm: I = U / R. R_SHUNT thường là 1k (1000 Ohm)
    float I_is = V_is / R_SHUNT;
    
    // 3. Quy đổi ra dòng điện thực tế chạy qua Motor (I_motor)
    // Dựa vào hệ số K_ILIS của chip (BTS7960 thường là 1950)
    float I_motor = I_is * K_ILIS;
    
    return I_motor;
}
float current_obs = 0; // Biến dòng điện đang quan sát
#define SAFE_LIMIT 0.3f // Ngưỡng an toàn (Ampe)

// Hàm này là "chìa khóa" để đổi kênh ADC linh hoạt
uint32_t ADC_Read_Manual(uint32_t channel) {
    ADC_ChannelConfTypeDef sConfig = {0};
    
    // BƯỚC 1: Xóa cấu hình cũ, ép ADC2 nhìn vào kênh mới (Channel 6 hoặc 7)
    sConfig.Channel = channel;
    sConfig.Rank = 1; // Luôn dùng Rank 1 vì mình đã tắt Scan Mode
    sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
    
    // Lệnh này ghi đè vào thanh ghi cấu hình của ADC
    HAL_ADC_ConfigChannel(&hadc2, &sConfig); 
    
    // BƯỚC 2: Ra lệnh cho ADC bắt đầu đo kênh vừa cấu hình
    HAL_ADC_Start(&hadc2);
    
    // BƯỚC 3: Đợi đo xong (Polling)
    if (HAL_ADC_PollForConversion(&hadc2, 10) == HAL_OK) {
        uint32_t val = HAL_ADC_GetValue(&hadc2);
        HAL_ADC_Stop(&hadc2);
        return val;
    }
    
    HAL_ADC_Stop(&hadc2);
    return 0;
}

void Update_CurrentFeedback() {
    // Dựa vào trạng thái motor để "ra lệnh" cho ADC nhìn vào đâu
    if (current_motor_status == MOTOR_SPEED_UP) {
        // Đang quay thuận -> Bảo ADC: "Ê, đọc chân R (CH7) cho tao"
        uint32_t raw_R = ADC_Read_Manual(ADC_CHANNEL_7);
        R_Ampe = Convert_ADC_to_Amper(raw_R);
        L_Ampe = 0.0f; // Hướng nghịch chắc chắn không có dòng, ép về 0 cho sạch
        current_obs = R_Ampe;
    } 
    else if (current_motor_status == MOTOR_SPEED_DOWN) {
        // Đang quay nghịch -> Bảo ADC: "Giờ thì chuyển sang chân L (CH6) đi"
        uint32_t raw_L = ADC_Read_Manual(ADC_CHANNEL_6);
        L_Ampe = Convert_ADC_to_Amper(raw_L);
        R_Ampe = 0.0f;
        current_obs = L_Ampe;
    }
    else {
        // Khi STOP: Đọc nhanh cả 2 để kiểm tra dòng rò hoặc kẹt nhẹ
        R_Ampe = Convert_ADC_to_Amper(ADC_Read_Manual(ADC_CHANNEL_7));
        L_Ampe = Convert_ADC_to_Amper(ADC_Read_Manual(ADC_CHANNEL_6));
        current_obs = (R_Ampe > L_Ampe) ? R_Ampe : L_Ampe;
    }
}
// Cap nhat flag de dao chieu 


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
	PID_Init(&myPID, myPID.Kp, myPID.Ki, myPID.Kd,1000);
	//Start Encoder, Timer and Interrupt UART
	HAL_UART_Receive_IT(&huart1, &rx_data, 1); 
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	HAL_TIM_Base_Start_IT(&htim4);
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
			if(isEncoderReceiving)
			{
					if(rx_data == '\n' || rx_data == '\r')
					{
							MatlabBufferEncoder[rx_index] = '\0';
							target_mm = atof(MatlabBufferEncoder);
							isEncoderReceiving = 0;
							isMoving = 1;
							rx_index = 0;
							
							PID_Reset(&myPID);    // X�a s?ch Integral v� Error cu
					}
					else if (rx_index < 8) // Tránh lưu đè chữ G vào buffer
					{
							MatlabBufferEncoder[rx_index++] = rx_data;
					}
			}
			//speed mode
					else if(speed_mode == 1)
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
										sscanf(MatlabBufferPID1, "%f I %f D %f S %f M %f", &myPID.Kp, &myPID.Ki, &myPID.Kd, &target_mm, &myPID.target_velocity);
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
							
			}
		handle_simple_command(&rx_data);
		Encoder_Update();	
		Motor_handle();
    myPID.current_pos =  Encoder_GetDistance() ;
			
		if (HAL_GetTick() - last_pid_time >= 20) 
    {
			Update_CurrentFeedback();
			last_pid_time = HAL_GetTick();
			if(isMoving) // For Encoder without PID 
			{
				if(PIDinUSED) // PID in used
				{
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);
						float error = target_mm - myPID.current_pos;
						current_vel = (myPID.current_pos - myPID.last_pos) / 0.02f;
						//vel_filtered = (alpha * current_vel_raw) + (1.0f - alpha) * vel_filtered;
						myPID.last_pos = myPID.current_pos;
						float distance_to_go = __fabs(error);

						// 1. Tạo dốc vận tốc mục tiêu
						if(v_acc < myPID.target_velocity) v_acc += acc_step; 

						// 2. Hệ số phanh: Càng gần đích v_limit càng nhỏ
						float v_limit = distance_to_go * 2.0f; // Hệ số 1.5-2.0 là vừa đẹp

						// 3. Vận tốc cuối cùng không được vượt quá v_limit
						float v_final = (v_acc < v_limit) ? v_acc : v_limit;

						// 4. Chỉ dùng 1 hàm PID duy nhất điều khiển VẬN TỐC
						float v_target = (error > 0) ? v_final : -v_final;
						output = PID_Compute(&myPID, v_target, current_vel, 0.02f);
						if (error > 0) {
											//myPID.integral_Stored = 0;
											current_motor_status = MOTOR_SPEED_UP;
											current_speed = (uint16_t)output;
									} else {
											current_motor_status = MOTOR_SPEED_DOWN;
											current_speed = (uint16_t)__fabs(output);
									}
							// Dừng motor nếu sai số quá nhỏ (Dead-band)
						if (__fabs(target_mm - myPID.current_pos) < 0.1f) {
									isMoving = 0;
									current_motor_status = MOTOR_STOP;
									current_speed = 0;
									v_acc = 0;
									PID_Reset(&myPID);
							}
						
					}
				
				}
		}
		//PID Vel first, when near the goal, PID Pos
	//	Update_CurrentFeedback();
    // Sending to Matlab
		//	Update_CurrentFeedback();
		
		
		if (HAL_GetTick() - last_send_time >= 100) 
		{
				
				// Định dạng: V[số],E[số]\n  (Ví dụ: V150,E12.50\n)
			int len = sprintf(buffer, "V%d E%.2f\n", current_speed  , myPID.current_pos);
				HAL_UART_Transmit(&huart1, (uint8_t*)buffer, len, sizeof(buffer));
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
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
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
  hadc2.Init.ContinuousConvMode = DISABLE;
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
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
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
  huart1.Init.BaudRate = 9600;
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

  /*Configure GPIO pins : PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

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
