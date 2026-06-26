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
#include "stdio.h"    /* ← thêm: fix lỗi sscanf */
#include "stdlib.h"
#include "math.h" 
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
typedef struct {
    float Kp, Ki, Kd;
    float integral;
    float prev_error;
    float integral_limit;  // chống integral windup
    float output_limit;    // giới hạn output tối đa
} PID_t;

// Kalman Test
typedef struct {
    float kalmanExtrapolate;
	  float kalmanEstimation;
	  float kalmanUncertainty_currFlex;
	  float kalmanUncertainty_exFlex;
} KalmanParam;
KalmanParam angle1;
KalmanParam angle2;
KalmanParam angle3;

PID_t pid1 = {
    .Kp = 500.0f,    // Tăng để phản ứng nhanh hơn
    .Ki = 15.0f,      // Nhỏ thôi, tránh dao động
    .Kd = 100.0f,     // Hãm phanh khi gần đích
    .integral = 0,
    .prev_error = 0,
    .integral_limit = 300.0f,
    .output_limit = 25000.0f
};

PID_t pid2 = {
//    .Kp = 800.0f,
//    .Ki = 50.0f,
//    .Kd = 50.0f,
	.Kp = 800.0f,
    .Ki = 50.0f,
    .Kd = 55.0f,
    .integral = 0,
    .prev_error = 0,
    .integral_limit = 3000.0f,
    .output_limit = 90000.0f
};

PID_t pid3 = {
		//    .Kp = 400.0f,
		//    .Ki = 3.0f,
		//    .Kd = 30.0f,
    .Kp = 400.0f,
    .Ki = 3.0f,
    .Kd = 35.0f,
    .integral = 0,
    .prev_error = 0,
    .integral_limit = 100.0f,
    .output_limit = 25000.0f
};
#define TCA9548A_ADDR     (0x70 << 1)
#define AS5600_ADDR       (0x36 << 1)
#define AS5600_RAW_ANGLE_REG  0x0C

#define MUX_CH_J1   0
#define MUX_CH_J2   1
#define MUX_CH_J3   2
#define GEAR_RATIO_J1   9.375f
#define GEAR_RATIO_J2   9.375f
#define ANGLE_MAX_J3  300.0f
#define HOOK_OPEN    180.0f
#define HOOK_CLOSE   0.0f

#define L0  108.0f   // ← THÊM
#define L1  220.0f   // ← THÊM
#define L2  235.0f   // ← THÊM
#define J2_ZERO_OFFSET   5.0f
#define VEL_PROFILE_J1  60.0f   // J1: 60°/s
#define VEL_PROFILE_J2  60.0f   // J2: 60°/s
#define VEL_PROFILE_J3  30.0f   // J3: chậm hơn, motor yếu
#define LOOP_DT         0.05f
#define PROCESS_NOISE   0.20f
#define MEAS_UNCERTAINTY (2.0 * 2.0)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
//volatile float servo_target_angle = 90.0f;
volatile uint16_t debug_as5600_raw1 = 0;
volatile uint16_t debug_as5600_raw2 = 0;
volatile uint16_t debug_as5600_raw3 = 0;
volatile uint8_t  debug_as5600_status1 = 0;
volatile uint8_t  debug_as5600_status2 = 0;
volatile uint8_t  debug_as5600_status3 = 0;
float as5600_zero_offset1 = 0.0f;
float as5600_zero_offset2 = 0.0f;
float as5600_zero_offset3 = 0.0f;
uint8_t as5600_invert1 = 0; // 1 nếu chiều quay ngược so với chiều bạn cần
uint8_t as5600_invert2 = 0;
uint8_t as5600_invert3 = 0;

volatile uint8_t home_button_pressed = 0; 
volatile uint8_t home_done = 0;              // 0 = chưa home, 1 = đã home
volatile uint8_t control_enabled = 0;        // 0 = không nhận data, 1 = nhận data
volatile uint8_t home_requested = 0;         // 0 = không cần home, 1 = yêu cầu home

volatile float home_angle1 = 38.7f;          // Home position J1
volatile float home_angle2 = 47.5f;          // Home position J2
volatile float home_angle3 = 147.5f;         // Home position J3

#define HOMING_TIMEOUT_MS 30000
float target_smooth1 = 0.0f;  // ← THÊM
float target_smooth2 = 0.0f;  // ← THÊM
float target_smooth3 = 0.0f;  // ← THÊM
float filtered_j1 = 0.0f;     // ← THÊM
float filtered_j2 = 0.0f;     // ← THÊM
float filtered_j3 = 0.0f;     // ← THÊM
uint8_t uart2_rx_byte;
uint8_t uart2_buf[32];
uint8_t uart2_index = 0;
uint8_t uart6_rx_byte;
uint8_t uart6_buf[32];
uint8_t uart6_index = 0;
volatile uint8_t uart_mode = 1;
volatile float watch_joint1_real = 0.0f;  // góc thực J1 sau gear ratio
volatile float watch_joint2_real = 0.0f; 
volatile float watch_joint3_real = 0.0f;
volatile float watch_angle1 = 0.0f;
volatile float watch_angle2 = 0.0f;
volatile int32_t debug_err1 = 0;
volatile uint8_t debug_dir1 = 0;
volatile int32_t debug_err3 = 0;
volatile uint8_t debug_dir3 = 0;
volatile int32_t debug_err2 = 0;
volatile uint8_t debug_dir2 = 0;
volatile uint8_t btn_servo_last = 1;
volatile uint8_t servo_open     = 0;
volatile uint32_t uart_irq_count = 0;
uint8_t uart_rx_byte;
uint8_t uart_buf[32];
uint8_t uart_index = 0;

volatile float uart_angle1 = 0;
volatile float uart_angle2 = 0;
volatile float uart_angle3 = 0;
volatile uint8_t uart_data_ready = 0;
volatile uint8_t button_val = 0; 
  // 0=bình thường, 1=về home
volatile uint8_t is_homing = 0;       // 0=không homing, 1=đang homing
#define  FILTER_SIZE 8   // Lấy trung bình 8 mẫu


volatile float angle_current1 = 0;
volatile float angle_current2 = 0;
volatile float angle_current3 = 0;
volatile int32_t pos1 = 0, pos2 = 0, pos3 = 0;



#define STEPS_PER_REV   1600
#define MAX_SPEED        8000
#define MIN_SPEED        50
#define ACCEL_STEP       200
volatile uint8_t hook_state = 0;  // 0=đóng, 1=mở
typedef struct { float x, y, z; } Position3D_t;
volatile Position3D_t tool_pos = {0, 0, 0};
typedef struct {
    int32_t  target_speed;
    int32_t  current_speed;
    uint8_t  direction;
		int32_t  max_speed;    // ← Thêm
    int32_t  accel_step;   // ← Thêm
} MotorState_t;

MotorState_t state1 = {0, 0, 0, 1000, 50};
MotorState_t state2 = {0, 0, 0, 1000, 50};
MotorState_t state3 = {0, 0, 0, 3000 , 50 };
typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    GPIO_TypeDef *dir_port;
    uint16_t dir_pin;
    GPIO_TypeDef *en_port;
    uint16_t en_pin;
    uint16_t current_speed;
    uint8_t direction;
} StepperMotor_t;

StepperMotor_t motor1 = {
    .htim = &htim4,
    .channel = TIM_CHANNEL_1,
    .dir_port = GPIOA,
    .dir_pin = GPIO_PIN_1,
    .en_port = GPIOA,
    .en_pin = GPIO_PIN_2,
    .current_speed = 0,
    .direction = 0
};
StepperMotor_t motor2 = {
    .htim = &htim3,
    .channel = TIM_CHANNEL_1,
    .dir_port = GPIOA,
    .dir_pin = GPIO_PIN_7,
    .en_port = GPIOB,
    .en_pin = GPIO_PIN_0,
    .current_speed = 0,
    .direction = 0
};
StepperMotor_t motor3 = {
    .htim = &htim1,
    .channel = TIM_CHANNEL_1,
    .dir_port = GPIOB,
    .dir_pin = GPIO_PIN_7,
    .en_port = GPIOB,
    .en_pin = GPIO_PIN_8,
    .current_speed = 0,
    .direction = 0
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
void Slave_Feedback_Read(void);
HAL_StatusTypeDef kp(uint8_t channel);
HAL_StatusTypeDef AS5600_ReadRawAngle(uint16_t *raw_angle);
float AS5600_ReadAngleDeg(uint8_t mux_channel, float zero_offset, uint8_t invert, volatile uint16_t *out_raw, volatile uint8_t *out_status);
//void Servo_TIM9_SetAngle(float angle);  // ← THÊM
void Button_Servo_Handle(void);  
void Servo_DebugControl(void);// ← THÊM
void Parse_UART_Data(char *buf);
void Go_Home(void);
uint8_t Is_At_Home(void);
void Process_ADC_Data(void);
void ADC_Filter_Update(void);
void Motor_MoveToAngle(StepperMotor_t *motor, MotorState_t *state, volatile int32_t *pos, float target_angle);
void Motor_Init(StepperMotor_t *motor);
void Motor_SetSpeed(StepperMotor_t *motor, uint16_t speed);
void Motor_SetDirection(StepperMotor_t *motor, uint8_t dir);
void Motor_Enable(StepperMotor_t *motor);
void Motor_Disable(StepperMotor_t *motor);
//float ADC_ToAngle(uint16_t adc_val);
void Motor_SetSpeedSmooth(StepperMotor_t *motor, MotorState_t *state, int32_t target_steps_per_sec);
void Servo_SetAngle(float angle);
void Hook_Control(uint8_t state);
Position3D_t Forward_Kinematics(float t1, float t2, float t3); // ← THÊM
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
	
	__HAL_RCC_GPIOD_CLK_ENABLE();
GPIOD->MODER |= (1 << 26);
GPIOD->MODER |= (1 << 24);
pos1 = 0; pos2 = 0; pos3 = 0;

HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);

// Không start TIM motor ở đây — Motor_Init tự lo
Motor_Init(&motor1);   // TIM4 CH1 → PB6
Motor_Init(&motor2);   // TIM3 CH1 → PB4
Motor_Init(&motor3);   // TIM1 CH1 → PE9

// Servo TIM9 start riêng
//HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
//Servo_TIM9_SetAngle(0.0f);

HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
HAL_UART_Receive_IT(&huart2, &uart2_rx_byte, 1);  // ← đổi uart_rx_byte → uart2_rx_byte

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	button_val = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
  while (1)
  {
//		Servo_DebugControl();  
//		Servo_TIM9_SetAngle(servo_target_angle);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_RESET)
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);  // LED sáng khi nhấn
}
else
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET); // LED tắt khi nhả
}
		 GPIOD->ODR ^= (1 << 13);
    // Đọc nút HOME
    uint8_t btn_home_now = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
    if(home_button_pressed == 0 && btn_home_now == GPIO_PIN_RESET)
    {
        HAL_Delay(20);
        if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET)
        {
            home_button_pressed = 1;
        uart_mode = !uart_mode;
        if(uart_mode == 0)
            home_requested = 1;  // ← HOME mode
        else
            home_requested = 0; // ← Toggle uart_mode: 0↔1
        }
    }
    if(btn_home_now == GPIO_PIN_SET)
    {
        home_button_pressed = 0;  // ← Reset khi nhả nút
    }
 //Button_Servo_Handle();
		
  Slave_Feedback_Read();   // ← ra ngoài, luôn chạy
HAL_Delay(50);

if(uart_mode == 0)  // ← HOME mode
{
    Go_Home();
}
else  // ← MASTER-SLAVE mode (uart_mode = 1)
{
    Process_ADC_Data();
  //  Hook_Control(hook_state);
}
//    Motor_Enable(&motor1);
//    Motor_Enable(&motor2);
//
//    Motor_SetDirection(&motor1, 0);
//    Motor_SetDirection(&motor2, 0);
//
//    Motor_SetSpeedSmooth(&motor1, &state1, 2000);
//    Motor_SetSpeedSmooth(&motor2, &state2, 2000);
//
//    HAL_Delay(50);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 99;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
sConfigOC.OCMode = TIM_OCMODE_PWM1;
sConfigOC.Pulse = 50;
sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
{
    Error_Handler();
}
  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  htim2.Init.Prescaler = 99;
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
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 99;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 99;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);  // ← THÊM
HAL_NVIC_EnableIRQ(USART2_IRQn);   
  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DIR_MOTOR_1_Pin|EN_MOTOR_2_Pin|DIR_MOTOR_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, EN_MOTOR_2B0_Pin|DIR_MOTOR_3_Pin|EN_MOTOR_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF3_TIM9;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR_MOTOR_1_Pin DIR_MOTOR_2_Pin */
  GPIO_InitStruct.Pin = DIR_MOTOR_1_Pin|DIR_MOTOR_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : EN_MOTOR_2_Pin */
  GPIO_InitStruct.Pin = EN_MOTOR_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(EN_MOTOR_2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_MOTOR_2B0_Pin EN_MOTOR_3_Pin */
  GPIO_InitStruct.Pin = EN_MOTOR_2B0_Pin|EN_MOTOR_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN_SERVO_Pin */
  GPIO_InitStruct.Pin = BTN_SERVO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BTN_SERVO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DIR_MOTOR_3_Pin */
  GPIO_InitStruct.Pin = DIR_MOTOR_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DIR_MOTOR_3_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
	GPIO_InitTypeDef GPIO_InitStruct3 = {0};
GPIO_InitStruct3.Pin = GPIO_PIN_0;
GPIO_InitStruct3.Mode = GPIO_MODE_INPUT;
GPIO_InitStruct3.Pull = GPIO_PULLUP;
HAL_GPIO_Init(GPIOA, &GPIO_InitStruct3);

	
	


  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
float PID_Compute(PID_t *pid, float error)
{
    // ---- P term ----
    float P_term = pid->Kp * error;

    // ---- I term (với anti-windup: chỉ tích khi chưa bão hòa) ----
    pid->integral += error;
    // Giới hạn integral để tránh "windup" — tích quá lớn gây overshoot
    if(pid->integral >  pid->integral_limit) pid->integral =  pid->integral_limit;
    if(pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
    float I_term = pid->Ki * pid->integral;

    // ---- D term ----
    // delta_error âm = sai số đang giảm = robot đang tiến đến đích
    // D_term âm sẽ làm giảm output → phanh lại
    float delta_error = error - pid->prev_error;
    float D_term = pid->Kd * delta_error/0.05;
    pid->prev_error = error;

    // ---- Tổng output ----
    float output = P_term + I_term*0 + D_term;

    // Giới hạn output (steps/s tối đa của motor)
    if(output >  pid->output_limit) output =  pid->output_limit;
    if(output < -pid->output_limit) output = -pid->output_limit;

    return output;
}

/**
 * @brief Reset PID về 0 (dùng khi đến đích hoặc sau homing)
 */
void PID_Reset(PID_t *pid)
{
    pid->integral   = 0;
    pid->prev_error = 0;
}

//void Servo_TIM9_SetAngle(float angle)
//{
// //   if(servo_target_angle < 0.0f) servo_target_angle = 0.0f;
// //   if(servo_target_angle > 180.0f) servo_target_angle = 180.0f;
//	 // Tính pulse: 0° = 1000µs, 180° = 2000µs
//    uint32_t pulse = (uint32_t)(600  + (angle / 180.0f) * 2600);
//    
//    // Ghi vào TIM5 Channel 1
//    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, pulse);
//}

void Parse_UART_Data(char *buf)
{	
		if(buf == NULL || buf[0] == '\0') return;
		if(uart_mode == 0) return;
    // ← THÊM: check lệnh đổi mode trước
    if(strncmp(buf, "MODE:APP", 8) == 0) {
        uart_mode = 0;
        return;
    }
    if(strncmp(buf, "MODE:ARM", 8) == 0) {
        uart_mode = 1;
        return;
    }

    // giữ nguyên phần cũ
    float a1, a2, a3;
    if(sscanf(buf, "$%f,%f,%f", &a1, &a2, &a3) == 3)
    {
        uart_angle1 = a1;
        uart_angle2 = a2;
        uart_angle3 = a3;
        uart_data_ready = 1;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
   if(huart->Instance == USART1)
    {
        uart_irq_count++;
				uint32_t now = HAL_GetTick();  // ← THÊM DÒNG NÀY
        static uint32_t last_update_time = 0;
    if(uart_rx_byte == '\n') {
        uart_buf[uart_index] = '\0';

        last_update_time = now;
        // ← SỬA: MODE commands luôn được parse
        if(uart_mode == 1 || strncmp((char*)uart_buf, "MODE:", 5) == 0)
            Parse_UART_Data((char*)uart_buf);
        uart_index = 0;
    } else {
        if(uart_index < 31)
            uart_buf[uart_index++] = uart_rx_byte;
    }
    HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
    }
    else if(huart->Instance == USART2)                  // ← THÊM block mới
    {
        uart_irq_count++;
    if(uart2_rx_byte == '\n') {
        uart2_buf[uart2_index] = '\0';
        if(uart_mode == 0 || strncmp((char*)uart2_buf, "MODE:", 5) == 0)
            Parse_UART_Data((char*)uart2_buf);
        uart2_index = 0;
    } else {
        if(uart2_index < 31)
            uart2_buf[uart2_index++] = uart2_rx_byte;
    }
    HAL_UART_Receive_IT(&huart2, &uart2_rx_byte, 1);;
    }
    
}
uint8_t Is_At_Home(void)
{
    // Kiểm tra cả 3 motor đã về 0° chưa
    return (labs(pos1) <= 5 && labs(pos2) <= 5 && labs(pos3) <= 5);
}



//void Servo_SetAngle(float angle)
//{
//    if(angle < 0)   angle = 0;
//    if(angle > 180) angle = 180;
//    uint32_t pulse = (uint32_t)(50 + (angle / 180.0f) * 200);
//    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pulse);
//}


//void Hook_Control(uint8_t state)
//{
//     if(state == 1)
//        Servo_TIM9_SetAngle(HOOK_OPEN);
//    else
//        Servo_TIM9_SetAngle(HOOK_CLOSE);
//}
HAL_StatusTypeDef kp(uint8_t channel)
{
    if(channel > 7) return HAL_ERROR;
    uint8_t data = (uint8_t)(1 << channel);
    return HAL_I2C_Master_Transmit(&hi2c2, TCA9548A_ADDR, &data, 1, 20);
}

HAL_StatusTypeDef AS5600_ReadRawAngle(uint16_t *raw_angle)
{
    uint8_t reg = AS5600_RAW_ANGLE_REG;
    uint8_t buf[2];
    HAL_StatusTypeDef status;

    status = HAL_I2C_Master_Transmit(&hi2c2, AS5600_ADDR, &reg, 1, 20);
    if(status != HAL_OK) return status;

    status = HAL_I2C_Master_Receive(&hi2c2, AS5600_ADDR, buf, 2, 20);
    if(status != HAL_OK) return status;

    *raw_angle = (uint16_t)(((buf[0] << 8) | buf[1]) & 0x0FFF);
    return HAL_OK;
}

float AS5600_ReadAngleDeg(uint8_t mux_channel, float zero_offset, uint8_t invert, volatile uint16_t *out_raw, volatile uint8_t *out_status)
{
    uint16_t raw;
    HAL_StatusTypeDef status;

    status = kp(mux_channel);
    if(status != HAL_OK) {
        if(out_status) *out_status = (uint8_t)status;
        return -1.0f;
    }

    status = AS5600_ReadRawAngle(&raw);
    if(out_status) *out_status = (uint8_t)status;
    if(status != HAL_OK) return -1.0f;

    if(out_raw) *out_raw = raw;

    float deg = (float)raw * 360.0f / 4096.0f;
    if(invert) deg = 360.0f - deg;

    deg -= zero_offset;
    if(deg < 0)    deg += 360.0f;
    if(deg >= 360.0f) deg -= 360.0f;

    return deg;
}
Position3D_t Forward_Kinematics(float theta1_deg, float theta2_deg, float theta3_deg)
{
    Position3D_t pos;
    float t1 = theta1_deg * 3.14159f / 180.0f;
    float t2 = theta2_deg * 3.14159f / 180.0f;
    float t3 = theta3_deg * 3.14159f / 180.0f;
    float r  = L1 * cosf(t2) + L2 * cosf(t2 + t3);
    pos.x = r * cosf(t1);
    pos.y = r * sinf(t1);
    pos.z = L0 + L1 * sinf(t2) + L2 * sinf(t2 + t3);
    return pos;
}
void Motor_MoveToAngle(StepperMotor_t *motor, MotorState_t *state, volatile int32_t *pos, float target_angle)
{
    // Chuyển góc mục tiêu sang steps
    int32_t target_steps = (int32_t)(target_angle * STEPS_PER_REV / 360.0f);
    int32_t error = target_steps - *pos;

    // Đã đến đích → dừng
    if(labs(error) <= 2)
    {
        state->target_speed = 0;
        state->current_speed = 0;
        __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, 0);
        return;
    }

    // Xác định chiều quay
    if(error > 0)
        Motor_SetDirection(motor, 0);
    else
        Motor_SetDirection(motor, 1);

    // Tốc độ tỉ lệ với khoảng cách còn lại (giảm tốc khi gần đích)
    int32_t speed;
    int32_t dist = labs(error);
    if(dist > 500)
        speed = state->max_speed;
    else
				speed = MIN_SPEED + (int32_t)((float)dist / 500.0f * (state->max_speed - MIN_SPEED));

    Motor_SetSpeedSmooth(motor, state, speed);

    // Đếm steps
    if(motor->direction == 0)
        (*pos)++;
    else
        (*pos)--;
}
void Go_Home(void)
{
     static uint8_t homing_active = 0;
    static uint32_t homing_start = 0;
    static uint32_t last_send_time = 0;
		
    // ← INIT LẦN ĐẦU
    if(!homing_active && home_requested)
    {
        homing_start = HAL_GetTick();
        homing_active = 1;
        last_send_time = 0;
				

        PID_Reset(&pid1);
        PID_Reset(&pid2);
        PID_Reset(&pid3);
        return;
    }

    if(!homing_active) return;

    // ===== TIMEOUT =====
    if(HAL_GetTick() - homing_start > HOMING_TIMEOUT_MS)
    {
        Motor_SetSpeedSmooth(&motor1, &state1, 0);
        Motor_SetSpeedSmooth(&motor2, &state2, 0);
        Motor_SetSpeedSmooth(&motor3, &state3, 0);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
        home_requested = 0;
        homing_active = 0;
        return;
    }


    
		// ===== Cập nhật uart_angle = home_angle =====
  
    // ===== ĐỌC GÓC THỰC TỪ ADC =====
     
    // ===== TÍNH SAI SỐ =====
    float err1 = home_angle1 - filtered_j1;
    float err2 = home_angle2 - filtered_j2;
    float err3 = home_angle3 - filtered_j3;

    // ===== KIỂM TRA ĐẠT HOME? =====
    if(fabsf(err1) <= 2.0f && fabsf(err2) <= 2.0f && fabsf(err3) <= 2.0f)
    {
        Motor_SetSpeedSmooth(&motor1, &state1, 0);
        Motor_SetSpeedSmooth(&motor2, &state2, 0);
        Motor_SetSpeedSmooth(&motor3, &state3, 0);
        home_requested = 0;
        control_enabled = 0;
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
        home_done = 1;
				homing_active = 0;
        return;
    }

    // ===== ĐIỀU KHIỂN MOTOR 1 =====
    if(fabsf(err1) > 1.0f)
    {
        float pid_out1 = PID_Compute(&pid1, err1);
        uint8_t dir1 = pid_out1 > 0 ? 0 : 1;
        Motor_SetDirection(&motor1, dir1);
        int32_t spd1 = (int32_t)fabsf(pid_out1);
        if(spd1 > 20000) spd1 = 20000;
        if(spd1 < 500) spd1 = 500;
        Motor_SetSpeedSmooth(&motor1, &state1, spd1);
    }
    else
    {
        Motor_SetSpeedSmooth(&motor1, &state1, 0);
    }

    // ===== ĐIỀU KHIỂN MOTOR 2 =====
    if(fabsf(err2) > 1.0f)
    {
        float pid_out2 = PID_Compute(&pid2, err2);
        uint8_t dir2 = pid_out2 > 0 ? 1 : 0;
        Motor_SetDirection(&motor2, dir2);
        int32_t spd2 = (int32_t)fabsf(pid_out2);
        if(spd2 > 20000) spd2 = 20000;
        if(spd2 < 500) spd2 = 500;
        Motor_SetSpeedSmooth(&motor2, &state2, spd2);
    }
    else
    {
        Motor_SetSpeedSmooth(&motor2, &state2, 0);
    }

    // ===== ĐIỀU KHIỂN MOTOR 3 =====
    if(fabsf(err3) > 1.0f)
    {
        float pid_out3 = PID_Compute(&pid3, err3);
        uint8_t dir3 = pid_out3 > 0 ? 1 : 0;
        Motor_SetDirection(&motor3, dir3);
        int32_t spd3 = (int32_t)fabsf(pid_out3);
        if(spd3 > 5000) spd3 = 5000;
        if(spd3 < 200) spd3 = 200;
        Motor_SetSpeedSmooth(&motor3, &state3, spd3);
    }
    else
    {
        Motor_SetSpeedSmooth(&motor3, &state3, 0);
    }
}
/**
  * @brief  Khoi tao dong co
  * @param  motor: con tro den struct dong co
  */
void Motor_Init(StepperMotor_t *motor)
{
    HAL_GPIO_WritePin(motor->en_port, motor->en_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, GPIO_PIN_RESET);

    __HAL_TIM_SET_AUTORELOAD(motor->htim, 999);
    __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, 0); // ← 500→0

    HAL_TIM_PWM_Start(motor->htim, motor->channel);

    if(motor->htim->Instance == TIM1)
        __HAL_TIM_MOE_ENABLE(motor->htim);  // ← sau Start ✅

    HAL_Delay(10);
    HAL_GPIO_WritePin(motor->en_port, motor->en_pin, GPIO_PIN_RESET);
}

/**
  * @brief  Dat toc do dong co
  * @param  motor: con tro den struct dong co
  * @param  speed: 0-4095 (tu ADC)
  */
void Motor_SetSpeed(StepperMotor_t *motor, uint16_t speed)
{
    motor->current_speed = speed;
    
    // Toc do qua thap -> Dung dong co
    if(speed < 100)
    {
        __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, 0);
        return;
    }
    
    // Chuyen doi ADC (0-4095) sang toc do buoc/giay
    // Pham vi: 50 -> 8000 steps/s
    uint32_t steps_per_sec = 50 + ((uint32_t)speed * 7950) / 4095;
    
    // Tinh ARR:
    // Timer clock = 1MHz (sau Prescaler)
    // Tan so PWM = steps_per_sec
    // ARR = 1,000,000 / (2 * steps_per_sec) - 1
    uint32_t arr = (1000000UL /  steps_per_sec) - 1;
    
    // Gioi han ARR trong pham vi hop le
    if(arr > 65535) arr = 65535;
    if(arr < 50)    arr = 50;
    
    // Cap nhat Timer
    __HAL_TIM_SET_AUTORELOAD(motor->htim, arr);
    __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, arr / 2); // 50% duty cycle
}

/**
  * @brief  Dat huong quay dong co
  * @param  motor: con tro den struct dong co
  * @param  dir: 0 = thuan, 1 = nguoc
  */
void Motor_SetDirection(StepperMotor_t *motor, uint8_t dir)
{
    motor->direction = dir;
    HAL_GPIO_WritePin(
        motor->dir_port,
        motor->dir_pin,
        dir ? GPIO_PIN_SET : GPIO_PIN_RESET
    );
}

/**
  * @brief  Bat dong co (EN = LOW)
  * @param  motor: con tro den struct dong co
  */
void Motor_Enable(StepperMotor_t *motor)
{
    HAL_GPIO_WritePin(motor->en_port, motor->en_pin, GPIO_PIN_RESET);
}

/**
  * @brief  Tat dong co (EN = HIGH)
  * @param  motor: con tro den struct dong co
  */
void Motor_Disable(StepperMotor_t *motor)
{
    HAL_GPIO_WritePin(motor->en_port, motor->en_pin, GPIO_PIN_SET);
}

void kalmanFilter (KalmanParam* pKalman, float measure) {
  float kalman_gain = pKalman->kalmanUncertainty_exFlex / (pKalman->kalmanUncertainty_exFlex + MEAS_UNCERTAINTY);
	pKalman->kalmanEstimation = (pKalman->kalmanExtrapolate) + kalman_gain * (measure - pKalman->kalmanExtrapolate);
	pKalman->kalmanUncertainty_currFlex = (1 - kalman_gain) * (pKalman->kalmanUncertainty_exFlex);
	pKalman->kalmanUncertainty_exFlex = pKalman->kalmanUncertainty_currFlex + PROCESS_NOISE;
	pKalman->kalmanExtrapolate = pKalman->kalmanEstimation;
}
/**
  * @brief  Xu ly du lieu ADC va cap nhat toc do 3 motor
  *         Goi trong vong lap chinh moi 10ms
  */


void Slave_Feedback_Read(void)
{
	float a1 = AS5600_ReadAngleDeg(MUX_CH_J1, as5600_zero_offset1, as5600_invert1, &debug_as5600_raw1, &debug_as5600_status1);
	float a2 = AS5600_ReadAngleDeg(MUX_CH_J2, as5600_zero_offset2, as5600_invert2, &debug_as5600_raw2, &debug_as5600_status2);
	float a3 = AS5600_ReadAngleDeg(MUX_CH_J3, as5600_zero_offset3, as5600_invert3, &debug_as5600_raw3, &debug_as5600_status3);

	    if(a1 >= 0) watch_joint1_real = a1;
	    if(a2 >= 0) watch_joint2_real = a2;
	    if(a3 >= 0) watch_joint3_real = a3;
	    // LUÔN gửi FB về Arduino qua USART1

    // LUÔN gửi FB về Arduino qua USART1
    char buf[64];
    int len = snprintf(buf, sizeof(buf),
        "FB:%.1f,%.1f,%.1f\n",
        watch_joint1_real, watch_joint2_real, watch_joint3_real);
    HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, 30);

    // Log về PC
    
}
void Motor_SetSpeedSmooth(StepperMotor_t *motor, MotorState_t *state, int32_t target_steps_per_sec)
{
    state->target_speed = target_steps_per_sec;

    if(state->current_speed < state->target_speed)
    {
						state->current_speed += state->accel_step;        if(state->current_speed > state->target_speed)
            state->current_speed = state->target_speed;
    }
    else if(state->current_speed > state->target_speed)
    {
        state->current_speed -= state->accel_step;
        if(state->current_speed < state->target_speed)
            state->current_speed = state->target_speed;
    }

    if(state->current_speed < MIN_SPEED || state->target_speed == 0)
    {
				state->current_speed = 0;
        __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, 0);
        return;
    }

    uint32_t arr = (1000000UL / state->current_speed) - 1;
    if(arr > 65535) arr = 65535;
    if(arr < 50)    arr = 50;

    __HAL_TIM_SET_AUTORELOAD(motor->htim, arr);
    __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, arr / 2);
}
void Process_ADC_Data(void){
	
	  // ===== Cờ: chỉ chạy sau khi nhận lệnh đầu tiên =====
    static uint8_t first_cmd = 0;
		if(uart_mode == 0)
    {
        first_cmd = 0;
        return;
    }
    if(uart_data_ready) {
        uart_data_ready = 0;
        if(!first_cmd) {
            // Lần đầu: init target_smooth = góc thực để không nhảy giật
            target_smooth1 = watch_joint1_real;
            target_smooth2 = watch_joint2_real;
            target_smooth3 = watch_joint3_real;
            first_cmd = 1;
        }
    }
    if(!first_cmd) return;

    // Đọc góc thực (low-pass filter — giữ nguyên)
   filtered_j1 = filtered_j1 * 0.8f + watch_joint1_real * 0.2f;
   filtered_j2 = filtered_j2 * 0.8f + watch_joint2_real * 0.2f;
   filtered_j3 = filtered_j3 * 0.8f + watch_joint3_real * 0.2f;
//     kalmanFilter(&angle1, watch_joint1_real);
//		kalmanFilter(&angle2, watch_joint2_real);
//		kalmanFilter(&angle3, watch_joint3_real);
//		float filtered_j1 = angle1.kalmanEstimation;
//    float filtered_j2 = angle2.kalmanEstimation;
//    float filtered_j3 = angle3.kalmanEstimation;

    // ===== MOTION PROFILING: target_smooth tiến dần đến uart_angle =====
    float step1 = VEL_PROFILE_J1 * LOOP_DT;  // = 3°/chu kỳ
    float step2 = VEL_PROFILE_J2 * LOOP_DT;  // = 3°/chu kỳ
    float step3 = VEL_PROFILE_J3 * LOOP_DT;  // = 1.5°/chu kỳ

    // J1
    if(target_smooth1 < uart_angle1 - step1)
        target_smooth1 += step1;
    else if(target_smooth1 > uart_angle1 + step1)
        target_smooth1 -= step1;
    else
        target_smooth1 = uart_angle1;  // đủ gần → snap đến đích

    // J2
    if(target_smooth2 < uart_angle2 - step2)
        target_smooth2 += step2;
    else if(target_smooth2 > uart_angle2 + step2)
        target_smooth2 -= step2;
    else
        target_smooth2 = uart_angle2;

    // J3
    if(target_smooth3 < uart_angle3 - step3)
        target_smooth3 += step3;
    else if(target_smooth3 > uart_angle3 + step3)
        target_smooth3 -= step3;
    else
        target_smooth3 = uart_angle3;

    // ===== Tính sai số — DÙNG target_smooth THAY VÌ uart_angle =====
    float err1 = target_smooth1 - filtered_j1;   // ← thay uart_angle1
    float err2 = target_smooth2 - filtered_j2;   // ← thay uart_angle2
    float err3 = target_smooth3 - filtered_j3;   // ← thay uart_angle3

    // ===== Debug =====
    debug_err1 = (int32_t)err1;
    debug_err2 = (int32_t)err2;
    debug_err3 = (int32_t)err3;

int32_t speed_cap1 = fabsf(err1) > 10.0f ? 90000
                   : fabsf(err1) >  5.0f ? 40000
                   :                       15000;
    // ===== JOINT 1 — PID =====
    // Ngưỡng dead-band 0.5°: tránh motor rung khi đã đủ gần
//    if(fabsf(err1) > 1.0f)
//    {
//        float pid_out1 = PID_Compute(&pid1, err1);
//        // Dấu output quyết định chiều quay
//        uint8_t dir1 = pid_out1 > 0 ? 0 : 1;
//        Motor_SetDirection(&motor1, dir1);
//        debug_dir1 = dir1;
//        // Lấy trị tuyệt đối làm tốc độ
//        int32_t spd1 = (int32_t)fabsf(pid_out1);
//				if     (fabsf(err1) > 5.0f) spd1 = spd1;            // xa: full speed
//    else if(fabsf(err1) > 2.0f) spd1 = spd1 * 4 / 10;   // trung: 40%
//    else                         spd1 = spd1 * 15 / 100; // gần: 15% — bò vào đích
//
//				if(spd1 > speed_cap1) spd1 = speed_cap1;  // ← THÊM clamp
//				if(spd1 < 0) spd1 = 0;
//        Motor_SetSpeedSmooth(&motor1, &state1, spd1);
//    }
//    else
//    {
//        // Đến đích → dừng và reset PID
//        // Reset quan trọng: xóa integral và prev_error
//        // để lần di chuyển tiếp theo bắt đầu sạch
//        PID_Reset(&pid1);
//        Motor_SetSpeedSmooth(&motor1, &state1, 0);
//        debug_err1 = 0;
//    }

    	// ===== JOINT 1 — TEST KHÔNG PID =====
    	if(fabsf(err1) > 1.0f)
    	{
    	    uint8_t dir1 = err1 > 0 ? 1 : 0;

    	    Motor_SetDirection(&motor1, dir1);
    	    debug_dir1 = dir1;

    	    int32_t spd1 = 1000;

    	    Motor_SetSpeedSmooth(&motor1, &state1, spd1);
    	}
    	else
    	{
    	    Motor_SetSpeedSmooth(&motor1, &state1, 0);
    	    debug_err1 = 0;
    	}
int32_t speed_cap2 = fabsf(err2) > 10.0f ? 90000
                   : fabsf(err2) >  5.0f ? 40000
                   :                       15000;
    // ===== JOINT 2 — PID =====
//    if(fabsf(err2) > 1.0f)
//    {
//        float pid_out2 = PID_Compute(&pid2, err2);
//        uint8_t dir2 = pid_out2 > 0 ? 1 : 0;  // J2 đảo chiều so với J1
//        Motor_SetDirection(&motor2, dir2);
//        debug_dir2 = dir2;
//        int32_t spd2 = (int32_t)fabsf(pid_out2);
//				if     (fabsf(err2) > 20.0f) spd2 = spd2;            // xa: full speed
//    else if(fabsf(err2) > 10.0f) spd2 = spd2 * 1 / 10;   // trung: 40%
//    else                         spd2 = spd2 * 5 / 100; // gần: 15%
//				if(spd2 > speed_cap2) spd2 = speed_cap2;
//				if(spd2 < 0) spd2 = 0;
//        Motor_SetSpeedSmooth(&motor2, &state2, spd2);
//    }
//    else
//    {
//       // PID_Reset(&pid2);
//        Motor_SetSpeedSmooth(&motor2, &state2, 0);
//        debug_err2 = 0;
//    }
// ===== JOINT 2 — TEST KHÔNG PID =====
if(fabsf(err2) > 1.0f)
{
    uint8_t dir2 = err2 > 0 ? 0 : 1;

    Motor_SetDirection(&motor2, dir2);
    debug_dir2 = dir2;

    int32_t spd2 = 1000;

    Motor_SetSpeedSmooth(&motor2, &state2, spd2);
}
else
{
    Motor_SetSpeedSmooth(&motor2, &state2, 0);
    debug_err2 = 0;
}

    // ===== JOINT 3 — PID =====
    if(fabsf(err3) > 2.0f)
    {
        float pid_out3 = PID_Compute(&pid3, err3);
        uint8_t dir3 = pid_out3 > 0 ? 1 : 0;
        Motor_SetDirection(&motor3, dir3);
        debug_dir3 = dir3;
        int32_t spd3 = (int32_t)fabsf(pid_out3);
				if     (fabsf(err3) > 20.0f) spd3 = spd3;            // xa: full
    else if(fabsf(err3) >  15.0f) spd3 = spd3 * 3 / 10;   // trung: 30%
    else                          spd3 = spd3 * 5 / 100; 
				if(spd3 > state3.max_speed) spd3 = state3.max_speed;  // ← THÊM clamp
				if(spd3 < 0) spd3 = 0;
        Motor_SetSpeedSmooth(&motor3, &state3, spd3);
    }
    else
    {
        PID_Reset(&pid3);
        Motor_SetSpeedSmooth(&motor3, &state3, 0);
        debug_err3 = 0;
    }

    // Cập nhật angle_current
    angle_current1 = filtered_j1;
    angle_current2 = filtered_j2;
    angle_current3 = filtered_j3;

    // Forward Kinematics
    tool_pos = Forward_Kinematics(angle_current1, angle_current2, angle_current3);
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
 
    // B?t LED d? PD14 (c� s?n tr�n Discovery)
    __HAL_RCC_GPIOD_CLK_ENABLE();
    GPIOD->MODER |= (1 << 28);
    GPIOD->ODR   |= (1 << 14);
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
