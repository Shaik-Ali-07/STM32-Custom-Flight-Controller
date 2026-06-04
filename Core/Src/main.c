/* USER CODE BEGIN Header */
/**
******************************************************************************
* @file : main.c
* @brief : Main program body
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
#include "iir.h"
#include "mpu6500.h"
#include<math.h>
#include<stdio.h>
#include "MadgwickAHRS.h"
#include "pid.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
FLIGHT_IDLE,
FLIGHT_CLIMB,
FLIGHT_HOVER,
FLIGHT_LAND
} FlightState_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define sampling_freq 1000.0000000000f
#define cutoff_freq 30.0000000000f
#define ESC_MIN_PULSE 1000 // 1 ms = 0% throttle
#define ESC_MAX_PULSE 2000 // 2 ms = 100% throttle

// --- NEW TEST FLIGHT SETTINGS ---
#define HOVER_THROTTLE 1400.0f // Approx 35% throttle (Adjust for your weight!)
#define CLIMB_RATE 0.3f // Increases throttle by 0.5 every 10ms
#define LAND_RATE 0.2f // Decreases throttle by 0.3 every 10ms
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
static float esc_filtered[4] = {1000,1000,1000,1000};
#define OUTPUT_ALPHA 0.3f // Low-pass filter PWM

MPU6500_t mpu6500;
IIR_filter iir_a[3],iir_g[3];
PID pid_roll, pid_pitch;

float accel_f[3], gyro_f[3];

float pitch,roll;

float m1 = 0, m2 = 0, m3 = 0, m4 = 0;
volatile float throttle;
float setpoint_roll = 0.0f, setpoint_pitch = 0.0f;

FlightState_t current_state = FLIGHT_IDLE;
uint32_t state_start_time = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
float constrain(float value, float min, float max);

void set_esc_speed1(uint16_t pulse_us)
{
// Constrain the value between min and max
if(pulse_us < ESC_MIN_PULSE) pulse_us = ESC_MIN_PULSE;
if(pulse_us > ESC_MAX_PULSE) pulse_us = ESC_MAX_PULSE;

__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse_us);
}

void set_esc_speed2(uint16_t pulse_us)
{
// Constrain the value between min and max
if(pulse_us < ESC_MIN_PULSE) pulse_us = ESC_MIN_PULSE;
if(pulse_us > ESC_MAX_PULSE) pulse_us = ESC_MAX_PULSE;

__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulse_us);
}

void set_esc_speed3(uint16_t pulse_us)
{
// Constrain the value between min and max
if(pulse_us < ESC_MIN_PULSE) pulse_us = ESC_MIN_PULSE;
if(pulse_us > ESC_MAX_PULSE) pulse_us = ESC_MAX_PULSE;

__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pulse_us);
}

void set_esc_speed4(uint16_t pulse_us)
{
// Constrain the value between min and max
if(pulse_us < ESC_MIN_PULSE) pulse_us = ESC_MIN_PULSE;
if(pulse_us > ESC_MAX_PULSE) pulse_us = ESC_MAX_PULSE;

__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pulse_us);
}

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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

if(MPU6500_Init(&mpu6500, &hspi1, CS_GPIO_Port, CS_Pin) != HAL_OK){
while(1){
HAL_Delay(1000);
}
}
MPU6500_CalibrateGyro(&mpu6500,1000);

for(int i = 0; i < 3; i++) {
IIR_Init(&iir_a[i], cutoff_freq, sampling_freq);
IIR_Init(&iir_g[i], cutoff_freq, sampling_freq);
}

mpu6500.dma_transfer_complete = true;

// Initialize cascaded PIDs (tune these values)
PID_Init(&pid_roll);
pid_roll.Kp = 1.4153f, pid_roll.Ki = 0.0f, pid_roll.Kd = 0.0f;//1.4153
pid_roll.Tau = 0.02f;
pid_roll.maxlim = 400.0f, pid_roll.minlim = -400.0f;

PID_Init(&pid_pitch);
pid_pitch.Kp = 1.4153f, pid_pitch.Ki = 0.0f, pid_pitch.Kd =0.0f;
pid_pitch.Tau = 0.02f;
pid_pitch.maxlim = 400.0f, pid_pitch.minlim = -400.0f;

HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

set_esc_speed1(ESC_MIN_PULSE);
set_esc_speed2(ESC_MIN_PULSE);
set_esc_speed3(ESC_MIN_PULSE);
set_esc_speed4(ESC_MIN_PULSE);

// ESC Arming Sequence - VERY IMPORTANT
// Send minimum throttle for longer to ensure ESC is ready

HAL_Delay(8000);
HAL_TIM_Base_Start_IT(&htim3);
throttle = 1000;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
while (1)
{
if (fabs(roll) > 30.0f || fabs(pitch) > 30.0f) {
throttle = 1000.0f; // Instant 0% throttle
current_state = FLIGHT_IDLE; // Reset state machine
state_start_time = 0; // Reset timer
while(1){
	HAL_Delay(10);
}
}

switch (current_state) {
case FLIGHT_IDLE:
throttle = 1000.0f;

// Initial start after 13s (to let gyro calibrate and you to step back)
// Then wait 10s between subsequent flights
if (state_start_time == 0) {
if (HAL_GetTick() > 10000) {
current_state = FLIGHT_CLIMB;
}
} else {
if (HAL_GetTick() - state_start_time > 10000) {
current_state = FLIGHT_CLIMB;
}
}
break;

case FLIGHT_CLIMB:
// Smoothly ramp up throttle
if (throttle < HOVER_THROTTLE) {
throttle += CLIMB_RATE;
} else {
// Target reached, switch to Hover
current_state = FLIGHT_HOVER;
state_start_time = HAL_GetTick();
}
break;

case FLIGHT_HOVER:
throttle = HOVER_THROTTLE;
// Hold altitude for 5 seconds
if (HAL_GetTick() - state_start_time > 15000) {
current_state = FLIGHT_LAND;
}
break;

case FLIGHT_LAND:
// Smoothly ramp down
if (throttle > 1000.0f) {
throttle -= LAND_RATE;
} else {
// Landed
throttle = 1000.0f;
current_state = FLIGHT_IDLE;
state_start_time = HAL_GetTick(); // Mark time of landing
}
break;
}

// This ensures CLIMB_RATE adds up smoothly over time
HAL_Delay(10);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 200;
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  htim2.Init.Prescaler = 100-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000-1;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 100-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Debug_Pin_GPIO_Port, Debug_Pin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : Debug_Pin_Pin CS_Pin */
  GPIO_InitStruct.Pin = Debug_Pin_Pin|CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
if(hspi->Instance == SPI1) {
// End CS HIGH
HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

//Debug pin
HAL_GPIO_WritePin(Debug_Pin_GPIO_Port, Debug_Pin_Pin, GPIO_PIN_SET);

// Parse raw DMA data
MPU6500_DMA_Complete_Callback(&mpu6500);

// Convert to engineering units (g, rad/s)
mpu6500.accel_g[0] = (float)mpu6500.accel_raw[0] / mpu6500.accel_scale;
mpu6500.accel_g[1] = (float)mpu6500.accel_raw[1] / mpu6500.accel_scale;
mpu6500.accel_g[2] = (float)mpu6500.accel_raw[2] / mpu6500.accel_scale;

mpu6500.gyro_dps[0] = ((float)mpu6500.gyro_raw[0] / mpu6500.gyro_scale) - mpu6500.gyro_bias[0];
mpu6500.gyro_dps[1] = ((float)mpu6500.gyro_raw[1] / mpu6500.gyro_scale) - mpu6500.gyro_bias[1];
mpu6500.gyro_dps[2] = ((float)mpu6500.gyro_raw[2] / mpu6500.gyro_scale) - mpu6500.gyro_bias[2];

mpu6500.gyro_rad[0] = mpu6500.gyro_dps[0] * 0.017453292f; // deg_to_rad
mpu6500.gyro_rad[1] = mpu6500.gyro_dps[1] * 0.017453292f;
mpu6500.gyro_rad[2] = mpu6500.gyro_dps[2] * 0.017453292f;

// Apply IIR filtering
for(int i = 0; i < 3; i++){
accel_f[i] = IIR_Update(&iir_a[i], mpu6500.accel_g[i]);
gyro_f[i] = IIR_Update(&iir_g[i], mpu6500.gyro_rad[i]);
}

// Update orientation filter
MadgwickAHRSupdateIMU(gyro_f[0], gyro_f[1], gyro_f[2],
accel_f[0], accel_f[1], accel_f[2]);

roll = atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2)) * 57.2958f;
pitch = asinf(-2.0f*(q1*q3 - q0*q2)) * 57.2958f;

// Mark ready for next cycle
mpu6500.dma_transfer_complete = true;

// SINGLE-LOOP PID: Angle error → PWM directly
float pid_out_roll = PID_Update(&pid_roll, setpoint_roll, roll);
float pid_out_pitch = PID_Update(&pid_pitch, setpoint_pitch, pitch);

// Motor mixing (unchanged)
m1 = throttle + pid_out_roll - pid_out_pitch;//front - left
m2 = throttle - pid_out_roll - pid_out_pitch;//front-right
m3 = throttle - pid_out_roll + pid_out_pitch;//rear-right
m4 = throttle + pid_out_roll + pid_out_pitch;//rear-left

esc_filtered[0] = OUTPUT_ALPHA * m1 + (1.0f - OUTPUT_ALPHA) * esc_filtered[0];
esc_filtered[1] = OUTPUT_ALPHA * m2 + (1.0f - OUTPUT_ALPHA) * esc_filtered[1];
esc_filtered[2] = OUTPUT_ALPHA * m3 + (1.0f - OUTPUT_ALPHA) * esc_filtered[2];
esc_filtered[3] = OUTPUT_ALPHA * m4 + (1.0f - OUTPUT_ALPHA) * esc_filtered[3];

// Output to ESCs (unchanged)
set_esc_speed1((uint16_t)constrain(m1, 1000, 2000));
set_esc_speed2((uint16_t)constrain(m2, 1000, 2000));
set_esc_speed3((uint16_t)constrain(m3, 1000, 2000));
set_esc_speed4((uint16_t)constrain(m4, 1000, 2000));

HAL_GPIO_WritePin(Debug_Pin_GPIO_Port, Debug_Pin_Pin, GPIO_PIN_RESET);
}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
if(htim->Instance == TIM3){
// Only trigger new DMA read if previous one is complete
if(mpu6500.dma_transfer_complete == true){
MPU6500_ReadRaw_DMA(&mpu6500); // Non-blocking!
mpu6500.dma_transfer_complete = false;
}
// If DMA busy, skip this cycle (no overflow)
}
}

/* Add this to main.c */

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
if(hspi->Instance == SPI1) {
// 1. Force CS HIGH (release the MPU6500)
HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

// 2. Clear the 'Busy' flag so the loop can try again next time
mpu6500.dma_transfer_complete = true;

// Optional: Toggle the Debug Pin to see on logic analyzer/scope
HAL_GPIO_TogglePin(Debug_Pin_GPIO_Port, Debug_Pin_Pin);
}
}

float constrain(float value, float min, float max) {
if(value < min) return min;
if(value > max) return max;
return value;
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
__disable_irq();
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
