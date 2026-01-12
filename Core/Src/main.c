/* USER CODE END Header */
#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Variables */
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile int32_t g_total_left_ticks = 0;
volatile int32_t g_total_right_ticks = 0;

typedef struct { float Kp; float Ki; float Kd; float prevError; float integral; } PID_Config;


PID_Config pid_R = {50.0f, 0.01f, 0.1f, 0.0f, 0.0f};
PID_Config pid_L = {50.0f, 0.01f, 0.1f, 0.0f, 0.0f};

volatile float target_RPM_R = 0.0f;
volatile float target_RPM_L = 0.0f;
volatile float current_RPM_R = 0.0f;
volatile float current_RPM_L = 0.0f;
float pwm_output_R = 0.0f;
float pwm_output_L = 0.0f;

const float RPM_CONVERSION_FACTOR = 0.79f;

uint8_t rx_byte_temp;
char rx_buffer[32];
uint8_t rx_index = 0;
char tx_buffer[64];

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART2_UART_Init(void);
float PID_Compute(PID_Config *pid, float target, float current);
void Set_Motor_Right(float pwm_val);
void Set_Motor_Left(float pwm_val);

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_TIM2_CLK_ENABLE();
  __HAL_RCC_TIM3_CLK_ENABLE();
  __HAL_RCC_TIM4_CLK_ENABLE();
  __HAL_RCC_TIM5_CLK_ENABLE();
  __HAL_RCC_USART2_CLK_ENABLE();

  HAL_NVIC_SetPriority(TIM5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM5_IRQn);
  HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);

  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_USART2_UART_Init();

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_Base_Start_IT(&htim5);

  HAL_UART_Receive_IT(&huart2, &rx_byte_temp, 1);

  while (1)
  {

      int len = sprintf(tx_buffer, "L:%ld,R:%ld\n", g_total_left_ticks, g_total_right_ticks);
      HAL_UART_Transmit(&huart2, (uint8_t*)tx_buffer, len, 10);
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
      HAL_Delay(100);
  }
}

// 1. PID LOOP & ODOMETER UPDATE
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM5)
  {
      int16_t delta_R = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
      int16_t delta_L = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);

      __HAL_TIM_SET_COUNTER(&htim4, 0);
      __HAL_TIM_SET_COUNTER(&htim3, 0);

      // --- POLARITY FLIP (FORWARD FIX) ---
      delta_R = -delta_R;
      delta_L = -delta_L;

      g_total_right_ticks += delta_R;
      g_total_left_ticks  += delta_L;

      current_RPM_R = (float)delta_R * RPM_CONVERSION_FACTOR;
      current_RPM_L = (float)delta_L * RPM_CONVERSION_FACTOR;

      pwm_output_R = PID_Compute(&pid_R, target_RPM_R, current_RPM_R);
      pwm_output_L = PID_Compute(&pid_L, target_RPM_L, current_RPM_L);

      Set_Motor_Right(pwm_output_R);
      Set_Motor_Left(pwm_output_L);
  }
}

// 2. UART RECEIVE (Dual Targets: "L,R")
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
      if (rx_byte_temp == '\n' || rx_byte_temp == '\r')
      {
          rx_buffer[rx_index] = '\0';
          if (rx_index > 0)
          {
              // Parse "100.0,50.0"
              char *comma_ptr = strchr(rx_buffer, ',');
              if (comma_ptr != NULL) {
                  *comma_ptr = '\0';
                  float val_L = atof(rx_buffer);
                  float val_R = atof(comma_ptr + 1);
                  target_RPM_L = val_L;
                  target_RPM_R = val_R;
              } else {
                  // Fallback for single value
                  float val = atof(rx_buffer);
                  target_RPM_L = val;
                  target_RPM_R = val;
              }

              // Emergency Reset
              if (target_RPM_L == 0.0f && target_RPM_R == 0.0f) {
                  pid_R.integral = 0; pid_R.prevError = 0;
                  pid_L.integral = 0; pid_L.prevError = 0;
                  Set_Motor_Right(0);
                  Set_Motor_Left(0);
              }
          }
          rx_index = 0;
      }
      else
      {
          if (rx_index < 30) rx_buffer[rx_index++] = rx_byte_temp;
      }
      HAL_UART_Receive_IT(&huart2, &rx_byte_temp, 1);
  }
}

float PID_Compute(PID_Config *pid, float target, float current) {
    float error = target - current;
    float P = pid->Kp * error;
    pid->integral += error;
    if (pid->integral > 5000.0f) pid->integral = 5000.0f;
    if (pid->integral < -5000.0f) pid->integral = -5000.0f;
    float I = pid->Ki * pid->integral;
    float D = pid->Kd * (error - pid->prevError);
    pid->prevError = error;
    float output = P + I + D;
    if (output > 999.0f) output = 999.0f;
    if (output < -999.0f) output = -999.0f;
    return output;
}
void Set_Motor_Right(float pwm_val) {
    if (pwm_val > 0) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
        pwm_val = -pwm_val;
    }
    if(pwm_val > 999) pwm_val = 999;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint32_t)pwm_val);
}

void Set_Motor_Left(float pwm_val) {
    if (pwm_val > 0) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
        pwm_val = -pwm_val;
    }
    if(pwm_val > 999) pwm_val = 999;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (uint32_t)pwm_val);
}

static void MX_USART2_UART_Init(void) {
  huart2.Instance = USART2; huart2.Init.BaudRate = 115200; huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1; huart2.Init.Parity = UART_PARITY_NONE; huart2.Init.Mode = UART_MODE_TX_RX;
  HAL_UART_Init(&huart2);
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3; GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH; GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
static void MX_TIM2_Init(void) {
  TIM_OC_InitTypeDef sConfigOC = {0};
  htim2.Instance = TIM2; htim2.Init.Prescaler = 71; htim2.Init.Period = 999; htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  HAL_TIM_PWM_Init(&htim2);
  sConfigOC.OCMode = TIM_OCMODE_PWM1; sConfigOC.Pulse = 0; sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH; sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1); HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2);
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1; GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
static void MX_TIM3_Init(void) {
  TIM_Encoder_InitTypeDef sConfig = {0};
  htim3.Instance = TIM3; htim3.Init.Prescaler = 0; htim3.Init.Period = 65535;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1; sConfig.IC1Polarity = TIM_ICPOLARITY_RISING; sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  HAL_TIM_Encoder_Init(&htim3, &sConfig);
  HAL_TIMEx_MasterConfigSynchronization(&htim3, NULL);
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7; GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3; HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
static void MX_TIM4_Init(void) {
  TIM_Encoder_InitTypeDef sConfig = {0};
  htim4.Instance = TIM4; htim4.Init.Prescaler = 0; htim4.Init.Period = 65535;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1; sConfig.IC1Polarity = TIM_ICPOLARITY_RISING; sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  HAL_TIM_Encoder_Init(&htim4, &sConfig);
  HAL_TIMEx_MasterConfigSynchronization(&htim4, NULL);
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7; GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4; HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}
static void MX_TIM5_Init(void) {
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  htim5.Instance = TIM5; htim5.Init.Prescaler = 7199; htim5.Init.Period = 99; htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  HAL_TIM_Base_Init(&htim5); sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL; HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig);
}
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9|GPIO_PIN_11, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13; GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_11; GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_13; HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0}; RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  __HAL_RCC_PWR_CLK_ENABLE(); __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI; RCC_OscInitStruct.HSIState = RCC_HSI_ON; RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON; RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI; RCC_OscInitStruct.PLL.PLLM = 8; RCC_OscInitStruct.PLL.PLLN = 72; RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2; RCC_OscInitStruct.PLL.PLLQ = 4;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1; RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2; RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}
void Error_Handler(void) { __disable_irq(); while (1) {} }
