/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
#define testVariable
#include "main.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "Motors.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    float   center;
    uint8_t count;
} Cluster;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUM_SENSORS     16
#define BASE_SPEED      300
#define MIN_SPEED       0
#define MAX_SPEED       1000

#define KP              16.9f
#define KI              0.0004f
#define KD              39.0f

#define TURBINE_ARM     1000
#define TURBINE_SPEED   1350
#define TURBINE_MAX     2000

#define OVERSAMPLING    4
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

/* USER CODE BEGIN PV */
static uint16_t          adc_raw[1];
static uint16_t          sensor_values[NUM_SENSORS];
static volatile uint8_t  current_sensor   = 0;
static volatile uint8_t  sample_count     = 0;
static volatile uint32_t sensor_accumulator[NUM_SENSORS];
static volatile uint8_t  sensors_ready    = 0;
static volatile uint8_t  scan_in_progress = 0;
static volatile uint16_t threshold_val[NUM_SENSORS];

static float   pid_error      = 0.0f;
static float   pid_last_error = 0.0f;
static float   pid_integral   = 0.0f;
static float   pid_position   = 75.0f;
static int32_t left_speed     = 0;
static int32_t right_speed    = 0;
static uint8_t line_is_lost   = 0;
static uint8_t junction_detected = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);

/* USER CODE BEGIN PFP */
static void start_sensor_scan(void);
static void pid_update(void);
static void update_leds(void);
static void calibrate_sensors(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void set_mux(uint8_t ch)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, (ch & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, ((ch >> 1) & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, ((ch >> 2) & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, ((ch >> 3) & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    for (int i = 0; i < 30; i++) { __NOP(); }
}

static void start_sensor_scan(void)
{
    if (scan_in_progress) return;

    scan_in_progress = 1;
    sensors_ready    = 0;
    current_sensor   = 0;
    sample_count     = 0;

    set_mux(current_sensor);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_raw, 1);
}

static uint8_t find_clusters(Cluster *clusters)
{
    uint8_t num_clusters = 0;
    uint8_t in_cluster   = 0;
    float   weighted_sum = 0.0f;
    float   total_weight = 0.0f;
    uint8_t count        = 0;

    for (uint8_t i = 0; i < NUM_SENSORS; i++)
    {
        if (sensor_values[i] > threshold_val[i])
        {
            weighted_sum += (i * 10.0f) * (float)sensor_values[i];
            total_weight += (float)sensor_values[i];
            count++;
            in_cluster = 1;
        }
        else if (in_cluster)
        {
            if (num_clusters < 4)
            {
                clusters[num_clusters].center = weighted_sum / total_weight;
                clusters[num_clusters].count  = count;
                num_clusters++;
            }
            weighted_sum = 0.0f;
            total_weight = 0.0f;
            count        = 0;
            in_cluster   = 0;
        }
    }

    if (in_cluster && num_clusters < 4)
    {
        clusters[num_clusters].center = weighted_sum / total_weight;
        clusters[num_clusters].count  = count;
        num_clusters++;
    }
    return num_clusters;
}

static void pid_update(void)
{
    Cluster clusters[4];
    uint8_t num_clusters = find_clusters(clusters);

    if (num_clusters == 0)
    {
        line_is_lost = 1;
        pid_integral = 0.0f;

        if (pid_last_error > 0.0f)
        {
            left_speed  = BASE_SPEED;
            right_speed = 0;
        }
        else
        {
            left_speed  = 0;
            right_speed = BASE_SPEED;
        }
    }
    else
    {
        line_is_lost      = 0;
        junction_detected = (num_clusters > 1 || clusters[0].count > 8) ? 1 : 0;

        float best_pos = clusters[0].center;
        float min_diff = fabsf(clusters[0].center - pid_position);
        for (uint8_t i = 1; i < num_clusters; i++)
        {
            float diff = fabsf(clusters[i].center - pid_position);
            if (diff < min_diff)
            {
                min_diff = diff;
                best_pos = clusters[i].center;
            }
        }

        pid_position  = best_pos;
        pid_error     = pid_position - 75.0f;

        pid_integral += pid_error;
        if (pid_integral >  5000.0f) pid_integral =  5000.0f;
        if (pid_integral < -5000.0f) pid_integral = -5000.0f;

        float derivative = pid_error - pid_last_error;
        pid_last_error   = pid_error;

        float current_base;
        if (junction_detected)
            current_base = BASE_SPEED * 0.5f;
        else if (fabsf(pid_error) > 40.0f)
            current_base = BASE_SPEED * 0.6f;
        else
            current_base = (float)BASE_SPEED;

        float correction = (KP * pid_error) + (KI * pid_integral) + (KD * derivative);

        left_speed  = (int32_t)(current_base + correction);
        right_speed = (int32_t)(current_base - correction);
    }

    if (left_speed  > MAX_SPEED) left_speed  = MAX_SPEED;
    if (left_speed  < MIN_SPEED) left_speed  = MIN_SPEED;
    if (right_speed > MAX_SPEED) right_speed = MAX_SPEED;
    if (right_speed < MIN_SPEED) right_speed = MIN_SPEED;

    set_lmotor_speed(left_speed);
    set_rmotor_speed(right_speed);
}

static void update_leds(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12 | GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2  | GPIO_PIN_3,  GPIO_PIN_RESET);

    if (line_is_lost)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12 | GPIO_PIN_13, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2  | GPIO_PIN_3,  GPIO_PIN_SET);
        return;
    }

    if      (pid_position <  40.0f) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    else if (pid_position <  70.0f) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    else if (pid_position < 110.0f) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3,  GPIO_PIN_SET);
    else                            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2,  GPIO_PIN_SET);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance != ADC1) return;

    sensor_accumulator[current_sensor] += adc_raw[0];
    sample_count++;

    if (sample_count < OVERSAMPLING)
    {
        HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_raw, 1);
    }
    else
    {
        sensor_values[current_sensor] = (uint16_t)(sensor_accumulator[current_sensor] / OVERSAMPLING);
        sensor_accumulator[current_sensor] = 0;
        sample_count = 0;
        current_sensor++;

        if (current_sensor < NUM_SENSORS)
        {
            set_mux(current_sensor);
            HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_raw, 1);
        }
        else
        {
            scan_in_progress = 0;
            sensors_ready    = 1;
        }
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM5)
    {
        start_sensor_scan();
    }
}

static void calibrate_sensors(void)
{
    uint16_t min_val[NUM_SENSORS];
    uint16_t max_val[NUM_SENSORS];

    for (int i = 0; i < NUM_SENSORS; i++) {
        min_val[i] = 4095;
        max_val[i] = 0;
    }

    HAL_TIM_Base_Stop_IT(&htim5);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12 | GPIO_PIN_13, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2  | GPIO_PIN_3,  GPIO_PIN_SET);

    uint32_t t_start = HAL_GetTick();
    while ((HAL_GetTick() - t_start) < 3000)
    {
        scan_in_progress = 0;
        start_sensor_scan();
        while (!sensors_ready);
        sensors_ready = 0;

        for (int i = 0; i < NUM_SENSORS; i++)
        {
            if (sensor_values[i] < min_val[i]) min_val[i] = sensor_values[i];
            if (sensor_values[i] > max_val[i]) max_val[i] = sensor_values[i];
        }
    }

    for (int i = 0; i < NUM_SENSORS; i++)
        threshold_val[i] = (min_val[i] + max_val[i]) / 2;

    uint32_t sum = 0;
    for (int i = 0; i < NUM_SENSORS; i++)
        sum += threshold_val[i];
    uint16_t avg_threshold = (uint16_t)(sum / NUM_SENSORS);

    if (avg_threshold < 1024 || avg_threshold > 3072)
    {
        for (int i = 0; i < NUM_SENSORS; i++)
            threshold_val[i] = 2048;
    }

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12 | GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2  | GPIO_PIN_3,  GPIO_PIN_RESET);

    HAL_TIM_Base_Start_IT(&htim5);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USB_DEVICE_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

  HAL_TIM_Base_Start_IT(&htim5);

  init_left_driver();
  init_right_driver();

  calibrate_sensors();

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  TIM3->CCR2 = TURBINE_MAX;
  HAL_Delay(2000);

  TIM3->CCR2 = TURBINE_ARM;
  HAL_Delay(3000);

  TIM3->CCR2 = TURBINE_SPEED;

  start_sensor_scan();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (sensors_ready)
    {
        sensors_ready = 0;
        pid_update();
        update_leds();
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

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM       = 8;
  RCC_OscInitStruct.PLL.PLLN       = 168;
  RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ       = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                   | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) Error_Handler();
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

  hadc1.Instance                   = ADC1;
  hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV6;
  hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode          = DISABLE;
  hadc1.Init.ContinuousConvMode    = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion       = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();

  sConfig.Channel      = ADC_CHANNEL_14;
  sConfig.Rank         = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();

  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef      sConfigOC     = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */

  htim2.Instance               = TIM2;
  htim2.Init.Prescaler         = 84 - 1;
  htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim2.Init.Period            = 1000 - 1;
  htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) Error_Handler();

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) Error_Handler();

  sConfigOC.OCMode     = TIM_OCMODE_PWM1;
  sConfigOC.Pulse      = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) Error_Handler();

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef      sConfigOC     = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */

  htim3.Instance               = TIM3;
  htim3.Init.Prescaler         = 84;
  htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim3.Init.Period            = 1674;
  htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) Error_Handler();

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) Error_Handler();

  sConfigOC.OCMode     = TIM_OCMODE_PWM1;
  sConfigOC.Pulse      = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) Error_Handler();

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef      sConfigOC     = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */

  htim4.Instance               = TIM4;
  htim4.Init.Prescaler         = 84;
  htim4.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim4.Init.Period            = 1000 - 1;
  htim4.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) Error_Handler();

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK) Error_Handler();

  sConfigOC.OCMode     = TIM_OCMODE_PWM1;
  sConfigOC.Pulse      = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) Error_Handler();

  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

  HAL_TIM_MspPostInit(&htim4);
}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{
  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef  sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig      = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */

  htim5.Instance               = TIM5;
  htim5.Init.Prescaler         = 84 - 1;
  htim5.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim5.Init.Period            = 10000 - 1;
  htim5.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK) Error_Handler();

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK) Error_Handler();

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK) Error_Handler();

  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{
  __HAL_RCC_DMA2_CLK_ENABLE();
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
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

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_10 | GPIO_PIN_11, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2  | GPIO_PIN_3  | GPIO_PIN_6  | GPIO_PIN_7,  GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1  | GPIO_PIN_2  | GPIO_PIN_12 | GPIO_PIN_13, GPIO_PIN_RESET);

  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_10 | GPIO_PIN_11;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_6 | GPIO_PIN_7;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_12 | GPIO_PIN_13;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
  __disable_irq();
  while (1) {}
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
  (void)file; (void)line;
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
