/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Line Follower 
  
  ******************************************************************************
  */
/* USER CODE END Header */

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

/* USER CODE BEGIN PD */
#define NUM_SENSORS     16
#define BASE_SPEED      300
#define MIN_SPEED       0
#define MAX_SPEED       1000

#define KP              16.9f
#define KI              0.0004f
#define KD              39.0f


#define TURBINE_ARM     1000   /* Min throttle pulse (µs equivalent ticks) */
#define TURBINE_SPEED   1350   /* Running throttle */
#define TURBINE_MAX     2000   /* Max throttle (used for arming high signal) */

#define OVERSAMPLING    4      /* ADC samples averaged per sensor per scan */
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

/* USER CODE BEGIN PV */
/* --- Sensor state --------------------------------------------------------- */
static uint16_t adc_raw[1];
static uint16_t sensor_values[NUM_SENSORS];
static volatile uint8_t  current_sensor  = 0;
static volatile uint8_t  sample_count    = 0;   /* samples taken for THIS sensor */
static volatile uint32_t sensor_accumulator[NUM_SENSORS];
static volatile uint8_t  sensors_ready   = 0;   /* set by ISR, cleared by main */
static volatile uint8_t  scan_in_progress = 0;  /* guard against TIM5 overlap */
static volatile uint16_t threshold_val[NUM_SENSORS];

/* --- PID state ------------------------------------------------------------ */
static float    pid_error      = 0.0f;
static float    pid_last_error = 0.0f;
static float    pid_integral   = 0.0f;
static float    pid_position   = 75.0f;  /* centred on 0–150 scale */
static int32_t  left_speed     = 0;
static int32_t  right_speed    = 0;
static uint8_t  line_is_lost   = 0;
static uint8_t  junction_detected = 0;


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

/* USER CODE BEGIN 0 */

/* --------------------------------------------------------------------------
 * MUX control — selects which of the 16 sensors feeds ADC_CHANNEL_14
 * Bits: A0=PB2, A1=PB1, A2=PA7, A3=PA6
 * -------------------------------------------------------------------------- */
static void set_mux(uint8_t ch)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, (ch & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, ((ch >> 1) & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, ((ch >> 2) & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, ((ch >> 3) & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    /* Short settle time for mux + RC filter */
    for (int i = 0; i < 30; i++) { __NOP(); }
}

/* --------------------------------------------------------------------------
 * Begin a full 16-sensor scan.
 * Called from TIM5 ISR (and directly during calibration).
 * Guard: does nothing if a scan is already running.
 * -------------------------------------------------------------------------- */
static void start_sensor_scan(void)
{
    if (scan_in_progress) return;   /* previous scan not yet finished — skip */

    scan_in_progress = 1;
    sensors_ready    = 0;
    current_sensor   = 0;
    sample_count     = 0;           /* reset per-sensor oversampling counter */

    set_mux(current_sensor);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_raw, 1);
}

/* --------------------------------------------------------------------------
 * Cluster detector
 * Groups consecutive above-threshold sensors into weighted-centroid clusters.
 * -------------------------------------------------------------------------- */
typedef struct {
	float center;
	uint8_t count;
}Cluster;

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
    /* Close any cluster still open at the array end */
    if (in_cluster && num_clusters < 4)
    {
        clusters[num_clusters].center = weighted_sum / total_weight;
        clusters[num_clusters].count  = count;
        num_clusters++;
    }
    return num_clusters;
}

/* --------------------------------------------------------------------------
 * PID update — call from main loop ONLY, never from an ISR.
 * -------------------------------------------------------------------------- */
static void pid_update(void)
{
    Cluster clusters[4];
    uint8_t num_clusters = find_clusters(clusters);

    if (num_clusters == 0)
    {
        /* Line lost — spin toward the side it was last seen */
        line_is_lost  = 1;
        pid_integral  = 0.0f;

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
        line_is_lost = 0;

        /*
         * Junction detection: more than one distinct cluster means a fork.
         * A single very wide cluster (>8 sensors) is treated as a junction too.
         * Previously this was only num_clusters==0 check — fixed.
         */
        junction_detected = (num_clusters > 1 || clusters[0].count > 8) ? 1 : 0;

        /* Pick the cluster closest to where we tracked last frame */
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

        pid_position = best_pos;
        pid_error    = pid_position - 75.0f;   /* setpoint = centre of 0–150 range */

        pid_integral += pid_error;
        if (pid_integral >  5000.0f) pid_integral =  5000.0f;
        if (pid_integral < -5000.0f) pid_integral = -5000.0f;

        float derivative   = pid_error - pid_last_error;
        pid_last_error     = pid_error;

        /*
         * Reduce base speed on sharp turns AND at junctions
         * (junction slowdown is new — was not handled before)
         */
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

    /* Clamp to allowed range — MIN_SPEED = 0, so no reverse */
    if (left_speed  > MAX_SPEED) left_speed  = MAX_SPEED;
    if (left_speed  < MIN_SPEED) left_speed  = MIN_SPEED;
    if (right_speed > MAX_SPEED) right_speed = MAX_SPEED;
    if (right_speed < MIN_SPEED) right_speed = MIN_SPEED;

    set_lmotor_speed(left_speed);
    set_rmotor_speed(right_speed);
}

/* --------------------------------------------------------------------------
 * LED visual feedback — call from main loop after pid_update().
 * Was defined but never called in the original code.
 * -------------------------------------------------------------------------- */
static void update_leds(void)
{
    /* Clear all */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12 | GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2  | GPIO_PIN_3,  GPIO_PIN_RESET);

    if (line_is_lost)
    {
        /* All LEDs on = searching */
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12 | GPIO_PIN_13, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2  | GPIO_PIN_3,  GPIO_PIN_SET);
        return;
    }

    /* Single LED shows which quadrant the line is in */
    if      (pid_position <  40.0f) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    else if (pid_position <  70.0f) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    else if (pid_position < 110.0f) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3,  GPIO_PIN_SET);
    else                            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2,  GPIO_PIN_SET);
}

/* --------------------------------------------------------------------------
 * DMA conversion complete callback — runs in interrupt context.
 *
 * ONLY accumulates samples and advances the scan state.
 * NO floating-point, NO motor writes, NO PID here.
 * -------------------------------------------------------------------------- */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance != ADC1) return;

    /* Accumulate one sample for the current sensor */
    sensor_accumulator[current_sensor] += adc_raw[0];
    sample_count++;

    if (sample_count < OVERSAMPLING)
    {
        /* Need more samples for this sensor — re-trigger immediately */
        HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_raw, 1);
    }
    else
    {
        /* This sensor is done — compute average and move on */
        sensor_values[current_sensor] = (uint16_t)(sensor_accumulator[current_sensor] / OVERSAMPLING);
        sensor_accumulator[current_sensor] = 0;

        sample_count = 0;   /* reset BEFORE moving to next sensor */
        current_sensor++;

        if (current_sensor < NUM_SENSORS)
        {
            /* Advance mux and sample the next sensor */
            set_mux(current_sensor);
            HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_raw, 1);
        }
        else
        {
            /* All 16 sensors sampled — signal main loop */
            scan_in_progress = 0;
            sensors_ready    = 1;
        }
    }
}

/* --------------------------------------------------------------------------
 * TIM5 period elapsed — triggers a new scan every 10 ms.
 * Guard in start_sensor_scan() prevents overlap if a scan is still running.
 * -------------------------------------------------------------------------- */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM5)
    {
        start_sensor_scan();
    }
}

/* --------------------------------------------------------------------------
 * Sensor calibration — call once at startup with TIM5 stopped.
 * Sweeps robot slowly over the line for 3 s, records min/max, sets threshold.
 * -------------------------------------------------------------------------- */
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

    /* Compute per-sensor thresholds */
    for (int i = 0; i < NUM_SENSORS; i++)
        threshold_val[i] = (min_val[i] + max_val[i]) / 2;

    /* Average all thresholds to check calibration quality */
    uint32_t sum = 0;
    for (int i = 0; i < NUM_SENSORS; i++)
        sum += threshold_val[i];
    uint16_t avg_threshold = (uint16_t)(sum / NUM_SENSORS);

    /* If average is too far from midpoint calibration was bad — reset to default */
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

/* --------------------------------------------------------------------------
 * Application entry point
 * -------------------------------------------------------------------------- */
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_TIM2_Init();
    MX_TIM4_Init();
    MX_USB_DEVICE_Init();
    MX_TIM3_Init();
    MX_TIM5_Init();

    /* Start motor PWM timers */
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

    /* Start scan timer (disabled inside calibrate_sensors, re-enabled after) */
    HAL_TIM_Base_Start_IT(&htim5);

    /* Init motor drivers */
    init_left_driver();
    init_right_driver();

    /* Calibrate sensors — TIM5 is stopped inside this call */
    calibrate_sensors();

    /* Start ESC PWM output */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

    /*
     * ESC arming sequence:
     *  1. Send max throttle so ESC enters programming / arming mode
     *  2. Drop to minimum throttle — ESC beeps and arms
     *  3. Set running speed
     *
     * FIX: was writing to CCR1; TIM_CHANNEL_2 maps to CCR2.
     */
    TIM3->CCR2 = TURBINE_MAX;   /* high signal — enter ESC arm mode */
    HAL_Delay(2000);

    TIM3->CCR2 = TURBINE_ARM;   /* drop to min — ESC beeps and confirms arm */
    HAL_Delay(3000);

    TIM3->CCR2 = TURBINE_SPEED; /* set running speed */

    /* Kick off the first sensor scan */
    start_sensor_scan();

    /* -----------------------------------------------------------------------
     * Main loop — all heavy processing happens here, NOT in interrupts.
     * ----------------------------------------------------------------------- */
    while (1)
    {
        if (sensors_ready)
        {
            sensors_ready = 0;   /* acknowledge before processing */
            pid_update();
            update_leds();
        }
        /* Other non-time-critical tasks (USB telemetry, etc.) can go here */
    }
}

/* --------------------------------------------------------------------------
 * System Clock Configuration
 * HSE 8 MHz → PLL → 168 MHz SYSCLK
 * -------------------------------------------------------------------------- */
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

/* --------------------------------------------------------------------------
 * ADC1 — single channel, software trigger, 12-bit, DMA mode
 * -------------------------------------------------------------------------- */
static void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

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
}

/* --------------------------------------------------------------------------
 * TIM2 — Right motor PWM (CH1), 1 kHz, 0–999 duty
 * 84 MHz / 84 prescaler / 1000 period = 1 kHz
 * -------------------------------------------------------------------------- */
static void MX_TIM2_Init(void)
{
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef      sConfigOC     = {0};

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

    HAL_TIM_MspPostInit(&htim2);
}

/* --------------------------------------------------------------------------
 * TIM3 — ESC / Turbine PWM (CH2), ~50 Hz RC servo signal
 * 84 MHz / 85 prescaler / 1675 period ≈ 50 Hz
 * CCR2 range: 1000 (1 ms, min) to 2000 (2 ms, max)
 *
 * FIX: original code used CCR1 but channel is CH2 → CCR2. Corrected here
 * and in the arming sequence in main().
 * -------------------------------------------------------------------------- */
static void MX_TIM3_Init(void)
{
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef      sConfigOC     = {0};

    htim3.Instance               = TIM3;
    htim3.Init.Prescaler         = 84;          /* 84 MHz / 85 = ~988 kHz tick */
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = 1674;         /* ~50 Hz */
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

    HAL_TIM_MspPostInit(&htim3);
}

/* --------------------------------------------------------------------------
 * TIM4 — Left motor PWM (CH4), 1 kHz, 0–999 duty
 * 84 MHz / 85 prescaler / 1000 period = ~988 Hz
 * -------------------------------------------------------------------------- */
static void MX_TIM4_Init(void)
{
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef      sConfigOC     = {0};

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

    HAL_TIM_MspPostInit(&htim4);
}

/* --------------------------------------------------------------------------
 * TIM5 — Scan trigger timer, interrupt every 10 ms
 *
 * FIX: was 1 ms (period 999), which is shorter than one full 16-sensor scan.
 * Now 10 ms (period 9999) — comfortably longer than worst-case scan time.
 *
 * 84 MHz / 84 prescaler = 1 MHz tick → period 9999 = 10 ms
 * -------------------------------------------------------------------------- */
static void MX_TIM5_Init(void)
{
    TIM_ClockConfigTypeDef  sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig      = {0};

    htim5.Instance               = TIM5;
    htim5.Init.Prescaler         = 84 - 1;
    htim5.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim5.Init.Period            = 10000 - 1;   /* 10 ms — FIX: was 999 (1 ms) */
    htim5.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim5) != HAL_OK) Error_Handler();

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK) Error_Handler();

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK) Error_Handler();
}

/* --------------------------------------------------------------------------
 * DMA — ADC1 uses DMA2 Stream0
 * -------------------------------------------------------------------------- */
static void MX_DMA_Init(void)
{
    __HAL_RCC_DMA2_CLK_ENABLE();
    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

/* --------------------------------------------------------------------------
 * GPIO
 * -------------------------------------------------------------------------- */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* Output default levels */
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
}

/* --------------------------------------------------------------------------
 * Error handler
 * -------------------------------------------------------------------------- */
void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file; (void)line;
}
#endif
