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
#include "oled.h"
#include "buck_pwm.h"
#include "buck_adc.h"
#include "buck_display_filter.h"
#include "buck_voltage_display.h"
#include <math.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Safe fallback for builds outside this CMake configuration. */
#ifndef ADC_ONLY_TEST
#define ADC_ONLY_TEST 1
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint32_t adc_schedule_missed;
volatile HAL_StatusTypeDef adc_init_status;
/* Display-only values; buck_input/output_voltage remain unsmoothed. */
volatile float buck_input_voltage_display = NAN;
volatile float buck_output_voltage_display = NAN;
static BuckDisplayFilter input_display_filter;
static BuckDisplayFilter output_display_filter;
static BuckDisplayDigits input_display_digits;
static BuckDisplayDigits output_display_digits;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#if ADC_ONLY_TEST
static void App_HoldPWMInactive(void)
{
    GPIO_InitTypeDef gpio = {0};
    /* No TIM1 initialization, alternate function, counter, MOE or start.
     * Load the output latch LOW before selecting GPIO output mode. */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13 | GPIO_PIN_15, GPIO_PIN_RESET);
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Pin = GPIO_PIN_10;
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = GPIO_PIN_13 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOB, &gpio);
}
#endif

static void App_SetVoltageLine(uint8_t row, const char *label, float voltage, uint32_t estimate)
{
    char line[22];
    BuckVoltageDisplay_Format(line, sizeof(line), label, voltage, estimate);
    OLED_SetTextLine(row, line);
}

static void App_UpdateVoltageMonitor(void)
{
    float input = BuckADC_GetInputVoltageForDisplay();
    float output = BuckADC_GetOutputVoltageForDisplay();
    uint32_t input_estimate = (adc_input_error_code & BUCK_ADC_ERR_REFERENCE) != 0U;
    uint32_t output_estimate = (adc_output_error_code & BUCK_ADC_ERR_REFERENCE) != 0U;
    uint32_t input_source = input_estimate | ((adc_input_error_code & BUCK_ADC_ERR_NEGATIVE) ? 2U : 0U);
    uint32_t output_source = output_estimate | ((adc_output_error_code & BUCK_ADC_ERR_NEGATIVE) ? 2U : 0U);
    uint32_t now = HAL_GetTick();
    /* A sign anomaly must not be hidden by the old positive digit history. */
    float filtered_input = BuckDisplayFilter_Update(&input_display_filter, input, now, input_source);
    float filtered_output = BuckDisplayFilter_Update(&output_display_filter, output, now, output_source);
    /* Invalidate digit history immediately, even between OLED refreshes. */
    if (!isfinite(filtered_input))
        buck_input_voltage_display = BuckDisplayDigits_Update(&input_display_digits, NAN, now, input_source);
    if (!isfinite(filtered_output))
        buck_output_voltage_display = BuckDisplayDigits_Update(&output_display_digits, NAN, now, output_source);
}

static void App_UpdateVoltageDisplay(void)
{
    char line[22];
    /* Recheck current ADC validity before showing a stored filter value. */
    float input = BuckADC_GetInputVoltageForDisplay();
    float output = BuckADC_GetOutputVoltageForDisplay();
    uint32_t input_estimate = (adc_input_error_code & BUCK_ADC_ERR_REFERENCE) != 0U;
    uint32_t output_estimate = (adc_output_error_code & BUCK_ADC_ERR_REFERENCE) != 0U;
    uint32_t input_source = input_estimate | ((adc_input_error_code & BUCK_ADC_ERR_NEGATIVE) ? 2U : 0U);
    uint32_t output_source = output_estimate | ((adc_output_error_code & BUCK_ADC_ERR_NEGATIVE) ? 2U : 0U);
    if (!isfinite(input)) BuckDisplayFilter_Reset(&input_display_filter);
    if (!isfinite(output)) BuckDisplayFilter_Reset(&output_display_filter);
    uint32_t now = HAL_GetTick();
    input = BuckDisplayFilter_Get(&input_display_filter, now);
    output = BuckDisplayFilter_Get(&output_display_filter, now);
    input = BuckDisplayDigits_Update(&input_display_digits, input, now, input_source);
    output = BuckDisplayDigits_Update(&output_display_digits, output, now, output_source);
    buck_input_voltage_display = input;
    buck_output_voltage_display = output;
    App_SetVoltageLine(0U, "VIN", input, input_estimate);
    App_SetVoltageLine(1U, "VOUT", output, output_estimate);

    /* A valid ADC conversion at a rail is an input-range diagnostic, not
     * a conversion failure. Keep the module flags/validity unchanged. */
    uint32_t conversion_faults = adc_error_code
        & ~(BUCK_ADC_ERR_REFERENCE | BUCK_ADC_ERR_NEGATIVE | BUCK_ADC_ERR_RAIL);
    uint32_t input_limit = adc_input_error_code & BUCK_ADC_ERR_RAIL;
    uint32_t output_limit = adc_output_error_code & BUCK_ADC_ERR_RAIL;
    const char *signal_state = NULL;
    if (conversion_faults != 0U)
        signal_state = "ADC ERROR";
    else if (input_limit != 0U && output_limit != 0U)
        signal_state = "VIN VOUT ADC LIMIT";
    else if (input_limit != 0U)
        signal_state = "VIN ADC LIMIT";
    else if (output_limit != 0U)
        signal_state = "VOUT ADC LIMIT";
    else if ((adc_error_code & BUCK_ADC_ERR_NEGATIVE) != 0U)
        signal_state = "NEGATIVE DIFF";

    const char *state = "ADC OK";
    if ((adc_error_code & BUCK_ADC_ERR_VREFBUF_READY) != 0U)
        state = "VREF NOT READY";
    else if ((adc_error_code & BUCK_ADC_ERR_REFERENCE_MODE) != 0U)
        state = "VREF CONFIG ERROR";
    else if (signal_state != NULL)
        state = signal_state;
    else if (BUCK_ADC_INTERNAL_REFERENCE_TEST && (adc_input_estimate_valid || adc_output_estimate_valid)
             && (adc_error_code & BUCK_ADC_ERR_REFERENCE) != 0U)
        state = "NOMINAL VREF 2.5V";
    else if ((adc_error_code & BUCK_ADC_ERR_REFERENCE) != 0U)
        state = "REF UNCONFIRMED";
    OLED_SetTextLine(2U, state);
    if (adc_raw_valid != 0U)
    {
        snprintf(line, sizeof(line), "IN +%4lu -%4lu",
                 (unsigned long)adc_input_raw_plus, (unsigned long)adc_input_raw_minus);
        OLED_SetTextLine(3U, line);
        snprintf(line, sizeof(line), "OUT+%4lu -%4lu",
                 (unsigned long)adc_raw_plus, (unsigned long)adc_raw_minus);
        OLED_SetTextLine(4U, line);
    }
    else
    {
        OLED_SetTextLine(3U, "IN +---- -----");
        OLED_SetTextLine(4U, "OUT+---- -----");
    }
    snprintf(line, sizeof(line), "EI:%04lX EO:%04lX",
             (unsigned long)adc_input_error_code, (unsigned long)adc_output_error_code);
    OLED_SetTextLine(5U, line);
    snprintf(line, sizeof(line), "ADC N:%lu", (unsigned long)adc_update_count);
    OLED_SetTextLine(6U, line);
#if ADC_ONLY_TEST
    OLED_SetTextLine(7U, "ADC ONLY - PWM OFF");
#else
    OLED_SetTextLine(7U, "OPEN LOOP PWM");
#endif
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

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
#if ADC_ONLY_TEST
  App_HoldPWMInactive();
#endif
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* An HSE clock failure enters NMI_Handler(), which stops the PWM outputs. */
  HAL_RCC_EnableCSS();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
#if !ADC_ONLY_TEST
  BuckPWM_Init();
#endif
  OLED_Init();
  adc_init_status = BuckADC_Init();
  if (adc_init_status == HAL_OK) (void)BuckADC_ReadVoltage();
  App_UpdateVoltageMonitor();
  App_UpdateVoltageDisplay();
#if !ADC_ONLY_TEST
  /* Only compiled after explicitly selecting ADC_ONLY_TEST=OFF. */
  BuckPWM_Start();
#endif
  uint32_t sample_tick = HAL_GetTick();
  uint32_t display_tick = sample_tick;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();
    uint32_t elapsed = now - sample_tick;
    if (elapsed >= BUCK_ADC_UPDATE_MS)
    {
      /* Skip expired slots rather than burst-sampling to catch up. */
      uint32_t periods = elapsed / BUCK_ADC_UPDATE_MS;
      adc_schedule_missed += periods - 1U;
      sample_tick += periods * BUCK_ADC_UPDATE_MS;
      if (adc_init_status == HAL_OK) (void)BuckADC_ReadVoltage();
      App_UpdateVoltageMonitor();
    }
    now = HAL_GetTick();
    if ((uint32_t)(now - display_tick) >= 200U)
    {
      display_tick = now;
      App_UpdateVoltageDisplay();
    }
    /* Software SPI runs only with at least 3 ms until the next sample slot.
     * Check adc_schedule_missed on hardware; this is cooperative monitoring,
     * not a PWM-synchronous control-loop sampling schedule. */
    if ((uint32_t)(HAL_GetTick() - sample_tick) < BUCK_ADC_UPDATE_MS - 3U)
      OLED_Process();
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  /* Board crystal: 8 MHz, crystal/resonator mode (not external-clock bypass). */

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  /* 8 MHz / 2 * 84 / 2 = 168 MHz SYSCLK and TIM1 clock. */
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

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
  BuckPWM_Stop();
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
