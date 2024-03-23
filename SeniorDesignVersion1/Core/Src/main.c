/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32h7xx_hal.h"
#include <string.h>

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

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_HS_USB_Init(void);
/* USER CODE BEGIN PFP */

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

  /* USER CODE BEGIN 2 */
MX_GPIO_Init();
MX_ADC1_Init();
MX_TIM1_Init();
MX_USART3_UART_Init();
MX_USB_OTG_HS_USB_Init();
MX_IWDG_Init();
// Initial PWM settings
htim1.Instance->ARR = initial_period; // Set initial auto-reload value for frequency
htim1.Instance->CCR1 = initial_duty_cycle; // Set initial compare value for duty cycle
htim1.Instance->BDTR = initial_dead_time; // Set initial dead time
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // enable the DC-DC converters and H-bridge, verify voltage
	  power_up_sequence();

	  // pet the watch dog, Perform system checks
	  HAL_IWDG_Refresh(&hiwdg); // must be done more frequently than timeout period or system will reset
	  check_system_parameters();

	  // Start ADC in interrupt mode
	  HAL_ADC_Start_IT(&hadc1);

	  // Start PWM
	  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

	  // Measure from ADCs, Adjust PWM, Dead time accordingly
	  HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc1);

	  // pet the watch dog, Perform system checks
	  HAL_IWDG_Refresh(&hiwdg);
	  check_system_parameters();

	  // End PWM
	  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1); // This should be ended when the user specifies

	  // Power Down
	  power_down_sequence();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/* USER FUNCTIONS BEGIN */

// Verify the voltages are present so the power up sequence can begin
HAL_StatusTypeDef verify_voltage(ADC_HandleTypeDef* hadc, uint32_t ADCChannel, uint32_t ThresholdVoltage)
{
    // Initialize the ADC channel configuration structure to zeros.
    ADC_ChannelConfTypeDef sConfig = {0};

    // Set the desired channel to be measured.
    sConfig.Channel = ADCChannel;

    // Set the rank. If only one channel is being measured, it will be rank 1.
    sConfig.Rank = ADC_REGULAR_RANK_1;

    // Set the sampling time to a very short duration, assuming the voltage signal is stable.
    sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;

    // Configure the ADC channel with the specified settings.
    HAL_ADC_ConfigChannel(hadc, &sConfig);

    // Start the ADC peripheral.
    HAL_ADC_Start(hadc);

    // Poll the ADC for the conversion result, timeout set to 10ms.
    HAL_StatusTypeDef status = HAL_ADC_PollForConversion(hadc, 10);

    // If the ADC finishes the conversion successfully,
    // check if the value meets the required threshold.
    if (status == HAL_OK)
    {
        // Get the ADC conversion result.
        uint32_t ADCValue = HAL_ADC_GetValue(hadc);

        // Compare the conversion result to the threshold.
        if (ADCValue < ThresholdVoltage)
        {
            // If the result is less than the threshold, update status to HAL_ERROR.
            status = HAL_ERROR;
        }
    }

    // Stop the ADC to save power, especially if not sampling continuously.
    HAL_ADC_Stop(hadc);

    // Return the status which indicates whether the voltage is above the threshold.
    return status;
}

// Enable all DC-DC converters and H-bridge (15V)
void power_up_sequence() {
  // 1. Enable 15V power supply
  HAL_GPIO_WritePin(POWER_SUPPLY_15V_ENABLE_GPIO_Port, POWER_SUPPLY_15V_ENABLE_Pin, GPIO_PIN_SET);

  // 2. Verify 15V is available
  if(verify_voltage(&hadc1, ADC_CHANNEL_15V, VOLTAGE_15V_THRESHOLD) != HAL_OK) { // There is a warning because these are not defined in main.h
	  // Make the RED led blink if the 15V is not present
	  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET); // Assuming there is an error LED
	  while(1) {
	      HAL_Delay(1000);
	      HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin); // Blink an LED to indicate error
	      // Send an error message over UART
	      char* errorMsg = "Error: 15V supply not present.\r\n";
	      HAL_UART_Transmit(&huart3, (uint8_t*)errorMsg, strlen(errorMsg), HAL_MAX_DELAY);
	  }
  }

  // 3. Enable 3.3V bias voltage
  HAL_GPIO_WritePin(POWER_SUPPLY_3V3_ENABLE_GPIO_Port, POWER_SUPPLY_3V3_ENABLE_Pin, GPIO_PIN_SET);

  // 4. Verify 3.3V is available
  if(verify_voltage(&hadc1, ADC_CHANNEL_3V3, VOLTAGE_3V3_THRESHOLD) != HAL_OK) {
	  // Make the RED led blink if the 3.3V is not present
	  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET); // Assuming there is an error LED
	  while(1) {
	      HAL_Delay(1000);
	      HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin); // Blink an LED to indicate error
	      // Send an error message over UART
	      char* errorMsg = "Error: 3.3V supply not present.\r\n";
	      HAL_UART_Transmit(&huart3, (uint8_t*)errorMsg, strlen(errorMsg), HAL_MAX_DELAY);
	  }
  }

  // 5. Enable 500V power supply
  HAL_GPIO_WritePin(POWER_SUPPLY_500V_ENABLE_GPIO_Port, POWER_SUPPLY_500V_ENABLE_Pin, GPIO_PIN_SET);

  // 6. Verify 500V is available
  if(verify_voltage(&hadc1, ADC_CHANNEL_500V, VOLTAGE_500V_THRESHOLD) != HAL_OK) {
	  // Make the RED led blink if the 500V is not present
	  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET); // Assuming there is an error LED
	  while(1) {
	      HAL_Delay(1000);
	      HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin); // Blink an LED to indicate error
	      // Send an error message over UART
	      char* errorMsg = "Error: 500V supply not present.\r\n";
	      HAL_UART_Transmit(&huart3, (uint8_t*)errorMsg, strlen(errorMsg), HAL_MAX_DELAY);
	  }
  }

  // 7. Drive H-bridge
  HAL_GPIO_WritePin(H_BRIDGE_ENABLE_GPIO_Port, H_BRIDGE_ENABLE_Pin, GPIO_PIN_SET);

  // 8. Verify that 1-2kVrms is available
  if(verify_voltage(&hadc1, ADC_CHANNEL_1KV2KV, VOLTAGE_1KV2KV_THRESHOLD) != HAL_OK) {
	  // Make the RED led blink if the 1-2kVrms is not present
	  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET); // Assuming there is an error LED
	  while(1) {
	      HAL_Delay(1000);
	      HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin); // Blink an LED to indicate error
	      // Send an error message over UART
	      char* errorMsg = "Error: 1-2kVrms supply not present.\r\n";
	      HAL_UART_Transmit(&huart3, (uint8_t*)errorMsg, strlen(errorMsg), HAL_MAX_DELAY);
	  }
  }

  // 9. Done.
}

void power_down_sequence() {
    // 1. Stop driving H-bridge
    HAL_GPIO_WritePin(H_BRIDGE_ENABLE_GPIO_Port, H_BRIDGE_ENABLE_Pin, GPIO_PIN_RESET);

    // 2. Verify if 1-2kVrms is available, if yes send error
    if(verify_voltage(&hadc1, ADC_CHANNEL_1KV2KV, VOLTAGE_1KV2KV_THRESHOLD) == HAL_OK) {
    	// Make the RED led blink if the 1-2kVrms is present
    	HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET); // Assuming there is an error LED
    	while(1) {
    		HAL_Delay(1000);
    		HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin); // Blink an LED to indicate error
    		// Send an error message over UART
    		char* errorMsg = "Error: 1-2kVrms supply is still present.\r\n";
    		HAL_UART_Transmit(&huart3, (uint8_t*)errorMsg, strlen(errorMsg), HAL_MAX_DELAY);
    	}
    }

    // 3. Disable 500V power supply
    HAL_GPIO_WritePin(POWER_SUPPLY_500V_ENABLE_GPIO_Port, POWER_SUPPLY_500V_ENABLE_Pin, GPIO_PIN_RESET);

    // 4. Verify if 500V is available, if yes then send error
    if(verify_voltage(&hadc1, ADC_CHANNEL_500V, VOLTAGE_500V_THRESHOLD) == HAL_OK) {
    	// Make the RED led blink if the 500V is present
    	HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET); // Assuming there is an error LED
    	while(1) {
    		HAL_Delay(1000);
    	    HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin); // Blink an LED to indicate error
    	    // Send an error message over UART
    	    char* errorMsg = "Error: 500V supply is still present.\r\n";
    	    HAL_UART_Transmit(&huart3, (uint8_t*)errorMsg, strlen(errorMsg), HAL_MAX_DELAY);
    	}
    }

    // 5. Disable 3.3V bias voltage
    HAL_GPIO_WritePin(POWER_SUPPLY_3V3_ENABLE_GPIO_Port, POWER_SUPPLY_3V3_ENABLE_Pin, GPIO_PIN_RESET);

    // 6. Verify if 3.3V is present, if yes then send error
    if(verify_voltage(&hadc1, ADC_CHANNEL_3V3, VOLTAGE_3V3_THRESHOLD) == HAL_OK) {
    	// Make the RED led blink if the 3.3V is present
    	HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET); // Assuming there is an error LED
    	while(1) {
    	 	HAL_Delay(1000);
    	    HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin); // Blink an LED to indicate error
    	    // Send an error message over UART
    	    char* errorMsg = "Error: 3.3V supply is still present.\r\n";
    	    HAL_UART_Transmit(&huart3, (uint8_t*)errorMsg, strlen(errorMsg), HAL_MAX_DELAY);
    	}
    }

    // 7. Disable 15V power supply
    HAL_GPIO_WritePin(POWER_SUPPLY_15V_ENABLE_GPIO_Port, POWER_SUPPLY_15V_ENABLE_Pin, GPIO_PIN_RESET);

    // 8. Verify if 15V is present, if yes then send error
    if(verify_voltage(&hadc1, ADC_CHANNEL_15V, VOLTAGE_15V_THRESHOLD) == HAL_OK) {
    	// Make the RED led blink if the 15V is present
    	HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET); // Assuming there is an error LED
    	while(1) {
    	HAL_Delay(1000);
    	HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin); // Blink an LED to indicate error
    	// Send an error message over UART
    	char* errorMsg = "Error: 15V supply is still present.\r\n";
    	HAL_UART_Transmit(&huart3, (uint8_t*)errorMsg, strlen(errorMsg), HAL_MAX_DELAY);
    	}
    }

    // 9. Done
}

// Function to adjust PWM frequency based on delta_f
void adjust_pwm_frequency(uint32_t delta_f)
{
    // Calculate the new period
    uint32_t new_period = htim1.Instance->ARR - delta_f;

    // Make sure the new period value does not exceed the maximum value
    if(new_period > TIM_PERIOD_MAX){
    	new_period = TIM_PERIOD_MAX;
    }

    // Update the timer's auto-reload register to adjust the frequency
    __HAL_TIM_SET_AUTORELOAD(&htim1, new_period);
}

// Function to adjust PWM dead time based on delta_t
void adjust_pwm_deadtime(uint32_t delta_t)
{
    // Update the timer's dead-time register to adjust the dead time
    __HAL_TIM_SET_DEADTIME(&htim1, delta_t);
}

// ADC interrupt callback function
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if(hadc->Instance == ADC1)
    {
        // Read ADC values
        uint32_t bridge_current = HAL_ADC_GetValue(hadc); // Example, actual implementation may vary
        uint32_t plasma_voltage = HAL_ADC_GetValue(hadc); // Example, actual implementation may vary

        // Calculate adjustments
        uint32_t delta_f = K_1 * (bridge_current - previous_bridge_current) / (I_smax - I_smin);
        uint32_t delta_t = K_2 * (V_desired - plasma_voltage);

        // Adjust PWM parameters
        adjust_pwm_frequency(delta_f);
        adjust_pwm_deadtime(delta_t);

        // Start the next ADC conversion
        HAL_ADC_Start_IT(hadc);
    }
}

// initialize watch dog
void MX_IWDG_Init(void) {
    IWDG_HandleTypeDef hiwdg;

    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_64; // Adjust prescaler
    hiwdg.Init.Reload = 4095; // Adjust reload value to get the desired timeout period
    HAL_IWDG_Init(&hiwdg);
}

void check_system_parameters(void) {
    if (!is_voltage_ok() || !is_temperature_ok() || !is_fan_speed_ok())
    {
        // Handle error
        error_handler();
    }
}

/* USER FUNCTIONS END */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_CRSInitTypeDef RCC_CRSInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 24;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the SYSCFG APB clock
  */
  __HAL_RCC_CRS_CLK_ENABLE();

  /** Configures CRS
  */
  RCC_CRSInitStruct.Prescaler = RCC_CRS_SYNC_DIV1;
  RCC_CRSInitStruct.Source = RCC_CRS_SYNC_SOURCE_LSE;
  RCC_CRSInitStruct.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  RCC_CRSInitStruct.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000,32768);
  RCC_CRSInitStruct.ErrorLimitValue = 34;
  RCC_CRSInitStruct.HSI48CalibrationValue = 32;

  HAL_RCCEx_CRSConfig(&RCC_CRSInitStruct);
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
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
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_HS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_HS_USB_Init(void)
{

  /* USER CODE BEGIN USB_OTG_HS_Init 0 */

  /* USER CODE END USB_OTG_HS_Init 0 */

  /* USER CODE BEGIN USB_OTG_HS_Init 1 */

  /* USER CODE END USB_OTG_HS_Init 1 */
  /* USER CODE BEGIN USB_OTG_HS_Init 2 */

  /* USER CODE END USB_OTG_HS_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_FS_PWR_EN_GPIO_Port, USB_FS_PWR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_RED_Pin */
  GPIO_InitStruct.Pin = LED_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_RED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_PWR_EN_Pin */
  GPIO_InitStruct.Pin = USB_FS_PWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_FS_PWR_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_OVCR_Pin */
  GPIO_InitStruct.Pin = USB_FS_OVCR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_FS_OVCR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_FS_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_FS_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_ID_Pin */
  GPIO_InitStruct.Pin = USB_FS_ID_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_HS;
  HAL_GPIO_Init(USB_FS_ID_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_YELLOW_Pin */
  GPIO_InitStruct.Pin = LED_YELLOW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_YELLOW_GPIO_Port, &GPIO_InitStruct);

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
