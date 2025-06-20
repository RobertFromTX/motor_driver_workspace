/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
	//cannot set values here since no memory allocated in declaration
	float Ts; //sampling time
	float proportional_gain;
	float integral_gain;
	float derivative_gain;

	float tau; // 1/(cutoff frequency of low pass filter after derivative component)

	float proportional_out;
	float integral_out;
	float derivative_out;

	int16_t error; //error is signed
	uint16_t measured_pos;

	float total_out;
	float out_max;
	float out_min;
} pid_controller;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_BUF_SIZE	2
#define MOTOR_HISTORY_LEN	3 //PID only needs 3 max since it is at most second order
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_tim1_ch1;

/* USER CODE BEGIN PV */
volatile uint8_t position_ready = 0; //flag representing whether or not position data is ready from adc
uint16_t raw_pos_values[ADC_BUF_SIZE];

uint16_t motor1_pos[MOTOR_HISTORY_LEN];
uint16_t motor2_pos[MOTOR_HISTORY_LEN];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void initialize_PID(pid_controller *controller, uint16_t updated_measured_pos);
void set_gains_PID(pid_controller *controller, float Kp, float Ki, float Kd);
void update_PID(pid_controller *controller, uint16_t updated_measured_pos, uint16_t set_point);
void update_motor_input(int16_t new_out, uint32_t **active_buffer, uint32_t **inactive_buffer);
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
	MX_TIM1_Init();
	MX_ADC1_Init();
	MX_TIM2_Init();
	/* USER CODE BEGIN 2 */

	//Initialize PWM
	//creating two buffers for a single motor. When changing speed, can update the inactive buffer and swap it with the active buffer
	//to avoid two devices trying to write to the same buffer.
	volatile uint32_t bufferA = 400; //out of 999, determines on time of PWM signal
	volatile uint32_t bufferB = 750;
	volatile uint32_t *active_buffer = &bufferA;
	volatile uint32_t *inactive_buffer = &bufferB;
	HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t*) active_buffer, 1); //send the pulse to the timer peripheral, which determines duty cycle
	//Initialize ADC
	if (HAL_TIM_Base_Start(&htim2) != HAL_OK)
	{
		Error_Handler();
	}

	if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK) //calibrate ADC to get rid of offset error, Ex stands for extended. Special marking for function in HAL that might differ on other stm32 lines.
	{ //No Ex in HAL functions means you can easily copy code to other stm32 families.
		Error_Handler();
	}

	if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*) raw_pos_values, 2) != HAL_OK)
	{
		Error_Handler();
	}

	//initialize PID controller
	pid_controller motor1_controller;
	initialize_PID(&motor1_controller, 0);
	set_gains_PID(&motor1_controller, 6, 200, .015); //RED MOTOR (also works for yellow but less aggressive)
	//set_gains_PID(&motor1_controller, 12, 600, .02); //YELLOW MOTOR

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		//The code updates the angle every 1 second, it is slow just for demo purposes. The analog output of the motor encoders will be connected to either pin A0 or A1.
		//The angle values will be in motor1_pos and motor2_pos. Must view live expressions and add these arrays to see the angles.
		while (!position_ready);
		position_ready = 0;	//set flag back to 0
		uint16_t motor_position1 = motor1_pos[0];
		update_PID(&motor1_controller, motor_position1, 180);
		update_motor_input((int16_t) motor1_controller.total_out, &active_buffer, &inactive_buffer);
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
	RCC_OscInitTypeDef RCC_OscInitStruct =
	{ 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct =
	{ 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit =
	{ 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_TIM1;
	PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
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

	ADC_MultiModeTypeDef multimode =
	{ 0 };
	ADC_ChannelConfTypeDef sConfig =
	{ 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
	hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 2;
	hadc1.Init.DMAContinuousRequests = ENABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
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
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.SamplingTime = ADC_SAMPLETIME_19CYCLES_5;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_2;
	sConfig.Rank = ADC_REGULAR_RANK_2;
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

	TIM_MasterConfigTypeDef sMasterConfig =
	{ 0 };
	TIM_OC_InitTypeDef sConfigOC =
	{ 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig =
	{ 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 7;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 999;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig =
	{ 0 };
	TIM_MasterConfigTypeDef sMasterConfig =
	{ 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 7;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 99;
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
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	/* DMA1_Channel2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct =
	{ 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_RESET);

	/*Configure GPIO pins : PB4 PB5 */
	GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void initialize_PID(pid_controller *controller, uint16_t updated_measured_pos)
{

	controller->Ts = 0.0001; //equates to 10kHz
	controller->tau = .001;

	controller->out_max = 800;
	controller->out_min = -800;

	controller->proportional_gain = 0;
	controller->integral_gain = 0;
	controller->derivative_gain = 0;

	controller->proportional_out = 0;
	controller->integral_out = 0;
	controller->derivative_out = 0;
	controller->total_out = 0;

	controller->error = 0;
	controller->measured_pos = updated_measured_pos;
}
void set_gains_PID(pid_controller *controller, float Kp, float Ki, float Kd)
{
	controller->proportional_gain = Kp;
	controller->integral_gain = Ki;
	controller->derivative_gain = Kd;
}
void update_PID(pid_controller *controller, uint16_t updated_measured_pos, uint16_t set_point)
{
	int16_t adjusted_measured_pos = updated_measured_pos;

	int16_t updated_error = set_point - updated_measured_pos;

	//this block makes sure that if the setpoint is near boundaries (0 or 359 degrees), can still approach the setpoint
	//from the direction that has the angle measurement spike from 0 to 359 degrees or 359 to 0 degrees
	//this is done by adjusting the error term
	if (updated_measured_pos > set_point + 180 && set_point < 180)
	{
		adjusted_measured_pos = adjusted_measured_pos - 360;
	}
	else if (updated_measured_pos < set_point - 180 && set_point >= 180)
	{
		adjusted_measured_pos = adjusted_measured_pos + 360;
	}
	updated_error = set_point - adjusted_measured_pos;

	//calculate difference for derivative term, but need to account for when motor goes from 359->0 and 0->359
	//make sure that when it goes 359->0, the difference is 1, and 0->359 is -1
	int16_t position_difference = updated_measured_pos - controller->measured_pos;
	if (position_difference > 300) //when it goes from 0 to 359
	{
		position_difference = position_difference - 360;
	}
	else if (position_difference < -300) //when it goes from 359 to 0
	{
		position_difference = position_difference + 360;
	}

	//updated the outputs of the P, I, and D components of the controller
	controller->proportional_out = controller->proportional_gain * updated_error;
	controller->integral_out = controller->integral_gain * controller->Ts * (updated_error + controller->error) / 2.0 + controller->integral_out;
	controller->derivative_out = ((controller->derivative_gain * 2) * (position_difference) //
	+ (2 * controller->tau - controller->Ts) * controller->derivative_out) / (2 * controller->tau + controller->Ts);
	//note: derivative term uses measured value instead of error term to avoid kick back

	//clamp integrator implementation
	float integral_min, integral_max;
	//determine integrator limits
	if (controller->out_max > controller->proportional_out)
	{
		integral_max = controller->out_max - controller->proportional_out + controller->derivative_out; //see total_out comment to see why adding derivative term instead of subtracting here
	}
	else
	{
		integral_max = 0;
	}
	if (controller->out_min < controller->proportional_out)
	{
		integral_min = controller->out_min - controller->proportional_out + controller->derivative_out; //see total_out comment to see why adding derivative term instead of subtracting here
	}
	else
	{
		integral_min = 0;
	}

	//get absolute error
	int16_t absval_error = controller->error;
	if (absval_error < 0)
	{
		absval_error = -1 * absval_error;
	}

	if (absval_error < 30) //limit integrator even more once closer to desired angle.
	{
		integral_max = 340;
		integral_min = -340;
	}
	if (absval_error < 10) //limit integrator even more once closer to desired angle.
	{
		integral_max = 320; //RED MOTOR
		integral_min = -320;
	}
	//clamping of integrator
	if (controller->integral_out > integral_max)
	{
		controller->integral_out = integral_max;
	}
	else if (controller->integral_out < integral_min)
	{
		controller->integral_out = integral_min;
	}

	//compute total output of controller
	controller->total_out = controller->proportional_out + controller->integral_out - controller->derivative_out; //note negative sign on derivative term, this is correct since it is on the feedback loop

	//limit total output of controller
	if (controller->total_out > controller->out_max)
	{
		controller->total_out = controller->out_max;
	}
	else if (controller->total_out < controller->out_min)
	{
		controller->total_out = controller->out_min;
	}

	//updated the error and measured position
	controller->error = updated_error;
	controller->measured_pos = updated_measured_pos;

}
void update_motor_input(int16_t new_out, uint32_t **active_buffer_address, uint32_t **inactive_buffer_address)
{
	if (new_out > 0)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);

	}
	else if (new_out < 0)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
		new_out = new_out * -1;
	}

	**inactive_buffer_address = new_out;
	uint32_t *temp_uint32_address = *active_buffer_address;
	*active_buffer_address = *inactive_buffer_address;
	*inactive_buffer_address = temp_uint32_address;

}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) //once buffer that stores angular position data from motor encoder is full, stop the ADC and run this action
{
	for (int delay = MOTOR_HISTORY_LEN - 1; delay > 0; --delay) //shift all elements to right
	{
		motor1_pos[delay] = motor1_pos[delay - 1];
		motor2_pos[delay] = motor2_pos[delay - 1];
	}
	motor1_pos[0] = raw_pos_values[0] * 0.08789; //multiply by 360/4096 to convert ADC steps to angle in degrees
	motor2_pos[0] = raw_pos_values[1] * 0.08789;

	position_ready = 1; //signal to main loop that adc data ready
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
