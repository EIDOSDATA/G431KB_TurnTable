/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ros.h"
#include "encoder_msgs/EncoderInfo.h" // 이건 어디냐!!!!!!
#include "std_msgs/String.h" // ros 에 포함
#include "std_msgs/UInt16.h" // ros 에 포함
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define BAUDRATE      115200 // BPS
#define COUNTERCLOCKWISE  0 // FLAG
#define CLOCKWISE         1 // FLAG
#define GO                499 // 127/256(0~255) Duty:50% >>> 499 / 1000(0~999) Duty:50%
#define SLOW              799 // 180/256(0~255) Duty:70.3125% >>> 699 / 1000(0~999) Duty:70%
#define STOP              999 // 127/256(0~255) Duty:100% >>> 499 / 1000(0~999) Duty:100%
#define IR_THRESHOLD      3400 // 일단 아두이노 범위가 0~1023 인것에서의 값인 850 임 >> 이 G431KB는 12bit ADC 이므로 나중에 변경이 필요할 수 있다 3400
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
// 운동부 함수
void controlCallback(const std_msgs::String &ctl_msg);
void initMotor(void);
void makeMotion(void);
void EXTI3_IRQHandler(void);
void doEncoderB(void);
void doEncoderA(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_PIN);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart2);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart2);
void setup(void); // ROS INIT
void loop(void); // ROS TEST PRINT
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
ros::NodeHandle nh;

std_msgs::String ctl_msg; // 컨트롤 메시지
std_msgs::UInt16 ir_msg; // IR 메시지
encoder_msgs::EncoderInfo enc_msg; // 엔코더 메시지
std_msgs::String str_msg; // USART 출력 테스트용 메시지

ros::Subscriber<std_msgs::String> ctl_sub("control_motor", &controlCallback);
ros::Publisher ir_value("ir_value", &ir_msg);
ros::Publisher encoder_info("encoder_info", &enc_msg);
ros::Publisher chatter("chatter", &str_msg); // 테스트 출력

bool edgeFlag = false;
int speed = GO;
int dir = CLOCKWISE;
int encoderPos = 0;

char hello[] = "Hello world!";

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
  MX_USART2_UART_Init();
  MX_ADC2_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	//HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3); // 이미
	//ROS init
	setup();
	// PWM
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // PWM START
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		loop();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		if (dir == 1) // digitalWrite(AIN2_PIN, dir); // GPIO출력으로 대체(dir은 그냥 on / off 플래그라고 생각하자)
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET); // AIN2_PIN dir=1
			//__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, speed); // PWM 출력
		}
		else
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); // AIN2_PIN dir=0
			//__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, speed); // PWM 출력
		}
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, speed); // PWM 출력
		//HAL_DMA

		// ADC
		HAL_ADC_Start(&hadc2);
		HAL_ADC_PollForConversion(&hadc2, 100); // 아두이노에서 아날로그 읽는데 걸리는 시간이 100ms이다
		int res = HAL_ADC_GetValue(&hadc2); // IR_PIN
		HAL_ADC_Stop(&hadc2); // 읽고 멈춰
		ir_msg.data = res;
		ir_value.publish(&ir_msg);

		if (edgeFlag == true) // || edgeFlag == false 추가함
		{
			//enc_msg.timeStamp = HAL_GetTick(); // 손좀 봐야 하는거 이 친구는 ms 단위의 시간을 반환해 준다.
			encoder_info.publish(&enc_msg);
			edgeFlag = false;
		}
		nh.spinOnce();

		/* 아두이노에서 주석처리된 부분
		 if (speed == STOP)
		 encoder_info.publish(&enc_msg);
		 Serial.println(encoderPos);
		 if (encoderPos >= 1560)
		 {
		 Serial.println(encoderPos);
		 encoderPos = encoderPos % 1560;
		 }
		 */
		//nh.spinOnce();
		//HAL_Delay(1); // 이것도 아두이노에서 주석처리
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
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
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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

  ADC_AnalogWDGConfTypeDef AnalogWDGConfig = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analog WatchDog 1
  */
  AnalogWDGConfig.WatchdogNumber = ADC_ANALOGWATCHDOG_1;
  AnalogWDGConfig.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG;
  AnalogWDGConfig.Channel = ADC_CHANNEL_1;
  AnalogWDGConfig.ITMode = DISABLE;
  AnalogWDGConfig.HighThreshold = 0;
  AnalogWDGConfig.LowThreshold = 0;
  AnalogWDGConfig.FilteringConfig = ADC_AWD_FILTERING_NONE;
  if (HAL_ADC_AnalogWDGConfig(&hadc2, &AnalogWDGConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 169;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB6 LD2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_6|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

void controlCallback(const std_msgs::String &ctl_msg)
{
	if (strcmp(ctl_msg.data, "stop") == 0)
		speed = STOP;
	else if (strcmp(ctl_msg.data, "go") == 0)
		speed = GO;
	else if (strcmp(ctl_msg.data, "slow") == 0)
		speed = SLOW;
	else if (strcmp(ctl_msg.data, "cw") == 0)
		dir = CLOCKWISE;
	else if (strcmp(ctl_msg.data, "ccw") == 0)
		dir = COUNTERCLOCKWISE;
	else if (strcmp(ctl_msg.data, "init") == 0)
		initMotor();
	else if (strcmp(ctl_msg.data, "motion") == 0)
		makeMotion();
}

void initMotor()
{
	speed = GO;
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, speed);
	//htim2.Instance->CCR1 = speed; // Duty 값 변경 analogWrite(AIN1_PIN, speed); // PWM 출력
	while (speed == GO)
	{
		HAL_ADC_Start(&hadc2);
		HAL_ADC_PollForConversion(&hadc2, 100); // 아두이노에서 아날로그 읽는데 걸리는 시간이 100ms이다
		HAL_ADC_Stop(&hadc2); // 읽고 멈춰
		int res = HAL_ADC_GetValue(&hadc2); // int res = analogRead(IR_PIN);
		HAL_ADC_Stop(&hadc2); // 읽고 멈춰
		if ((res >= IR_THRESHOLD))
		{
			speed = STOP;
			encoderPos = 0;
		}
	}
	makeMotion();
}

void makeMotion()
{
	speed = GO;
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, speed);
	//htim2.Instance->CCR1 = speed; // Duty 값 변경 analogWrite(AIN1_PIN, speed); // PWM 출력
	HAL_Delay(210); // 기어비에 따라서 초기화 함으로 원 위치에 온다고한다. 209 ~ 210 위에 라이다를 올릴것을 감안한다면 210을 추천한다.
	speed = STOP;
}

void doEncoderB() // 인터럽트 발생시에만 엔코더 작동
{
	edgeFlag = true;
	encoderPos = encoderPos + 1;

	if (encoderPos >= 2169) //2174
	{
		encoderPos = 0;
		//speed = STOP;
	}
	enc_msg.encoderValue = encoderPos;
	enc_msg.timeStamp = HAL_GetTick(); // 손좀 봐야 하는거 이 친구는 ms 단위의 시간을 반환해 준다.
}

void doEncoderA() // 인터럽트 발생시에만 엔코더 작동
{
	edgeFlag = true;
	encoderPos = encoderPos + 1;

	if (encoderPos >= 2169) //2174
	{
		encoderPos = 0;
		//speed = STOP;
	}
	//enc_msg.encoderValue = encoderPos;
	//enc_msg.timeStamp = HAL_GetTick(); // 손좀 봐야 하는거 이 친구는 ms 단위의 시간을 반환해 준다.
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_PIN) // 인터럽트 발생시 할것들
{
	switch (GPIO_PIN)
	{
	case GPIO_PIN_3:
		doEncoderB();
		break;

		//case GPIO_PIN_12:
		//doEncoderA();
		//break;

	default:
		doEncoderB();
		break;
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart2)
{
	nh.getHardware()->flush(); // 보내주고 지움
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart2)
{
	nh.getHardware()->reset_rbuf(); // 받고 DMA
}

void setup(void) // ROS INIT
{
	nh.initNode();
	nh.subscribe(ctl_sub);
	nh.advertise(ir_value);
	nh.advertise(encoder_info);
	enc_msg.header.frame_id = "encoder_link"; // 테스트 및 엔코더 작동시 출력할 것
	nh.advertise(chatter);
}

//테스트용 코드이며 현재 사용 하지 않음
void loop(void)
{
	//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
	str_msg.data = hello; // 테스트용 값 출력
	chatter.publish(&str_msg);
	//encoder_info.publish(&enc_msg); // 이건 그냥 테스트용이다.
	HAL_Delay(100);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
