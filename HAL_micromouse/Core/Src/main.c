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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stdio.h"
#include "VL53L0x.h"
#include "../../MPU6050/mpu6050.h"
#include "move.h"
#include "encoders.h"
#include "SoftwareCRC.h"
#include "mapping.h"
#include "FloodFill.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//wyższa matematyka
#define PI 3.14159265359


//interwały zdarzań w pętli main w ms
#define DIST_READ_INTERVAL 100
#define BLINK_INTERVAL 500

//makra do obsługi diody LED
#define LED_ON()   HAL_GPIO_WritePin(GPIOB, RED_LED_Pin, 0)
#define LED_OFF()  HAL_GPIO_WritePin(GPIOB, RED_LED_Pin, 1)
#define LED_TOGGLE() HAL_GPIO_TogglePin(GPIOB, RED_LED_Pin)

//makro do obsługi przycisku
#define BUTTON_STATE() HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//zmienne dla regulatora PID
volatile float eps = 0.0f;
volatile uint8_t pidChangedFlag = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//przekierowanie printf dla portu szeregowego
int _write(int file, char *ptr, int len)
{
	HAL_UART_Transmit(&huart2, (uint8_t*) ptr, len, 50);
	return len;
}

//timer dla regulatora PID
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM10)
	{
		eps = leftTotal - rightTotal;
		pidChangedFlag = 1;
	}
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

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART2_UART_Init();
	MX_I2C1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	MX_TIM10_Init();
	/* USER CODE BEGIN 2 */

	//uruchomienie timera dla PID
	HAL_TIM_Base_Start_IT(&htim10);

	//zmienne lokalne funkcji main - ustawione jako lokalne by były od razu widoczne w debuggerze
	/************************************************************************/

	//zmienne dla MPU6050
	float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0, temperature = 0;

	//zmienne do podglądu enkoderów
	uint32_t prawy = 0;
	uint32_t lewy = 0;

	//ramka danych na WDS
	char frameWDS[50] =
	{ 0 };

	//zmienne do testów VL53L0x
	volatile int dist_F_tmp;
	volatile int dist_L_tmp;
	volatile int dist_R_tmp;
	volatile int dist_FL_tmp;
	volatile int dist_FR_tmp;

	//zmienne do odmierzania interwałów czasowych za pomocą systick
	uint32_t time_ToF = 0;
	uint32_t time_blink = 0;

	/************************************************************************/

	//zapal LED
	LED_ON();

	//inicjalizacja czujników ToF
	initMicromouseVL53L0x();

	//inicjalizacja MPU6050

	MPU6050_Init(&hi2c1);
	MPU6050_SetInterruptMode(MPU6050_INTMODE_ACTIVEHIGH);
	MPU6050_SetInterruptDrive(MPU6050_INTDRV_PUSHPULL);
	MPU6050_SetInterruptLatch(MPU6050_INTLATCH_WAITCLEAR);
	MPU6050_SetInterruptLatchClear(MPU6050_INTCLEAR_STATUSREAD);
	MPU6050_SetIntEnableRegister(0);
	MPU6050_SetDlpf(MPU6050_DHPF_5);

	//inicjalizacja PWM dla silników
	motorsInit();

	//inicjalizacja enkoderów
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	//inicjalizacja PID
	pidInit(&pidSt,PID_PARAM_KP, PID_PARAM_KI, PID_PARAM_KD, PID_DT);
	HAL_Delay(3000);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	while (1)
	{
		uint32_t czas1 = HAL_GetTick();
		//odczyt danych z mpu6050
		MPU6050_GetAccelerometerScaled(&ax, &ay, &az);
		MPU6050_GetGyroscopeScaled(&gx, &gy, &gz);
		temperature = MPU6050_GetTemperatureCelsius();

		//odczyt z enkoderów
		encRead();
		prawy = leftTotal;
		lewy = rightTotal;

		//test portu szeregowego
		printf("Hello world\n");

		////////////////////////////////////////////////////
		static int x = 0;
		if (x == 0)
		{
			x = move_back(200, 100, &pidSt);
			if (x == 1)
			{
				HAL_Delay(1000);
			}
		}

		if(BUTTON_STATE() == 1)
		{
			HAL_Delay(300);
			x = 0;
		}
		////////////////////////////////////////////////////

		//testowy odczyt z czujników ToF
		if ((HAL_GetTick() - time_ToF) >= DIST_READ_INTERVAL)
		{
			dist_F_tmp = ToF_readRangeContinuousMillimeters(&ToF_F);
			dist_R_tmp = ToF_readRangeContinuousMillimeters(&ToF_R);
			dist_L_tmp = ToF_readRangeContinuousMillimeters(&ToF_L);
			dist_FL_tmp = ToF_readRangeContinuousMillimeters(&ToF_FL);
			dist_FR_tmp = ToF_readRangeContinuousMillimeters(&ToF_FR);
			time_ToF = HAL_GetTick();
		}

		if ((HAL_GetTick() - time_blink) >= BLINK_INTERVAL)
		{
			LED_TOGGLE();
			time_blink = HAL_GetTick();
		}

		//odczyt z czujników ToF z filtrem medianowym
		/*
		 dist_F_tmp = ToF_medianFilter(&ToF_F, 7);
		 dist_R_tmp = ToF_medianFilter(&ToF_R, 7);
		 dist_L_tmp = ToF_medianFilter(&ToF_L, 7);
		 dist_FL_tmp = ToF_medianFilter(&ToF_FL, 7);
		 dist_FR_tmp = ToF_medianFilter(&ToF_FR, 7);
		 */

		//generowanie ramki danych na WDS
		//makeFrame(frameString,left_encoder,right_encoder,ToF_L,ToF_FL,ToF_F,ToF_FR,ToF_R);
		//tak powinna wyglądać przykładowa gotowa ramka -> "X_00012_00034_0023_0234_0433_3444_0003_1886576405"
		makeFrame(frameWDS, lewy, prawy, dist_L_tmp, dist_FL_tmp, dist_F_tmp,
				dist_FR_tmp, dist_R_tmp);
		//wysłanie ramki
		printf("%s\n", frameWDS);

		uint32_t czas2 = HAL_GetTick() - czas1;
		HAL_Delay(0);

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

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 84;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
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
