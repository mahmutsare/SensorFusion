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
#include "sensor_fusion_lib.h"
#include "lcd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
volatile float theta_asil;//acilari depolamak icin degiskenler tanimliyoruz
volatile float phi_asil;
volatile float theta_SF;
volatile float phi_SF;
volatile int basladi;
volatile float elptime=0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

/* USER CODE BEGIN PV */
// veri gruplarimizi olusturuyoruz
sensor_t veriler;
//baslangic kosullarini ve filtrenin parametrelerini tanimliyoruz
SF_KF_t K_fil = {
	.Q= {    { 0.01 , 0, 0, 0},
			 { 0, 0.01 , 0, 0},
			 { 0, 0,0.01 , 0},
			 { 0, 0, 0,  0.01}},
    .R= {   {0.00001,  0,  0,   0},
    		{  0, 0.00001,  0,   0},
			{  0,  0, 0.00001,   0},
			{  0,  0,  0, 0.00001}},
	.P= {   {  5,  0,  0,   0},
		    {  0,  5,  0,   0},
			{  0,  0,  5,   0},
			{  0,  0,  0,   5}},
	.K_gain={   { 0.01,  0,  0,   0},
		    	{  0, 0.01,  0,   0},
				{  0,  0, 0.01,   0},
				{  0,  0,  0,  0.01}},
	.q_est=    { 1, 0, 0, 0 },
	.q_accel=  { 1, 0, 0, 0 },
	.q_pred=   { 1, 0, 0, 0 },
	.time_interval= 0.01
};


kucuk_kalman Gy_kf={
		.Ppredgy={0,0,0},
		.Kgain_gy={0 , 0 ,0},
		.Pgy={0.1 ,0.1 ,0.1},
		.Qgy={0.001 ,0.001 ,0.001},
		.Rgy={10,10,  10}
};

char temp1[8];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
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
  MX_GPIO_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  Lcd_HandleTypeDef lcd;
  Lcd_PortType ports[] = {GPIOB,  GPIOB , GPIOB  ,GPIOB  };
  Lcd_PinType pins[] = {GPIO_PIN_3,GPIO_PIN_4 ,GPIO_PIN_5 ,GPIO_PIN_6 };
  lcd= Lcd_create(ports, pins, GPIOB,GPIO_PIN_12, GPIOB, GPIO_PIN_13, LCD_4_BIT_MODE);

  Lcd_clear(&lcd);
basladi=MPU6050_init_and_conf(&hi2c2);//sensoru konfigure edip baslatiyoruz
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  Sensor_Fusion(&hi2c2, &K_fil   ,&veriler ,&Gy_kf);
	  theta_asil=veriler.theta_accel*180/3.14159;//ivmeolcerden aldigimiz theta acisini dereceye ceviriyoruz
	  phi_asil=veriler.phi_accel*180/3.14159;//ivmeolcerden aldigimiz phi acisini dereceye ceviriyoruz
	  theta_SF=veriler.THETA;//kalman filtresinden aldigimiz theta acisini dereceye ceviriyoruz
	  phi_SF=veriler.PHI;//kalman filtresinden aldigimiz phi acisini dereceye ceviriyoruz





	  sprintf(temp1, "%.1f",theta_asil);
	  Lcd_cursor(&lcd, 0,0);
	  Lcd_string(&lcd, temp1);


	  sprintf(temp1, "%.1f",phi_asil);
	  Lcd_cursor(&lcd, 0,8);
	  Lcd_string(&lcd, temp1);

	  sprintf(temp1, "%.1f",theta_SF);
	  Lcd_cursor(&lcd, 1,0);
	  Lcd_string(&lcd, temp1);


	  sprintf(temp1, "%.1f",phi_SF);
	  Lcd_cursor(&lcd, 1,8);
	  Lcd_string(&lcd, temp1);












	  while ((HAL_GetTick()- elptime)*1000<10000);//donguyu 0.01 saniyede sinirliyoruz
	  elptime=HAL_GetTick();




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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB12 PB13 PB3 PB4
                           PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
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
