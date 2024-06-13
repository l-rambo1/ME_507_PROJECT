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
#include "string.h"
#include <stdlib.h>
#include <stdio.h>
#include "stm32f411xe.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_hal_rcc.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define IMU_ADDR (0x28)
#define OP_MODE_REG (0x3D)
#define NDOF_MODE (0x0C)
#define EUL_X (0x1A)
#define GYR_Y_LSB (0x16)

#define velo_error velo_meas;
float   actuation_pwm;
float   actuation_acc;
float   actuation_acc_prev;
float actuation_pwm_prev;

uint32_t i;
uint32_t count;


//UART
char transmit_buff[150];
uint16_t n;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
//I2C
uint32_t scan;
uint32_t result;
uint16_t i2c_buff_write[1];
int16_t i2c_buff_read[6];
int16_t i2c_buff_gyro[2];
float eul_x, eul_y,eul_z;
int16_t gyr_y;
float eul_y_offset;


//controller
float pos_meas;  /// measurement from the IMU
uint16_t velo_meas; /// measurement from the IMU

float pos_setpoint;
float pos_error;

float kp;
float kd;
float e_sum;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM5_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
void set_motor(uint32_t servo_cmp);
void start_motor(void);
float get_pos(void);
int16_t get_vel(void);
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
  MX_USART2_UART_Init();
  MX_TIM5_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
  HAL_I2C_Init(&hi2c2);


  //testing stuff
  	  n = sprintf(transmit_buff,"UART initialized \r\n");
  	  HAL_UART_Transmit(&huart2, (uint8_t *) &transmit_buff, n, HAL_MAX_DELAY); //transmit the read data
  	//I2C initialization stuff

  	    	  for (i = 1; i < 128; i++) {
  	    	          result = HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t)(i << 1), 1, 10);

  	    	          if (result == HAL_OK) {
  	    	              n=sprintf(transmit_buff,"Device found at address 0x%02d\n\r",(int) i);
  	    	              HAL_UART_Transmit(&huart2, (uint8_t *) &transmit_buff, n, HAL_MAX_DELAY); //transmit the read data
  	    	          }
  	    	      }
  	    	      n=sprintf(transmit_buff,"Scan completed.\n\r");
  	    	      HAL_UART_Transmit(&huart2, (uint8_t *) &transmit_buff, n, HAL_MAX_DELAY); //transmit the read data


  	    	      scan = HAL_I2C_IsDeviceReady(&hi2c2,(uint16_t)(IMU_ADDR << 1) , 1, 10);

  	    	          if (scan == HAL_OK) {
  	    	              n=sprintf(transmit_buff,"IMU Found at 40\n\r");
  	    	              HAL_UART_Transmit(&huart2, (uint8_t *) &transmit_buff, n, HAL_MAX_DELAY); //transmit the read data

  	    	          }
  	    	          if (scan != HAL_OK) {
  	    	             n=sprintf(transmit_buff,"No IMU found\n\r");
  	    	             HAL_UART_Transmit(&huart2, (uint8_t *) &transmit_buff, n, HAL_MAX_DELAY); //transmit the read data
  	    	             HAL_Delay(100000);
  	    	          }

//put the devise in NDOF mode
i2c_buff_write[0]=NDOF_MODE;


//initialize motor
start_motor();

set_motor(160000);
HAL_Delay(3000);

for (i = 10; i >0; i--) {
	 n=sprintf(transmit_buff,"Launching in %ld\n\r",i);
	  	    	             HAL_UART_Transmit(&huart2, (uint8_t *) &transmit_buff, n, HAL_MAX_DELAY); //transmit the read data
	  	    	             HAL_Delay(1000);
}

HAL_I2C_Mem_Write (&hi2c2,IMU_ADDR << 1, OP_MODE_REG, I2C_MEMADD_SIZE_8BIT , (uint8_t *) &i2c_buff_write, 1 , HAL_MAX_DELAY );
//find offset for euler angle y
for (i=0; i<20;i++){
HAL_I2C_Mem_Read(&hi2c2, IMU_ADDR << 1,  EUL_X  , I2C_MEMADD_SIZE_8BIT,(uint8_t *)&i2c_buff_read,6,HAL_MAX_DELAY);
HAL_Delay(20);
}
HAL_I2C_Mem_Read(&hi2c2, IMU_ADDR << 1,  EUL_X  , I2C_MEMADD_SIZE_8BIT,(uint8_t *)&i2c_buff_read,6,HAL_MAX_DELAY);
eul_y_offset=((i2c_buff_read[3]<<8 | i2c_buff_read[2])/16);


//controller parameters initially
kp=5;
kd=0;
int ki=0;
pos_setpoint=0;
actuation_acc_prev=160000;
actuation_pwm_prev=0;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
    {


  	  //measure velocity and position

  	  pos_meas= get_pos();
      velo_meas=get_vel();



      pos_error=pos_setpoint-pos_meas;
      e_sum=e_sum+pos_error;



      // Simplified step-by-step calculation
      float kp_term = kp * pos_error;
      float kd_term = kd * velo_error;
      float ki_term= ki * e_sum;
      actuation_pwm = kp_term + kd_term + ki_term;


  	  //for loop to do the ramp input of pwm
  	  for (i = 1; i < 5; i++) {

  		  actuation_acc=i*actuation_pwm+actuation_acc_prev;;

  		  //these if statements limit pwm control within available range
  		  if(actuation_acc>190000){
  			actuation_acc=190000;
  		  }
  		 if(actuation_acc<96000){
  		   actuation_acc=96000;
  		  }
  		HAL_Delay(0);
  		set_motor(actuation_acc);



  	  }
  	 actuation_acc_prev=actuation_acc;

  	 n=sprintf(transmit_buff,"PWM: %ld\n\r",(int32_t)actuation_acc);
  	  	 	HAL_UART_Transmit(&huart2, (uint8_t *) &transmit_buff, n, HAL_MAX_DELAY); //transmit the read data



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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 96;
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
  hi2c2.Init.ClockSpeed = 200000;
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
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 1919999;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 96000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA6 PA7 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void set_motor(uint32_t servo_cmp)
	  {
	  	  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, servo_cmp);
	  	 //n=sprintf(transmit_buff,"Motor set: %ld\n\r",servo_cmp);
	  	 //HAL_UART_Transmit(&huart2, (uint8_t *) &transmit_buff, n, HAL_MAX_DELAY); //transmit the read data
	  }
void start_motor(){
	  	  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 143000);
	  	  HAL_Delay(3000);
	  	  n=sprintf(transmit_buff,"Motor start completed.\n\r");
	  	  HAL_UART_Transmit(&huart2, (uint8_t *) &transmit_buff, n, HAL_MAX_DELAY); //transmit the read data

	  }
float get_pos(){
	HAL_I2C_Mem_Read(&hi2c2, IMU_ADDR << 1,  EUL_X  , I2C_MEMADD_SIZE_8BIT,(uint8_t *)&i2c_buff_read,6,HAL_MAX_DELAY);

		  HAL_I2C_ModeTypeDef error = HAL_I2C_GetError(&hi2c2);
		  if (error!= HAL_I2C_ERROR_NONE){
			  n=sprintf(transmit_buff,"I2C error\n\r");
			  //HAL_UART_Transmit(&huart2, (uint16_t *) &transmit_buff, n, HAL_MAX_DELAY);
			  //HAL_Delay(500);

		  }



		  eul_x=(i2c_buff_read[1]<<8 | i2c_buff_read[0])/16;
		  eul_y=((i2c_buff_read[3]<<8 | i2c_buff_read[2])/16.0)-eul_y_offset;
		  int eul_y_int=(int)eul_y;
		  eul_z=(i2c_buff_read[5]<<8 | i2c_buff_read[4])/16;

		  n=sprintf(transmit_buff,"EUL_Y= %d\n\r", eul_y_int);
		  HAL_UART_Transmit(&huart2, (uint8_t *) &transmit_buff, n, HAL_MAX_DELAY);
		  return eul_y;

}
int16_t get_vel(){
	 HAL_I2C_Mem_Read(&hi2c2, IMU_ADDR << 1,  GYR_Y_LSB  , I2C_MEMADD_SIZE_8BIT,(uint8_t *)&i2c_buff_gyro,2,HAL_MAX_DELAY);

	          HAL_I2C_ModeTypeDef error = HAL_I2C_GetError(&hi2c2);
	  		  if (error!= HAL_I2C_ERROR_NONE){
	  			  n=sprintf(transmit_buff,"I2C error\n\r");
	  			  //HAL_UART_Transmit(&huart2, (uint16_t *) &transmit_buff, n, HAL_MAX_DELAY);
	  			  //HAL_Delay(500);

	  		  }



	  		  gyr_y=(i2c_buff_gyro[1]<<8 | i2c_buff_gyro[0])/16;
	  		  int gyr_y_int=(int)gyr_y;


	  		  n=sprintf(transmit_buff,"GYRO_Y=%d\n\r", gyr_y_int);
	  		  HAL_UART_Transmit(&huart2, (uint8_t *) &transmit_buff, n, HAL_MAX_DELAY);
		  return gyr_y;

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
