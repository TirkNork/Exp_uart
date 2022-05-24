/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include <stdio.h>
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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
char TxDataBuffer[32] ={ 0 };
char RxDataBuffer[1] ={ 0 };

char Checkfromlast[2]={0};
uint8_t subMenu = 0;
uint8_t checkout = 0;
uint16_t LEDtoggle = 500;
uint16_t LED_f = 1;
uint32_t LEDTime = 0;
uint8_t LEDMode = 0;
uint8_t checkclickB1 = 0;
uint8_t B1Status = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
int16_t UARTRecieveIT();
void CodeselectingMenu(uint16_t numselect);
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
  /* USER CODE BEGIN 2 */
  char Select_Menu[]="Please Select the Menu from Below\r\n 0.LED Control\r\n 1.button Status\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)Select_Menu, strlen(Select_Menu), 10);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	static uint32_t timeUpdate =0;
	if(HAL_GetTick()-timeUpdate>=0)
	{
		timeUpdate=HAL_GetTick();

//		HAL_UART_Receive_IT(&huart2, (uint8_t*)RxDataBuffer,1);
		HAL_UART_Receive(&huart2, (uint8_t*)RxDataBuffer,1,0);
		int16_t inputchar = UARTRecieveIT();
		if(inputchar!=-1)
		{
			sprintf(TxDataBuffer, "ReceivedChar:[%c]\r\n", inputchar);		// default function printf
			HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
			CodeselectingMenu(inputchar);

		}
		static uint32_t timeStamp = 0;
		switch (LEDMode){
		case 0:
				  LEDTime = HAL_GetTick();
				  if(LED_f>=1){
					 LEDtoggle=1000/(LED_f*2);
				  }
				  else{
					  LEDtoggle=0;
				  }
				  if(LEDTime - timeStamp >= LEDtoggle){
					  timeStamp = HAL_GetTick();
					  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
				  }
			break;
		case 1:
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,GPIO_PIN_RESET);
			break;
		}
	}
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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
int16_t UARTRecieveIT()		// this function is to check
{
	static uint32_t dataPos =0;		// track recently position
	int16_t data=-1;
	if(huart2.RxXferSize - huart2.RxXferCount!=dataPos)  // this will work if size - typein don't be zero
	{
		data=RxDataBuffer[dataPos];		//get data of char position in array of RxDataBuffer (the data that return is a number is ASCII)
		dataPos= (dataPos+1)%huart2.RxXferSize;		//calculate if the value is go to the end after mod with size of huart2.RxXferSize will continue to count the first once again
	}
	return data;
}

void CodeselectingMenu(uint16_t numselect){
	//Announce List Menu in char
	char LED_Menu[]="LED Control Menu\r\n a:Speed Up +1Hz\r\n s:Speed Down -1Hz\r\n d:On/off\r\n x:back\r\n";
	char button_Status[]="Button Status Menu\r\n x:back\r\n" ;
	char Selecting_speed_up_LED[]="You are selecting Speed Up +1Hz Function\r\n";
	char Selecting_speed_down_LED[]="You are selecting Speed Down -1Hz Function\r\n";
	char Selecting_On_or_Off_LED[]="You are selecting On/Off Function\r\n";
	char Selecting_back[]="You are selecting back Function\r\n";
	char Select_Menu[]="Please Select the Menu from Below\r\n 0.LED Control\r\n 1.button Status\r\n";
	char Do_not_Press_Button_B1[]="Button B1 don't press\r\n";
	char Press_Button_B1[]="Button B1 press\r\n";
	char Error[]="Input error please try again\r\n";
//	HAL_UART_Transmit(&huart2, (uint8_t*)Select_Menu, strlen(Select_Menu), 10);

	Checkfromlast[subMenu] = numselect;
	if(Checkfromlast[0] == 48 || Checkfromlast[0] == 49){
		switch(Checkfromlast[0]){
			case 48 :
				subMenu = 1;
				if(Checkfromlast[1] == 97 || Checkfromlast[1] == 115 || Checkfromlast[1] == 100 || Checkfromlast[1] == 120){
					switch(Checkfromlast[1]){
						case 97:
							HAL_UART_Transmit(&huart2, (uint8_t*)Selecting_speed_up_LED, strlen(Selecting_speed_up_LED), 10);		// Display Speed Up Function
							LED_f=LED_f+1;
							Checkfromlast[1] = 0;
							checkout=0;
							break;
						case 115:
							HAL_UART_Transmit(&huart2, (uint8_t*)Selecting_speed_down_LED, strlen(Selecting_speed_down_LED), 10);		// Display Speed Down Function
							LED_f=LED_f-1;
							Checkfromlast[1] = 0;
							checkout=0;
							break;
						case 100:
							HAL_UART_Transmit(&huart2, (uint8_t*)Selecting_On_or_Off_LED, strlen(Selecting_On_or_Off_LED), 10);		// Display On/Off Function
							LEDMode^=1;
							Checkfromlast[1] = 0;
							checkout=0;
							break;
						case 120:
							HAL_UART_Transmit(&huart2, (uint8_t*)Selecting_back, strlen(Selecting_back), 10);		// Display Back Function
							Checkfromlast[0] = 0;
							Checkfromlast[1] = 0;
							subMenu = 0;
							checkout=1;
							HAL_UART_Transmit(&huart2, (uint8_t*)Select_Menu, strlen(Select_Menu), 10);
							break;
					}
				}
				else if(Checkfromlast[1] == 0){
						// non data
				}
				else{
					HAL_UART_Transmit(&huart2, (uint8_t*)Error, strlen(Error), 10);
					HAL_UART_Transmit(&huart2, (uint8_t*)LED_Menu, strlen(LED_Menu), 10);
					checkout=1;
				}
				if(checkout!=1){
					HAL_UART_Transmit(&huart2, (uint8_t*)LED_Menu, strlen(LED_Menu), 10);		// Display LED_Menu
				}
				break;

			case 49 :
				HAL_UART_Transmit(&huart2, (uint8_t*)button_Status, strlen(button_Status), 10);		// Display button_Status_Menu
				subMenu = 1;
				checkclickB1 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
				if(checkclickB1 == 1){
					HAL_UART_Transmit(&huart2, (uint8_t*)Do_not_Press_Button_B1, strlen(Do_not_Press_Button_B1), 10);
					B1Status=0;
				}
				else if(checkclickB1 == 0){
					HAL_UART_Transmit(&huart2, (uint8_t*)Press_Button_B1, strlen(Press_Button_B1), 10);
					B1Status=1;
				}
				if (Checkfromlast[1] == 120){
					HAL_UART_Transmit(&huart2, (uint8_t*)Selecting_back, strlen(Selecting_back), 10);		// Display Back Function
					Checkfromlast[0] = 0;
					Checkfromlast[1] = 0;
					subMenu = 0;
					HAL_UART_Transmit(&huart2, (uint8_t*)Select_Menu, strlen(Select_Menu), 10);
				}
				else if(Checkfromlast[1] == 0){
					// non data

				}
				else{
					HAL_UART_Transmit(&huart2, (uint8_t*)Error, strlen(Error), 10);
					HAL_UART_Transmit(&huart2, (uint8_t*)button_Status, strlen(button_Status), 10);
				}
				break;
		}
	}
	else if(Checkfromlast[0] == 0){
		// non data
	}
	else{
		HAL_UART_Transmit(&huart2, (uint8_t*)Error, strlen(Error), 10);
		HAL_UART_Transmit(&huart2, (uint8_t*)Select_Menu, strlen(Select_Menu), 10);
	}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	char Press_Button_B1[]="Button B1 press\r\n";
	char Do_not_Press_Button_B1[]="Button B1 don't press\r\n";
	if(GPIO_Pin == GPIO_PIN_13)
	{
		if(Checkfromlast[0]==49){
			B1Status^=1;
			if(B1Status==1){
				HAL_UART_Transmit(&huart2, (uint8_t*)Press_Button_B1, strlen(Press_Button_B1), 10);
			}
			else{
				HAL_UART_Transmit(&huart2, (uint8_t*)Do_not_Press_Button_B1, strlen(Do_not_Press_Button_B1), 10);
			}
		}
	}
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

