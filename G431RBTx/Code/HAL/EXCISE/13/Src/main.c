/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd_hal.h"
#include "stdio.h"
#include "Lcd.h"
#include "key.h"
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

/* USER CODE BEGIN PV */
extern uint8_t Key_Flag;
extern uint8_t Key_Value;

uint8_t Real_PassWord[3]={1,2,3};
uint8_t Pass_Word[3]={1,8,5};
uint8_t Pass_Look[3]={0,0,0};
uint8_t Key_Effect=1;
uint8_t Tog_Led2=0;

uint16_t Error_Num=0;

uint8_t Rec_Str[8];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void PassWordAdd(uint8_t Add_Num);
uint8_t PassWordTest(uint8_t* PassWord,uint8_t* Real_PassWord,uint8_t* Pass_Look);
void ResetPSD(uint8_t* Pass_Look);
void OpenLed1(void);
void CloseLed1(void);
void TogLed2(void);
void ChangPassWord(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_UART_Receive_DMA(&huart1, Rec_Str, 7);
	ChangPassWord();
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static uint16_t Count_5s=0;
	if(++Count_5s==500)
	{
		Count_5s=0;
		if(Key_Effect==0)
		{
			CloseLed1();
			ResetPSD(Pass_Look);
			Key_Effect=1;
			ShowPSD(Pass_Word,Pass_Look);
			__HAL_TIM_SET_AUTORELOAD(&htim2,1000);
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,500);
		}
		else
		{
			Tog_Led2=0;
			CloseLed1();
		}
		HAL_TIM_Base_Stop_IT(&htim4);
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
  HAL_UART_Receive_IT(&huart1, Rec_Str, 7);
  LcdSetInit(White,Black);
  ShowPSD(Pass_Word,Pass_Look);
  CloseLed1();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	if(Key_Flag==1)
	{
		Key_Flag=0;
		if( Key_Effect==1)
		{
			if(Key_Value==1) PassWordAdd(0);
			if(Key_Value==2) PassWordAdd(1);
			if(Key_Value==3) PassWordAdd(2);
			ShowPSD(Pass_Word,Pass_Look);
		}
		if(Key_Value==4)
		{
			if(PassWordTest(Pass_Word,Real_PassWord,Pass_Look)==1)
			{
				Error_Num=0;
				__HAL_TIM_SET_AUTORELOAD(&htim2,500);
				__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,50);
				OpenLed1();
				Key_Effect=0;
				ShowSTA(2000,10);
				HAL_TIM_Base_Start_IT (&htim4);
			}
			else
			{
				ResetPSD(Pass_Look);
				ShowPSD(Pass_Word,Pass_Look);
				Error_Num++;
				if(Error_Num==3)
				{
					Error_Num=0;
					Tog_Led2=1;
					HAL_TIM_Base_Start_IT (&htim4);
				}
			}
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage 
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV3;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks 
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void PassWordAdd(uint8_t Add_Num)
{
	if(Add_Num<=2)
	{
		if(Pass_Look[Add_Num]==0) {
		Pass_Look[Add_Num]=1;
		Pass_Word[Add_Num]=0;
		}
		else{
			if(++Pass_Word[Add_Num]==10) Pass_Word[Add_Num]=0;
		}
	}
}

uint8_t PassWordTest(uint8_t* PassWord,uint8_t* Real_PassWord,uint8_t* Pass_Look)
{
	uint8_t i=0;
	if(Pass_Look[0]==1 && Pass_Look[1]==1 && Pass_Look[2]==1)
	{
		while(i<3){
			if(PassWord[i]==Real_PassWord[i]) i++;
			else return 0;//验证不通过
		}
		return 1;//验证通过
	}
	else return 0;//验证不通过
}
	
void ResetPSD(uint8_t* Pass_Look)
{
	Pass_Look[0]=0;
	Pass_Look[1]=0;
	Pass_Look[2]=0;
}

void OpenLed1(void)
{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);
}
void CloseLed1(void)
{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);
}

void TogLed2(void)
{
	static uint8_t state=0;
	if(state==0)
	{
		state=1;
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);
	}
	else
	{
		state=0;
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);
	}
}

void ChangPassWord(void)
{
	uint8_t i;
	while(i<3)
	{
		if((Rec_Str[i]-'0')==Real_PassWord[i]) i++;
		else return ;
	}
	
	Real_PassWord[0]=Rec_Str[4]-'0';
	Real_PassWord[1]=Rec_Str[5]-'0';
	Real_PassWord[2]=Rec_Str[6]-'0';
	return ;
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
