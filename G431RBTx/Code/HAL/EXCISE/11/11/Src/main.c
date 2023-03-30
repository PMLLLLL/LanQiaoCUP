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
#include "adc.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd_hal.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//按键宏定义
#define B1 HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0)
#define B2 HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1)
#define B3 HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2)
#define B4 HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//定义一个按键检测枚举
enum Key_All_state{
	Key_Check,
	Key_Press,
	Key_Release
};

enum Key_All_state Key_State=Key_Check;
uint32_t Key_Press_Time = 20 ;//第一次进入释放初始值
uint8_t Key_Value=0;//按键键值
uint8_t Key_Flag=0;//按键标志位

uint8_t Duty_PA6=10;//定义占空比变量
uint8_t Duty_PA7=10;

//显示界面变量
uint8_t Inerface_Num=0;
//控制模式
uint8_t Control_Mode=0;

//电压变量
uint32_t vlot_Value;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void LedAllClose(void);
void LcdDataInterface(float Vlot,int mode);
void LcdParaInterface(int Duty_PA6,int Duty_PA7);
void LcdInit(void);
void LedControl(void);
uint32_t AdcGet(void);
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
  MX_ADC2_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */
	LedAllClose();
	LCD_Init();//LCD初始化
	LcdInit();
	//开启PWM输出
	HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
	//清除中断
	__HAL_TIM_CLEAR_IT(&htim16, TIM_CHANNEL_1);
	__HAL_TIM_CLEAR_IT(&htim17, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //读取电压值
	  vlot_Value = AdcGet();
	  //按键检测
	  if(Key_Flag)
	  {
		Key_Flag = 0;
		  //界面切换按键
		if(Key_Value == 1) 
		{
			++Inerface_Num;
			Inerface_Num %= 2;
		}
		//仅参数界面有效
		if(Key_Value == 2 && Inerface_Num==1) 
		{
			Duty_PA6 += 10;
			if(Duty_PA6 == 100) Duty_PA6=10;
		}
		//仅参数界面有效
		if(Key_Value == 3 && Inerface_Num==1) 
		{
			Duty_PA7 += 10;
			if(Duty_PA7 == 100) Duty_PA7=10;
		}
		if(Key_Value == 4) 
		{
			Control_Mode++;
			Control_Mode %= 2;
		}
	  }
	  
	  //自动模式控制占空比
	  if(Control_Mode == 0)
	  {
		  	//改变占空比
		  __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, vlot_Value * 10000 /4096 -1);
		  __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, vlot_Value * 5000 /4096 -1);
	  }
	  else
	  {
			//改变占空比
		  __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, Duty_PA6 * 100);
		  __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, Duty_PA7 * 100 /2);
	  }
	  //显示界面
	  if(Inerface_Num == 0) LcdDataInterface(vlot_Value * 3.3/4096,Control_Mode);//数据界面
	  else LcdParaInterface( Duty_PA6, Duty_PA7);//参数界面
	  
	  //LED控制
	  LedControl();
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//关闭所有LED
void LedAllClose(void)
{
	//PD2 开启LED使能控制
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
	//关闭所有LED
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 |
						GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_SET);
	//PD2 关闭LED使能控制
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
}

void LcdInit(void)
{
	LCD_SetTextColor(White);
	LCD_SetBackColor(Black); 
	LCD_DisplayStringLine(Line0, "                    ");
	LCD_DisplayStringLine(Line1, "                    ");
	LCD_DisplayStringLine(Line2, "                    ");
	LCD_DisplayStringLine(Line3, "                    ");
	LCD_DisplayStringLine(Line4, "                    ");
	LCD_DisplayStringLine(Line5, "                    ");
	LCD_DisplayStringLine(Line6, "                    ");
	LCD_DisplayStringLine(Line7, "                    ");
	LCD_DisplayStringLine(Line8, "                    ");
	LCD_DisplayStringLine(Line9, "                    ");
}

void LcdDataInterface(float Vlot,int mode)
{
	uint8_t StringLine2[20];
	//显示Data
	LCD_DisplayStringLine(Line0, "      Data          ");
	sprintf((char*)StringLine2,"    V:%.2fV     ",Vlot);
	LCD_DisplayStringLine(Line2, StringLine2);
	if(mode==0)
		LCD_DisplayStringLine(Line4, "    Mode:AUTO       ");
	else if(mode == 1)
		LCD_DisplayStringLine(Line4, "    Mode:MANU       ");
}

void LcdParaInterface(int Duty_PA6,int Duty_PA7)
{
	uint8_t StringLine2[20];
	uint8_t StringLine4[20];
	//显示Data
	LCD_DisplayStringLine(Line0, "      Para          ");
	sprintf((char*)StringLine2,"    PA6:%2d%%     ",Duty_PA6);
	LCD_DisplayStringLine(Line2, StringLine2);
	sprintf((char*)StringLine4,"    PA7:%2d%%     ",Duty_PA7);
	LCD_DisplayStringLine(Line4, StringLine4);
}

//按键扫描 10ms扫描一次
void KeyScan(void)
{
	switch(Key_State)
	{
		case Key_Check:
			if(B1 == GPIO_PIN_RESET | B2 == GPIO_PIN_RESET | B3 == GPIO_PIN_RESET | B4 == GPIO_PIN_RESET)
				Key_State = Key_Press;
			break;
		case Key_Press:
			Key_State = Key_Release;//等待按键释放
			//键值判断
			if(B1 == GPIO_PIN_RESET) Key_Value=1;
			else if(B2 == GPIO_PIN_RESET) Key_Value=2;
			else if(B3 == GPIO_PIN_RESET) Key_Value=3;
			else if(B4 == GPIO_PIN_RESET) Key_Value=4;
			else Key_State = Key_Check;//消抖判断
			break;
		case Key_Release:
			if(B1 == GPIO_PIN_RESET | B2 == GPIO_PIN_RESET | B3 == GPIO_PIN_RESET | B4 == GPIO_PIN_RESET)
				Key_Press_Time+=10;//按键持续按下
			else//按键释放
			{
				Key_Flag=1;//按键标志位
				Key_State = Key_Check;//按键状态复位
			}
			break;
		default:break;
	}
}

//LED控制
void LedControl(void)
{
	//PD2 开启LED使能控制
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
	//关闭所有LED
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 |
						GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_SET);
	//自动模式L1点亮 数据界面L2点亮
	if(Control_Mode == 0) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8 , GPIO_PIN_RESET);
	if(Inerface_Num == 0) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9 , GPIO_PIN_RESET);
	//PD2 关闭LED使能控制
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
}

//ADC获取
uint32_t AdcGet(void)
{
	uint32_t Value;
	HAL_ADC_Start(&hadc2);//开启ADC
	HAL_ADC_PollForConversion(&hadc2, 2);//开启adc转换
	Value = HAL_ADC_GetValue(&hadc2);//获取ADC的值
	HAL_ADC_Stop(&hadc2);
	return Value;
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
