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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd_hal.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */
//按键的宏定义
#define B1 HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0)
#define B2 HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1)
#define B3 HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2)
#define B4 HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)
/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//定义按键状态枚举
enum AllKeyState{
	Key_Check,
	Key_Press,
	Key_Release
};
enum AllKeyState Key_State = Key_Check;//定义一个按键变量
uint8_t Key_Flag=0;//按键标志位
uint8_t Key_Down_Time = 20;//按键持续时间
uint8_t Key_Value = 0;//按键键值

//定义电压上限值和电压下限值
double Max_Vlot=2.4;
double Min_Vlot=1.2;

//高亮显示行数
uint8_t H_Light_Line = 0;

//显示界面选择
uint8_t Interface_Mode = 0;//0代表数据界面 1代表参数设置界面

//读取到的ADC电压
float Adc_Value;

//当前电压状态
uint8_t Vlot_State = 0;//0代表超过上限 1 代表低于下限 2代表正常
//定义显示状态常量字符串
char Status_Strings[3][7]={
	"Upper",
	"Lower",
	"Normal"
};

//上限提醒指示灯和下限提醒指示灯位置
signed int Upper_Led = 1;
signed int Lower_Led = 2;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void CloseLed(void);
void Lcd_Hal_init(void);
void InerfaceDataDis(float Volt);
void InerfaceParaDis(uint8_t hightlight);
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
  /* USER CODE BEGIN 2 */
	CloseLed();//LED关闭
	Lcd_Hal_init();//LCD初始化
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //读取ADC电压
	  Adc_Value = AdcGet() * 3.3 / 4096;
	  //判断电压状态
	if(Adc_Value > Max_Vlot) Vlot_State = 0;
	else if(Adc_Value < Min_Vlot) Vlot_State = 1;
	else Vlot_State = 2;
	  //执行界面切换
	  if(Interface_Mode ==0 ) InerfaceDataDis(Adc_Value);
	  else if(Interface_Mode ==1 ) InerfaceParaDis(H_Light_Line);
	  
	  //按键判断
	  if(Key_Flag ==1) 
	  {
		Key_Flag=0;
		  //按键界面切换
		if(Key_Value == 1) {
			Interface_Mode ++;
			Interface_Mode %= 2;
		}
		//高亮选择切换
		if(Key_Value == 2 && Interface_Mode == 1 ) {
			H_Light_Line ++;
			H_Light_Line %= 4;
		}
		if(Key_Value == 3 && Interface_Mode == 1 ) {
			//电压+0.3
			switch(H_Light_Line)
			{
				case 0:if(Max_Vlot < 3.2) Max_Vlot += 0.3;break;
				case 1:if(Min_Vlot < 3.2) Min_Vlot += 0.3;break;
				case 2:
					if(Upper_Led < 8) Upper_Led++;
					if(Upper_Led == Lower_Led) Upper_Led ++;
					if(Upper_Led == 9) Upper_Led-=2;
					break;
				case 3:
					if(Lower_Led < 8) Lower_Led++;
					if(Upper_Led == Lower_Led) Lower_Led ++;
					if(Lower_Led == 9) Lower_Led-=2;
					break;
				default:break;
			}
		}
		if(Key_Value == 4 && Interface_Mode == 1 ) {
			switch(H_Light_Line)
			{
				case 0:
					if(Max_Vlot > 0.3) Max_Vlot = Max_Vlot - 0.3;
					else Max_Vlot = 0.0;
				break;
				case 1:
					if(Min_Vlot > 0.3) Min_Vlot = Min_Vlot - 0.3;
					else Min_Vlot = 0.0;
				break;
				case 2:
					if(Upper_Led > 1) Upper_Led--;
					if(Upper_Led == Lower_Led) Upper_Led --;
					if(Upper_Led == 0) Upper_Led+=2;
					break;
				case 3:
					if(Lower_Led > 1) Lower_Led--;
					if(Upper_Led == Lower_Led) Lower_Led --;
					if(Lower_Led == 0) Lower_Led+=2;
					break;
				default:break;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void CloseLed(void)
{
	//开启LED控制
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
	//关闭所有LED
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8 |  GPIO_PIN_9 |  GPIO_PIN_10 |  GPIO_PIN_11 |  GPIO_PIN_12 |  GPIO_PIN_13 |  GPIO_PIN_14 |  GPIO_PIN_15, GPIO_PIN_SET);
	//关闭LED控制
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
}

void Lcd_Hal_init(void)
{
	LCD_Init();
	LCD_SetTextColor(White);
	LCD_SetBackColor(Black);
	LCD_Clear(White);
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

void InerfaceDataDis(float Volt)
{
	char String_Line2[20];
	char String_Line3[20];
	
	LCD_DisplayStringLine(Line0, "        Main        ");
	
	//显示电压的数值
	sprintf(String_Line2,"    Volt:%.2fV      ",Volt);
	LCD_DisplayStringLine(Line1, (uint8_t*)String_Line2);
	
	//根据电压不同显示不同的状态
	sprintf(String_Line3,"    Status:%s    ",Status_Strings[Vlot_State]);

	//填补空缺
	LCD_DisplayStringLine(Line2, (uint8_t*)String_Line3);
	LCD_DisplayStringLine(Line3, "                    ");
	LCD_DisplayStringLine(Line4, "                    ");
}

void InerfaceParaDis(uint8_t hightlight)
{
	char String_Line1[20];
	char String_Line2[20];
	char String_Line3[20];
	char String_Line4[20];
	LCD_DisplayStringLine(Line0, "      Setting       ");
	//显示电压上限值
	sprintf(String_Line1,"   Max Volt:%.1fV    ",Max_Vlot);
	if(hightlight == 0) LCD_SetBackColor(Blue);//高亮选择
	LCD_DisplayStringLine(Line1, (uint8_t*)String_Line1);
	LCD_SetBackColor(Black);
	//显示电压下限值
	sprintf(String_Line2,"   Min Volt:%.1fV    ",Min_Vlot);
	if(hightlight == 1) LCD_SetBackColor(Blue);//高亮选择
	LCD_DisplayStringLine(Line2, (uint8_t*)String_Line2);
	LCD_SetBackColor(Black);
	//显示上限提醒指示灯
	sprintf(String_Line3,"   Upper:LD%d        ",Upper_Led);
	if(hightlight == 2) LCD_SetBackColor(Blue);//高亮选择
	LCD_DisplayStringLine(Line3, (uint8_t*)String_Line3);
	LCD_SetBackColor(Black);
	//显示上限提醒指示灯
	sprintf(String_Line4,"   Lower:LD%d        ",Lower_Led);
	if(hightlight == 3) LCD_SetBackColor(Blue);//高亮选择
	LCD_DisplayStringLine(Line4, (uint8_t*)String_Line4);
	LCD_SetBackColor(Black);
}

//获取模拟电压的值
//输出为0-4096
uint32_t AdcGet(void)
{
	uint32_t adcvalue;
	HAL_ADC_Start(&hadc2);//启动ADC
	HAL_ADC_PollForConversion(&hadc2, 2);//启动ADC转换
	adcvalue = HAL_ADC_GetValue(&hadc2);//获取ADC转换之后的值
	HAL_ADC_Stop(&hadc2);//关闭ADC
	return adcvalue;
}

void KeyScan(void)
{
	switch(Key_State)
	{
		case Key_Check:
			//按键按下检测
			if(B1 == RESET | B2 == RESET | B3 == RESET | B4 == RESET )
				//进入按键按下判断
				Key_State = Key_Press;
			break;
		case Key_Press:
			Key_State = Key_Release;
			//错误检测判断
			if(B1 == RESET) Key_Value = 1;
			else if(B2 == RESET) Key_Value = 2;
			else if(B3 == RESET) Key_Value = 3;
			else if(B4 == RESET) Key_Value = 4;
			else Key_State = Key_Check;
			break;
		case Key_Release:
			if(B1 == RESET | B2 == RESET | B3 == RESET | B4 == RESET )
			{
				//持续按键判断
				Key_Down_Time +=10;
			}
			else{
				Key_Flag = 1;//按键松开
				Key_State = Key_Check;//状态复位
			}
			break;
		default:
			break;
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
