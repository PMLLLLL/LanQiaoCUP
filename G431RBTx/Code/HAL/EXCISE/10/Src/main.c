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
//�����ĺ궨��
#define B1 HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0)
#define B2 HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1)
#define B3 HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2)
#define B4 HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)
/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//���尴��״̬ö��
enum AllKeyState{
	Key_Check,
	Key_Press,
	Key_Release
};
enum AllKeyState Key_State = Key_Check;//����һ����������
uint8_t Key_Flag=0;//������־λ
uint8_t Key_Down_Time = 20;//��������ʱ��
uint8_t Key_Value = 0;//������ֵ

//�����ѹ����ֵ�͵�ѹ����ֵ
double Max_Vlot=2.4;
double Min_Vlot=1.2;

//������ʾ����
uint8_t H_Light_Line = 0;

//��ʾ����ѡ��
uint8_t Interface_Mode = 0;//0�������ݽ��� 1����������ý���

//��ȡ����ADC��ѹ
float Adc_Value;

//��ǰ��ѹ״̬
uint8_t Vlot_State = 0;//0���������� 1 ����������� 2��������
//������ʾ״̬�����ַ���
char Status_Strings[3][7]={
	"Upper",
	"Lower",
	"Normal"
};

//��������ָʾ�ƺ���������ָʾ��λ��
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
	CloseLed();//LED�ر�
	Lcd_Hal_init();//LCD��ʼ��
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //��ȡADC��ѹ
	  Adc_Value = AdcGet() * 3.3 / 4096;
	  //�жϵ�ѹ״̬
	if(Adc_Value > Max_Vlot) Vlot_State = 0;
	else if(Adc_Value < Min_Vlot) Vlot_State = 1;
	else Vlot_State = 2;
	  //ִ�н����л�
	  if(Interface_Mode ==0 ) InerfaceDataDis(Adc_Value);
	  else if(Interface_Mode ==1 ) InerfaceParaDis(H_Light_Line);
	  
	  //�����ж�
	  if(Key_Flag ==1) 
	  {
		Key_Flag=0;
		  //���������л�
		if(Key_Value == 1) {
			Interface_Mode ++;
			Interface_Mode %= 2;
		}
		//����ѡ���л�
		if(Key_Value == 2 && Interface_Mode == 1 ) {
			H_Light_Line ++;
			H_Light_Line %= 4;
		}
		if(Key_Value == 3 && Interface_Mode == 1 ) {
			//��ѹ+0.3
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
	//����LED����
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
	//�ر�����LED
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8 |  GPIO_PIN_9 |  GPIO_PIN_10 |  GPIO_PIN_11 |  GPIO_PIN_12 |  GPIO_PIN_13 |  GPIO_PIN_14 |  GPIO_PIN_15, GPIO_PIN_SET);
	//�ر�LED����
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
	
	//��ʾ��ѹ����ֵ
	sprintf(String_Line2,"    Volt:%.2fV      ",Volt);
	LCD_DisplayStringLine(Line1, (uint8_t*)String_Line2);
	
	//���ݵ�ѹ��ͬ��ʾ��ͬ��״̬
	sprintf(String_Line3,"    Status:%s    ",Status_Strings[Vlot_State]);

	//���ȱ
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
	//��ʾ��ѹ����ֵ
	sprintf(String_Line1,"   Max Volt:%.1fV    ",Max_Vlot);
	if(hightlight == 0) LCD_SetBackColor(Blue);//����ѡ��
	LCD_DisplayStringLine(Line1, (uint8_t*)String_Line1);
	LCD_SetBackColor(Black);
	//��ʾ��ѹ����ֵ
	sprintf(String_Line2,"   Min Volt:%.1fV    ",Min_Vlot);
	if(hightlight == 1) LCD_SetBackColor(Blue);//����ѡ��
	LCD_DisplayStringLine(Line2, (uint8_t*)String_Line2);
	LCD_SetBackColor(Black);
	//��ʾ��������ָʾ��
	sprintf(String_Line3,"   Upper:LD%d        ",Upper_Led);
	if(hightlight == 2) LCD_SetBackColor(Blue);//����ѡ��
	LCD_DisplayStringLine(Line3, (uint8_t*)String_Line3);
	LCD_SetBackColor(Black);
	//��ʾ��������ָʾ��
	sprintf(String_Line4,"   Lower:LD%d        ",Lower_Led);
	if(hightlight == 3) LCD_SetBackColor(Blue);//����ѡ��
	LCD_DisplayStringLine(Line4, (uint8_t*)String_Line4);
	LCD_SetBackColor(Black);
}

//��ȡģ���ѹ��ֵ
//���Ϊ0-4096
uint32_t AdcGet(void)
{
	uint32_t adcvalue;
	HAL_ADC_Start(&hadc2);//����ADC
	HAL_ADC_PollForConversion(&hadc2, 2);//����ADCת��
	adcvalue = HAL_ADC_GetValue(&hadc2);//��ȡADCת��֮���ֵ
	HAL_ADC_Stop(&hadc2);//�ر�ADC
	return adcvalue;
}

void KeyScan(void)
{
	switch(Key_State)
	{
		case Key_Check:
			//�������¼��
			if(B1 == RESET | B2 == RESET | B3 == RESET | B4 == RESET )
				//���밴�������ж�
				Key_State = Key_Press;
			break;
		case Key_Press:
			Key_State = Key_Release;
			//�������ж�
			if(B1 == RESET) Key_Value = 1;
			else if(B2 == RESET) Key_Value = 2;
			else if(B3 == RESET) Key_Value = 3;
			else if(B4 == RESET) Key_Value = 4;
			else Key_State = Key_Check;
			break;
		case Key_Release:
			if(B1 == RESET | B2 == RESET | B3 == RESET | B4 == RESET )
			{
				//���������ж�
				Key_Down_Time +=10;
			}
			else{
				Key_Flag = 1;//�����ɿ�
				Key_State = Key_Check;//״̬��λ
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
