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
#include "lcd.h"
#include "led.h"
#include "key.h"
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

/* USER CODE BEGIN PV */

struct CarInfer{
	u8 state;//车位的使用状态 0代表没有使用 1代表已经被使用
	uint8_t CarType[5];//停车的类型
	uint8_t ID[5];//车号
	u16 Year;//停车时间年份
	u8 Month;//停车时间月份
	u8 Day;//停车时间天
	u8 hour;//停车时间小时
	u8 min;//停车时间分
	u8 sec;//停车时间秒
};

//建立一个停车场数组
struct CarInfer All_Set[8];
//建立一个临时信息数组
struct CarInfer TempData;

uint16_t Cnber_Car_Num=0;
uint16_t Vnber_Car_Num=0;
uint16_t Rest_Car_Num=8;
double Cnber_Car_Rate=3.5;
double Vnber_Car_Rate=2.0;

uint8_t Ds_Mode=0;//0代表车位显示界面 1代表费率显示界面
uint8_t Ds_Mode_old=1;

uint8_t Pluse_Out=0;

uint8_t Rx_Str[44];
char Tx_Str[40];

uint8_t CarSection;
char temp1[20];
char temp2[20];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void InterFaceOneKeyFunc(void);
void InterFaceTwoKeyFunc(void);
int FormRight(uint8_t * Rx_Str);
int CreateCar(uint8_t* RxStr);
void InitCar(void);
void StoreCar(uint8_t* RxStr,int i);
int CarExist(uint8_t* RxStr);
uint8_t CaculateTatolFee(uint8_t* Rx_Str,int CarSection);
void SendAndCacualte(uint8_t time,int CarSection);
void LedControl(void);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_UART_Receive_DMA(&huart1,(uint8_t*)Rx_Str,22);
	if(FormRight(Rx_Str))
	{
		//格式正确不发送消息
		//判断是进入停车场还是出去停车场
		CarSection = CarExist(Rx_Str);
		if(CarSection == 8)
		{
			//进入停车场创建停车信息
			CreateCar(Rx_Str);
		}
		else
		{
			//出去停车场计算费用信息 计算停车时间和费用
			SendAndCacualte(CaculateTatolFee(Rx_Str,CarSection),CarSection);
		}
		//对车位进行修改
	}
	else
	{
		//格式错误发送Error
		HAL_UART_Transmit_DMA(&huart1,"Error\r\n",11);
	}
	
	//更新屏幕显示
	if(Ds_Mode == 0) Dis_One_All( All_Set[0].Day, Vnber_Car_Num, 8-Vnber_Car_Num-Cnber_Car_Num);
	if(Ds_Mode == 1) Dis_Two_All( Cnber_Car_Rate, Vnber_Car_Rate);
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
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,0);
	//接受到多少个字符后进入中断
	HAL_UART_Receive_DMA(&huart1,Rx_Str,22);
	InitCar();//初始化车位状态
	CloseAllLed();
	LCD_Init();
	LCD_Clear(Black);
	LCD_SetTextColor(White);
	LCD_SetBackColor(Black);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(Ds_Mode != Ds_Mode_old )
	  {
		if(Ds_Mode == 0) Dis_One_All( Cnber_Car_Num, Vnber_Car_Num, 8-Vnber_Car_Num-Cnber_Car_Num);
		if(Ds_Mode == 1) Dis_Two_All( Cnber_Car_Rate, Vnber_Car_Rate);
		  
		Ds_Mode_old = Ds_Mode;
	  }
	  
	  //按键标志位进行按键判断
	  if(Key_Flag == 1)
	  {
		Key_Flag=0;//清空标志位
		if(Ds_Mode == 1) InterFaceOneKeyFunc();		//费率显示界面专用按键函数
		else InterFaceTwoKeyFunc();					//车位显示界面专用按键函数
	  }
	  
	  sprintf(temp2,"%s",All_Set[7].CarType);//可能有四字节对齐，显示接下来的内容
	  //sprintf(temp2,"%s%s***",All_Set[0].ID,All_Set[0].CarType);
	  sprintf(temp1,"%d %d %d %d %d %d %d",All_Set[7].Year,All_Set[7].Month,All_Set[7].Day,All_Set[7].hour,All_Set[7].min,All_Set[7].sec,CarSection);
	  LCD_DisplayStringLine(Line9,(uint8_t*)temp1);
	  LCD_DisplayStringLine(Line8,(uint8_t*)temp2);
	  LCD_DisplayStringLine(Line2,(uint8_t*)Rx_Str);
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

//定义界面1按键功能函数 界面1为车位显示界面
void InterFaceOneKeyFunc(void)
{
	//界面1按键功能执行
	switch(Key_Value)
	{
		case 1:
			if(++ Ds_Mode == 2) Ds_Mode = 0;
			break;
		case 2:
			//调整费率
			Cnber_Car_Rate += 0.5;
			Vnber_Car_Rate += 0.5;
			//更新显示
			Dis_Two_All( Cnber_Car_Rate, Vnber_Car_Rate);
			break;
		case 3:
			//调整费率
			if(Vnber_Car_Rate > 0)
			{
				Cnber_Car_Rate -= 0.5;
				Vnber_Car_Rate -= 0.5;
			}
			//更新显示
			Dis_Two_All( Cnber_Car_Rate, Vnber_Car_Rate);
			break;
		case 4:
			Pluse_Out = !Pluse_Out;
			if(Pluse_Out == 0) __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,0);
			else __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,100);
			break;
	}
}

//定义界面2按键功能函数 界面2为费率设置界面
void InterFaceTwoKeyFunc(void)
{
	switch(Key_Value)
	{
		case 1:
			if(++ Ds_Mode == 2) Ds_Mode = 0;
			break;
		case 2:
			//无功能
			break;
		case 3:
			//无功能
			break;
		case 4:
			Pluse_Out = !Pluse_Out;
			if(Pluse_Out == 0) __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,0);
			else __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,100);
			break;
	}
}

//检查接受字符串是否格式正确 返回1代表格式正确 返回0代表格式错误
int FormRight(uint8_t * Rx_Str)
{
	int i;
	int stream=0;//流程变量，每一次判断通过之后加一，达到指定的值输出1
	//分号判断
	if(stream == 0 && Rx_Str[9] == ':' && Rx_Str[4] == ':') stream++;
	//日期是数字判断
	if(stream == 1)
	{
		i=10;
		//判断从Rx_Str[10]到Rx_Str[21]是不是数字
		while(Rx_Str[i] >= '0' && Rx_Str[i] <= '9' && i<=21) i++;
		if(i==22) stream++;
		else return 0;
	}
	//停车类型只能是VNBR和CNBR判断
	if(Rx_Str[0] == 'V' || Rx_Str[0] == 'C')
	{
		if(Rx_Str[1] == 'N' && Rx_Str[2] == 'B' && Rx_Str[3] == 'R') stream++;
	}
	
	//所有格式判断通过
	if(stream==3) return 1;
	
	return 0;
}

//创建一个车位函数 代表这个车位被使用了
//返回1代表成功停车 0代表车位已经满了
int CreateCar(uint8_t* RxStr)
{
	int i;
	//查找空闲的车位
	for(i=0;i<8;i++)
	{
		if(All_Set[i].state == 0)//发现空闲车位
		{
			//存入空闲车位的信息
			StoreCar(RxStr,i);
			//存入完成 输出1
			return 1;
		}
	}
	return 0;
}

//存入车位信息
void StoreCar(uint8_t* RxStr,int i)
{
	//车的类型
	All_Set[i].CarType[0]=RxStr[0];
	All_Set[i].CarType[1]=RxStr[1];
	All_Set[i].CarType[2]=RxStr[2];
	All_Set[i].CarType[3]=RxStr[3];
	
	//车号
	All_Set[i].ID[0]=RxStr[5];
	All_Set[i].ID[1]=RxStr[6];
	All_Set[i].ID[2]=RxStr[7];
	All_Set[i].ID[3]=RxStr[8];
	
	//年份
	All_Set[i].Year=(RxStr[10]-48)*10+(RxStr[11]-48);
	
	//月份
	All_Set[i].Month=(RxStr[12]-48)*10+(RxStr[13]-48);
	
	//天
	All_Set[i].Day=(RxStr[14]-48)*10+(RxStr[15]-48);
	
	//时分秒
	All_Set[i].hour=(RxStr[16]-48)*10+(RxStr[17]-48);
	All_Set[i].min=(RxStr[18]-48)*10+(RxStr[19]-48);
	All_Set[i].sec=(RxStr[20]-48)*10+(RxStr[21]-48);
	
	//车位已经被占
	All_Set[i].state=1;
}

//停车场车位状态数据初始化
void InitCar()
{
	int i;
	for(i=0;i<8;i++)
	{
		//初始化所有车位都没有使用
		All_Set[i].state=0;
	}
}

//返回参数i 代表这个车已经存在第i个停车位停车场
//返回参数8 代表这个车不存在停车场
int CarExist(uint8_t* RxStr)
{
	int i;
	//从有车停的车位之中找到相同的车号
	//可以认为是出停车场
	for(i=0;i<8;i++)
	{
		if(All_Set[i].state == 1)//从已经停了车的位置找到和车号相同的车
		{
			if(RxStr[5]==All_Set[i].ID[0] && RxStr[6]==All_Set[i].ID[1] && RxStr[7]==All_Set[i].ID[2] && RxStr[8]==All_Set[i].ID[3])
			{
				//找到了相同的车号
				//判断是出停车场
				return i;
			}
		}
	}
	return 8;
}


//根据当前停车信息和之前的入场信息进行时间
//函数返回小时数
uint8_t CaculateTatolFee(uint8_t* Rx_Str,int CarSection)
{
	uint8_t temp=0;
	//将接受到的新时间存入到临时结构体
	StoreCar(Rx_Str,0);
	

	if(All_Set[CarSection].min < (Rx_Str[18]-48)*10+(Rx_Str[19]-48)) temp=1;//不足一小时取一小时
	else if(All_Set[CarSection].min == (Rx_Str[18]-48)*10+(Rx_Str[19]-48)) 
	{
		if(All_Set[CarSection].sec < (Rx_Str[20]-48)*10+(Rx_Str[21]-48)) temp=1;//不足一小时取一小时
	}
	
	temp += ((Rx_Str[16]-48)*10+(Rx_Str[17]-48) - All_Set[CarSection].hour);
	
	return temp;
}

//计算总停车费用和发送要求的字符串
void SendAndCacualte(uint8_t time,int CarSection)
{
	double temp_rate;
	if(All_Set[CarSection].CarType[0]=='V') temp_rate = Vnber_Car_Rate;
	else temp_rate = Cnber_Car_Rate;
	sprintf(Tx_Str,"%s:%s:%d:%.2f  \r\n",All_Set[CarSection].CarType,All_Set[CarSection].ID,time,time*temp_rate);
	HAL_UART_Transmit_DMA(&huart1,(uint8_t*)Tx_Str,strlen(Tx_Str));
}

void LedControl(void)
{
	int free,i;
	//查询是否有空闲车位
	for(i=0;i<8;i++)
	{
		if(All_Set[i].state==0)
		{
			free=1;
			break;
		}
		else free=0;
	}
	
	//根据free和Pluse_Out标志来选择led的亮灭
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 |
							GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15 ,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);
	if(free) HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8 ,GPIO_PIN_RESET);
	if(Pluse_Out) HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9 ,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);
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
