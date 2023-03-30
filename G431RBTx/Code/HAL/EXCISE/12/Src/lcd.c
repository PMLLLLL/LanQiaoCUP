#include "lcd.h"
#include "lcd_hal.h"
#include "stdio.h"
#include "string.h"

char Lcd_Line0[20];
char Lcd_Line1[20];
char Lcd_Line2[20];
char Lcd_Line3[20];
char Lcd_Line4[20];
char Lcd_Line5[20];
char Lcd_Line6[20];
char Lcd_Line7[20];
char Lcd_Line8[20];
char Lcd_Line9[20];

void Edit_Dis_One(uint16_t Cnber_Car_Num,uint16_t Vnber_Car_Num,uint16_t Rest_Car_Num)
{
	sprintf(Lcd_Line1,"       Data         ");
	sprintf(Lcd_Line3,"   CNBR:%d          ",Cnber_Car_Num);
	sprintf(Lcd_Line5,"   VNBR:%d          ",Vnber_Car_Num);
	sprintf(Lcd_Line7,"   IDLE:%d          ",Rest_Car_Num);
}

void Edit_Dis_Two(float Cnber_Car_Rate,float Vnber_Car_Rate)
{
	sprintf(Lcd_Line1,"       Para         ");
	sprintf(Lcd_Line3,"   CNBR:%.2f        ",Cnber_Car_Rate);
	sprintf(Lcd_Line5,"   VNBR:%.2f        ",Vnber_Car_Rate);
	sprintf(Lcd_Line7,"                    ");
}

void Dis_Updata(void)
{
	LCD_DisplayStringLine(Line0,(uint8_t*)Lcd_Line0);
	LCD_DisplayStringLine(Line1,(uint8_t*)Lcd_Line1);
	LCD_DisplayStringLine(Line2,(uint8_t*)Lcd_Line2);
	LCD_DisplayStringLine(Line3,(uint8_t*)Lcd_Line3);
	LCD_DisplayStringLine(Line4,(uint8_t*)Lcd_Line4);
	LCD_DisplayStringLine(Line5,(uint8_t*)Lcd_Line5);
	LCD_DisplayStringLine(Line6,(uint8_t*)Lcd_Line6);
	LCD_DisplayStringLine(Line7,(uint8_t*)Lcd_Line7);
	LCD_DisplayStringLine(Line8,(uint8_t*)Lcd_Line8);
	LCD_DisplayStringLine(Line9,(uint8_t*)Lcd_Line9);
}

void Dis_One_All(int16_t Cnber_Car_Num,uint16_t Vnber_Car_Num,uint16_t Rest_Car_Num)
{
	Edit_Dis_One( Cnber_Car_Num, Vnber_Car_Num, Rest_Car_Num);
	Dis_Updata();
}

void Dis_Two_All(float Cnber_Car_Rate,float Vnber_Car_Rate)
{
	Edit_Dis_Two( Cnber_Car_Rate, Vnber_Car_Rate);
	Dis_Updata();
}

