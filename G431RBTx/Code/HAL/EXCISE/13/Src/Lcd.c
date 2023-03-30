#include "lcd_hal.h"
#include "stdio.h"

void ShowPSD(uint8_t* Pass_Word,uint8_t* Pass_Look)
{
	char Lcd_Line3[20];
	char Lcd_Line4[20];
	char Lcd_Line5[20];
	LCD_DisplayStringLine(Line1, "       PSD          ");
	
	if(Pass_Look[0]==0) sprintf(Lcd_Line3,"    B1:@          ");
	else sprintf(Lcd_Line3,"    B1:%d     ",Pass_Word[0]);
	if(Pass_Look[1]==0) sprintf(Lcd_Line4,"    B2:@          ");
	else sprintf(Lcd_Line4,"    B2:%d     ",Pass_Word[1]);
	if(Pass_Look[2]==0) sprintf(Lcd_Line5,"    B3:@          ");
	else sprintf(Lcd_Line5,"    B3:%d     ",Pass_Word[2]);
	LCD_DisplayStringLine(Line3, (uint8_t*)Lcd_Line3);
	LCD_DisplayStringLine(Line4, (uint8_t*)Lcd_Line4);
	LCD_DisplayStringLine(Line5, (uint8_t*)Lcd_Line5);
}

void ShowSTA(uint16_t FREQ,uint8_t DUTY)
{
	char Lcd_Line3[20];
	char Lcd_Line4[20];
	LCD_DisplayStringLine(Line1, "       STA          ");
	sprintf(Lcd_Line3,"    F:%dHZ",FREQ);
	sprintf(Lcd_Line4,"    D:%d%%",DUTY);
	LCD_DisplayStringLine(Line3, (uint8_t*)Lcd_Line3);
	LCD_DisplayStringLine(Line4, (uint8_t*)Lcd_Line4);
	LCD_DisplayStringLine(Line5, "                    ");
}



