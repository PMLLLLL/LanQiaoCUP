#include "key.h"
#include "main.h"

enum Key_All_State
{
	Key_Check,
	Key_Press,
	Key_Release
}Key_State=Key_Check;

uint8_t Key_Flag=0;
uint16_t Key_Press_Time=20;
uint8_t Key_Value=0;

void KeyScan(void)
{
	switch(Key_State)
	{
		case Key_Check:
			if(B1 == RESET || B2 == RESET || B3 == RESET || B4 == RESET )
				Key_State = Key_Press;
			break;
		case Key_Press:
			Key_State =  Key_Release;
			if(B1 == RESET) Key_Value = 1;
			else if(B2 == RESET) Key_Value = 2;
			else if(B3 == RESET) Key_Value = 3;
			else if(B4 == RESET) Key_Value = 4;
			else Key_State = Key_Check;
			break;
		case Key_Release:
			if(B1 == RESET || B2 == RESET || B3 == RESET || B4 == RESET )
			{
				Key_Press_Time +=10;
			}
			else
			{
				Key_Flag = 1;
				Key_State = Key_Check;
			}
			break;
	}
}

