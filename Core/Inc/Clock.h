#ifndef _Clock_H_
#define _Clock_H_

#include "fp7103.h"
#include "menu.h"
#include <stdio.h>
#include <stdbool.h>

extern  bool flagDecrementButton;      // Було натискання кнопки
extern  bool flagEnterButton;      // Було натискання кнопки
extern  bool flagIncrementButton;      // Було натискання кнопки
extern  bool flagDecrementButtonLong;  // Було довге утримання кнопки
extern  bool flagEnterButtonLong;  // Було довге утримання кнопки
extern  bool flagIncrementButtonLong;  // Було довге утримання кнопки


extern RTC_TimeTypeDef sTime;
extern RTC_DateTypeDef sDate;

int Clock(){
	char tmpClock[4]={};
	int j = 0;
	tmpClock[0] = sTime.Hours/10;
	if (sTime.Hours/10 == 0){j=1;}
	tmpClock[1] = sTime.Hours - tmpClock[0];
	tmpClock[2] = sTime.Minutes/10;
	tmpClock[3] = sTime.Minutes - tmpClock[0];
	if(sTime.Hours > 5 && sTime.Hours < 22 || flagDecrementButton || flagEnterButton || flagIncrementButton || flagDecrementButtonLong || flagEnterButtonLong || flagIncrementButtonLong)
	{
		for(int i = 0 + j; i<4;i++){
		 writeCHARSEG(tmpClock[i], i);
		 LL_mDelay(50);
	 }
	}
	return flagDecrementButtonLong&&flagIncrementButtonLong?0:1;
}

void setTimeNow(){
		sTime.Hours 	= menu[2].value;
		sTime.Minutes 	= menu[3].value;
		sTime.Seconds 	= 0;
	}
#endif
