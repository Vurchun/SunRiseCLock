#ifndef _fp7103_H_
#define _fp7103_H_
#define pinEN_ON() 		LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_15)
#define pinEN_OFF() 	LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_15)

#include "math.h"
#include "menu.h"
#include <stdio.h>
#include <stdbool.h>

extern RTC_TimeTypeDef sTime;
extern RTC_DateTypeDef sDate;

void pwmFP7103() {
	if (menu[11].value && menu[9].value * 60 >= timeWakeUp - timeNow) {
		int timeWakeUp 	= menu[6].value 	* 3600
					+ menu[7].value	* 60;
		int timeNow 	= sTime.Hours 		* 3600
					+ sTime.Minutes		* 60
					+ sTime.Seconds;
			pinEN_ON();
			TIM_Cmd(TIM21, ENABLE);
			TIM21->CCR1 = (int16_t) (65535 * pow((1 - timeNow / timeWakeUp), 2.24));
	} else {
			pinEN_OFF();
			TIM_Cmd(TIM21, DISABLE);
			TIM21->CCR1 = 0;
		}
}

#endif
