#include "led.h"
uint8_t R1=0,R2=0,R3=0,R4=0;
uint16_t num_gl=0;
//==============================
void segchar (uint8_t seg)
{
  uint8_t r;
  switch(seg)
  {
    case 0:
      r = 0xC0;//1100 0000
      break;
    case 1:
      r = 0xF9;//1111 1001
      break;
    case 2:
      r = 0xA4;//1010 0100
      break;
    case 3:
      r = 0xB0;//1011 0000
      break;
    case 4:
      r = 0x99;//1001 1001
      break;
    case 5:
      r = 0x92;//1001 0010
      break;
    case 6:
      r = 0x82;//1000 0010
      break;
    case 7:
      r = 0xF8;//1111 1000
      break;
    case 8:
      r = 0x80;//1000 0000
      break;
    case 9:
      r = 0x90;//1001 0000
      break;
    case 10:
      r = 0xFF;//1111 1111
      break;
  }
  MODIFY_REG(GPIOB->ODR, ~(r<<8), r<<8);
}
//==============================
void ledprint(uint16_t number)
{
  num_gl=number;
	R1 = number%10;
	R2 = number%100/10;
	R3 = number%1000/100;
	R4 = number/1000;
}
