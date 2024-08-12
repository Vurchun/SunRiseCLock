#ifndef _4digit7segment_H_
#define _4digit7segment_H_
#define LEDa_ON() 		LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_7)
#define LEDa_OFF() 		LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_7)
#define LEDb_ON() 		LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_1)
#define LEDb_OFF() 		LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_1)
#define LEDc_ON() 		LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_6)
#define LEDc_OFF() 		LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_6)
#define LEDd_ON() 		LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_5)
#define LEDd_OFF() 		LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_5)
#define LEDe_ON() 		LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_11)
#define LEDe_OFF() 		LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_11)
#define LEDf_ON() 		LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_9)
#define LEDf_OFF() 		LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_9)
#define LEDg_ON() 		LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_0)
#define LEDg_OFF() 		LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_0)
#define LEDdp_ON() 		LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_3)
#define LEDdp_OFF() 	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_3)

#define LEDD1_ON() 		LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_3)
#define LEDD1_OFF() 	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_3)
#define LEDD2_ON() 		LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4)
#define LEDD2_OFF() 	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_4)
#define LEDD3_ON() 		LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_12)
#define LEDD3_OFF() 	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_12)
#define LEDD4_ON() 		LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_4)
#define LEDD4_OFF() 	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_4)

#define LEDl1l2_ON() 	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_10)
#define LEDl1l2_OFF() 	LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_10)
#define LEDalarm_ON() 	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_8)
#define LEDalarm_OFF() 	LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_8)

#define pinEN_ON() 		LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_15)
#define pinEN_OFF() 	LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_15)


void writeCHARSEG(char CHAR, int seg){
	switch (seg) {
		case 0:
			LEDD1_ON();
			LEDD2_OFF();
			LEDD3_OFF();
			LEDD4_OFF();
			break;
		case 1:
			LEDD1_OFF();
			LEDD2_ON();
			LEDD3_OFF();
			LEDD4_OFF();
			break;
		case 2:
			LEDD1_OFF();
			LEDD2_OFF();
			LEDD3_ON();
			LEDD4_OFF();
			break;
		case 3:
			LEDD1_OFF();
			LEDD2_OFF();
			LEDD3_OFF();
			LEDD4_ON();
			break;
		default:
			LEDD1_OFF();
			LEDD2_OFF();
			LEDD3_OFF();
			LEDD4_OFF();
			break;
		}
	switch (CHAR) {
		case 'a':
			LEDa_ON();
			LEDb_ON();
			LEDc_ON();
			LEDd_ON();
			LEDe_ON();
			LEDf_ON();
			LEDg_OFF();
			LEDdp_OFF();
			break;
		case 'b':
			break;
		case 'c':
			break;
		case 'd':
			break;
		case 'e':
			break;
		case 'f':
			break;
		case 'g':
			break;
		case 'h':
			break;
		case 'i':
			break;
		case 'j':
			break;
		case 'k':
			break;
		case 'l':
			break;
		case 'm':
			break;
		case 'n':
			break;
		case 'o':
			break;
		case 'p':
			break;
		case 'q':
			break;
		case 'r':
			break;
		case 's':
			break;
		case 't':
			break;
		case 'u':
			break;
		case 'v':
			break;
		case 'w':
			break;
		case 'x':
			break;
		case 'y':
			break;
		case 'z':
			break;
		case '_':
			LEDa_OFF();
			LEDb_OFF();
			LEDc_ON();
			LEDd_OFF();
			LEDe_OFF();
			LEDf_OFF();
			LEDg_OFF();
			LEDdp_OFF();
			break;
		case '.':
			LEDa_OFF();
			LEDb_OFF();
			LEDc_OFF();
			LEDd_OFF();
			LEDe_OFF();
			LEDf_OFF();
			LEDg_OFF();
			LEDdp_ON();
			break;


		case '0':
				LEDa_ON();
				LEDb_ON();
				LEDc_ON();
				LEDd_ON();
				LEDe_ON();
				LEDf_ON();
				LEDg_OFF();
				LEDdp_OFF();
				break;
			case '1':
				LEDa_OFF();
				LEDb_ON();
				LEDc_ON();
				LEDd_OFF();
				LEDe_OFF();
				LEDf_OFF();
				LEDg_OFF();
				LEDdp_OFF();
				break;
			case '2':
				LEDa_ON();
				LEDb_ON();
				LEDc_OFF();
				LEDd_ON();
				LEDe_ON();
				LEDf_OFF();
				LEDg_ON();
				LEDdp_OFF();
				break;
			case '3':
				LEDa_ON();
				LEDb_ON();
				LEDc_ON();
				LEDd_ON();
				LEDe_OFF();
				LEDf_OFF();
				LEDg_ON();
				LEDdp_OFF();
				break;
			case '4':
				LEDa_OFF();
				LEDb_ON();
				LEDc_ON();
				LEDd_OFF();
				LEDe_OFF();
				LEDf_ON();
				LEDg_ON();
				LEDdp_OFF();
				break;
			case '5':
				LEDa_ON();
				LEDb_OFF();
				LEDc_ON();
				LEDd_ON();
				LEDe_OFF();
				LEDf_ON();
				LEDg_ON();
				LEDdp_OFF();
				break;
			case '6':
				LEDa_ON();
				LEDb_OFF();
				LEDc_ON();
				LEDd_ON();
				LEDe_ON();
				LEDf_ON();
				LEDg_ON();
				LEDdp_OFF();
				break;
			case '7':
				LEDa_ON();
				LEDb_ON();
				LEDc_ON();
				LEDd_OFF();
				LEDe_OFF();
				LEDf_OFF();
				LEDg_OFF();
				LEDdp_OFF();
				break;
			case '8':
				LEDa_ON();
				LEDb_ON();
				LEDc_ON();
				LEDd_ON();
				LEDe_ON();
				LEDf_ON();
				LEDg_ON();
				LEDdp_OFF();
				break;
			case '9':
				LEDa_ON();
				LEDb_ON();
				LEDc_ON();
				LEDd_ON();
				LEDe_OFF();
				LEDf_ON();
				LEDg_ON();
				LEDdp_OFF();
				break;
			default:
				LEDa_OFF();
				LEDb_OFF();
				LEDc_OFF();
				LEDd_OFF();
				LEDe_OFF();
				LEDf_OFF();
				LEDg_OFF();
				LEDdp_OFF();
				break;
	}
}
#endif
