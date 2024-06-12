#define LEDa_ON() 		SET_BIT(GPIOA->ODR,GPIO_ODR_ODR2)
#define LEDa_OFF() 		CLEAR_BIT(GPIOA->ODR,GPIO_ODR_ODR2)
#define LEDb_ON() 		SET_BIT(GPIOA->ODR,GPIO_ODR_ODR3)
#define LEDb_OFF() 		CLEAR_BIT(GPIOA->ODR,GPIO_ODR_ODR3)
#define LEDc_ON() 		SET_BIT(GPIOA->ODR,GPIO_ODR_ODR4)
#define LEDc_OFF() 		CLEAR_BIT(GPIOA->ODR,GPIO_ODR_ODR4)
#define LEDd_ON() 		SET_BIT(GPIOA->ODR,GPIO_ODR_ODR5)
#define LEDd_OFF() 		CLEAR_BIT(GPIOA->ODR,GPIO_ODR_ODR5)
#define LEDe_ON() 		SET_BIT(GPIOA->ODR,GPIO_ODR_ODR6)
#define LEDe_OFF() 		CLEAR_BIT(GPIOA->ODR,GPIO_ODR_ODR6)
#define LEDf_ON() 		SET_BIT(GPIOA->ODR,GPIO_ODR_ODR7)
#define LEDf_OFF() 		CLEAR_BIT(GPIOA->ODR,GPIO_ODR_ODR7)
#define LEDg_ON() 		SET_BIT(GPIOB->ODR,GPIO_ODR_ODR0)
#define LEDg_OFF() 		CLEAR_BIT(GPIOB->ODR,GPIO_ODR_ODR0)
#define LEDdp_ON() 		SET_BIT(GPIOB->ODR,GPIO_ODR_ODR1)
#define LEDdp_OFF() 	CLEAR_BIT(GPIOB->ODR,GPIO_ODR_ODR1)
#define LEDD1_ON() 		SET_BIT(GPIOA->ODR,GPIO_ODR_ODR6)
#define LEDD1_OFF() 	CLEAR_BIT(GPIOA->ODR,GPIO_ODR_ODR6)
#define LEDD2_ON() 		SET_BIT(GPIOA->ODR,GPIO_ODR_ODR7)
#define LEDD2_OFF() 	CLEAR_BIT(GPIOA->ODR,GPIO_ODR_ODR7)
#define LEDD3_ON() 		SET_BIT(GPIOB->ODR,GPIO_ODR_ODR0)
#define LEDD3_OFF() 	CLEAR_BIT(GPIOB->ODR,GPIO_ODR_ODR0)
#define LEDD4_ON() 		SET_BIT(GPIOB->ODR,GPIO_ODR_ODR1)
#define LEDD4_OFF() 	CLEAR_BIT(GPIOB->ODR,GPIO_ODR_ODR1)
#define LEDl1l2_ON() 	SET_BIT(GPIOB->ODR,GPIO_ODR_ODR10)
#define LEDl1l2_OFF() 	CLEAR_BIT(GPIOB->ODR,GPIO_ODR_ODR10)
#define LEDalarm_ON() 	SET_BIT(GPIOB->ODR,GPIO_ODR_ODR11)
#define LEDalarm_OFF() 	CLEAR_BIT(GPIOB->ODR,GPIO_ODR_ODR11)

void writeSEG(int num, int seg){

	  switch(seg)
	  {
	    case 1:
	    LEDD1_ON();
		LEDD2_OFF();
		LEDD3_OFF();
		LEDD4_OFF();
	      	  break;
	    case 2:
	    	LEDD1_OFF();
	    	LEDD2_ON() ;
	    	LEDD2_OFF();
	    	LEDD3_OFF();
	    	LEDD4_OFF();
	      	  break;
	    case 3:
	    	LEDD1_OFF();
	    	LEDD2_OFF();
	    	LEDD3_OFF();
	    	LEDD4_ON();
	    	LEDD4_OFF();
	      	  break;
	    case 4:
	    	LEDD1_OFF();
	    	LEDD2_OFF();
	    	LEDD3_OFF();
	    	LEDD4_ON();
	    	LEDD4_OFF();
	      	  break;
	    default:
	    	  break;
	  }

	  switch(num)
	  {
	    case 0:
	      	LEDa_ON();
			LEDb_ON();
			LEDc_ON();
			LEDd_OFF();
			LEDe_ON();
			LEDf_ON();
			LEDg_OFF();
	      break;
	    case 1:
	      	LEDa_OFF();
			LEDb_ON();
			LEDc_ON();
			LEDd_OFF();
			LEDe_OFF();
			LEDf_OFF();
			LEDg_OFF();
			LEDdp_OFF();
	      break;
	    case 2:
	      	LEDa_ON();
			LEDb_ON();
			LEDc_OFF();
			LEDd_ON();
			LEDe_ON();
			LEDf_OFF();
			LEDg_ON();
			LEDdp_OFF();
	      break;
	    case 3:
	      	LEDa_ON();
			LEDb_ON();
			LEDc_ON();
			LEDd_ON();
			LEDd_OFF();
			LEDe_OFF();
			LEDf_OFF();
			LEDg_ON();
			LEDdp_OFF();
	      break;
	    case 4:      
			LEDa_OFF();
			LEDb_ON();
			LEDc_ON();
			LEDd_OFF();
			LEDe_OFF();
			LEDf_ON();
			LEDg_ON();
			LEDdp_OFF();
	      break;
	    case 5:
	      	LEDa_ON();
			LEDb_OFF();
			LEDc_ON();
			LEDd_ON();
			LEDe_OFF();
			LEDf_ON();
			LEDg_ON();
			LEDdp_OFF();
	      break;
	    case 6:
	      	LEDa_ON();
			LEDb_OFF();
			LEDc_ON();
			LEDd_ON();
			LEDe_ON();
			LEDf_ON();
			LEDg_ON();
			LEDdp_OFF();
	      break;
	    case 7:
	      	LEDa_ON();
			LEDb_ON();
			LEDc_ON();
			LEDd_OFF();
			LEDe_OFF();
			LEDf_OFF();
			LEDg_OFF();
			LEDdp_OFF();
	      break;
	    case 8:
	      	LEDa_ON();
			LEDb_ON();
			LEDc_ON();
			LEDd_ON();
			LEDe_ON();
			LEDf_ON();
			LEDg_ON();
			LEDdp_OFF();
	      	  break;
	    case 9:
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
	    LEDa_ON();
			LEDb_OFF();
			LEDc_OFF();
			LEDd_OFF();
			LEDe_OFF();
			LEDf_OFF();
			LEDg_OFF();
	    	LEDdp_ON();
	    	  break;
	  }

}