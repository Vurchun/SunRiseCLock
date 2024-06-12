#include "stm32f10x.h"
#include "led.h"
//----------------------------------------------------------
#define SYSCLOCK 72000000U
//----------------------------------------------------------
#define TIM_EnableIT_UPDATE(TIMx) SET_BIT(TIMx->DIER, TIM_DIER_UIE)
#define TIM_EnableCounter(TIMx) SET_BIT(TIMx->CR1, TIM_CR1_CEN)
#define TIM_DisableCounter(TIMx) CLEAR_BIT(TIMx->CR1, TIM_CR1_CEN)
//----------------------------------------------------------
__IO uint32_t tmpreg;
__IO uint32_t SysTick_CNT = 0;
__IO uint8_t tim2_count = 0;
extern uint8_t R1,R2,R3,R4;
extern uint16_t num_gl;
//----------------------------------------------------------
__forceinline void delay(__IO uint32_t tck)
{
  while(tck)
  {
    tck--;
  }  
}
//----------------------------------------------------------
void delay_ms(uint32_t ms)
{
  MODIFY_REG(SysTick->VAL,SysTick_VAL_CURRENT_Msk,SYSCLOCK / 1000 - 1);
  SysTick_CNT = ms;
  while(SysTick_CNT) {}
}
//-----------------------------------------------------------
void RCC_DeInit(void)
{
  SET_BIT(RCC->CR, RCC_CR_HSION);
  while(READ_BIT(RCC->CR, RCC_CR_HSIRDY == RESET)) {}
  MODIFY_REG(RCC->CR, RCC_CR_HSITRIM, 0x80U);
  CLEAR_REG(RCC->CFGR);
  while (READ_BIT(RCC->CFGR, RCC_CFGR_SWS) != RESET) {}
  CLEAR_BIT(RCC->CR, RCC_CR_PLLON);
  while (READ_BIT(RCC->CR, RCC_CR_PLLRDY) != RESET) {}
  CLEAR_BIT(RCC->CR, RCC_CR_HSEON | RCC_CR_CSSON);
  while (READ_BIT(RCC->CR, RCC_CR_HSERDY) != RESET) {}
  CLEAR_BIT(RCC->CR, RCC_CR_HSEBYP);
  //Reset all CSR flags
  SET_BIT(RCC->CSR, RCC_CSR_RMVF);
}
//----------------------------------------------------------
void SetSysClockTo72(void)
{
  SET_BIT(RCC->CR, RCC_CR_HSEON);
  while(READ_BIT(RCC->CR, RCC_CR_HSERDY == RESET)) {}
  //Enable the Prefetch Buffer
  CLEAR_BIT(FLASH->ACR, FLASH_ACR_PRFTBE);
  SET_BIT(FLASH->ACR, FLASH_ACR_PRFTBE);
  MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, FLASH_ACR_LATENCY_2);
  MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, RCC_CFGR_HPRE_DIV1);
  MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, RCC_CFGR_PPRE2_DIV1);
  MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, RCC_CFGR_PPRE1_DIV2);
  MODIFY_REG(RCC->CFGR, RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL,
             RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL9);
  SET_BIT(RCC->CR, RCC_CR_PLLON);
  while(READ_BIT(RCC->CR, RCC_CR_PLLRDY) != (RCC_CR_PLLRDY)) {}
  MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_PLL);
  while(READ_BIT(RCC->CFGR, RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {}
}
//----------------------------------------------------------
void SysTick_Init(void)
{
  MODIFY_REG(SysTick->LOAD,SysTick_LOAD_RELOAD_Msk,SYSCLOCK / 1000 - 1);
  CLEAR_BIT(SysTick->VAL, SysTick_VAL_CURRENT_Msk);
  SET_BIT(SysTick->CTRL, SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk);
}
//----------------------------------------------------------
static void TIM2_Init(void)
{
  SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM2EN);
  NVIC_EnableIRQ(TIM2_IRQn);
  WRITE_REG(TIM2->PSC, 3599);
  WRITE_REG(TIM2->ARR, 100);
}
//----------------------------------------------------------
static void GPIO_Init(void)
{
  SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPAEN |RCC_APB2ENR_IOPBEN);
  MODIFY_REG(GPIOA->CRL, GPIO_CRL_CNF7_0 | GPIO_CRL_CNF6_0 | GPIO_CRL_CNF5_0 | GPIO_CRL_CNF4_0,\
             GPIO_CRL_MODE7_0 | GPIO_CRL_MODE6_0 | GPIO_CRL_MODE5_0 | GPIO_CRL_MODE4_0);
  MODIFY_REG(GPIOB->CRH, GPIO_CRH_CNF15_0 | GPIO_CRH_CNF14_0 | GPIO_CRH_CNF13_0 | GPIO_CRH_CNF12_0 | \
             GPIO_CRH_CNF11_0 | GPIO_CRH_CNF10_0 | GPIO_CRH_CNF9_0 | GPIO_CRH_CNF8_0,\
             GPIO_CRH_MODE15_0 | GPIO_CRH_MODE14_0 | GPIO_CRH_MODE13_0 | GPIO_CRH_MODE12_0 | \
             GPIO_CRH_MODE11_0 | GPIO_CRH_MODE10_0 | GPIO_CRH_MODE9_0 | GPIO_CRH_MODE8_0);
  SET_BIT(GPIOB->ODR, GPIO_ODR_ODR15 | GPIO_ODR_ODR14 | GPIO_ODR_ODR13 | GPIO_ODR_ODR12 |\
             GPIO_ODR_ODR11 | GPIO_ODR_ODR10 | GPIO_ODR_ODR9 | GPIO_ODR_ODR8);
}
//----------------------------------------------------------
int main(void)
{
  uint16_t i;
  RCC_DeInit();
  SET_BIT(RCC->APB2ENR, RCC_APB2ENR_AFIOEN);
  //Delay after an RCC peripheral clock enabling
  tmpreg = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_AFIOEN);
  //NOJTAG: JTAG-DP Disabled and SW-DP Enabled 
  CLEAR_BIT(AFIO->MAPR,AFIO_MAPR_SWJ_CFG);
  SET_BIT(AFIO->MAPR, AFIO_MAPR_SWJ_CFG_JTAGDISABLE);
  SetSysClockTo72();
  SysTick_Init();
  TIM2_Init();
  GPIO_Init();
  TIM_EnableIT_UPDATE(TIM2);
  TIM_EnableCounter(TIM2);
  while(1)
	{
    for(i=0;i<10000;i++)
    {
      ledprint(i);
      delay_ms(100);
    }
  }
}
//----------------------------------------------------------
void SysTick_Handler(void)
{
  if(SysTick_CNT > 0)  SysTick_CNT--;
}
//----------------------------------------------------------
void TIM2_IRQHandler(void)
{
  if(READ_BIT(TIM2->SR, TIM_SR_UIF))
  {
    CLEAR_BIT(TIM2->SR, TIM_SR_UIF);
    if(tim2_count==0)
    {
      MODIFY_REG(GPIOA->ODR, GPIO_ODR_ODR5 | GPIO_ODR_ODR6 | GPIO_ODR_ODR7, GPIO_ODR_ODR4);
      segchar(R1);
    }
    if(tim2_count==1)
    {
      MODIFY_REG(GPIOA->ODR, GPIO_ODR_ODR4 | GPIO_ODR_ODR6 | GPIO_ODR_ODR7, GPIO_ODR_ODR5);
      segchar(R2);
      if(num_gl<10)
        SET_BIT(GPIOB->ODR, GPIO_ODR_ODR15 | GPIO_ODR_ODR14 | GPIO_ODR_ODR13 | GPIO_ODR_ODR12 |\
                GPIO_ODR_ODR11 | GPIO_ODR_ODR10 | GPIO_ODR_ODR9 | GPIO_ODR_ODR8);
    }
    if(tim2_count==2)
    {
      MODIFY_REG(GPIOA->ODR, GPIO_ODR_ODR4 | GPIO_ODR_ODR5 | GPIO_ODR_ODR7, GPIO_ODR_ODR6);
      segchar(R3);
      if(num_gl<100)
        SET_BIT(GPIOB->ODR, GPIO_ODR_ODR15 | GPIO_ODR_ODR14 | GPIO_ODR_ODR13 | GPIO_ODR_ODR12 |\
                GPIO_ODR_ODR11 | GPIO_ODR_ODR10 | GPIO_ODR_ODR9 | GPIO_ODR_ODR8);
    }
    if(tim2_count==3)
    {
      MODIFY_REG(GPIOA->ODR, GPIO_ODR_ODR4 | GPIO_ODR_ODR5 | GPIO_ODR_ODR6, GPIO_ODR_ODR7);
      segchar(R4);
      if(num_gl<1000)
        SET_BIT(GPIOB->ODR, GPIO_ODR_ODR15 | GPIO_ODR_ODR14 | GPIO_ODR_ODR13 | GPIO_ODR_ODR12 |\
                GPIO_ODR_ODR11 | GPIO_ODR_ODR10 | GPIO_ODR_ODR9 | GPIO_ODR_ODR8);
    }
    tim2_count++;
    if(tim2_count>3) tim2_count=0;
  }
}
//----------------------------------------------------------
