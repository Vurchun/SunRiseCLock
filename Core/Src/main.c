/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include <stdio.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SYSCLOCK 32000000U

#define LEDa_ON() 		GPIOA->BSRR = GPIO_BSRR_BS_7
#define LEDa_OFF() 		GPIOA->BSRR = GPIO_BSRR_BR_7
#define LEDb_ON() 		GPIOB->BSRR = GPIO_BSRR_BS_1
#define LEDb_OFF() 		GPIOB->BSRR = GPIO_BSRR_BR_1
#define LEDc_ON() 		GPIOA->BSRR = GPIO_BSRR_BS_6
#define LEDc_OFF() 		GPIOA->BSRR = GPIO_BSRR_BR_6
#define LEDd_ON() 		GPIOA->BSRR = GPIO_BSRR_BS_5
#define LEDd_OFF() 		GPIOA->BSRR = GPIO_BSRR_BR_5
#define LEDe_ON() 		GPIOA->BSRR = GPIO_BSRR_BS_11
#define LEDe_OFF() 		GPIOA->BSRR = GPIO_BSRR_BR_11
#define LEDf_ON() 		GPIOA->BSRR = GPIO_BSRR_BS_9
#define LEDf_OFF() 		GPIOA->BSRR = GPIO_BSRR_BR_9
#define LEDg_ON() 		GPIOB->BSRR = GPIO_BSRR_BS_0
#define LEDg_OFF() 		GPIOB->BSRR = GPIO_BSRR_BR_0
#define LEDdp_ON() 		GPIOB->BSRR = GPIO_BSRR_BS_3
#define LEDdp_OFF() 	GPIOB->BSRR = GPIO_BSRR_BR_3

#define LEDD1_ON() 		GPIOA->BSRR = GPIO_BSRR_BR_3
#define LEDD1_OFF() 	GPIOA->BSRR = GPIO_BSRR_BS_3
#define LEDD2_ON() 		GPIOA->BSRR = GPIO_BSRR_BR_4
#define LEDD2_OFF() 	GPIOA->BSRR = GPIO_BSRR_BS_4
#define LEDD3_ON() 		GPIOA->BSRR = GPIO_BSRR_BR_12
#define LEDD3_OFF() 	GPIOA->BSRR = GPIO_BSRR_BS_12
#define LEDD4_ON() 		GPIOB->BSRR = GPIO_BSRR_BR_4
#define LEDD4_OFF() 	GPIOB->BSRR = GPIO_BSRR_BS_4

#define LEDl1l2_ON() 	GPIOA->BSRR = GPIO_BSRR_BS_10
#define LEDl1l2_OFF() 	GPIOA->BSRR = GPIO_BSRR_BR_10
#define LEDalarm_ON() 	GPIOA->BSRR = GPIO_BSRR_BS_8
#define LEDalarm_OFF() 	GPIOA->BSRR = GPIO_BSRR_BR_8

#define pinEN_ON() 		GPIOC->BSRR = GPIO_BSRR_BS_15
#define pinEN_OFF() 	GPIOC->BSRR = GPIO_BSRR_BR_15

#define TIM_EnableCounter(TIMx) SET_BIT(TIMx->CR1, TIM_CR1_CEN)
#define TIM_DisableCounter(TIMx) CLEAR_BIT(TIMx->CR1, TIM_CR1_CEN)
#define TIM_EnableIT_UPDATE(TIMx) SET_BIT(TIMx->DIER, TIM_DIER_UIE)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
bool flagDecrementButton;      // Було натискання кнопки
bool flagEnterButton;      // Було натискання кнопки
bool flagIncrementButton;      // Було натискання кнопки
bool flagDecrementButtonLong;  // Було довге утримання кнопки
bool flagEnterButtonLong;  // Було довге утримання кнопки
bool flagIncrementButtonLong;  // Було довге утримання кнопки

int menuArraySize = 18;        	// Встановлюємо розмір масиву
int actualIndex = 0;
bool isParamEditMode = false;	// Прапорець режиму редагування параметра
int tmpVValue = 0;       		// Тимчасова змінна для зберігання змінюваного параметра



struct strMenu {
			int id;         // Унікальний ідентифікаційний індекс ID
			int parentid;   // ID батька
			bool isParam;   // Чи є пункт змінним параметром
			char _name[4];  // Назва
			int value;      // Поточне значення
			int _min;       // Мінімально можливе значення параметра
			int _max;       // Максимально можливе значення параметра
};
/* PPPP
 * P__0			Time_Now
 * 		P_0.0	Hour_Now
 * 		P_0.1	Minute_Now
 * 		P_0.2	Set
 * P__1			Time_Rise
 * 		P_1.0	Hour_Rise
 * 		P_1.1	Minute_Rise
 * P__2			Rising_Parametrs
 * 		P_2.0	Period_Rising
 * P__3			Alarm_Parametrs
 * 		P_3.0	Alarm_Status
 * 		P_3.1	Alarm_Melody
 * 		P_3.2	Alarm_Melody_test
 * P__4			Menu_Parametrs
 * 		P_4.0	Numbers_Change_Style
 * 		P_4.1	Menu_Night_Mode
 * P__5			Clock
 */
struct strMenu menu[] = {                         // Встановлюємо пункти меню
	  {0, -1,    false,	"PPPP",		0, 0, 	0},
	  //-----------------------------------------------------------------------
	  {1, 0,     false,	"P__0",		0, 0, 	0},
	  {2, 1,     true,	"P_00",	0, 2, 	0},
	  {3, 2,     true,	"P_01", 	0, 59, 	0},
	  {4, 1,     false, "P_02", 	0, 0, 	0},
	  //-----------------------------------------------------------------------
	  {5, 0,     false, "P__1", 	0, 0, 	0},
	  {6, 5,     true,	"P_10", 	0, 23, 	0},
	  {7, 5,     true,	"P_11", 	0, 59, 	0},
	  //-----------------------------------------------------------------------
	  {8, 0,   	 false, "P__2", 	0, 0, 	0},
	  {9, 8,     true,	"P_20", 	0, 255, 64},
	  //-----------------------------------------------------------------------
	  {10, 0,    false, "P__3", 	0, 0, 	0},
	  {11, 10,   true,	"P_30", 	0, 1, 	1},
	  {12, 10,   true,	"P_31", 	0, 8, 	0},
	  {13, 10,   false,	"P_32", 	0, 0, 	0},
	  //-----------------------------------------------------------------------
	  {14, 0,    false, "P__4", 	0, 0, 	1},
	  {15, 14,   true,	"P_40", 	0, 3, 	0},
	  {16, 14,   true,	"P_41", 	0, 1, 	1},
	  {17, 0,    false, "P__5", 	0, 0, 	0}
	  //-----------------------------------------------------------------------
};

int MusicStep = 0;
char PlayMusic = 0;
int sound_time;
int sound_counter;

#define C	261	//Do
#define C_	277 //Do#
#define D	293 //Re
#define D_	311 //Re#
#define E	239 //Mi
#define F	349 //Fa
#define F_	370 //Fa#
#define G 	392 //Sol
#define G_	415 //Sol#
#define A	440 //La
#define A_	466 //La#
#define H	494 //Si

#define t1		2000
#define t2		1000
#define t4		500
#define t8		250
#define t16		125

typedef struct
{
	uint16_t freq;
	uint16_t time;
}SoundTypeDef;

const SoundTypeDef Music[48] ={
	{C*2, t4},
	{G, t4},
	{A_, t8},
	{F, t8},
	{D_, t8},
	{F, t8},
	{G, t4},
	{C, t2},
	{C*2, t4},
	{G, t4},
	{A_, t8},
	{F, t8},
	{D_, t8},
	{F, t8},
	{G, t4},
	{C*2, t4},
	{0, t8},
	{D_, t8},
	{D_, t8},
	{D_, t8},
	{G, t8},
	{A_, t4},
	{D_*2, t8},
	{C_*2, t8},
	{C*2, t8},
	{C*2, t8},
	{C*2, t8},
	{C*2, t8},
	{A_, t8},
	{F, t8},
	{D_, t8},
	{F, t8},
	{G, t4},
	{C*2, t2},
	{C*2, t2},
	{A_, t8},
	{G_, t8},
	{G, t8},
	{G_, t8},
	{A_, t2},
	{A_, t4},
	{C*2, t4},
	{A_, t8},
	{F, t8},
	{D_, t8},
	{F, t8},
	{G, t4},
	{C*2, t2}
};


__IO uint32_t tmpreg;
__IO uint32_t SysTick_CNT = 0;
__IO uint8_t lptim_count = 0;
__IO uint8_t butIncrement_count = 0;
__IO uint8_t butIncrement_lock = 0;
__IO uint8_t butEnter_count = 0;
__IO uint8_t butEnter_lock = 0;
__IO uint8_t butDecrement_count = 0;
__IO uint8_t butDecrement_lock = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void deinitRCC(void);
void initRCC(void);
void initGPIO(void);
void initRTC(void);
static void initTIM2(void);
static void initTIM21(void);
static void initLPTIM1(void);
void initUSART2_UART(void);

void SysTick_Handler (void);
void EXTI0_1_IRQHandler (void);
void EXTI2_3_IRQHandler (void);
void EXTI4_15_IRQHandler (void);
void LPTIM1_IRQHandler (void);
void TIM21_IRQHandler (void);
void LPUART1_IRQHandler (void);

void pwmFP7103();
void setTimeNow();
int Clock();
void writeCHARSEG(char CHAR, int seg);
char* setActualMenu(int v, int h);
int getMenuIndexByID(int id);
int getNearMenuIndexByID(int parentid, int id, int side);
void StartMusic(int melody);
void sound (int freq, int time_ms);

void delay_ms(uint32_t ms);
/* USER CODE END PFP */

int main(void){
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* USER CODE BEGIN Init */
	initRCC();
	initGPIO();
  /* USER CODE END Init */
  /* Configure the system clock */


  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	int vmenu = 0; // Змінна, що зберігає дію по вертикалі 1 - вхід в меню, -1 - вихід з меню
	int hmenu = 0; // Змінна, що зберігає дію по горизонталі 1 - вправо, -1 - вліво
	char* tmpValue;

	while(1){

LEDdp_ON();

LEDD1_OFF();
	}
	 if (flagDecrementButton){
	  hmenu = 1;// Якщо при спаді лінії A на лінії B лог. одиниця, то обертання в один бік
	  flagDecrementButton = false;       // Действие обработано - сбрасываем флаг
	 }
	  					  // Прапорець обертання за годинниковою стрілкою
	 else if (flagIncrementButton){
	  hmenu = -1;	// Якщо при спаді лінії A на лінії B лог. нуль, то обертання в інший бік
	  flagIncrementButton = false;       // Действие обработано - сбрасываем флаг
	 }
	  						// Прапорець обертання проти годинникової стрілки
	 if (flagEnterButton) {           // Кнопка нажата
	  vmenu = 1;                // По нажатию кнопки - переходим на уровень вниз
	  flagEnterButton = false;       // Действие обработано - сбрасываем флаг
	 }
	 else if (flagEnterButtonLong){
	  vmenu = -1;
	  flagEnterButtonLong = false;               // Действие обработано - сбрасываем флаг
	 }
	 if (vmenu != 0 || hmenu != 0) tmpValue = setActualMenu(vmenu, hmenu); // Если было действие - реагируем на него
	 for(int i = 0; i<4;i++){
		 writeCHARSEG(tmpValue[i], i);
	 }
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */
//void deinitRCC(void){
////	Включим для начала HSI (внутренний генератор 8 МГц)
//	 SET_BIT(RCC->CR, RCC_CR_HSION);
////	 Дождёмся его стабилизации
//	 while(READ_BIT(RCC->CR, RCC_CR_HSIRDY == RESET)) {}
////	 Сбросим калибровку
////	 Полностью очистим конфигурационный регистр
////	 MODIFY_REG(RCC->CR, RCC_CR_HSITRIM, 0x80U);
//
////	 Дождёмся очистку бита SWS
////	 System clock switch status
////	 These bits are set and cleared by hardware to indicate which clock source is used as system clock.
//	 CLEAR_REG(RCC->CFGR);
//	 while (READ_BIT(RCC->CFGR, RCC_CFGR_SWS) != RESET) {}
////	 Аналогичным образом отключим PLL
//	 CLEAR_BIT(RCC->CR, RCC_CR_PLLON);
//	 while (READ_BIT(RCC->CR, RCC_CR_PLLRDY) != RESET) {}
////	 Выключим HSE и его детектор тактового сигнала, дождавшись затем отключения HSE
//	 CLEAR_BIT(RCC->CR, RCC_CR_HSEON);
//	 while (READ_BIT(RCC->CR, RCC_CR_HSERDY) != RESET) {}
////	 Сбросим бит, разрешающий использование внешнего генератора
//	 CLEAR_BIT(RCC->CR, RCC_CR_HSEBYP);
////	 Сбросим флаги всех прерываний от RCC
//	  CLEAR_BIT(RCC->CR, RCC_CR_HSEBYP);
//	  SET_BIT(RCC->CSR, RCC_CSR_RMVF);
////	 Также запретим все прерывания от RCC
////	   CLEAR_REG(RCC->CIR);
//}
void initRCC(void){
//	Включим наш HSE, дождавшись его стабилизации
	SET_BIT(RCC->CR, RCC_CR_HSEON);
	while(READ_BIT(RCC->CR, RCC_CR_HSERDY == RESET)) {}
//	Настроим значения всех делителей
	MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, RCC_CFGR_HPRE_DIV1);
	MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, RCC_CFGR_PPRE2_DIV1);
	MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, RCC_CFGR_PPRE1_DIV1);
//	Настроим PLL на коэффициент 9 и настроим его вход для работы от HSE
	MODIFY_REG(RCC->CFGR, RCC_CFGR_MCOSEL, RCC_CFGR_MCOSEL_HSI);
	MODIFY_REG(RCC->CFGR, RCC_CFGR_PLLDIV, RCC_CFGR_PLLDIV2);
//	Разрешим работу PLL, дождавшись затем его разблокировку
	SET_BIT(RCC->CR, RCC_CR_PLLON);
	while(READ_BIT(RCC->CR, RCC_CR_PLLRDY) != (RCC_CR_PLLRDY)) {}
//	Выберем PLL в качестве источника системного тактирования, дождавшись затем применения данного действия
	MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_PLL);
	while(READ_BIT(RCC->CFGR, RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {}

	// 1. Увімкнути тактуючий сигнал для PWR (для доступу до резервного домену)
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	// 2. Дозволити доступ до резервного домену
	PWR->CR |= PWR_CR_DBP;
	// 3. Увімкнути LSE (Low-Speed External) генератор
	RCC->CSR |= RCC_CSR_LSEON;
	// 4. Дочекатися готовності LSE
	while(!(RCC->CSR & RCC_CSR_LSERDY));
	// 5. Вибрати LSE як джерело тактуючого сигналу для RTC
	RCC->CSR |= RCC_CSR_RTCSEL_LSE;
	// 6. Увімкнути RTC
	RCC->CSR |= RCC_CSR_RTCEN;
}

void initGPIO(void){
	// сключаем тактирование порта A
	RCC->IOPENR = RCC_IOPENR_IOPAEN;
	// сключаем тактирование порта B
	RCC->IOPENR = RCC_IOPENR_IOPBEN;
	// сключаем тактирование порта C
	RCC->IOPENR = RCC_IOPENR_IOPCEN;

	// LEDa (PA7)
	MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODE7_Msk, 0b01 << GPIO_MODER_MODE7_Pos);
	GPIOA->OTYPER |= GPIO_OTYPER_OT_7;
	MODIFY_REG(GPIOA->OSPEEDR, GPIO_OSPEEDER_OSPEED7_Msk, 0b11 << GPIO_OSPEEDER_OSPEED7_Pos);

	// LEDb (PB1)
	MODIFY_REG(GPIOB->MODER, GPIO_MODER_MODE1_Msk, 0b01 << GPIO_MODER_MODE1_Pos);
	GPIOB->OTYPER |= GPIO_OTYPER_OT_1;
	MODIFY_REG(GPIOB->OSPEEDR, GPIO_OSPEEDER_OSPEED1_Msk, 0b11 << GPIO_OSPEEDER_OSPEED1_Pos);

	// LEDc (PA6)
	MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODE6_Msk, 0b01 << GPIO_MODER_MODE6_Pos);
	GPIOA->OTYPER |= GPIO_OTYPER_OT_6;
	MODIFY_REG(GPIOA->OSPEEDR, GPIO_OSPEEDER_OSPEED6_Msk, 0b11 << GPIO_OSPEEDER_OSPEED6_Pos);

	// LEDd (PA5)
	MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODE5_Msk, 0b01 << GPIO_MODER_MODE5_Pos);
	GPIOA->OTYPER |= GPIO_OTYPER_OT_5;
	MODIFY_REG(GPIOA->OSPEEDR, GPIO_OSPEEDER_OSPEED5_Msk, 0b11 << GPIO_OSPEEDER_OSPEED5_Pos);

	// LEDe (PA11)
	MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODE11_Msk, 0b01 << GPIO_MODER_MODE11_Pos);
	GPIOA->OTYPER |= GPIO_OTYPER_OT_11;
	MODIFY_REG(GPIOA->OSPEEDR, GPIO_OSPEEDER_OSPEED11_Msk, 0b11 << GPIO_OSPEEDER_OSPEED11_Pos);

	// LEDf (PA9)
	MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODE9_Msk, 0b01 << GPIO_MODER_MODE9_Pos);
	GPIOA->OTYPER |= GPIO_OTYPER_OT_9;
	MODIFY_REG(GPIOA->OSPEEDR, GPIO_OSPEEDER_OSPEED9_Msk, 0b11 << GPIO_OSPEEDER_OSPEED9_Pos);

	// LEDg (PB0)
	MODIFY_REG(GPIOB->MODER, GPIO_MODER_MODE0_Msk, 0b01 << GPIO_MODER_MODE0_Pos);
	GPIOB->OTYPER |= GPIO_OTYPER_OT_0;
	MODIFY_REG(GPIOB->OSPEEDR, GPIO_OSPEEDER_OSPEED0_Msk, 0b11 << GPIO_OSPEEDER_OSPEED0_Pos);

	// LEDdp (PB3)
	MODIFY_REG(GPIOB->MODER, GPIO_MODER_MODE3_Msk, 0b01 << GPIO_MODER_MODE3_Pos);
	GPIOB->OTYPER |= GPIO_OTYPER_OT_3;
	MODIFY_REG(GPIOB->OSPEEDR, GPIO_OSPEEDER_OSPEED3_Msk, 0b11 << GPIO_OSPEEDER_OSPEED3_Pos);

	// LEDD1 (PA3)
	MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODE3_Msk, 0b01 << GPIO_MODER_MODE3_Pos);
	GPIOA->OTYPER |= GPIO_OTYPER_OT_3;
	MODIFY_REG(GPIOA->OSPEEDR, GPIO_OSPEEDER_OSPEED3_Msk, 0b11 << GPIO_OSPEEDER_OSPEED3_Pos);

	// LEDD2 (PA4)
	MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODE4_Msk, 0b01 << GPIO_MODER_MODE4_Pos);
	GPIOA->OTYPER |= GPIO_OTYPER_OT_4;
	MODIFY_REG(GPIOA->OSPEEDR, GPIO_OSPEEDER_OSPEED4_Msk, 0b11 << GPIO_OSPEEDER_OSPEED4_Pos);

	// LEDD3 (PA12)
	MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODE12_Msk, 0b01 << GPIO_MODER_MODE12_Pos);
	GPIOA->OTYPER |= GPIO_OTYPER_OT_12;
	MODIFY_REG(GPIOA->OSPEEDR, GPIO_OSPEEDER_OSPEED12_Msk, 0b11 << GPIO_OSPEEDER_OSPEED12_Pos);

	// LEDD4 (PB4)
	MODIFY_REG(GPIOB->MODER, GPIO_MODER_MODE4_Msk, 0b01 << GPIO_MODER_MODE4_Pos);
	GPIOB->OTYPER |= GPIO_OTYPER_OT_4;
	MODIFY_REG(GPIOB->OSPEEDR, GPIO_OSPEEDER_OSPEED4_Msk, 0b11 << GPIO_OSPEEDER_OSPEED4_Pos);

	// LEDl1l2 (PA10)
	MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODE10_Msk, 0b01 << GPIO_MODER_MODE10_Pos);
	GPIOA->OTYPER |= GPIO_OTYPER_OT_10;
	MODIFY_REG(GPIOA->OSPEEDR, GPIO_OSPEEDER_OSPEED10_Msk, 0b11 << GPIO_OSPEEDER_OSPEED10_Pos);

	// LEDalarm (PA8)
	MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODE8_Msk, 0b01 << GPIO_MODER_MODE8_Pos);
	GPIOA->OTYPER |= GPIO_OTYPER_OT_8;
	MODIFY_REG(GPIOA->OSPEEDR, GPIO_OSPEEDER_OSPEED8_Msk, 0b11 << GPIO_OSPEEDER_OSPEED8_Pos);

	// pinEN (PC15)
	MODIFY_REG(GPIOC->MODER, GPIO_MODER_MODE15_Msk, 0b01 << GPIO_MODER_MODE15_Pos);
	GPIOC->OTYPER |= GPIO_OTYPER_OT_15;
	MODIFY_REG(GPIOC->OSPEEDR, GPIO_OSPEEDER_OSPEED15_Msk, 0b11 << GPIO_OSPEEDER_OSPEED15_Pos);
	
	// decrement (PA0)
	MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODE0_Msk, 0b00 << GPIO_MODER_MODE0_Pos);
	MODIFY_REG(GPIOA->OSPEEDR, GPIO_OSPEEDER_OSPEED0_Msk, 0b11 << GPIO_OSPEEDER_OSPEED0_Pos);
	MODIFY_REG(GPIOA->PUPDR, GPIO_PUPDR_PUPD0_Msk, 0b00 << GPIO_PUPDR_PUPD0_Pos);

	MODIFY_REG(SYSCFG->EXTICR[0], SYSCFG_EXTICR1_EXTI0_Msk, 0b000<< SYSCFG_EXTICR1_EXTI0_Pos);
	SET_BIT(EXTI->IMR, EXTI_IMR_IM0);
	SET_BIT(EXTI->FTSR, EXTI_FTSR_FT0);

	// enter (PA1)
	MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODE1_Msk, 0b00 << GPIO_MODER_MODE1_Pos);
	MODIFY_REG(GPIOA->OSPEEDR, GPIO_OSPEEDER_OSPEED1_Msk, 0b11 << GPIO_OSPEEDER_OSPEED1_Pos);
	MODIFY_REG(GPIOA->PUPDR, GPIO_PUPDR_PUPD1_Msk, 0b00 << GPIO_PUPDR_PUPD1_Pos);

	MODIFY_REG(SYSCFG->EXTICR[0], SYSCFG_EXTICR1_EXTI1_Msk, 0b000<< SYSCFG_EXTICR1_EXTI1_Pos);
	SET_BIT(EXTI->IMR, EXTI_IMR_IM1);
	SET_BIT(EXTI->FTSR, EXTI_FTSR_FT1);

	// increment (PA2)
	MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODE2_Msk, 0b00 << GPIO_MODER_MODE2_Pos);
	MODIFY_REG(GPIOA->OSPEEDR, GPIO_OSPEEDER_OSPEED2_Msk, 0b11 << GPIO_OSPEEDER_OSPEED2_Pos);
	MODIFY_REG(GPIOA->PUPDR, GPIO_PUPDR_PUPD2_Msk, 0b00 << GPIO_PUPDR_PUPD2_Pos);

	MODIFY_REG(SYSCFG->EXTICR[0], SYSCFG_EXTICR1_EXTI2_Msk, 0b000<< SYSCFG_EXTICR1_EXTI2_Pos);
	SET_BIT(EXTI->IMR, EXTI_IMR_IM2);
	SET_BIT(EXTI->FTSR, EXTI_FTSR_FT2);

	// pwr (PB9)		powerControlPIN
	MODIFY_REG(GPIOB->MODER, GPIO_MODER_MODE9_Msk, 0b00 << GPIO_MODER_MODE9_Pos);
	MODIFY_REG(GPIOB->OSPEEDR, GPIO_OSPEEDER_OSPEED9_Msk, 0b11 << GPIO_OSPEEDER_OSPEED9_Pos);
	MODIFY_REG(GPIOB->PUPDR, GPIO_PUPDR_PUPD9_Msk, 0b00 << GPIO_PUPDR_PUPD9_Pos);

	MODIFY_REG(SYSCFG->EXTICR[2], SYSCFG_EXTICR3_EXTI9_Msk, 0b000<< SYSCFG_EXTICR3_EXTI9_Pos);
	SET_BIT(EXTI->IMR, EXTI_IMR_IM9);
	SET_BIT(EXTI->RTSR, EXTI_RTSR_RT9);
	SET_BIT(EXTI->FTSR, EXTI_FTSR_FT9);
	}

void initRTC(void){

}

static void initTIM2(void){
  SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM2EN);
  NVIC_EnableIRQ(TIM2_IRQn);
  WRITE_REG(TIM2->PSC, 3599);
  WRITE_REG(TIM2->ARR, 50);
}

static void initTIM21(void){
  SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM2EN);
  NVIC_EnableIRQ(TIM21_IRQn);
  WRITE_REG(TIM21->PSC, 3599);
  WRITE_REG(TIM21->ARR, 50);
}
static void initLPTIM1(void){
  SET_BIT(RCC->APB1ENR, RCC_APB1ENR_LPTIM1EN);
  NVIC_EnableIRQ(LPTIM1_IRQn);
//  WRITE_REG(LPTIM1->PSC, 3599);
//  WRITE_REG(LPTIM1->ARR, 25);
}

void initLPUSART(void){

}

void SysTick_Handler() {}
void EXTI0_1_IRQHandler() {}
void EXTI2_3_IRQHandler() {}
void EXTI4_15_IRQHandler() {}
void LPTIM1_IRQHandler() {}
void TIM21_IRQHandler() {}
void LPUART1_IRQHandler() {}

void pwmFP7103() {
//	if (menu[11].value) {
//		int timeWakeUp 	= menu[6].value 	* 3600
//					+ menu[7].value	* 60;
//		int timeNow 	= sTime.Hours 		* 3600
//					+ sTime.Minutes		* 60
//					+ sTime.Seconds;
//		if(menu[9].value * 60 >= timeWakeUp - timeNow){
//			pinEN_ON();
//			//TIM_Cmd(TIM21, ENABLE);
//			TIM21->CCR1 = (int16_t) (65535 * pow((1 - timeNow / timeWakeUp), 2.24));
//		}
//	} else {
//			pinEN_OFF();
//			//TIM_Cmd(TIM21, DISABLE);
//			TIM21->CCR1 = 0;
//		}
}
//
int Clock(){
//	char tmpClock[4]={};
//	int j = 0;
//	tmpClock[0] = sTime.Hours/10;
//	if (sTime.Hours/10 == 0){j=1;}
//	tmpClock[1] = sTime.Hours - tmpClock[0];
//	tmpClock[2] = sTime.Minutes/10;
//	tmpClock[3] = sTime.Minutes - tmpClock[0];
//	if(sTime.Hours > 5 && sTime.Hours < 22 || flagDecrementButton || flagEnterButton || flagIncrementButton || flagDecrementButtonLong || flagEnterButtonLong || flagIncrementButtonLong)
//	{
//		for(int i = 0 + j; i<4;i++){
//		 writeCHARSEG(tmpClock[i], i);
//		 LL_mDelay(50);
//	 }
//	}
	return flagDecrementButtonLong&&flagIncrementButtonLong?0:1;
}
//
void setTimeNow(){
//		sTime.Hours 	= menu[2].value;
//		sTime.Minutes 	= menu[3].value;
//		sTime.Seconds 	= 00;
	}

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
		case 'A':
			LEDa_ON();
			LEDb_ON();
			LEDc_OFF();
			LEDd_ON();
			LEDe_ON();
			LEDf_ON();
			LEDg_ON();
			LEDdp_OFF();
			break;
		case 'P':
			LEDa_ON();
			LEDb_ON();
			LEDc_OFF();
			LEDd_OFF();
			LEDe_ON();
			LEDf_ON();
			LEDg_OFF();
			LEDdp_OFF();
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

char* setActualMenu(int v, int h) {
	if (v != 0) {               // Рухаємося по вертикалі
		if (v == -1) {            // Команда ВГОРУ (скасування)
			if (isParamEditMode) { // Якщо параметр у режимі редагування, то скасовуємо зміни
				isParamEditMode = false;
			} else { // Якщо пункт меню не у режимі редагування, переміщаємося до батька
				if (menu[actualIndex].parentid > 0) { // Якщо є куди переміщатися вгору (ParentID>0)
					actualIndex = getMenuIndexByID(menu[actualIndex].parentid);
				}
			}
		} else {                        // Якщо команда ВН�?З - входу/редагування
			if (menu[actualIndex].isParam && !isParamEditMode) { // Якщо не в режимі редагування, то ...
				isParamEditMode = true; // Переходимо в режим редагування параметра
				tmpVValue = menu[actualIndex].value; // Тимчасовій змінній присвоюємо актуальне значення параметра
			} else if (menu[actualIndex].isParam && isParamEditMode) { // Якщо в режимі редагування
				menu[actualIndex].value = tmpVValue; // Зберігаємо задане значення
				isParamEditMode = false;      // І виходимо з режиму редагування

			} else {
				bool nochild = true;  // Прапорець, чи є дочірні елементи
				for (int i = 0; i < menuArraySize; i++) {
					if (menu[i].parentid == menu[actualIndex].id) {
						actualIndex = i; // Якщо є, робимо перший попавшийся актуальним елементом
						nochild = false;  // Потомки є
						break;            // Виходимо з for
					}
				}
				if (nochild) { // Якщо ж потомків немає, воспринимаємо як команду
					switch (menu[actualIndex].id) { // Serial.println("Executing command...");         // І тут обробляємо за власним баченням
					case 4:						// Зберігаємо налаштування з комірки памті
						setTimeNow();
						break;
					case 13:						// Завантажуємо налаштування з комірки памті
						StartMusic(menu[12].value);
						break;
					case 17:
						while (Clock()){Clock();}
						break;
					default:
						break;
					}
				}
			}
		}
	}

	if (h != 0) {             // Якщо горизонтальна навігація
		if (isParamEditMode) {  // У режимі редагування параметра
			tmpVValue += h;        // Змінюємо його значення і ...
			// ... контролюємо, щоб воно залишилося в заданому діапазоні
			if (tmpVValue > menu[actualIndex]._max)
				tmpVValue = menu[actualIndex]._min;
			if (tmpVValue < menu[actualIndex]._min)
				tmpVValue = menu[actualIndex]._max;
		} else { // Якщо режим редагування не активний, навігація серед дочірніх одного батька
			actualIndex = getNearMenuIndexByID(menu[actualIndex].parentid,
					menu[actualIndex].id, h);
		}
	}
	// Отображаем информацию
	if (isParamEditMode) {
		int tmpV[4] = {};
		tmpV[0]=	tmpVValue/1000;
		tmpV[1]=	tmpVValue/100 	- 	tmpV[0]*10;
		tmpV[2]=	tmpVValue/10 	- 	tmpV[0]*100 	- tmpV[1]*10;
		tmpV[3]=	tmpVValue 		- 	tmpV[0]*1000 	- tmpV[1]*100 	- tmpV[2]*10;
		return tmpV;
	} else {
		return menu[actualIndex]._name;
	}
}

int getMenuIndexByID(int id) { // Функція отримання індексу пункту меню за його ID
	for (int i = 0; i < menuArraySize; i++) {
		if (menu[i].id == id)
			return i;
	}
	return -1;
}

int getNearMenuIndexByID(int parentid, int id, int side) { // Функція отримання індексу пункту меню наступного або попереднього від актуального
	int prevID = -1;      // Змінна для зберігання індексу попереднього елемента
	int nextID = -1;        // Змінна для зберігання індексу наступного елемента
	int actualID = -1;     // Змінна для зберігання індексу актуального елемента

	int firstID = -1;  // Змінна для зберігання індексу першого елемента
	int lastID = -1;   // Змінна для зберігання індексу останнього елемента

	for (int i = 0; i < menuArraySize; i++) {
		if (menu[i].parentid == parentid) { // Перебираємо всі елементи з одним батьківським ID
			if (firstID == -1)
				firstID = i;     // Запам'ятовуємо перший елемент списку

			if (menu[i].id == id) {
				actualID = i;  // Запам'ятовуємо актуальний елемент списку
			} else {
				if (actualID == -1) { // Якщо зустрівся елемент до актуального, робимо його попереднім
					prevID = i;
				} else if (actualID != -1 && nextID == -1) { // Якщо зустрівся елемент після актуального, робимо його наступним
					nextID = i;
				}
			}
			lastID = i;  // Кожний наступний елемент - останній
		}
	}

	if (nextID == -1)
		nextID = firstID; // Якщо наступного елемента немає - по колу видаємо перший
	if (prevID == -1)
		prevID = lastID; // Якщо попереднього елемента немає - по колу видаємо останній
	if (side == -1)
		return prevID; // В залежності від напрямку обертання, видаємо потрібний індекс
	else
		return nextID;
	return -1;
}

void StartMusic(int melody) {
	MusicStep = 0;
	PlayMusic = 1;
	sound(Music[MusicStep].freq, Music[MusicStep].time);
}

void sound (int freq, int time_ms) {
	if (freq > 0) {
//		TIM2->ARR = SYSCLK / timer.TIM_Prescaler / freq;
		TIM2->CCR1 = TIM2->ARR / 2;
	}
	else {
		TIM2->ARR = 1000;
		TIM2->CCR1 = 0;
	}
//	TIM_SetCounter(TIM2, 0);

//	sound_time = ((SYSCLK / timer.TIM_Prescaler / TIM2->ARR) * time_ms ) / 1000;
	sound_counter = 0;
//	TIM_Cmd(TIM2, ENABLE);
}

void delay_ms(uint32_t ms)
{
  MODIFY_REG(SysTick->VAL,SysTick_VAL_CURRENT_Msk,SYSCLOCK / 1000 - 1);
  SysTick_CNT = ms;
  while(SysTick_CNT) {}
}
/* USER CODE END 4 */

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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
