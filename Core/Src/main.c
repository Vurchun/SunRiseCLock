#include "main.h"
#include <stdbool.h>


// Макрос для налаштування GPIO
#define CONFIGURE_GPIO(PORT, PIN, MODE, TYPE, SPEED)                                         \
	MODIFY_REG(PORT->MODER, GPIO_MODER_MODE##PIN##_Msk, MODE << GPIO_MODER_MODE##PIN##_Pos); \
	MODIFY_REG(PORT->OTYPER, GPIO_OTYPER_OT_##PIN, TYPE << GPIO_OTYPER_OT_##PIN);            \
	MODIFY_REG(PORT->OSPEEDR, GPIO_OSPEEDER_OSPEED##PIN##_Msk, SPEED << GPIO_OSPEEDER_OSPEED##PIN##_Pos);

// Макрос для налаштування та ініціалізації EXTI (зовнішні переривання)
#define CONFIGURE_EXTI(PIN, PORT_SOURCE, CR, LINE)                                                                          \
	/* Налаштування лінії переривання на відповідний пін і порт */                                                                \
	MODIFY_REG(SYSCFG->EXTICR[CR], SYSCFG_EXTICR##LINE##_EXTI##PIN##_Msk, PORT_SOURCE << SYSCFG_EXTICR##LINE##_EXTI##PIN##_Pos);  \
	/* Встановлення маски переривання */                                                                                          \
	SET_BIT(EXTI->IMR, EXTI_IMR_IM##PIN);                                                                                         \
	/* Встановлення тригера на спадаючий фронт */                                                                                 \
	SET_BIT(EXTI->FTSR, EXTI_FTSR_FT##PIN);

#define BUTTON_PRESSED(PIN, PORT) (PORT->IDR & GPIO_IDR_ID##PIN)


#define ENTER 	1
#define EXIT 	0
// Макроси для вмикання/вимикання різних світлодіодів (LEDs)
#define LEDa_ON() GPIOA->BSRR = GPIO_BSRR_BS_7
#define LEDa_OFF() GPIOA->BSRR = GPIO_BSRR_BR_7
#define LEDb_ON() GPIOB->BSRR = GPIO_BSRR_BS_1
#define LEDb_OFF() GPIOB->BSRR = GPIO_BSRR_BR_1
#define LEDc_ON() GPIOA->BSRR = GPIO_BSRR_BS_6
#define LEDc_OFF() GPIOA->BSRR = GPIO_BSRR_BR_6
#define LEDd_ON() GPIOA->BSRR = GPIO_BSRR_BS_5
#define LEDd_OFF() GPIOA->BSRR = GPIO_BSRR_BR_5
#define LEDe_ON() GPIOA->BSRR = GPIO_BSRR_BS_11
#define LEDe_OFF() GPIOA->BSRR = GPIO_BSRR_BR_11
#define LEDf_ON() GPIOA->BSRR = GPIO_BSRR_BS_9
#define LEDf_OFF() GPIOA->BSRR = GPIO_BSRR_BR_9
#define LEDg_ON() GPIOB->BSRR = GPIO_BSRR_BS_0
#define LEDg_OFF() GPIOB->BSRR = GPIO_BSRR_BR_0
#define LEDdp_ON() GPIOB->BSRR = GPIO_BSRR_BS_3
#define LEDdp_OFF() GPIOB->BSRR = GPIO_BSRR_BR_3

// Макроси для керування дисплеями
#define LEDD1_OFF() GPIOA->BSRR = GPIO_BSRR_BR_3
#define LEDD1_ON() GPIOA->BSRR = GPIO_BSRR_BS_3
#define LEDD2_OFF() GPIOA->BSRR = GPIO_BSRR_BR_4
#define LEDD2_ON() GPIOA->BSRR = GPIO_BSRR_BS_4
#define LEDD3_OFF() GPIOA->BSRR = GPIO_BSRR_BR_12
#define LEDD3_ON() GPIOA->BSRR = GPIO_BSRR_BS_12
#define LEDD4_OFF() GPIOB->BSRR = GPIO_BSRR_BR_4
#define LEDD4_ON() GPIOB->BSRR = GPIO_BSRR_BS_4

// Макроси для додаткових індикаторів
#define LEDl1l2_ON() GPIOA->BSRR = GPIO_BSRR_BS_10
#define LEDl1l2_OFF() GPIOA->BSRR = GPIO_BSRR_BR_10
#define LEDalarm_ON() GPIOA->BSRR = GPIO_BSRR_BS_8
#define LEDalarm_OFF() GPIOA->BSRR = GPIO_BSRR_BR_8

// Макроси для керування enable
#define pinEN_ON() GPIOC->BSRR = GPIO_BSRR_BS_15
#define pinEN_OFF() GPIOC->BSRR = GPIO_BSRR_BR_15

#define SYSCLK 32000000

volatile uint16_t	decrementButtonCounter = 0;
volatile uint16_t	enterButtonCounter = 0;
volatile uint16_t	incrementButtonCounter = 0;

uint32_t 	DecrementButtonDebounce = 0;
uint32_t 	EnterButtonDebounce = 0;
uint32_t 	IncrementButtonDebounce = 0;
// Змінні для збереження часу натискання для кожної кнопки
uint8_t timeIncrementButtonDown = 0;
uint8_t timeDecrementButtonDown = 0;
uint8_t timeEnterButtonDown = 0;

uint8_t timeLastIncrementButtonPress = 0;
uint8_t timeLastDecrementButtonPress = 0;
uint8_t timeLastEnterButtonPress = 0;

// Прапори для обробки станів кнопок
bool flagIncrementButtonDown = false;
bool flagDecrementButtonDown = false;
bool flagEnterButtonDown = false;

bool flagIncrementButton = false;
bool flagDecrementButton = false;
bool flagEnterButton = false;

bool flagIncrementButtonLong = false;
bool flagDecrementButtonLong = false;
bool flagEnterButtonLong = false;

bool flagIncrementButtonDouble = false;
bool flagDecrementButtonDouble = false;
bool flagEnterButtonDouble = false;

bool flagPendingIncrementDoubleClick = false;
bool flagPendingDecrementDoubleClick = false;
bool flagPendingEnterDoubleClick = false;

#define menuArraySize  31		  // Встановлюємо розмір масиву меню
uint8_t actualIndex = 0;		  // Поточний індекс меню
bool isParamEditMode = false; // Прапорець режиму редагування параметра
uint8_t tmpVal = 0;			  // Тимчасова змінна для зберігання параметра

volatile uint32_t SysTimer_ms = 0;		// Системний таймер (аналог HAL_GetTick)
volatile uint16_t Delay_counter_ms = 0; // Лічильник для затримки


char tmpV[4] = {};

uint8_t vmenu = 0; // Змінна, що зберігає дію по вертикалі 1 - вхід в меню, -1 - вихід з меню
uint8_t hmenu = 0; // Змінна, що зберігає дію по горизонталі 1 - вправо, -1 - вліво
// Структура меню
struct strMenu
{
	uint8_t id;		   // Унікальний ідентифікаційний індекс ID
	uint8_t parentid;  // ID батька (вкладеність)
	bool isParam;  // Чи є пункт змінним параметром
	char _name[4]; // Назва пункту меню
	uint8_t value;	   // Поточне значення параметра
	uint8_t _min;	   // Мінімально можливе значеннял
	uint8_t _max;	   // Максимально можливе значення
};
/* PPPP
 *0 	P__0		Time_Now
 *1 		P_0.0	Hour_Now
 *2 		P_0.1	Minute_Now
 *3			P_0.2	Seconds_Now
 *4 		P_0.3	Day_Now
 *5 		P_0.4	Month_Now
 *6 		P_0.5	Year_Now
 *7 		P_0.6	WeekDay_Nows
 *8 		P_0.7	Set
 *9 	P__1		Time_Rise
 *10 		P_1.0	Hour_Rise
 *11 		P_1.1	Minute_Rise
 *12 	P__2		Rising_Parametrs
 *13 		P_2.0	Period_Rising
 *14 		P_2.1	ɣ_Coefient_Rising
 *15 	P__3		Alarm_Parametrs
 *16 		P_3.0	Alarm_Status
 *17		P_3.1	Alarm_Hours
 *18		P_3.2	Alarm_Minutes
 *19 		P_3.3	Alarm_Melody
 *20 		P_3.4	Alarm_Melody_test
 *21 	P__4		Menu_Parametrs
 *22		P_4.0	Numbers_Change_Style
 *23 		P_4.1	Menu_Night_Mode
 *24 	P__5		Menu_Parametrs
 *25 		P_5.0	debounceTime
 *26		P_5.1	timeButtonPressed
 *27 		P_5.2	timeButtonLongPressed
 *28 		P_5.3	timeDoubleClick
 *30 	P__6		Clock(StartWork)
 */
struct strMenu menu[] = {
	// Встановлюємо пункти меню
	{0, -1, false, "PPPP", 0, 0, 0},
	//-----------------------------------------------------------------------
	{1, 0, false, "P__0", 0, 0, 0},
	{2, 1, true,  "P_00", 0, 0, 24},
	{3, 1, true,  "P_01", 0, 0, 59},
	{4, 1, true,  "P_02", 0, 0, 59},
	{5, 1, true,  "P_03", 0, 0, 31},
	{6, 1, true,  "P_04", 0, 0, 12},
	{7, 1, true,  "P_05", 0, 0, 99},
	{8, 1, true,  "P_06", 0, 1, 7},
	{9, 1, false, "P_07", 0, 0, 0},
	//-----------------------------------------------------------------------
	{10, 0, false, "P__1", 0, 0, 0},
	{11, 10, true, "P_10", 0, 23, 0},
	{12, 10, true, "P_11", 0, 59, 0},
	//-----------------------------------------------------------------------
	{13, 0, false, "P__2", 0, 0, 0},
	{14, 13, true, "P_20", 0, 231, 30},
	{15, 13, true, "P_21", 0, 231, 224},
	//-----------------------------------------------------------------------
	{16, 0, false, "P__3", 0, 0, 0},
	{17, 16, true, "P_30", 0, 1, 1},
	{18, 16, true, "P_31", 0, 0, 24},
	{19, 16, true, "P_32", 0, 0, 59},
	{20, 16, true, "P_33", 0, 7, 0},
	{21, 16, false, "P_34", 0, 0, 0},
	//-----------------------------------------------------------------------
	{22, 0, false, "P__4", 0, 0, 1},
	{23, 22, true, "P_40", 0, 3, 0},
	{24, 22, true, "P_41", 0, 1, 1},
	//-----------------------------------------------------------------------
	{25, 0, false, "P__5", 0, 0, 0},
	{26, 25, true, "P_50", 0, 100, 75},
	{27, 25, true, "P_51", 0, 231, 125},
	{28, 25, true, "P_52", 0, 231, 200},
	{29, 25, true, "P_53", 0, 231, 125},
	//-----------------------------------------------------------------------
	{30, 0, false, "P__5", 0, 0, 0}
	//-----------------------------------------------------------------------
};

/* Sound/Buzzer variables ---------------------------------------------------------*/
uint16_t MusicStep = 0;
char PlayMusic = 0;
uint16_t sound_time;
uint16_t sound_counter;

#define C 261  // Do
#define C_ 277 // Do#
#define D 293  // Re
#define D_ 311 // Re#
#define E 239  // Mi
#define F 349  // Fa
#define F_ 370 // Fa#
#define G 392  // Sol
#define G_ 415 // Sol#
#define A 440  // La
#define A_ 466 // La#
#define H 494  // Si

#define t1 2000
#define t2 1000
#define t4 500
#define t8 250
#define t16 125

typedef struct
{
	uint16_t freq;
	uint16_t time;
} SoundTypeDef;

const SoundTypeDef Music[48] = {
	{C * 2, t4},
	{G, t4},
	{A_, t8},
	{F, t8},
	{D_, t8},
	{F, t8},
	{G, t4},
	{C, t2},
	{C * 2, t4},
	{G, t4},
	{A_, t8},
	{F, t8},
	{D_, t8},
	{F, t8},
	{G, t4},
	{C * 2, t4},
	{0, t8},
	{D_, t8},
	{D_, t8},
	{D_, t8},
	{G, t8},
	{A_, t4},
	{D_ * 2, t8},
	{C_ * 2, t8},
	{C * 2, t8},
	{C * 2, t8},
	{C * 2, t8},
	{C * 2, t8},
	{A_, t8},
	{F, t8},
	{D_, t8},
	{F, t8},
	{G, t4},
	{C * 2, t2},
	{C * 2, t2},
	{A_, t8},
	{G_, t8},
	{G, t8},
	{G_, t8},
	{A_, t2},
	{A_, t4},
	{C * 2, t4},
	{A_, t8},
	{F, t8},
	{D_, t8},
	{F, t8},
	{G, t4},
	{C * 2, t2}};
/* END S/BV */

/* Private function prototypes -----------------------------------------------*/
void CMSIS_FullInit(void);
uint8_t SysTickTimerInit(uint16_t ticks);
void SystemClock_Config(void);
void GPIO_Init(void);
void RTC_Init(void);
void TIM2_Init(void);
void TIM21_Init(void);
/*Handlers*/
void SysTick_Handler(void);
void Error_Handler(void);
void assert_failed(uint8_t *file, uint8_t line);
void EXTI0_1_IRQHandler(void);
void EXTI2_3_IRQHandler(void);
void EXTI4_15_IRQHandler(void);
void LPTIM1_IRQHandler(void);
void TIM2_IRQHandler(void);
void TIM21_IRQHandler(void);
/*----------------------------------------------------------------------------*/
void EnterLowPowerMode(uint8_t status);
void Delay_ms(uint16_t Milliseconds);
double custom_pow(double a, double x);
uint32_t custom_floor(double x);
/*----------------------------------------------------------------------------*/
void pwmFP7103();
/*----------------------------------------------------------------------------*/
void setTimeNow();
uint8_t Clock();
/*----------------------------------------------------------------------------*/
char intToChar(uint8_t num);
void writeCHARSEG(char CHAR, uint8_t seg);
char *setActualMenu(uint8_t v, uint8_t h);
uint8_t getMenuIndexByID(uint8_t id);
uint8_t getNearMenuIndexByID(uint8_t parentid, uint8_t id, uint8_t side);
/*----------------------------------------------------------------------------*/
void StartMusic(uint16_t melody);
void sound(uint16_t freq, uint16_t time_ms);
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
int main(void)
{

	char *tmpValue;

	CMSIS_FullInit(); // 1ms
	SystemClock_Config();

	GPIO_Init();
	RTC_Init();
	TIM2_Init();
	TIM21_Init();

	writeCHARSEG(' ', ' ');
	pinEN_OFF();
	tmpValue = setActualMenu(0, 0);

	while (1)
	{
		if(SysTimer_ms-DecrementButtonDebounce 	>= menu[25].value && READ_BIT(EXTI->IMR, EXTI_IMR_IM0) == 0) 	EXTI->IMR |= EXTI_IMR_IM0; 	// *25 		P_5.0	debounceTime
		if(SysTimer_ms-EnterButtonDebounce 		>= menu[25].value && READ_BIT(EXTI->IMR, EXTI_IMR_IM1) == 0) 	EXTI->IMR |= EXTI_IMR_IM1;	// *25 		P_5.0	debounceTime
		if(SysTimer_ms-IncrementButtonDebounce 	>= menu[25].value && READ_BIT(EXTI->IMR, EXTI_IMR_IM2) == 0) 	EXTI->IMR |= EXTI_IMR_IM2;	// *25 		P_5.0	debounceTime

		if (flagDecrementButton)
		{
			hmenu = 1;
			flagDecrementButton = false; // Действие обработано - сбрасываем флаг
		}
			else if(flagDecrementButtonLong){
				hmenu = 1;
			flagDecrementButtonLong = false; // Действие обработано - сбрасываем флаг
		}

		if (flagIncrementButton)
		{
			hmenu = -1;
			flagIncrementButton = false; // Действие обработано - сбрасываем флаг
		}
			else if(flagIncrementButtonLong){
				hmenu = -1;
			flagIncrementButtonLong = false; // Действие обработано - сбрасываем флаг
		}

		if (flagEnterButton)
		{							 // Кнопка нажата
			vmenu = 1;				 // По нажатию кнопки - переходим на уровень вниз
			flagEnterButton = false; // Действие обработано - сбрасываем флаг
		}
			else if (flagEnterButtonLong)
		{
			vmenu = -1;
			flagEnterButtonLong = false; // Действие обработано - сбрасываем флаг
		}
		
		if (vmenu != 0 || hmenu != 0)	tmpValue = setActualMenu(vmenu, hmenu); // Если было действие - реагируем на него
			if(SysTimer_ms%4==0)		writeCHARSEG(tmpValue[0], 0);
			if(SysTimer_ms%4==1)		writeCHARSEG(tmpValue[1], 1);
			if(SysTimer_ms%4==2)		writeCHARSEG(tmpValue[2], 2);
			if(SysTimer_ms%4==3)		writeCHARSEG(tmpValue[3], 3);
	}
	return 0;
}
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
uint8_t SysTickTimerInit(uint16_t ticks)
{

  if ((ticks - 1UL) > SysTick_LOAD_RELOAD_Msk)
  {
    return (1UL);                                                   /* Reload value impossible */
  }
  CLEAR_BIT(SysTick->CTRL,SysTick_CTRL_ENABLE_Msk);					/* Disenable SysTick Timer */
  SysTick->LOAD  = (uint16_t)(ticks - 1UL);                         /* set reload register */
  NVIC_SetPriority (SysTick_IRQn, (1UL << __NVIC_PRIO_BITS) - 1UL); /* set Priority for Systick uint16_terrupt */
  SysTick->VAL   = 0UL;                                             /* Load the SysTick Counter Value */
  SysTick->CTRL  = 	SysTick_CTRL_CLKSOURCE_Msk |
		  	  	  	SysTick_CTRL_TICKINT_Msk   |
					SysTick_CTRL_ENABLE_Msk;                         /* Enable SysTick IRQ and SysTick Timer */
  return (0UL);                                                     /* Function successful */
}

void CMSIS_FullInit(void)
{
    // *** Налаштування кешу, передвибірки і попереднього читання *** //

    // Вимкнути буфер кешу, якщо це налаштовано
		CLEAR_BIT(FLASH->ACR,FLASH_ACR_DISAB_BUF);
    // Включити попереднє читання, якщо це налаштовано
		SET_BIT(FLASH->ACR,FLASH_ACR_PRE_READ);
    // Включити буфер передвибірки, якщо це налаштовано
		SET_BIT(FLASH->ACR,FLASH_ACR_PRFTEN);
		SET_BIT(FLASH->ACR,FLASH_ACR_LATENCY);

    // *** Налаштування SysTick для переривань кожну 1 мс *** //

    uint16_t ticks = SYSCLK / 1000U;  // Розрахунок кількості тактів для 1 мс

    // Використовуємо SysTick_Config для налаштування таймера
    if (ticks > SysTick_LOAD_RELOAD_Msk) // Якщо кількість тактів більше дозволеного
    {
        while (1); // Помилка, зациклюємося
    }

    SysTickTimerInit(ticks);

    // Встановлення пріоритету для переривання SysTick
    uint32_t tickPriority = 0;  // Пріоритет для SysTick (без макросів)
    if (tickPriority < (1UL << __NVIC_PRIO_BITS))
    {
        NVIC_SetPriority(SysTick_IRQn, tickPriority);
    }
    else
    {
        while (1);  // Помилка пріоритету
    }
}

void SystemClock_Config(void)
{
	// Налаштування PWR CR для регулювання напруги
	MODIFY_REG(PWR->CR, PWR_CR_VOS_Msk, 0b01 << PWR_CR_VOS_Pos);

	// Вимкнення HSI16DIV
	CLEAR_BIT(RCC->CR, RCC_CR_HSIDIVEN);

	// Вимкнення MSI
	CLEAR_BIT(RCC->CR, RCC_CR_MSION);

	// Якщо джерело системної частоти не HSI16
	if ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_HSI)
	{
		// Увімкнення HSI16
		SET_BIT(RCC->CR, RCC_CR_HSION);

		// Очікування стабілізації HSI16
		while (!(RCC->CR & RCC_CR_HSIRDY))
		{
		}

		// Перемикання системної тактової частоти на HSI16
		MODIFY_REG(RCC->CFGR, RCC_CFGR_SW_Msk, RCC_CFGR_SW_HSI);

		// Очікування завершення перемикання
		while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_HSI)
		{
		}
	}

	// Налаштування MCO prescaler і джерела сигналу
	MODIFY_REG(RCC->CFGR, RCC_CFGR_MCOPRE_Msk, 0b000 << RCC_CFGR_MCOPRE_Pos);
	MODIFY_REG(RCC->CFGR, RCC_CFGR_MCOSEL_Msk, 0b000 << RCC_CFGR_MCOSEL_Pos);

	// Налаштування PLL
	MODIFY_REG(RCC->CFGR, RCC_CFGR_PLLDIV_Msk, 0b01 << RCC_CFGR_PLLDIV_Pos);
	MODIFY_REG(RCC->CFGR, RCC_CFGR_PLLMUL_Msk, 0b0001 << RCC_CFGR_PLLMUL_Pos);
	MODIFY_REG(RCC->CFGR, RCC_CFGR_PLLSRC_Msk, 0b0 << RCC_CFGR_PLLSRC_Pos);

	// Вимкнення системного пробудження від MSI після STOP режиму
	SET_BIT(RCC->CFGR, RCC_CFGR_STOPWUCK);

	// Налаштування прескалерів для шин APB1, APB2 та AHB
	MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2_Msk, 0b000 << RCC_CFGR_PPRE2_Pos);
	MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1_Msk, 0b000 << RCC_CFGR_PPRE1_Pos);
	MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE_Msk, 0b0000 << RCC_CFGR_HPRE_Pos);

	// Перемикання на PLL
	SET_BIT(RCC->CR, RCC_CR_PLLON);

	// Очікування стабілізації PLL
	while (!(RCC->CR & RCC_CR_PLLRDY))
	{
	}

	// Перемикання системної частоти на PLL
	MODIFY_REG(RCC->CFGR, RCC_CFGR_SW_Msk, RCC_CFGR_SW_PLL);

	// Очікування завершення перемикання
	while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_PLL)
	{
	}
}

void RTC_Init(void)
{
	// 1. Enable power and backup domain access
	SET_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN);
	SET_BIT(PWR->CR, PWR_CR_DBP);

	// 2. Enable LSE Oscillator
	SET_BIT(RCC->CSR, RCC_CSR_LSEON);
	while (!(READ_BIT(RCC->CSR, RCC_CSR_LSERDY)))
	{
	}

	// 3. Set LSE as RTC clock source and enable RTC
	MODIFY_REG(RCC->CSR, RCC_CSR_RTCSEL_Msk, 0b01 << RCC_CSR_RTCSEL_Pos); // LSE selected as RTC clock
	SET_BIT(RCC->CSR, RCC_CSR_RTCEN);

	// 4. Disable RTC write protection
	RTC->WPR = 0xCA; // Step 1
	RTC->WPR = 0x53; // Step 2

	// 5. Enter initialization mode
	SET_BIT(RTC->ISR, RTC_ISR_INIT);
	while (!(READ_BIT(RTC->ISR, RTC_ISR_INITF)))
	{
	}

	// 6. Set the time in BCD format (17:36:00)
	MODIFY_REG(RTC->TR,
			   RTC_TR_HT_Msk | RTC_TR_HU_Msk | RTC_TR_MNT_Msk | RTC_TR_MNU_Msk | RTC_TR_ST_Msk | RTC_TR_SU_Msk,
			   (0x1 << RTC_TR_HT_Pos) |		 // Hour tens (1 -> 17)
				   (0x7 << RTC_TR_HU_Pos) |	 // Hour units (7 -> 17)
				   (0x3 << RTC_TR_MNT_Pos) | // Minute tens (3 -> 36)
				   (0x6 << RTC_TR_MNU_Pos) | // Minute units (6 -> 36)
				   (0x0 << RTC_TR_ST_Pos) |	 // Second tens (0 -> 00)
				   (0x0 << RTC_TR_SU_Pos));	 // Second units (0 -> 00)

	// 7. Set the date in BCD format (01/02/2024, Monday)
	MODIFY_REG(RTC->DR,
			   RTC_DR_YT_Msk | RTC_DR_YU_Msk | RTC_DR_MT_Msk | RTC_DR_MU_Msk | RTC_DR_DT_Msk | RTC_DR_DU_Msk | RTC_DR_WDU_Msk,
			   (0x2 << RTC_DR_YT_Pos) |		 // Year tens (2 -> 2024)
				   (0x4 << RTC_DR_YU_Pos) |	 // Year units (4 -> 2024)
				   (0x0 << RTC_DR_MT_Pos) |	 // Month tens (1 -> April)
				   (0x4 << RTC_DR_MU_Pos) |	 // Month units (0 -> April)
				   (0x0 << RTC_DR_DT_Pos) |	 // Day tens (0 -> 01)
				   (0x1 << RTC_DR_DU_Pos) |	 // Day units (1 -> 01)
				   (0x2 << RTC_DR_WDU_Pos)); // Weekday (3 -> Monday)

	// 8. Exit initialization mode
	CLEAR_BIT(RTC->ISR, RTC_ISR_INIT);

	// 9. Re-enable RTC write protection
	RTC->WPR = 0xFE; // Disable write access for RTC register
	RTC->WPR = 0x64; //				-||-
}

void TIM2_Init(void)
{

	// Увімкнення тактування GPIOA (для PA15, як PWM вихід)
	RCC->IOPENR |= RCC_IOPENR_IOPAEN;
	// Настроить пин 15 на режим альтернативной функции
	CONFIGURE_GPIO(GPIOB, 15, 0b10, 0, 0b11); // BuzzerPin
	// Настроить альтернативную функцию AF1 для пина 15
	MODIFY_REG(GPIOA->AFR[1], GPIO_AFRH_AFSEL15_Msk, 0b0101 << GPIO_AFRH_AFSEL15_Pos);

	// Увімкнення тактування TIM2
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	CLEAR_BIT(TIM2->CR1, TIM_CR1_CEN);

	/*Настройка таймера 2*/
	// TIMx control register 1 (TIMx_CR1)
	// SET_BIT(TIM2->CR1, TIM_CR1_CEN);  //Запуск таймера
	CLEAR_BIT(TIM2->CR1, TIM_CR1_UDIS);								 // Генерировать событие Update
	CLEAR_BIT(TIM2->CR1, TIM_CR1_URS);								 // Генерировать прерывание
	CLEAR_BIT(TIM2->CR1, TIM_CR1_OPM);								 // One pulse mode off(Счетчик не останавливается при обновлении)
	CLEAR_BIT(TIM2->CR1, TIM_CR1_DIR);								 // Считаем вверх
	MODIFY_REG(TIM2->CR1, TIM_CR1_CMS_Msk, 0b00 << TIM_CR1_CMS_Pos); // Выравнивание по краю
	SET_BIT(TIM2->CR1, TIM_CR1_ARPE);								 // Auto-reload preload enable
	MODIFY_REG(TIM2->CR1, TIM_CR1_CKD_Msk, 0b00 << TIM_CR1_CKD_Pos); // Предделение выключено

	/*Настройка прерываний*/
	// TIMx DMA/uint16_terrupt enable register (TIMx_DIER)
	SET_BIT(TIM2->DIER, TIM_DIER_UIE); // Update uint16_terrupt enable

	// TIMx status register (TIMx_SR) - Статусные регистры

	TIM2->PSC = 3200 - 1;
	TIM2->ARR = 10000 - 1;

	NVIC_EnableIRQ(TIM2_IRQn); // Разрешить прерывания по таймеру 2
	/*Настройка шим(Канал 1)*/
	MODIFY_REG(TIM2->CCMR1, TIM_CCMR1_CC1S_Msk, 0b00 << TIM_CCMR1_CC1S_Pos);  // CC1 channel is configured as output
	CLEAR_BIT(TIM2->CCMR1, TIM_CCMR1_OC1FE);								  // Fast mode disable
	SET_BIT(TIM2->CCMR1, TIM_CCMR1_OC1PE);									  // Preload enable
	MODIFY_REG(TIM2->CCMR1, TIM_CCMR1_OC1M_Msk, 0b110 << TIM_CCMR1_OC1M_Pos); // PWM MODE 1
	CLEAR_BIT(TIM2->CCMR1, TIM_CCMR1_OC1CE);								  // OC1Ref is not affected by the ETRF input

	/*Запуск ШИМ*/
	// 15.4.9 TIMx capture/compare enable register (TIMx_CCER)
	SET_BIT(TIM2->CCER, TIM_CCER_CC1E);	   // On - OC1 signal is output on the corresponding output pin.
	CLEAR_BIT(TIM21->CCER, TIM_CCER_CC1P); // OC1 active high.

	SET_BIT(TIM2->CR1, TIM_CR1_CEN);
}

void TIM21_Init(void)
{
	// Увімкнення тактування GPIOB (для PB5, як PWM вихід)
	RCC->IOPENR |= RCC_IOPENR_IOPBEN;
	// Настроить пин 5 на режим альтернативной функции
	CONFIGURE_GPIO(GPIOB, 5, 0b10, 0, 0b11); // mainLED
	// Настроить альтернативную функцию AF1 для пина 5
	MODIFY_REG(GPIOB->AFR[0], GPIO_AFRL_AFSEL5_Msk, 0b0101 << GPIO_AFRL_AFSEL5_Pos);

	// Увімкнення тактування TIM21
	RCC->APB2ENR |= RCC_APB2ENR_TIM21EN;
	CLEAR_BIT(TIM21->CR1, TIM_CR1_CEN);

	/*Настройка таймера 21*/
	// TIMx control register 1 (TIMx_CR1)
	// SET_BIT(TIM21->CR1, TIM_CR1_CEN);  //Запуск таймера
	CLEAR_BIT(TIM21->CR1, TIM_CR1_UDIS);							  // Генерировать событие Update
	CLEAR_BIT(TIM21->CR1, TIM_CR1_URS);								  // Генерировать прерывание
	CLEAR_BIT(TIM21->CR1, TIM_CR1_OPM);								  // One pulse mode off(Счетчик не останавливается при обновлении)
	CLEAR_BIT(TIM21->CR1, TIM_CR1_DIR);								  // Считаем вверх
	MODIFY_REG(TIM21->CR1, TIM_CR1_CMS_Msk, 0b00 << TIM_CR1_CMS_Pos); // Выравнивание по краю
	SET_BIT(TIM21->CR1, TIM_CR1_ARPE);								  // Auto-reload preload enable
	MODIFY_REG(TIM21->CR1, TIM_CR1_CKD_Msk, 0b00 << TIM_CR1_CKD_Pos); // Предделение выключено

	/*Настройка прерываний*/
	// TIMx DMA/interrupt enable register (TIMx_DIER)
	SET_BIT(TIM21->DIER, TIM_DIER_UIE); // Update interrupt enable

	// TIMx status register (TIMx_SR) - Статусные регистры
	TIM21->PSC = 3200 - 1;
	TIM21->ARR = 10000 - 1;

	NVIC_EnableIRQ(TIM21_IRQn); // Разрешить прерывания по таймеру 21

	/*Настройка шим(Канал 1)*/
	MODIFY_REG(TIM21->CCMR1, TIM_CCMR1_CC1S_Msk, 0b00 << TIM_CCMR1_CC1S_Pos);  // CC1 channel is configured as output
	CLEAR_BIT(TIM21->CCMR1, TIM_CCMR1_OC1FE);								   // Fast mode disable
	SET_BIT(TIM21->CCMR1, TIM_CCMR1_OC1PE);									   // Preload enable
	MODIFY_REG(TIM21->CCMR1, TIM_CCMR1_OC1M_Msk, 0b110 << TIM_CCMR1_OC1M_Pos); // PWM MODE 1
	CLEAR_BIT(TIM21->CCMR1, TIM_CCMR1_OC1CE);								   // OC1Ref is not affected by the ETRF input

	/*Запуск ШИМ*/
	// TIMx capture/compare enable register (TIMx_CCER)
	SET_BIT(TIM21->CCER, TIM_CCER_CC1E); // On - OC1 signal is output on the corresponding output pin.
	SET_BIT(TIM21->CCER, TIM_CCER_CC1P); // OC1 active high.

	SET_BIT(TIM21->CR1, TIM_CR1_CEN);
}

void GPIO_Init(void)
{
	// Включення тактування портів A, B, C
	RCC->IOPENR |= RCC_IOPENR_IOPAEN | RCC_IOPENR_IOPBEN | RCC_IOPENR_IOPCEN;

	// Налаштування світлодіодів (виводи PA, PB)
	CONFIGURE_GPIO(GPIOA, 7, 0b01, 0, 0b11);  // LEDa
	CONFIGURE_GPIO(GPIOB, 1, 0b01, 0, 0b11);  // LEDb
	CONFIGURE_GPIO(GPIOA, 6, 0b01, 0, 0b11);  // LEDc
	CONFIGURE_GPIO(GPIOA, 5, 0b01, 0, 0b11);  // LEDd
	CONFIGURE_GPIO(GPIOA, 11, 0b01, 0, 0b11); // LEDe
	CONFIGURE_GPIO(GPIOA, 9, 0b01, 0, 0b11);  // LEDf
	CONFIGURE_GPIO(GPIOB, 0, 0b01, 0, 0b11);  // LEDg
	CONFIGURE_GPIO(GPIOB, 3, 0b01, 0, 0b11);  // LEDdp
	CONFIGURE_GPIO(GPIOA, 3, 0b01, 0, 0b11);  // LEDD1
	CONFIGURE_GPIO(GPIOA, 4, 0b01, 0, 0b11);  // LEDD2
	CONFIGURE_GPIO(GPIOA, 12, 0b01, 0, 0b11); // LEDD3
	CONFIGURE_GPIO(GPIOB, 4, 0b01, 0, 0b11);  // LEDD4
	CONFIGURE_GPIO(GPIOA, 10, 0b01, 0, 0b11); // LEDl1l2
	CONFIGURE_GPIO(GPIOA, 8, 0b01, 0, 0b11);  // LEDalarm
	CONFIGURE_GPIO(GPIOC, 15, 0b01, 0, 0b11); // pinEN

	// Налаштування кнопок із EXTI
	CONFIGURE_GPIO(GPIOA, 0, 0b00, 0, 0b11); // decrement
	CONFIGURE_EXTI(0, 0b000, 0, 1);		 // EXTI для decrement

	CONFIGURE_GPIO(GPIOA, 1, 0b00, 0, 0b11); // enter
	CONFIGURE_EXTI(1, 0b000, 0, 1);		 // EXTI для enter

	CONFIGURE_GPIO(GPIOA, 2, 0b00, 0, 0b11); // increment
	CONFIGURE_EXTI(2, 0b000, 0, 1);		 // EXTI для increment

	CONFIGURE_GPIO(GPIOB, 9, 0b00, 0, 0b11); // pwr
	CONFIGURE_EXTI(9, 0b000, 2, 3);		 // EXTI для pwr з обробкою по зростанню

	/* Включення переривання */                                                                                                   \
	NVIC_EnableIRQ(EXTI0_1_IRQn);                                                                                                  \
	NVIC_EnableIRQ(EXTI2_3_IRQn);
	NVIC_EnableIRQ(EXTI4_15_IRQn);
}

void LowPowerMode(uint8_t status)
{
    if(status){// Встановлюємо режим STOP з RTC працюючим у нормальному режимі
//	pinEN_OFF();
//	RCC->APB1ENR &= ~RCC_APB1ENR_TIM2EN;
//	RCC->APB1ENR &= ~RCC_APB2ENR_TIM21EN;
//	RCC->IOPENR &= ~RCC_IOPENR_IOPAEN | ~RCC_IOPENR_IOPBEN | ~RCC_IOPENR_IOPCEN;
//	RCC->IOPENR |= RCC_IOPENR_IOPBEN | RCC_IOPENR_IOPCEN;
//	CONFIGURE_GPIO(GPIOC, 15, 0b01, 0, 0b11); // pinEN
//	CONFIGURE_GPIO(GPIOB, 9, 0b00, 0, 0b11); // pwr
//	CONFIGURE_EXTI(9, 0b000, 2, 3);		 // EXTI для pwr з обробкою по зростанню
//
////    PWR->CR |= PWR_CR_LPDS;  // Налаштовуємо режим глибокого сну
////    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;  // Налаштовуємо контролер для глибокого сну
////
////    // Входимо у режим STOP, поки не відбудеться переривання від RTC чи EXTI
////     __WFI();  // Чекаємо на переривання для виходу з режиму STOP

    }else{
//	pinEN_ON();
//	GPIO_Init();
//	TIM2_Init();
//	TIM21_Init();
////    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;  // Вимикаємо режим глибокого сну
//    PWR->CR &= ~PWR_CR_LPDS;  // Відновлюємо нормальний режим живлення
}
}

void Delay_ms(uint16_t Miliseconds)
{
	Delay_counter_ms = Miliseconds;
	while (Delay_counter_ms != 0)
	{
		Delay_counter_ms--;
	}
}

uint32_t custom_floor(double x)
{
	uint32_t result = (uint32_t)x; // Приведення до uint32_t обрізає дробову частину
	if (x < 0 && x != result)
	{
		result--; // Якщо x від'ємне і не ціле, округляємо до меншого
	}
	return result;
}

double custom_pow(double a, double x)
{
	// Разделяем x на целую и дробную части
	uint16_t intPart = (uint16_t)x;		   // целая часть x
	double fracPart = x - intPart; // дробная часть x

	// Возведение a в целую степень
	double result = 1.0;
	for (uint16_t i = 0; i < intPart; ++i)
	{
		result *= a;
	}

	// Аппроксимация дробной части (для x между 2 и 3 можно ограничиться линейной аппроксимацией)
	double fractionalMultiplier = 1.0;
	if (fracPart > 0.0)
	{
		fractionalMultiplier = 1.0 + fracPart * (a - 1.0); // Пример аппроксимации для дробной части
	}

	return result * fractionalMultiplier;
}

char intToChar(uint8_t num)
{
	switch (num)
	{
	case 0:
		return '0';
	case 1:
		return '1';
	case 2:
		return '2';
	case 3:
		return '3';
	case 4:
		return '4';
	case 5:
		return '5';
	case 6:
		return '6';
	case 7:
		return '7';
	case 8:
		return '8';
	case 9:
		return '9';
	default:
		return '?'; // Повертаємо '?' для невідомого числа
	}
}

void writeCHARSEG(char CHAR, uint8_t seg)
{
	switch (seg)
	{
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
	switch (CHAR)
	{
	case 'P':
		LEDa_ON();
		LEDb_ON();
		LEDc_OFF();
		LEDd_OFF();
		LEDe_ON();
		LEDf_ON();
		LEDg_ON();
		LEDdp_OFF();
		break;
	case '_':
		LEDa_OFF();
		LEDb_OFF();
		LEDc_OFF();
		LEDd_ON();
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

void pwmFP7103()
{
	if (menu[16].value)
	{											//*16 		P_3.0	Alarm_Status
		uint16_t timeWakeUp = menu[10].value * 3600	//*10 		P_1.0	Hour_Rise
						 + menu[11].value * 60; //*11 		P_1.1	Minute_Rise
		uint16_t timeNow = (READ_BIT(RTC->TR, RTC_TR_HT) * 10 + READ_BIT(RTC->TR, RTC_TR_HU)) * 3600 + (READ_BIT(RTC->TR, RTC_TR_MNT) * 10 + READ_BIT(RTC->TR, RTC_TR_MNU)) * 60 + (READ_BIT(RTC->TR, RTC_TR_ST) * 10 + READ_BIT(RTC->TR, RTC_TR_SU));
		if (menu[13].value * 60 >= timeWakeUp - timeNow)
		{ // *13 		P_2.0	Period_Rising
			pinEN_ON();
			SET_BIT(TIM21->CR1, TIM_CR1_CEN); // Запуск таймера
			TIM21->CCR1 = custom_floor(TIM2->ARR * custom_pow((1 - timeNow / timeWakeUp), menu[14].value / 100));
			// *14 		P_2.1	ɣ_Coefient_Rising
		}
	}
	else
	{
		pinEN_OFF();
		CLEAR_BIT(TIM21->CR1, TIM_CR1_CEN); // Призупинення таймера
		TIM21->CCR1 = 0;
	}
}

uint8_t Clock()
{
	char tmpClock[4] = {};
	uint8_t i = 0;
	uint8_t j = 0;
	tmpClock[0] = READ_BIT(RTC->TR, RTC_TR_HT);
	if (tmpClock[0] == 0)	i = 1;
	tmpClock[1] = READ_BIT(RTC->TR, RTC_TR_HU);
	tmpClock[2] = READ_BIT(RTC->TR, RTC_TR_MNT);
	if (tmpClock[2] == 0)	j = 1;
	tmpClock[3] = READ_BIT(RTC->TR, RTC_TR_MNU);

	pwmFP7103();
//	 *23 		P_4.1	Menu_Night_Mode
	if (((READ_BIT(RTC->TR, RTC_TR_HT) * 10 + READ_BIT(RTC->TR, RTC_TR_HU) > 5) && (READ_BIT(RTC->TR, RTC_TR_HT) * 10 + READ_BIT(RTC->TR, RTC_TR_HU) < 22) && menu[23].value) || flagDecrementButton || flagEnterButton || flagIncrementButton || menu[23].value)
	{
		if(SysTimer_ms%4==0 && i == 0) 	writeCHARSEG(tmpClock[0], 0);
		if(SysTimer_ms%4==1)			writeCHARSEG(tmpClock[1], 1);
		if(SysTimer_ms%4==2 && j == 0)	writeCHARSEG(tmpClock[2], 2);
		if(SysTimer_ms%4==3)			writeCHARSEG(tmpClock[3], 3);
	}
	return flagDecrementButtonLong || flagEnterButtonLong || flagIncrementButtonLong ? 0 : 1;
}

void setTimeNow()
{
	MODIFY_REG(RTC->TR,
			   RTC_TR_HT_Msk | RTC_TR_HU_Msk | RTC_TR_MNT_Msk | RTC_TR_MNU_Msk | RTC_TR_ST_Msk | RTC_TR_SU_Msk,
			   (menu[1].value / 10 << RTC_TR_HT_Pos) |		// Hour tens (1 -> 17)
				   (menu[1].value % 10 << RTC_TR_HU_Pos) |	// Hour units (7 -> 17)
				   (menu[2].value / 10 << RTC_TR_MNT_Pos) | // Minute tens (3 -> 36)
				   (menu[2].value % 10 << RTC_TR_MNU_Pos) | // Minute units (6 -> 36)
				   (menu[3].value / 10 << RTC_TR_ST_Pos) |	// Second tens (0 -> 00)
				   (menu[3].value % 10 << RTC_TR_SU_Pos));	// Second units (0 -> 00)
	MODIFY_REG(RTC->DR,
			   RTC_DR_YT_Msk | RTC_DR_YU_Msk | RTC_DR_MT_Msk | RTC_DR_MU_Msk | RTC_DR_DT_Msk | RTC_DR_DU_Msk | RTC_DR_WDU_Msk,
			   (menu[6].value / 10 << RTC_DR_YT_Pos) |	   // Year tens (2 -> 24)
				   (menu[6].value % 10 << RTC_DR_YU_Pos) | // Year units (4 -> 24)
				   (menu[5].value / 10 << RTC_DR_MT_Pos) | // Month tens (1 -> April)
				   (menu[5].value % 10 << RTC_DR_MU_Pos) | // Month units (0 -> April)
				   (menu[4].value / 10 << RTC_DR_DT_Pos) | // Day tens (0 -> 01)
				   (menu[4].value % 10 << RTC_DR_DU_Pos) | // Day units (1 -> 01)
				   (menu[7].value << RTC_DR_WDU_Pos));	   // Weekday (3 -> Monday)
}

char *setActualMenu(uint8_t v, uint8_t h)
{
	if (v != 0)
	{ // Рухаємося по вертикалі
		if (v == -1)
		{ // Команда ВГОРУ (скасування)
			if (isParamEditMode)
			{ // Якщо параметр у режимі редагування, то скасовуємо зміни
				isParamEditMode = false;
			}
			else
			{ // Якщо пункт меню не у режимі редагування, переміщаємося до батька
				if (menu[actualIndex].parentid > 0)
				{ // Якщо є куди переміщатися вгору (ParentID>0)
					actualIndex = getMenuIndexByID(menu[actualIndex].parentid);
				}
			}
		}
		else
		{ // Якщо команда ВН�?З - входу/редагування
			if (menu[actualIndex].isParam && !isParamEditMode)
			{										 // Якщо не в режимі редагування, то ...
				isParamEditMode = true;				 // Переходимо в режим редагування параметра
				tmpVal = menu[actualIndex].value; // Тимчасовій змінній присвоюємо актуальне значення параметра
			}
			else if (menu[actualIndex].isParam && isParamEditMode)
			{										 // Якщо в режимі редагування
				menu[actualIndex].value = tmpVal; // Зберігаємо задане значення
				isParamEditMode = false;			 // І виходимо з режиму редагування
			}
			else
			{
				bool nochild = true; // Прапорець, чи є дочірні елементи
				for (uint8_t i = 0; i < menuArraySize; i++)
				{
					if (menu[i].parentid == menu[actualIndex].id)
					{
						actualIndex = i; // Якщо є, робимо перший попавшийся актуальним елементом
						nochild = false; // Потомки є
						break;			 // Виходимо з for
					}
				}
				if (nochild)
				{ // Якщо ж потомків немає, воспринимаємо як команду
					switch (menu[actualIndex].id)
					{		// Serial.pruint16_tln("Executing command...");         // І тут обробляємо за власним баченням
					case 4: // Зберігаємо налаштування з комірки памті
						setTimeNow();
						break;
					case 13:						// Завантажуємо налаштування з комірки памті
						StartMusic(menu[19].value); // *19 		P_3.3	Alarm_Melody
						break;
					case 17:
						while (Clock())
						{
							Clock();
						}
						break;
					default:
						break;
					}
				}
			}
		}
	}

	if (h != 0)
	{ // Якщо горизонтальна навігація
		if (isParamEditMode)
		{					// У режимі редагування параметра
			tmpVal += h; // Змінюємо його значення і ...
			// ... контролюємо, щоб воно залишилося в заданому діапазоні
			if (tmpVal > menu[actualIndex]._max)
				tmpVal = menu[actualIndex]._min;
			if (tmpVal < menu[actualIndex]._min)
				tmpVal = menu[actualIndex]._max;
		}
		else
		{ // Якщо режим редагування не активний, навігація серед дочірніх одного батька
			actualIndex = getNearMenuIndexByID(menu[actualIndex].parentid,
											   menu[actualIndex].id, h);
		}
	}
	// Отображаем информацию
	if (isParamEditMode)
	{
		tmpV[0] = intToChar(tmpVal / 1000);
		tmpV[1] = intToChar(tmpVal / 100 - tmpV[0] * 10);
		tmpV[2] = intToChar(tmpVal / 10 - tmpV[0] * 100 - tmpV[1] * 10);
		tmpV[3] = intToChar(tmpVal - tmpV[0] * 1000 - tmpV[1] * 100 - tmpV[2] * 10);
		return tmpV;
	}
	else
	{
		return menu[actualIndex]._name;
	}
}

uint8_t getMenuIndexByID(uint8_t id)
{ // Функція отримання індексу пункту меню за його ID
	for (uint8_t i = 0; i < menuArraySize; i++)
	{
		if (menu[i].id == id){return i;}
	}
	return -1;
}

uint8_t getNearMenuIndexByID(uint8_t parentid, uint8_t id, uint8_t side)
{					   // Функція отримання індексу пункту меню наступного або попереднього від актуального
	uint8_t prevID = -1;   // Змінна для зберігання індексу попереднього елемента
	uint8_t nextID = -1;   // Змінна для зберігання індексу наступного елемента
	uint8_t actualID = -1; // Змінна для зберігання індексу актуального елемента

	uint8_t firstID = -1; // Змінна для зберігання індексу першого елемента
	uint8_t lastID = -1;  // Змінна для зберігання індексу останнього елемента

	for (uint8_t i = 0; i < menuArraySize; i++)
	{
		if (menu[i].parentid == parentid)
		{ // Перебираємо всі елементи з одним батьківським ID
			if (firstID == -1)
				firstID = i; // Запам'ятовуємо перший елемент списку

			if (menu[i].id == id)
			{
				actualID = i; // Запам'ятовуємо актуальний елемент списку
			}
			else
			{
				if (actualID == -1)
				{ // Якщо зустрівся елемент до актуального, робимо його попереднім
					prevID = i;
				}
				else if (actualID != -1 && nextID == -1)
				{ // Якщо зустрівся елемент після актуального, робимо його наступним
					nextID = i;
				}
			}
			lastID = i; // Кожний наступний елемент - останній
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

void StartMusic(uint16_t melody)
{
	MusicStep = 0;
	PlayMusic = 1;
	sound(Music[MusicStep].freq, Music[MusicStep].time);
}

void sound(uint16_t freq, uint16_t time_ms)
{
	if (freq > 0)
	{
		TIM2->ARR = SYSCLK / TIM2->PSC / freq;
		TIM2->CCR1 = TIM2->ARR / 2;
	}
	else
	{
		TIM2->ARR = 1000;
		TIM2->CCR1 = 0;
	}
	TIM2->CNT = 0;

	sound_time = ((SYSCLK / TIM2->PSC / TIM2->ARR) * time_ms) / 1000;
	sound_counter = 0;
	SET_BIT(TIM2->CR1, TIM_CR1_CEN); // Запуск таймера;
}
/* Handlers--------------------------------------------------------*/
void SysTick_Handler(void)
{
	SysTimer_ms++;
	if (Delay_counter_ms)
	{
		Delay_counter_ms--;
	}
}

void EXTI0_1_IRQHandler(void)
{
    // Перевірка для DecrementButton (EXTI3)
    if (EXTI->PR & EXTI_PR_PR0)
    {
        EXTI->IMR &= ~EXTI_IMR_IM0;
		DecrementButtonDebounce = SysTimer_ms;
            if (!flagDecrementButtonDown)
            {
                timeDecrementButtonDown = SysTimer_ms;
                flagDecrementButtonDown = true;

                EXTI->RTSR &= ~EXTI_RTSR_RT3;
                EXTI->FTSR |= EXTI_FTSR_FT3;
            }
            else
            {
                uint8_t pressDecrementButtonDuration = SysTimer_ms - timeDecrementButtonDown;

                EXTI->FTSR &= ~EXTI_FTSR_FT0;
                EXTI->RTSR |= EXTI_RTSR_RT0;

                // *26		P_5.1	timeButtonPressed
                // *27 		P_5.2	timeButtonLongPressed
                if (pressDecrementButtonDuration < menu[27].value && pressDecrementButtonDuration > menu[26].value)
                {
                    flagDecrementButton = true;
                    if (flagPendingDecrementDoubleClick && (SysTimer_ms - timeLastDecrementButtonPress <= menu[28].value))// *28 		P_5.3	timeDoubleClick
                                        {
                                            flagDecrementButtonDouble = true;
                                            flagDecrementButton = false;
                                            flagPendingDecrementDoubleClick = false;
                                        }
                                        else
                                        {
                flagPendingDecrementDoubleClick = true;
                timeLastDecrementButtonPress = SysTimer_ms;
                }
                }
                else if (pressDecrementButtonDuration >= menu[27].value)
                {
                	flagDecrementButtonLong = true;
                }

                flagDecrementButtonDown = false;
            }

        EXTI->PR = EXTI_PR_PR0;
    }

    // Перевірка для EnterButton (EXTI1)
    if (EXTI->PR & EXTI_PR_PR1)
    {
        EXTI->IMR &= ~EXTI_IMR_IM1;
		EnterButtonDebounce = SysTimer_ms;
            if (!flagEnterButtonDown)
            {
                timeEnterButtonDown = SysTimer_ms;
                flagEnterButtonDown = true;

                EXTI->RTSR &= ~EXTI_RTSR_RT1;
                EXTI->FTSR |= EXTI_FTSR_FT1;
            }
            else
            {
                uint8_t pressEnterButtonDuration = SysTimer_ms - timeEnterButtonDown;

                EXTI->FTSR &= ~EXTI_FTSR_FT1;
                EXTI->RTSR |= EXTI_RTSR_RT1;

                // *26		P_5.1	timeButtonPressed
                // *27 		P_5.2	timeButtonLongPressed
                if (pressEnterButtonDuration > menu[26].value && pressEnterButtonDuration < menu[27].value)
                {
                     flagEnterButton = true;

                     if (flagPendingEnterDoubleClick && (SysTimer_ms - timeLastEnterButtonPress <= menu[28].value))// *28 		P_5.3	timeDoubleClick
                                         {
                                             flagEnterButtonDouble = true;
                                             flagEnterButton = false;
                                             flagPendingEnterDoubleClick = false;
                                         }
                                         else
                                         {
                                             flagPendingEnterDoubleClick = true;
                                             timeLastEnterButtonPress = SysTimer_ms;
                                         }
                }
                else if (pressEnterButtonDuration >= menu[27].value)
                {
                	flagEnterButtonLong = true;


                }

                flagEnterButtonDown = false;
            }

        EXTI->PR = EXTI_PR_PR1;
    }
}

void EXTI2_3_IRQHandler(void)
{
    // Перевірка, чи було переривання від лінії EXTI2
    if (EXTI->PR & EXTI_PR_PR2)
    {
        // Забороняємо переривання для даної лінії, поки не завершимо обробку
        EXTI->IMR &= ~EXTI_IMR_IM2;
            if ( !flagIncrementButtonDown )
            {
                // Початок натискання
                timeIncrementButtonDown = SysTimer_ms;
                flagIncrementButtonDown = true;

                // Переводимо переривання на спадаючий фронт для відстеження відпускання
                EXTI->RTSR &= ~EXTI_RTSR_RT2; // Вимикаємо переривання по зростаючому фронту
                EXTI->FTSR |= EXTI_FTSR_FT2;  // Включаємо переривання по спадаючому фронту
            }
            else
            {
                // Кнопка відпущена
                uint8_t pressIncrementButtonDuration = SysTimer_ms - timeIncrementButtonDown;

                // Переводимо переривання на зростаючий фронт для відстеження наступного натискання
				EXTI->FTSR &= ~EXTI_FTSR_FT2; // Вимикаємо переривання по спадаючому фронту
                EXTI->RTSR |= EXTI_RTSR_RT2;  // Включаємо переривання по зростаючому фронту
            
                // Обробка короткого та довгого натискання
                if (pressIncrementButtonDuration < menu[27].value && pressIncrementButtonDuration > menu[26].value)// *27 		P_5.2	timeButtonLongPressed
                {
                    //  Коротке натискання
                     flagIncrementButton= true;
                     // Обробка подвійного натискання
                     if (flagPendingIncrementDoubleClick && (SysTimer_ms - timeLastIncrementButtonPress <= menu[28].value))// *28 		P_5.3	timeDoubleClick
                     {
                         flagIncrementButtonDouble = true;
                         flagIncrementButton = false;
                         flagPendingIncrementDoubleClick = false;
                     }
                     else
                     {
                         flagPendingIncrementDoubleClick = true;
                         timeLastIncrementButtonPress = SysTimer_ms;
                     }
                }
                else if (pressIncrementButtonDuration > menu[27].value)
                {
                    //  Довге натискання
                	flagIncrementButtonLong = true;


                }

                // Скидаємо прапорець натискання
                flagIncrementButtonDown = false;
            }
        // Скидаємо прапорець EXTI2
        EXTI->PR = EXTI_PR_PR2;
    }
}

void EXTI4_15_IRQHandler(void)
{
    // Перевіряємо, чи було переривання від лінії EXTI9
    if (EXTI->PR & EXTI_PR_PR9)
    {
        // Забороняємо переривання для лінії EXTI9
        EXTI->IMR &= ~EXTI_IMR_IM9;

        // Перевірка стану піну 9 (припускаємо, що сигнал 0 – активний)
        if (BUTTON_PRESSED(9,GPIOB) == 0)  // Якщо сигнал 0
        {
            // Переводимо переривання на зростаючий фронт (чекаємо на сигнал 1)
            EXTI->RTSR |= EXTI_RTSR_RT9;   // Увімкнення переривання на зростаючий фронт
            EXTI->FTSR &= ~EXTI_FTSR_FT9;  // Вимкнення переривання на спадний фронт

            // Входимо в режим мінімального енергоспоживання
            LowPowerMode(ENTER);
        }
        else  // Якщо сигнал 1 (відновлення сигналу)
        {
            // Повертаємо контролер у нормальний режим роботи
            LowPowerMode(EXIT);

            // Переводимо переривання на спадний фронт (чекаємо на сигнал 0)
            EXTI->FTSR |= EXTI_FTSR_FT9;   // Увімкнення переривання на спадний фронт
            EXTI->RTSR &= ~EXTI_RTSR_RT9;  // Вимкнення переривання на зростаючий фронт
        }

        // Скидаємо прапорець переривання на лінії EXTI9
        EXTI->PR = EXTI_PR_PR9;

        // Увімкнення переривань після обробки
        EXTI->IMR |= EXTI_IMR_IM9;
    }
}

void TIM2_IRQHandler(void)
{
	if (READ_BIT(TIM2->SR, TIM_SR_UIF))
	{
		//		CounterTIM2++;
		CLEAR_BIT(TIM2->SR, TIM_SR_UIF); // Сбросим флаг прерывания
	}
}

void TIM21_IRQHandler(void)
{
	if (READ_BIT(TIM21->SR, TIM_SR_UIF))
	{
		//		CounterTIM2++;
		CLEAR_BIT(TIM21->SR, TIM_SR_UIF); // Сбросим флаг прерывания
	}
}

void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pouinter to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint8_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	   ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
