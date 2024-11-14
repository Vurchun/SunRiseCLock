#include "main.h"
// Макрос для налаштування GPIO
#define CONFIGURE_GPIO(PORT, PIN, MODE, TYPE, SPEED)                                         \
	MODIFY_REG(PORT->MODER, GPIO_MODER_MODE##PIN##_Msk, MODE << GPIO_MODER_MODE##PIN##_Pos); \
	MODIFY_REG(PORT->OTYPER, GPIO_OTYPER_OT_##PIN, TYPE << GPIO_OTYPER_OT_##PIN);            \
	MODIFY_REG(PORT->OSPEEDR, GPIO_OSPEEDER_OSPEED##PIN##_Msk, SPEED << GPIO_OSPEEDER_OSPEED##PIN##_Pos);

// Макрос для налаштування та ініціалізації EXTI (зовнішні переривання)
#define CONFIGURE_EXTI(PIN, PORT_SOURCE, CR, LINE)                                                                               \
	/* Налаштування лінії переривання на відповідний пін і порт */              \
	MODIFY_REG(SYSCFG->EXTICR[CR], SYSCFG_EXTICR##LINE##_EXTI##PIN##_Msk, PORT_SOURCE << SYSCFG_EXTICR##LINE##_EXTI##PIN##_Pos); \
	/* Встановлення маски переривання */                                                             \
	SET_BIT(EXTI->IMR, EXTI_IMR_IM##PIN);                                                                                        \
	/* Встановлення тригера на спадаючий фронт */                                             \
	SET_BIT(EXTI->FTSR, EXTI_FTSR_FT##PIN);

#define BUTTON_PRESSED(PIN, PORT) (PORT->IDR & GPIO_IDR_ID##PIN)

#define ENTER 1
#define EXIT 0

// Макроси для вмикання/вимикання різних світлодіодів (LEDs)
#define LEDa_ON() SET_BIT(GPIOA->BSRR, GPIO_BSRR_BS_7)
#define LEDa_OFF() SET_BIT(GPIOA->BSRR, GPIO_BSRR_BR_7)
#define LEDb_ON() SET_BIT(GPIOB->BSRR, GPIO_BSRR_BS_1)
#define LEDb_OFF() SET_BIT(GPIOB->BSRR, GPIO_BSRR_BR_1)
#define LEDc_ON() SET_BIT(GPIOA->BSRR, GPIO_BSRR_BS_6)
#define LEDc_OFF() SET_BIT(GPIOA->BSRR, GPIO_BSRR_BR_6)
#define LEDd_ON() SET_BIT(GPIOA->BSRR, GPIO_BSRR_BS_5)
#define LEDd_OFF() SET_BIT(GPIOA->BSRR, GPIO_BSRR_BR_5)
#define LEDe_ON() SET_BIT(GPIOA->BSRR, GPIO_BSRR_BS_11)
#define LEDe_OFF() SET_BIT(GPIOA->BSRR, GPIO_BSRR_BR_11)
#define LEDf_ON() SET_BIT(GPIOA->BSRR, GPIO_BSRR_BS_9)
#define LEDf_OFF() SET_BIT(GPIOA->BSRR, GPIO_BSRR_BR_9)
#define LEDg_ON() SET_BIT(GPIOB->BSRR, GPIO_BSRR_BS_0)
#define LEDg_OFF() SET_BIT(GPIOB->BSRR, GPIO_BSRR_BR_0)
#define LEDdp_ON() SET_BIT(GPIOB->BSRR, GPIO_BSRR_BS_3)
#define LEDdp_OFF() SET_BIT(GPIOB->BSRR, GPIO_BSRR_BR_3)

// Макроси для керування дисплеями
#define LEDD1_ON() SET_BIT(GPIOA->BSRR, GPIO_BSRR_BS_3)
#define LEDD1_OFF() SET_BIT(GPIOA->BSRR, GPIO_BSRR_BR_3)
#define LEDD2_ON() SET_BIT(GPIOA->BSRR, GPIO_BSRR_BS_4)
#define LEDD2_OFF() SET_BIT(GPIOA->BSRR, GPIO_BSRR_BR_4)
#define LEDD3_ON() SET_BIT(GPIOA->BSRR, GPIO_BSRR_BS_12)
#define LEDD3_OFF() SET_BIT(GPIOA->BSRR, GPIO_BSRR_BR_12)
#define LEDD4_ON() SET_BIT(GPIOB->BSRR, GPIO_BSRR_BS_4)
#define LEDD4_OFF() SET_BIT(GPIOB->BSRR, GPIO_BSRR_BR_4)

// Макроси для додаткових індикаторів
#define LEDl1l2_ON() SET_BIT(GPIOA->BSRR, GPIO_BSRR_BS_10)
#define LEDl1l2_OFF() SET_BIT(GPIOA->BSRR, GPIO_BSRR_BR_10)
#define LEDalarm_ON() SET_BIT(GPIOA->BSRR, GPIO_BSRR_BS_8)
#define LEDalarm_OFF() SET_BIT(GPIOA->BSRR, GPIO_BSRR_BR_8)

// Макроси для керування enable
#define pinEN_ON() SET_BIT(GPIOC->BSRR, GPIO_BSRR_BS_15)
#define pinEN_OFF() SET_BIT(GPIOC->BSRR, GPIO_BSRR_BR_15)

#define SYSCLK 32000000
#define bool uint8_t
#define true 1
#define false 0

// Змінні для збереження часу натискання для кожної кнопки
uint16_t timeIncrementButtonDown = 0;
uint16_t timeDecrementButtonDown = 0;
uint16_t timeEnterButtonDown = 0;

uint16_t IncrementButtonDebounce = 0;
uint16_t DecrementButtonDebounce = 0;
uint16_t EnterButtonDebounce = 0;

uint16_t timeLastIncrementButtonPress = 0;
uint16_t timeLastDecrementButtonPress = 0;
uint16_t timeLastEnterButtonPress = 0;

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

bool switchONDisplay = false;
bool lowpowerModeStatus = false;

#define menuArraySize 31	  // Встановлюємо розмір масиву меню
uint8_t actualIndex = 0;	  // Поточний індекс меню
bool isParamEditMode = false; // Прапорець режиму редагування параметра
uint8_t tmpVal = 0;			  // Тимчасова змінна для зберігання параметра


volatile uint32_t SysTimer_ms = 0;		// Системний таймер (аналог HAL_GetTick)
volatile uint16_t Delay_counter_ms = 0; // Лічильник для затримки
uint32_t timeWakeUp =0;
uint32_t timeNow =0;

char tmpV[4] = {};
char tmpClock[4] = {};
int8_t vmenu = 0; // Змінна, що зберігає дію по вертикалі 1 - вхід в меню, -1 - вихід з меню
int8_t hmenu = 0; // Змінна, що зберігає дію по горизонталі 1 - вправо, -1 - вліво
char *tmpValue;

// Структура меню
struct strMenu
{
	int8_t id;		 // Унікальний ідентифікаційний індекс ID
	int8_t parentid; // ID батька (вкладеність)
	bool isParam;	 // Чи є пункт змінним параметром
	char _name[4];	 // Назва пункту меню
	uint8_t value;	 // Поточне значення параметра
	uint8_t _min;	 // Мінімально можливе значеннял
	uint8_t _max;	 // Максимально можливе значення
};
/*0 PPPP
 *1 	P__0		Time_Now
 *2 		P_0.0	Hour_Now
 *3 		P_0.1	Minute_Now
 *4			P_0.2	Seconds_Now
 *5 		P_0.3	Day_Now
 *6 		P_0.4	Month_Now
 *7 		P_0.5	Year_Now
 *8 		P_0.6	WeekDay_Nows
 *9 		P_0.7	Set
 *10 	P__1		Time_Rise
 *11 		P_1.0	Hour_Rise
 *12 		P_1.1	Minute_Rise
 *13 	P__2		Rising_Parametrs
 *14 		P_2.0	Period_Rising
 *15 		P_2.1	ɣ_Coefient_Rising
 *16 		P_2.2	Test lamp
 *17 	P__3		Alarm_Parametrs
 *18 		P_3.0	Alarm_Status
 *19		P_3.1	Alarm_Hours
 *20		P_3.2	Alarm_Minutes
 *21 		P_3.3	Alarm_Melody
 *22 		P_3.4	Alarm_Melody_test
 *23 	P__4		Menu_Parametrs
 *24		P_4.0	Numbers_Change_Style
 *25 		P_4.1	Menu_Night_Mode
 *26 		P_4.2	Menu_Night_Mode_delay
 *27 	P__5		Menu_Parametrs
 *28 		P_5.0	debounceTime
 *29 		P_5.2	timeButtonLongPressed
 *30 	P__6		Clock(StartWork)
 */
struct strMenu menu[] = {
	// Встановлюємо пункти меню
	{0, -1, false, "PPPP", 0, 0, 0},
	//-----------------------------------------------------------------------
	{1, 0, false,"P_0_", 0, 0, 0},
	{2, 1, true, "P_00", 5, 0, 24},
	{3, 1, true, "P_01", 30, 0, 59},
	{4, 1, true, "P_02", 0, 0, 59},
	{5, 1, true, "P_03", 1, 1, 31},
	{6, 1, true, "P_04", 1, 1, 12},
	{7, 1, true, "P_05", 0, 0, 231},
	{8, 1, true, "P_06", 1, 1, 7},
	{9, 1, false,"P_07", 0, 0, 0},
	//-----------------------------------------------------------------------
	{10, 0, false, "P_1_", 0, 0, 0},
	{11, 10, true, "P_10", 5, 0, 23},
	{12, 10, true, "P_11", 45, 0, 59},
	//-----------------------------------------------------------------------
	{13, 0, false, "P_2_", 0, 0, 0},
	{14, 13, true, "P_20", 15, 0, 231},
	{15, 13, true, "P_21", 224, 224, 224},
	{16, 13,false, "P_22", 0, 0, 0},
	//-----------------------------------------------------------------------
	{17, 0, false, "P_3_", 0, 0, 0},
	{18, 17, true, "P_30", 1, 0, 1},
	{19, 17, true, "P_31", 5, 0, 24},
	{20, 17, true, "P_32", 50, 0, 59},
	{21, 17, true, "P_33", 0, 0, 7},
	{22, 17, false,"P_34", 0, 0, 0},
	//-----------------------------------------------------------------------
	{23, 0, false, "P_4_", 0, 0, 1},
	{24, 23, true, "P_40", 0, 0, 3},
	{25, 23, true, "P_41", 0, 0, 1},
	{26, 23, true, "P_42", 5, 0, 231},
	//-----------------------------------------------------------------------
	{27, 0, false, "P_5_", 0, 0, 0},
	{28, 27, true, "P_50", 30, 0, 231},
	{29, 27, true, "P_51", 100, 0, 231},
	//-----------------------------------------------------------------------
	{30, 0, false, "P_6_", 0, 0, 0}
	//-----------------------------------------------------------------------
};

/* Private function prototypes -----------------------------------------------*/
void CMSIS_FullInit(void);
uint8_t SysTickTimerInit(uint32_t ticks);
void SystemClock_Config(void);
void GPIO_Init(void);
void RTC_Init(void);
void TIM2_Init(void);
void TIM21_Init(void);
void interaptTIMDebounce(void);
/*----------------------------------------------------------------------------*/
//void Reset_Handler(void);
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
double pow224(int i);
uint16_t map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max);
/*----------------------------------------------------------------------------*/
void pwmFP7103();
void testMainLamp(void);
/*----------------------------------------------------------------------------*/
uint8_t secondsDecimal();
uint8_t minutesDecimal();
uint8_t hoursDecimal();
void setTimeNow();
uint8_t Clock();
/*----------------------------------------------------------------------------*/
char intToChar(uint8_t num);
void writeCHARSEG(char CHAR, uint8_t seg);
char *setActualMenu(int8_t v, int8_t h);
uint8_t getMenuIndexByID(int8_t id);
uint8_t getNearMenuIndexByID(int8_t parentid, int8_t id, int8_t side);
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
int main(void)
{
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
		vmenu = 0; // Скидання вертикального флагу після обробки
		hmenu = 0; // Скидання горизонтального флагу після обробки


		LEDl1l2_OFF();
		LEDalarm_OFF();
		pinEN_OFF();


		interaptTIMDebounce();

		if (flagDecrementButton)
		{
			hmenu = -1;					 // Переміщення по меню вниз
			flagDecrementButton = false; // Скидаємо флаг
		}
		if (flagDecrementButtonLong)
		{
			hmenu = -5; // Швидке переміщення по меню вниз (довге натискання)
			flagDecrementButtonLong = false;
		}
		if (flagIncrementButton)
		{
			hmenu = 1; // Переміщення по меню вгору
			flagIncrementButton = false;
		}
		if (flagIncrementButtonLong)
		{
			hmenu = 5; // Швидке переміщення по меню вгору (довге натискання)
			flagIncrementButtonLong = false;
		}
		if (flagEnterButton)
		{
			vmenu = 1; // Вхід у підменю або редагування параметра
			flagEnterButton = false;
		}
		if (flagEnterButtonLong)
		{
			vmenu = -1; // Вихід з підменю або відміна редагування
			flagEnterButtonLong = false;
		}

		if (vmenu != 0 || hmenu != 0)
		{
			tmpValue = setActualMenu(vmenu, hmenu); // Оновлюємо меню, якщо було натискання
		}

		if (SysTimer_ms % 4 == 0)
			writeCHARSEG(tmpValue[0], 0);
		if (SysTimer_ms % 4 == 1)
			writeCHARSEG(tmpValue[1], 1);
		if (SysTimer_ms % 4 == 2)
			writeCHARSEG(tmpValue[2], 2);
		if (SysTimer_ms % 4 == 3)
			writeCHARSEG(tmpValue[3], 3);
	}
	return 0;
}
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
uint8_t SysTickTimerInit(uint32_t ticks)
{

	if ((ticks - 1UL) > SysTick_LOAD_RELOAD_Msk)
	{
		return (1UL); /* Reload value impossible */
	}
	CLEAR_BIT(SysTick->CTRL, SysTick_CTRL_ENABLE_Msk);				 /* Disenable SysTick Timer */
	SysTick->LOAD = (uint32_t)(ticks - 1UL);						 /* set reload register */
	NVIC_SetPriority(SysTick_IRQn, (1UL << __NVIC_PRIO_BITS) - 1UL); /* set Priority for Systick uint16_terrupt */
	SysTick->VAL = 0UL;												 /* Load the SysTick Counter Value */
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
					SysTick_CTRL_TICKINT_Msk |
					SysTick_CTRL_ENABLE_Msk; /* Enable SysTick IRQ and SysTick Timer */
	return (0UL);							 /* Function successful */
}

void CMSIS_FullInit(void)
{
	// *** Налаштування кешу, передвибірки і попереднього читання *** //

	// Вимкнути буфер кешу, якщо це налаштовано
	CLEAR_BIT(FLASH->ACR, FLASH_ACR_DISAB_BUF);
	// Включити попереднє читання, якщо це налаштовано
	SET_BIT(FLASH->ACR, FLASH_ACR_PRE_READ);
	// Включити буфер передвибірки, якщо це налаштовано
	SET_BIT(FLASH->ACR, FLASH_ACR_PRFTEN);
	SET_BIT(FLASH->ACR, FLASH_ACR_LATENCY);

	// *** Налаштування SysTick для переривань кожну 1 мс *** //

	uint32_t ticks = SYSCLK / 1000U; // Розрахунок кількості тактів для 1 мс

	// Використовуємо SysTick_Config для налаштування таймера
	if (ticks > SysTick_LOAD_RELOAD_Msk) // Якщо кількість тактів більше дозволеного
	{
		while (1)
			; // Помилка, зациклюємося
	}

	SysTickTimerInit(ticks);

	// Встановлення пріоритету для переривання SysTick
	uint32_t tickPriority = 0; // Пріоритет для SysTick (без макросів)
	if (tickPriority < (1UL << __NVIC_PRIO_BITS))
	{
		NVIC_SetPriority(SysTick_IRQn, tickPriority);
	}
	else
	{
		while (1)
			; // Помилка пріоритету
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

    // 4. Set LSE as RTC clock source and enable RTC
    MODIFY_REG(RCC->CSR, RCC_CSR_RTCSEL_Msk, (0b01 << RCC_CSR_RTCSEL_Pos)); // Select LSE as RTC clock
    SET_BIT(RCC->CSR, RCC_CSR_RTCEN);         // Enable RTC

    // 5. Disable RTC write protection
    RTC->WPR = 0xCA;  // Write protection key 1
    RTC->WPR = 0x53;  // Write protection key 2

    // 6. Enter initialization mode
    SET_BIT(RTC->ISR, RTC_ISR_INIT);
    while (!(READ_BIT(RTC->ISR, RTC_ISR_INITF))) {} // Wait until initialization mode is ready

    // 7. Set the time in BCD format (17:36:00)
    MODIFY_REG(RTC->TR,
               RTC_TR_HT_Msk | RTC_TR_HU_Msk | RTC_TR_MNT_Msk | RTC_TR_MNU_Msk | RTC_TR_ST_Msk | RTC_TR_SU_Msk,
               (0x1 << RTC_TR_HT_Pos) |  // Hour tens (1 -> 17)
               (0x7 << RTC_TR_HU_Pos) |  // Hour units (7 -> 17)
               (0x3 << RTC_TR_MNT_Pos) | // Minute tens (3 -> 36)
               (0x6 << RTC_TR_MNU_Pos) | // Minute units (6 -> 36)
               (0x0 << RTC_TR_ST_Pos) |  // Second tens (0 -> 00)
               (0x0 << RTC_TR_SU_Pos));  // Second units (0 -> 00)

    // 8. Set the date in BCD format (01/02/2024, Monday)
    MODIFY_REG(RTC->DR,
               RTC_DR_YT_Msk | RTC_DR_YU_Msk | RTC_DR_MT_Msk | RTC_DR_MU_Msk | RTC_DR_DT_Msk | RTC_DR_DU_Msk | RTC_DR_WDU_Msk,
               (0x2 << RTC_DR_YT_Pos) |  // Year tens (2 -> 2024)
               (0x4 << RTC_DR_YU_Pos) |  // Year units (4 -> 2024)
               (0x0 << RTC_DR_MT_Pos) |  // Month tens (1 -> April)
               (0x4 << RTC_DR_MU_Pos) |  // Month units (0 -> April)
               (0x0 << RTC_DR_DT_Pos) |  // Day tens (0 -> 01)
               (0x1 << RTC_DR_DU_Pos) |  // Day units (1 -> 01)
               (0x1 << RTC_DR_WDU_Pos)); // Weekday (1 -> Monday)

    // 9. Exit initialization mode
    CLEAR_BIT(RTC->ISR, RTC_ISR_INIT);

    // 10. Re-enable RTC write protection
    RTC->WPR = 0xFE;  // Write protection key 1
    RTC->WPR = 0x64;  // Write protection key 2
}

void TIM2_Init(void)
{

	// Увімкнення тактування GPIOA (для PA15, як PWM вихід)
	RCC->IOPENR |= RCC_IOPENR_IOPAEN;
	// Настроить пин 15 на режим альтернативной функции
	CONFIGURE_GPIO(GPIOA, 15, 0b10, 0, 0b11); // BuzzerPin
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

	TIM2->PSC = 32 - 1;
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

	CLEAR_BIT(TIM2->CR1, TIM_CR1_CEN);
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
	TIM21->PSC = 32 - 1;
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

	CLEAR_BIT(TIM21->CR1, TIM_CR1_CEN);
}

void GPIO_Init(void)
{
	// Включення тактування портів A, B, C
	RCC->IOPENR |= RCC_IOPENR_IOPAEN | RCC_IOPENR_IOPBEN | RCC_IOPENR_IOPCEN;

	// Налаштування світлодіодів (виводи PA, PB)
	CONFIGURE_GPIO(GPIOA, 7, 0b01, 0, 0b00);  // LEDa
	CONFIGURE_GPIO(GPIOB, 1, 0b01, 0, 0b00);  // LEDb
	CONFIGURE_GPIO(GPIOA, 6, 0b01, 0, 0b00);  // LEDc
	CONFIGURE_GPIO(GPIOA, 5, 0b01, 0, 0b00);  // LEDd
	CONFIGURE_GPIO(GPIOA, 11,0b01, 0, 0b00);  // LEDe
	CONFIGURE_GPIO(GPIOA, 9, 0b01, 0, 0b00);  // LEDf
	CONFIGURE_GPIO(GPIOB, 0, 0b01, 0, 0b00);  // LEDg
	CONFIGURE_GPIO(GPIOB, 3, 0b01, 0, 0b00);  // LEDdp
	CONFIGURE_GPIO(GPIOA, 3, 0b01, 0, 0b00);  // LEDD1
	CONFIGURE_GPIO(GPIOA, 4, 0b01, 0, 0b00);  // LEDD2
	CONFIGURE_GPIO(GPIOA, 12,0b01, 0, 0b00);  // LEDD3
	CONFIGURE_GPIO(GPIOB, 4, 0b01, 0, 0b00);  // LEDD4
	CONFIGURE_GPIO(GPIOA, 10,0b01, 0, 0b00);  // LEDl1l2
	CONFIGURE_GPIO(GPIOA, 8, 0b01, 0, 0b00);  // LEDalarm
	CONFIGURE_GPIO(GPIOC, 15,0b01, 0, 0b00);  // pinEN

	// Налаштування кнопок із EXTI
	CONFIGURE_GPIO(GPIOA, 0, 0b00, 0, 0b11); // decrement
	CONFIGURE_EXTI(0, 0b000, 0, 1);			 // EXTI для decrement

	CONFIGURE_GPIO(GPIOA, 1, 0b00, 0, 0b11); // enter
	CONFIGURE_EXTI(1, 0b000, 0, 1);			 // EXTI для enter

	CONFIGURE_GPIO(GPIOA, 2, 0b00, 0, 0b11); // increment
	CONFIGURE_EXTI(2, 0b000, 0, 1);			 // EXTI для increment

	CONFIGURE_GPIO(GPIOB, 9, 0b00, 0, 0b11); // pwr
	CONFIGURE_EXTI(9, 0b000, 2, 3);			 // EXTI для pwr з обробкою по зростанню

	/* Включення переривання */
	NVIC_EnableIRQ(EXTI0_1_IRQn);
	NVIC_EnableIRQ(EXTI2_3_IRQn);
	NVIC_EnableIRQ(EXTI4_15_IRQn);
}

void LowPowerMode(uint8_t status)
{
	if (status)
	{
		lowpowerModeStatus = true;
		// Встановлюємо режим STOP з RTC працюючим у нормальному режимі
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
	}
	else
	{
		lowpowerModeStatus = false;
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

// Функція для обчислення факторіала
double pow224(int i) {
	return 0.00000000020754957593*i*i*i + 0.00000084626309852429*i*i - 0.00005473036906744611*i + 0.00203250994804093921;
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
	case '?':
		LEDa_ON();
		LEDb_ON();
		LEDc_OFF();
		LEDd_OFF();
		LEDe_ON();
		LEDf_ON();
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

uint16_t map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max) {
      return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

void pwmFP7103()
{
	if (menu[18].value)
	{		//*16 		P_3.0	Alarm_Status
		timeWakeUp 	= menu[11].value * 3600	 	//*11 		P_1.0	Hour_Rise
					+ menu[12].value * 60		//*12 		P_1.1	Minute_Rise
					+ 1 ;
		timeNow = hoursDecimal() * 3600 + minutesDecimal() * 60 + secondsDecimal() + 1;
		if (menu[14].value * 60 >= (timeWakeUp - timeNow))
		{ 	// *14 		P_2.0	Period_Rising
			// *15 		P_2.1	ɣ_Coefient_Rising
			pinEN_ON();
			SET_BIT(TIM21->CR1, TIM_CR1_CEN); // Запуск таймера
			TIM21->CCR1 = (uint16_t)((TIM21->ARR + 1)*pow224(map((uint16_t) (10000*((double)timeNow /timeWakeUp)),
					(uint16_t)(((double)(timeWakeUp - menu[14].value * 60) /timeWakeUp) * (10000)),
					(10000),
					0,
					(1000))));
		}
		if (!(menu[14].value * 60 >= (timeWakeUp - timeNow)) && (flagDecrementButtonLong || flagIncrementButtonLong))
		{
			pinEN_OFF();
			CLEAR_BIT(TIM21->CR1, TIM_CR1_CEN); // Призупинення таймера
			TIM21->CCR1 = 0;
		}
	}
	else
	{
		pinEN_OFF();
		CLEAR_BIT(TIM21->CR1, TIM_CR1_CEN); // Призупинення таймера
		TIM21->CCR1 = 0;
	}
}

void testMainLamp(void){
	pinEN_ON();
	SET_BIT(TIM21->CR1, TIM_CR1_CEN); // Запуск таймера
	// *15 		P_2.2	ɣ_Coefient_Rising
	while(SysTimer_ms % 10000 != 9999){
		if (SysTimer_ms % 10000 < 1000)	TIM21->CCR1 = 0;
		if (SysTimer_ms % 10000 < 3000)	TIM21->CCR1 = 3000;
		if (SysTimer_ms % 10000 < 6000)	TIM21->CCR1 = 6000;
		if (SysTimer_ms % 10000 < 9000)	TIM21->CCR1 = 9000;
	}
	TIM21->CCR1 = 0;
	pinEN_OFF();
	CLEAR_BIT(TIM21->CR1, TIM_CR1_CEN); // Зупинення таймера
}

uint8_t hoursDecimal(){
    // Extract HT (hour tens) and HU (hour units)
    uint8_t hoursBCD = ((READ_BIT(RTC->TR, RTC_TR_HT | RTC_TR_HU)) >> RTC_TR_HU_Pos) & 0xFF;
    // Convert BCD to decimal
    return ((hoursBCD >> 4) & 0xF) * 10 + (hoursBCD & 0xF);
}

uint8_t minutesDecimal(){
    // Extract MNT (minute tens) and MNU (minute units) fields from the TR register in BCD format
    uint8_t minutesBCD = ((READ_BIT(RTC->TR, RTC_TR_MNT | RTC_TR_MNU)) >> RTC_TR_MNU_Pos) & 0xFF;
    // Convert BCD to decimal
    return ((minutesBCD >> 4) & 0xF) * 10 + (minutesBCD & 0xF);
}

uint8_t secondsDecimal(){
    // Extract MNT (minute tens) and MNU (minute units) fields from the TR register in BCD format
    uint8_t secondsBCD = ((READ_BIT(RTC->TR, RTC_TR_ST | RTC_TR_SU)) >> RTC_TR_SU_Pos) & 0xFF;
    // Convert BCD to decimal
    return ((secondsBCD >> 4) & 0xF) * 10 + (secondsBCD & 0xF);
}

uint8_t Clock()
{
	uint8_t i = 0;
	uint8_t j = 0;
	interaptTIMDebounce();

	tmpClock[0] = intToChar(hoursDecimal()/10);
	if (tmpClock[0] == '0')
		i = 1;
	tmpClock[1] = intToChar(hoursDecimal()%10);
	tmpClock[2] = intToChar(minutesDecimal()/10);
	if (tmpClock[2] == '0')
		j = 1;
	tmpClock[3] = intToChar(minutesDecimal()%10);

	if (flagDecrementButton || flagEnterButton || flagIncrementButton) switchONDisplay = true;
	//	 *24 		P_4.2	Menu_Night_Mode_delay
	if (SysTimer_ms % menu[24].value == menu[26].value-1) switchONDisplay = false;
	//	 *24 		P_4.2	Menu_Night_Mode
	if ( (hoursDecimal()>5 && hoursDecimal() <22) || switchONDisplay || menu[24].value == 0)
	{
		(menu[18].value == 1)?LEDalarm_ON():LEDalarm_OFF();

		if (SysTimer_ms % 4 == 0 && i == 0)
			writeCHARSEG(tmpClock[0], 0);
		if (SysTimer_ms % 4 == 1)
			writeCHARSEG(tmpClock[1], 1);
		if (SysTimer_ms % 4 == 2 && j == 0)
			writeCHARSEG(tmpClock[2], 2);
		if (SysTimer_ms % 4 == 3)
			writeCHARSEG(tmpClock[3], 3);

		(SysTimer_ms % 2000 <= 1000)?LEDl1l2_ON():LEDl1l2_OFF();

		pwmFP7103();
	}
	return (flagEnterButtonLong) ? 0 : 1;
}

void setTimeNow()
{
	// Disable RTC write protection
	RTC->WPR = 0xCA; // Step 1
	RTC->WPR = 0x53; // Step 2

	// Enter initialization mode
	SET_BIT(RTC->ISR, RTC_ISR_INIT);
	while (!(READ_BIT(RTC->ISR, RTC_ISR_INITF)))
	{
	}

	MODIFY_REG(RTC->TR,
			   RTC_TR_HT_Msk | RTC_TR_HU_Msk | RTC_TR_MNT_Msk | RTC_TR_MNU_Msk | RTC_TR_ST_Msk | RTC_TR_SU_Msk,
			   	   (menu[2].value / 10 << RTC_TR_HT_Pos) |	// Hour tens
				   (menu[2].value % 10 << RTC_TR_HU_Pos) |	// Hour units
				   (menu[3].value / 10 << RTC_TR_MNT_Pos)|  // Minute tens
				   (menu[3].value % 10 << RTC_TR_MNU_Pos)|  // Minute units
				   (menu[4].value / 10 << RTC_TR_ST_Pos) |	// Second tens
				   (menu[4].value % 10 << RTC_TR_SU_Pos));	// Second units
	MODIFY_REG(RTC->DR,
			   RTC_DR_YT_Msk | RTC_DR_YU_Msk | RTC_DR_MT_Msk | RTC_DR_MU_Msk | RTC_DR_DT_Msk | RTC_DR_DU_Msk | RTC_DR_WDU_Msk,
			   	   (menu[7].value / 10 << RTC_DR_YT_Pos) | // Year tens
				   (menu[7].value % 10 << RTC_DR_YU_Pos) | // Year units
				   (menu[6].value / 10 << RTC_DR_MT_Pos) | // Month tens
				   (menu[6].value % 10 << RTC_DR_MU_Pos) | // Month units
				   (menu[5].value / 10 << RTC_DR_DT_Pos) | // Day tens
				   (menu[5].value % 10 << RTC_DR_DU_Pos) | // Day units
				   (menu[8].value << RTC_DR_WDU_Pos));	   // Weekday

	// Exit initialization mode
	CLEAR_BIT(RTC->ISR, RTC_ISR_INIT);

	// Re-enable RTC write protection
	RTC->WPR = 0xFE; // Disable write access for RTC register
	RTC->WPR = 0x64; //
}

char *setActualMenu(int8_t v, int8_t h)
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
			{									  // Якщо не в режимі редагування, то ...
				isParamEditMode = true;			  // Переходимо в режим редагування параметра
				tmpVal = menu[actualIndex].value; // Тимчасовій змінній присвоюємо актуальне значення параметра
			}
			else if (menu[actualIndex].isParam && isParamEditMode)
			{									  // Якщо в режимі редагування
				menu[actualIndex].value = tmpVal; // Зберігаємо задане значення
				isParamEditMode = false;		  // І виходимо з режиму редагування
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
					{
					case 9: 						//  *9 		P_0.7	Set
						setTimeNow();
						break;
					case 16: 						 //*16 		P_2.2	Test lamp
						testMainLamp();
						break;
					case 22:						// *22 		P_3.4	Alarm_Melody_test
						SET_BIT(TIM2->CR1, TIM_CR1_CEN); // Запуск таймера;
						while (!(SysTimer_ms%10000 == 9999)){StartMusic(menu[21].value);} // *21 		P_3.3	Alarm_Melody
						CLEAR_BIT(TIM2->CR1, TIM_CR1_CEN); // Stop таймера;
						break;
					case 30:
						while (Clock());			//*29 	P__6		Clock(StartWork)
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
		{				 // У режимі редагування параметра
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
		tmpV[0] = ((tmpVal / 1000) % 10 !=0) 								? intToChar((tmpVal / 1000) % 10) : ' ';
		tmpV[1] = ((tmpVal / 100) % 10 !=0 || (tmpVal / 1000) % 10 !=0) 	? intToChar((tmpVal / 100) % 10) : ' ';
		tmpV[2] = ((tmpVal / 10) % 10 !=0 || (tmpVal / 100) % 10 !=0)		? intToChar((tmpVal / 10) % 10) : ' ';
		tmpV[3] = 															  intToChar(tmpVal % 10);
		return tmpV;
	}
	else
	{
		return menu[actualIndex]._name;
	}
}

uint8_t getMenuIndexByID(int8_t id)
{ // Функція отримання індексу пункту меню за його ID
	for (uint8_t i = 0; i < menuArraySize; i++)
	{
		if (menu[i].id == id)
		{
			return i;
		}
	}
	return -1;
}

uint8_t getNearMenuIndexByID(int8_t parentid, int8_t id, int8_t side)
{						  // Функція отримання індексу пункту меню наступного або попереднього від актуального
	int8_t prevID = -1;	  // Змінна для зберігання індексу попереднього елемента
	int8_t nextID = -1;	  // Змінна для зберігання індексу наступного елемента
	int8_t actualID = -1; // Змінна для зберігання індексу актуального елемента

	int8_t firstID = -1; // Змінна для зберігання індексу першого елемента
	int8_t lastID = -1;	 // Змінна для зберігання індексу останнього елемента

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
	TIM21->ARR = 99;
	TIM21->CCR1 = 49;
}

void interaptTIMDebounce(void)
{
// *26 		P_5.1	debounceTime
if (((SysTimer_ms - DecrementButtonDebounce) > menu[28].value) && (READ_BIT(EXTI->IMR, EXTI_IMR_IM0) == 0))
	SET_BIT(EXTI->IMR, EXTI_IMR_IM0);
if (((SysTimer_ms - EnterButtonDebounce) > menu[28].value) && (READ_BIT(EXTI->IMR, EXTI_IMR_IM1) == 0))
	SET_BIT(EXTI->IMR, EXTI_IMR_IM1);
if (((SysTimer_ms - IncrementButtonDebounce) > menu[28].value) && (READ_BIT(EXTI->IMR, EXTI_IMR_IM2) == 0))
	SET_BIT(EXTI->IMR, EXTI_IMR_IM2);
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
		CLEAR_BIT(EXTI->IMR, EXTI_IMR_IM0);
		DecrementButtonDebounce = SysTimer_ms;

		if (!flagDecrementButtonDown)
		{
			timeDecrementButtonDown = SysTimer_ms;
			flagDecrementButtonDown = true;

			CLEAR_BIT(EXTI->FTSR, EXTI_FTSR_FT0);
			SET_BIT(EXTI->RTSR, EXTI_RTSR_RT0);
		}
		else
		{
			flagDecrementButtonDown = false;
			CLEAR_BIT(EXTI->RTSR, EXTI_RTSR_RT0);
			SET_BIT(EXTI->FTSR, EXTI_FTSR_FT0);

			// *26 		P_5.1	timeButtonLongPressed
			if (SysTimer_ms - timeDecrementButtonDown > 4 * menu[29].value)
			{
				flagDecrementButton = false;
				flagDecrementButtonLong = true;
			}
			else
			{
				flagDecrementButtonLong = false;
				flagDecrementButton = true;
			}
		}

		EXTI->PR = EXTI_PR_PR0;
	}

	// Перевірка для EnterButton (EXTI1)
	if (EXTI->PR & EXTI_PR_PR1)
	{
		CLEAR_BIT(EXTI->IMR, EXTI_IMR_IM1);
		EnterButtonDebounce = SysTimer_ms;

		if (!flagEnterButtonDown)
		{
			timeEnterButtonDown = SysTimer_ms;
			flagEnterButtonDown = true;

			CLEAR_BIT(EXTI->FTSR, EXTI_FTSR_FT1);
			SET_BIT(EXTI->RTSR, EXTI_RTSR_RT1);
		}
		else
		{
			flagEnterButtonDown = false;
			CLEAR_BIT(EXTI->RTSR, EXTI_RTSR_RT1);
			SET_BIT(EXTI->FTSR, EXTI_FTSR_FT1);

			// *26 		P_5.1	timeButtonLongPressed
			if (SysTimer_ms - timeEnterButtonDown > 4 * menu[29].value)
			{
				flagEnterButton = false;
				flagEnterButtonLong = true;
			}
			else
			{
				flagEnterButtonLong = false;
				flagEnterButton = true;
			}
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
		CLEAR_BIT(EXTI->IMR, EXTI_IMR_IM2);
		IncrementButtonDebounce = SysTimer_ms;

		if (!flagIncrementButtonDown)
		{
			// Початок натискання
			timeIncrementButtonDown = SysTimer_ms;
			flagIncrementButtonDown = true;

			// Переводимо переривання на спадаючий фронт для відстеження відпускання
			CLEAR_BIT(EXTI->FTSR, EXTI_FTSR_FT2); // Включаємо переривання по спадаючому фронту
			SET_BIT(EXTI->RTSR, EXTI_RTSR_RT2);	  // Вимикаємо переривання по зростаючому фронту
		}
		else
		{
			// Кнопка відпущена
			// Скидаємо прапорець натискання
			flagIncrementButtonDown = false;
			// Переводимо переривання на зростаючий фронт для відстеження наступного натискання
			CLEAR_BIT(EXTI->RTSR, EXTI_RTSR_RT2); // Включаємо переривання по зростаючому фронту
			SET_BIT(EXTI->FTSR, EXTI_FTSR_FT2);	  // Вимикаємо переривання по спадаючому фронту
			// Обробка короткого та довгого натискання
			if (SysTimer_ms - timeIncrementButtonDown > 4 * menu[29].value) // *27 		P_5.2	timeButtonLongPressed
			{
				//  Довге натискання
				flagIncrementButtonLong = true;
				flagIncrementButton = false;
			}
			else
			{
				//  Коротке натискання
				flagIncrementButton = true;
				flagIncrementButtonLong = false;
			}
		}
		// Скидаємо прапорець EXTI2
		EXTI->PR = EXTI_PR_PR2;
	}
}

/*
 *
 *Проверить сигнал пвр
 *
 */

void EXTI4_15_IRQHandler(void)
{
	// Перевіряємо, чи було переривання від лінії EXTI9
	if (EXTI->PR & EXTI_PR_PR9)
	{
		// Забороняємо переривання для лінії EXTI9
		EXTI->IMR &= ~EXTI_IMR_IM9;

		// Перевірка стану піну 9 (припускаємо, що сигнал 0 – активний)
		if (BUTTON_PRESSED(9, GPIOB) == 0) // Якщо сигнал 0
		{
			// Переводимо переривання на зростаючий фронт (чекаємо на сигнал 1)
			EXTI->RTSR |= EXTI_RTSR_RT9;  // Увімкнення переривання на зростаючий фронт
			EXTI->FTSR &= ~EXTI_FTSR_FT9; // Вимкнення переривання на спадний фронт

			// Входимо в режим мінімального енергоспоживання
			LowPowerMode(ENTER);
		}
		else // Якщо сигнал 1 (відновлення сигналу)
		{
			// Повертаємо контролер у нормальний режим роботи
			LowPowerMode(EXIT);

			// Переводимо переривання на спадний фронт (чекаємо на сигнал 0)
			EXTI->FTSR |= EXTI_FTSR_FT9;  // Увімкнення переривання на спадний фронт
			EXTI->RTSR &= ~EXTI_RTSR_RT9; // Вимкнення переривання на зростаючий фронт
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
