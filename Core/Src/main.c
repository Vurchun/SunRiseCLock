#include "main.h" // Включення основного заголовного файлу проекту.

#define bool uint8_t // Визначення типу bool як uint8_t.
#define true 1 // Визначення значення true як 1.
#define false 0 // Визначення значення false як 0.

// Макрос для налаштування GPIO
#define CONFIGURE_GPIO(PORT, PIN, MODE, TYPE, SPEED)                                         \
    MODIFY_REG(PORT->MODER, GPIO_MODER_MODE##PIN##_Msk, MODE << GPIO_MODER_MODE##PIN##_Pos); \
    MODIFY_REG(PORT->OTYPER, GPIO_OTYPER_OT_##PIN, TYPE << GPIO_OTYPER_OT_##PIN);            \
    MODIFY_REG(PORT->OSPEEDR, GPIO_OSPEEDER_OSPEED##PIN##_Msk, SPEED << GPIO_OSPEEDER_OSPEED##PIN##_Pos);

// Макрос для налаштування та ініціалізації EXTI (зовнішні переривання)
#define CONFIGURE_EXTI(PIN, PORT_SOURCE, CR, LINE)                                                                               \
    /* Налаштування лінії переривання на відповідний пін і порт */                                                               \
    MODIFY_REG(SYSCFG->EXTICR[CR], SYSCFG_EXTICR##LINE##_EXTI##PIN##_Msk, PORT_SOURCE << SYSCFG_EXTICR##LINE##_EXTI##PIN##_Pos); \
    /* Встановлення маски переривання */                                                                                         \
    SET_BIT(EXTI->IMR, EXTI_IMR_IM##PIN);                                                                                        \
    /* Встановлення тригера на спадаючий фронт */                                                                                \
    SET_BIT(EXTI->FTSR, EXTI_FTSR_FT##PIN);

#define BUTTON_PRESSED(PIN, PORT) (PORT->IDR & GPIO_IDR_ID##PIN) // Макрос для перевірки натискання кнопки.

#define SYSCLK 32000000 // Встановлення системної тактової частоти на 32 МГц.
#define ENTER 1 // Визначення значення ENTER як 1.
#define EXIT 0 // Визначення значення EXIT як 0.

// Макроси для вмикання/вимикання різних світлодіодів (LEDs)
#define LEDa_ON() SET_BIT(GPIOA->BSRR, GPIO_BSRR_BS_7) // Включення світлодіода A.
#define LEDa_OFF() SET_BIT(GPIOA->BSRR, GPIO_BSRR_BR_7) // Вимкнення світлодіода A.
#define LEDb_ON() SET_BIT(GPIOB->BSRR, GPIO_BSRR_BS_1) // Включення світлодіода B.
#define LEDb_OFF() SET_BIT(GPIOB->BSRR, GPIO_BSRR_BR_1) // Вимкнення світлодіода B.
#define LEDc_ON() SET_BIT(GPIOA->BSRR, GPIO_BSRR_BS_6) // Включення світлодіода C.
#define LEDc_OFF() SET_BIT(GPIOA->BSRR, GPIO_BSRR_BR_6) // Вимкнення світлодіода C.
#define LEDd_ON() SET_BIT(GPIOA->BSRR, GPIO_BSRR_BS_5) // Включення світлодіода D.
#define LEDd_OFF() SET_BIT(GPIOA->BSRR, GPIO_BSRR_BR_5) // Вимкнення світлодіода D.
#define LEDe_ON() SET_BIT(GPIOA->BSRR, GPIO_BSRR_BS_11) // Включення світлодіода E.
#define LEDe_OFF() SET_BIT(GPIOA->BSRR, GPIO_BSRR_BR_11) // Вимкнення світлодіода E.
#define LEDf_ON() SET_BIT(GPIOA->BSRR, GPIO_BSRR_BS_9) // Включення світлодіода F.
#define LEDf_OFF() SET_BIT(GPIOA->BSRR, GPIO_BSRR_BR_9) // Вимкнення світлодіода F.
#define LEDg_ON() SET_BIT(GPIOB->BSRR, GPIO_BSRR_BS_0) // Включення світлодіода G.
#define LEDg_OFF() SET_BIT(GPIOB->BSRR, GPIO_BSRR_BR_0) // Вимкнення світлодіода G.
#define LEDdp_ON() SET_BIT(GPIOB->BSRR, GPIO_BSRR_BS_3) // Включення десяткової точки.
#define LEDdp_OFF() SET_BIT(GPIOB->BSRR, GPIO_BSRR_BR_3) // Вимкнення десяткової точки.

// Макроси для керування дисплеями
#define LEDD1_ON() SET_BIT(GPIOA->BSRR, GPIO_BSRR_BS_3) // Включення дисплея 1.
#define LEDD1_OFF() SET_BIT(GPIOA->BSRR, GPIO_BSRR_BR_3) // Вимкнення дисплея 1.
#define LEDD2_ON() SET_BIT(GPIOA->BSRR, GPIO_BSRR_BS_4) // Включення дисплея 2.
#define LEDD2_OFF() SET_BIT(GPIOA->BSRR, GPIO_BSRR_BR_4) // Вимкнення дисплея 2.
#define LEDD3_ON() SET_BIT(GPIOA->BSRR, GPIO_BSRR_BS_12) // Включення дисплея 3.
#define LEDD3_OFF() SET_BIT(GPIOA->BSRR, GPIO_BSRR_BR_12) // Вимкнення дисплея 3.
#define LEDD4_ON() SET_BIT(GPIOB->BSRR, GPIO_BSRR_BS_4) // Включення дисплея 4.
#define LEDD4_OFF() SET_BIT(GPIOB->BSRR, GPIO_BSRR_BR_4) // Вимкнення дисплея 4.

// Макроси для додаткових індикаторів
#define LEDl1l2_ON() SET_BIT(GPIOA->BSRR, GPIO_BSRR_BS_10) // Включення індикатора l1l2.
#define LEDl1l2_OFF() SET_BIT(GPIOA->BSRR, GPIO_BSRR_BR_10) // Вимкнення індикатора l1l2.
#define LEDalarm_ON() SET_BIT(GPIOA->BSRR, GPIO_BSRR_BS_8) // Включення індикатора тривоги.
#define LEDalarm_OFF() SET_BIT(GPIOA->BSRR, GPIO_BSRR_BR_8) // Вимкнення індикатора тривоги.

// Макроси для керування enable
#define pinEN_ON() SET_BIT(GPIOC->BSRR, GPIO_BSRR_BS_15) // Включення enable піну.
#define pinEN_OFF() SET_BIT(GPIOC->BSRR, GPIO_BSRR_BR_15) // Вимкнення enable піну.

// Змінні для збереження часу натискання для кожної кнопки
uint16_t timeIncrementButtonDown = 0; // Час натискання кнопки збільшення.
uint16_t timeDecrementButtonDown = 0; // Час натискання кнопки зменшення.
uint16_t timeEnterButtonDown = 0; // Час натискання кнопки введення.

uint16_t IncrementButtonDebounce = 0; // Дебаунс для кнопки збільшення.
uint16_t DecrementButtonDebounce = 0; // Дебаунс для кнопки зменшення.
uint16_t EnterButtonDebounce = 0; // Дебаунс для кнопки введення.

uint16_t timeLastIncrementButtonPress = 0; // Час останнього натискання кнопки збільшення.
uint16_t timeLastDecrementButtonPress = 0; // Час останнього натискання кнопки зменшення.
uint16_t timeLastEnterButtonPress = 0; // Час останнього натискання кнопки введення.

// Прапори для обробки станів кнопок
bool flagIncrementButtonDown = false; // Прапор натискання кнопки збільшення.
bool flagDecrementButtonDown = false; // Прапор натискання кнопки зменшення.
bool flagEnterButtonDown = false; // Прапор натискання кнопки введення.

bool flagIncrementButton = false; // Прапор стану кнопки збільшення.
bool flagDecrementButton = false; // Прапор стану кнопки зменшення.
bool flagEnterButton = false; // Прапор стану кнопки введення.

bool flagIncrementButtonLong = false; // Прапор довгого натискання кнопки збільшення.
bool flagDecrementButtonLong = false; // Прапор довгого натискання кнопки зменшення.
bool flagEnterButtonLong = false; // Прапор довгого натискання кнопки введення.

bool switchONDisplay = false; // Прапор вмикання дисплея.
bool lowpowerModeStatus = false; // Прапор режиму низького енергоспоживання.

#define menuArraySize 31      // Встановлюємо розмір масиву меню.
uint8_t actualIndex = 0;      // Поточний індекс меню.

bool switchONDisplay = false; // Прапор вмикання дисплея.
bool lowpowerModeStatus = false; // Прапор режиму низького енергоспоживання.

#define menuArraySize 31 // Встановлюємо розмір масиву меню.
uint8_t actualIndex = 0; // Поточний індекс меню.
bool isParamEditMode = false; // Прапорець режиму редагування параметра.
uint8_t tmpVal = 0; // Тимчасова змінна для зберігання параметра.

volatile uint32_t SysTimer_ms = 0; // Системний таймер (аналог HAL_GetTick).
volatile uint16_t Delay_counter_ms = 0; // Лічильник для затримки.
uint32_t timeWakeUp = 0; // Час пробудження.
uint32_t timeNow = 0; // Поточний час.

char tmpV[4] = {}; // Тимчасовий масив для значень.
char tmpClock[4] = {}; // Тимчасовий масив для годинника.
int8_t vmenu = 0; // Змінна, що зберігає дію по вертикалі (1 - вхід в меню, -1 - вихід з меню).
int8_t hmenu = 0; // Змінна, що зберігає дію по горизонталі (1 - вправо, -1 - вліво).
char *tmpValue; // Вказівник на тимчасове значення.

struct strMenu // Структура меню.
{
    int8_t id; // Унікальний ідентифікаційний індекс ID.
    int8_t parentid; // ID батька (вкладеність).
    bool isParam; // Чи є пункт змінним параметром.
    char _name[4]; // Назва пункту меню.
    uint8_t value; // Поточне значення параметра.
    uint8_t _min; // Мінімально можливе значення.
    uint8_t _max; // Максимально можливе значення.
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
 *29 		P_5.1	timeButtonLongPressed
 *30 	P__6		Clock(StartWork)
 */

// Встановлюємо пункти меню.
struct strMenu menu[] = {
    {0, -1, false, "PPPP", 0, 0, 0},
    {1, 0, false, "P_0_", 0, 0, 0},
    {2, 1, true, "P_00", 17, 0, 24},
    {3, 1, true, "P_01", 36, 0, 59},
    {4, 1, true, "P_02", 0, 0, 59},
    {5, 1, true, "P_03", 1, 1, 31},
    {6, 1, true, "P_04", 4, 1, 12},
    {7, 1, true, "P_05", 0, 0, 231},
    {8, 1, true, "P_06", 1, 1, 7},
    {9, 1, false, "P_07", 0, 0, 0},
    {10, 0, false, "P_1_", 0, 0, 0},
    {11, 10, true, "P_10", 6, 0, 23},
    {12, 10, true, "P_11", 0, 0, 59},
    {13, 0, false, "P_2_", 0, 0, 0},
    {14, 13, true, "P_20", 15, 0, 231},
    {15, 13, true, "P_21", 224, 224, 224},
    {16, 13, false, "P_22", 0, 0, 0},
    {17, 0, false, "P_3_", 0, 0, 0},
    {18, 17, true, "P_30", 1, 0, 1},
    {19, 17, true, "P_31", 5, 0, 24},
    {20, 17, true, "P_32", 50, 0, 59},
    {21, 17, true, "P_33", 0, 0, 7},
    {22, 17, false, "P_34", 0, 0, 0},
    {23, 0, false, "P_4_", 0, 0, 1},
    {24, 23, true, "P_40", 0, 0, 3},
    {25, 23, true, "P_41", 0, 0, 1},
    {26, 23, true, "P_42", 5, 0, 231},
    {27, 0, false, "P_5_", 0, 0, 0},
    {28, 27, true, "P_50", 55, 0, 231},
    {29, 27, true, "P_51", 100, 0, 231},
    {30, 0, false, "P_6_", 0, 0, 0}
};

/* Private function prototypes -----------------------------------------------*/
void CMSIS_FullInit(void); // Прототип функції повної ініціалізації CMSIS.
uint8_t SysTickTimerInit(uint32_t ticks); // Прототип функції ініціалізації SysTick таймера.
void SystemClock_Config(void); // Прототип функції конфігурації системного годинника.
void GPIO_Init(void); // Прототип функції ініціалізації GPIO.
void RTC_Init(void); // Прототип функції ініціалізації RTC.
void TIM2_Init(void); // Прототип функції ініціалізації TIM2.
void TIM21_Init(void); // Прототип функції ініціалізації TIM21.
void interaptTIMDebounce(void); // Прототип функції обробки переривань TIM для дебаунсу.

// Прототипи обробників переривань
void SysTick_Handler(void); // Обробник переривання SysTick.
void Error_Handler(void); // Обробник помилок.
void assert_failed(uint8_t *file, uint8_t line); // Обробник помилок асерта.
void EXTI0_1_IRQHandler(void); // Обробник переривання EXTI для ліній 0-1.
void EXTI2_3_IRQHandler(void); // Обробник переривання EXTI для ліній 2-3.
void EXTI4_15_IRQHandler(void); // Обробник переривання EXTI для ліній 4-15.
void LPTIM1_IRQHandler(void); // Обробник переривання LPTIM1.
void TIM2_IRQHandler(void); // Обробник переривання TIM2.
void TIM21_IRQHandler(void); // Обробник переривання TIM21.

// Інші функції
void EnterLowPowerMode(uint8_t status); // Функція для входу в режим низького енергоспоживання.
void Delay_ms(uint16_t Milliseconds); // Функція затримки в мілісекундах.
double pow2_24(double c); // Функція обчислення степеня 2 до 24.
uint16_t map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max); // Функція для мапування значень.

void pwmFP7103(); // Функція для керування PWM.
void testMainLamp(void); // Функція тестування головної лампи.

uint8_t secondsDecimal(); // Функція для отримання секунд у десятковому форматі.
uint8_t minutesDecimal(); // Функція для отримання хвилин у десятковому форматі.
uint8_t hoursDecimal(); // Функція для отримання годин у десятковому форматі.
void setTimeNow(); // Функція для встановлення поточного часу.
uint8_t Clock(); // Функція для роботи з годинником.

char intToChar(uint8_t num); // Функція для перетворення числа в символ.
void writeCHARSEG(char CHAR, uint8_t seg); // Функція для запису символу на сегмент.
char *setActualMenu(int8_t v, int8_t h); // Функція для встановлення поточного меню.
uint8_t getMenuIndexByID(int8_t id); // Функція для отримання індексу меню за ID.
uint8_t getNearMenuIndexByID(int8_t parentid, int8_t id, int8_t side); // Функція для отримання найближчого індексу меню за ID.
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
int main(void)
{
    CMSIS_FullInit(); // Ініціалізація CMSIS та таймеру на 1 мс.
    SystemClock_Config(); // Конфігурація системного годинника.

    GPIO_Init(); // Ініціалізація GPIO.
    RTC_Init(); // Ініціалізація реального часу (RTC).
    TIM2_Init(); // Ініціалізація таймера TIM2.
    TIM21_Init(); // Ініціалізація таймера TIM21.

    LEDl1l2_OFF(); // Вимкнення індикатора l1l2.
    LEDalarm_OFF(); // Вимкнення індикатора тривоги.
    pinEN_OFF(); // Вимкнення enable піну.
    writeCHARSEG(' ', ' '); // Очищення сегментів дисплея.

    tmpValue = setActualMenu(0, 0); // Встановлення початкового меню.
    while (1)
    {
        vmenu = 0; // Скидання вертикального флагу після обробки.
        hmenu = 0; // Скидання горизонтального флагу після обробки.

        LEDl1l2_OFF(); // Вимкнення індикатора l1l2.
        LEDalarm_OFF(); // Вимкнення індикатора будильника.
        pinEN_OFF(); // Вимкнення enable піну.

        interaptTIMDebounce(); // Обробка дебаунсу за допомогою таймера.

        if (flagDecrementButton)
        {
            hmenu = -1; // Переміщення по меню вниз.
            flagDecrementButton = false; // Скидання флагу.
        }
        if (flagDecrementButtonLong)
        {
            hmenu = -5; // Швидке переміщення по меню вниз (довге натискання).
            flagDecrementButtonLong = false; // Скидання флагу.
        }
        if (flagIncrementButton)
        {
            hmenu = 1; // Переміщення по меню вгору.
            flagIncrementButton = false; // Скидання флагу.
        }
        if (flagIncrementButtonLong)
        {
            hmenu = 5; // Швидке переміщення по меню вгору (довге натискання).
            flagIncrementButtonLong = false; // Скидання флагу.
        }
        if (flagEnterButton)
        {
            vmenu = 1; // Вхід у підменю або редагування параметра.
            flagEnterButton = false; // Скидання флагу.
        }
        if (flagEnterButtonLong)
        {
            vmenu = -1; // Вихід з підменю або відміна редагування.
            flagEnterButtonLong = false; // Скидання флагу.
        }

        if (vmenu != 0 || hmenu != 0)
        {
            tmpValue = setActualMenu(vmenu, hmenu); // Оновлення меню, якщо було натискання.
        }

        // Оновлення дисплея з інтервалом 4 мс.
        if (SysTimer_ms % 4 == 0)
            writeCHARSEG(tmpValue[0], 0); // Запис символу на сегмент 0.
        if (SysTimer_ms % 4 == 1)
            writeCHARSEG(tmpValue[1], 1); // Запис символу на сегмент 1.
        if (SysTimer_ms % 4 == 2)
            writeCHARSEG(tmpValue[2], 2); // Запис символу на сегмент 2.
        if (SysTimer_ms % 4 == 3)
            writeCHARSEG(tmpValue[3], 3); // Запис символу на сегмент 3.
    }
    return 0; // Завершення функції main.
}

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
uint8_t SysTickTimerInit(uint32_t ticks)
{
    if ((ticks - 1UL) > SysTick_LOAD_RELOAD_Msk)
    {
        return (1UL); /* Неприпустиме значення перезавантаження */
    }
    CLEAR_BIT(SysTick->CTRL, SysTick_CTRL_ENABLE_Msk); /* Вимкнення таймера SysTick */
    SysTick->LOAD = (uint32_t)(ticks - 1UL); /* Встановлення значення перезавантаження */
    NVIC_SetPriority(SysTick_IRQn, (1UL << __NVIC_PRIO_BITS) - 1UL); /* Встановлення пріоритету переривання SysTick */
    SysTick->VAL = 0UL; /* Скидання значення лічильника SysTick */
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                    SysTick_CTRL_TICKINT_Msk |
                    SysTick_CTRL_ENABLE_Msk; /* Включення SysTick IRQ і таймера SysTick */
    return (0UL); /* Успішне завершення функції */
}

void CMSIS_FullInit(void)
{
    // *** Налаштування кешу, передвибірки і попереднього читання *** //
    CLEAR_BIT(FLASH->ACR, FLASH_ACR_DISAB_BUF); // Вимкнення буфера кешу, якщо це налаштовано
    SET_BIT(FLASH->ACR, FLASH_ACR_PRE_READ); // Включення попереднього читання, якщо це налаштовано
    SET_BIT(FLASH->ACR, FLASH_ACR_PRFTEN); // Включення буфера передвибірки, якщо це налаштовано
    SET_BIT(FLASH->ACR, FLASH_ACR_LATENCY); // Встановлення затримки доступу до флеш-пам'яті

    // *** Налаштування SysTick для переривань кожну 1 мс *** //
    uint32_t ticks = SYSCLK / 1000U; // Розрахунок кількості тактів для 1 мс

    // Використовуємо SysTick_Config для налаштування таймера
    if (ticks > SysTick_LOAD_RELOAD_Msk) // Якщо кількість тактів більше дозволеного
    {
        while (1); // Помилка, зациклюємося
    }

    SysTickTimerInit(ticks); // Ініціалізація таймера SysTick

    // Встановлення пріоритету для переривання SysTick
    uint32_t tickPriority = 0; // Пріоритет для SysTick
    if (tickPriority < (1UL << __NVIC_PRIO_BITS))
    {
        NVIC_SetPriority(SysTick_IRQn, tickPriority); // Встановлення пріоритету переривання SysTick
    }
    else
    {
        while (1); // Помилка пріоритету, зациклюємося
    }
}

void SystemClock_Config(void)
{
    MODIFY_REG(PWR->CR, PWR_CR_VOS_Msk, 0b01 << PWR_CR_VOS_Pos); // Налаштування PWR CR для регулювання напруги

    CLEAR_BIT(RCC->CR, RCC_CR_HSIDIVEN); // Вимкнення HSI16DIV

    CLEAR_BIT(RCC->CR, RCC_CR_MSION); // Вимкнення MSI

    if ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_HSI)
    {
        SET_BIT(RCC->CR, RCC_CR_HSION); // Увімкнення HSI16
        while (!(RCC->CR & RCC_CR_HSIRDY)); // Очікування стабілізації HSI16

        MODIFY_REG(RCC->CFGR, RCC_CFGR_SW_Msk, RCC_CFGR_SW_HSI); // Перемикання системної тактової частоти на HSI16
        while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_HSI); // Очікування завершення перемикання
    }

    MODIFY_REG(RCC->CFGR, RCC_CFGR_MCOPRE_Msk, 0b000 << RCC_CFGR_MCOPRE_Pos); // Налаштування MCO prescaler
    MODIFY_REG(RCC->CFGR, RCC_CFGR_MCOSEL_Msk, 0b000 << RCC_CFGR_MCOSEL_Pos); // Налаштування джерела сигналу для MCO

    MODIFY_REG(RCC->CFGR, RCC_CFGR_PLLDIV_Msk, 0b01 << RCC_CFGR_PLLDIV_Pos); // Налаштування PLL divisor
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PLLMUL_Msk, 0b0001 << RCC_CFGR_PLLMUL_Pos); // Налаштування множника PLL
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PLLSRC_Msk, 0b0 << RCC_CFGR_PLLSRC_Pos); // Налаштування джерела сигналу для PLL

    SET_BIT(RCC->CFGR, RCC_CFGR_STOPWUCK); // Вимкнення системного пробудження від MSI після STOP режиму

    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2_Msk, 0b000 << RCC_CFGR_PPRE2_Pos); // Налаштування прескалера для APB2
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1_Msk, 0b000 << RCC_CFGR_PPRE1_Pos); // Налаштування прескалера для APB1
    MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE_Msk, 0b0000 << RCC_CFGR_HPRE_Pos); // Налаштування прескалера для AHB

    SET_BIT(RCC->CR, RCC_CR_PLLON); // Увімкнення PLL
    while (!(RCC->CR & RCC_CR_PLLRDY)); // Очікування стабілізації PLL

    MODIFY_REG(RCC->CFGR, RCC_CFGR_SW_Msk, RCC_CFGR_SW_PLL); // Перемикання системної частоти на PLL
    while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_PLL); // Очікування завершення перемикання
}

void RTC_Init(void)
{
    // 1. Увімкнення доступу до резервного домену та живлення
    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN); // Увімкнення тактування блоку живлення
    SET_BIT(PWR->CR, PWR_CR_DBP); // Розблокування доступу до резервного домену

    // 2. Налаштування LSE в режимі обхідного шляху
    CLEAR_BIT(RCC->CSR, RCC_CSR_LSEON); // Переконатись, що LSE вимкнений
    SET_BIT(RCC->CSR, RCC_CSR_LSEBYP); // Увімкнути режим обходу LSE (зовнішній генератор)
    SET_BIT(RCC->CSR, RCC_CSR_LSEON); // Увімкнути генератор LSE

    while (!(READ_BIT(RCC->CSR, RCC_CSR_LSERDY))) {} // Очікувати, поки LSE не стабілізується

    // 4. Вибір LSE як джерела сигналу для RTC та увімкнення RTC
    MODIFY_REG(RCC->CSR, RCC_CSR_RTCSEL_Msk, (0b01 << RCC_CSR_RTCSEL_Pos)); // Вибір LSE як джерела сигналу для RTC
    SET_BIT(RCC->CSR, RCC_CSR_RTCEN); // Увімкнення RTC

    // 5. Вимкнення захисту запису RTC
    RTC->WPR = 0xCA; // Ключ захисту запису 1
    RTC->WPR = 0x53; // Ключ захисту запису 2

    // 6. Перехід у режим ініціалізації
    SET_BIT(RTC->ISR, RTC_ISR_INIT); // Встановлення режиму ініціалізації
    while (!(READ_BIT(RTC->ISR, RTC_ISR_INITF))) {} // Очікування готовності режиму ініціалізації

    // 7. Встановлення часу у форматі BCD (17:36:00)
    MODIFY_REG(RTC->TR,
               RTC_TR_HT_Msk | RTC_TR_HU_Msk | RTC_TR_MNT_Msk | RTC_TR_MNU_Msk | RTC_TR_ST_Msk | RTC_TR_SU_Msk,
               (0x1 << RTC_TR_HT_Pos) |  // Десятки годин (1 -> 17)
               (0x7 << RTC_TR_HU_Pos) |  // Одиниці годин (7 -> 17)
               (0x3 << RTC_TR_MNT_Pos) | // Десятки хвилин (3 -> 36)
               (0x6 << RTC_TR_MNU_Pos) | // Одиниці хвилин (6 -> 36)
               (0x0 << RTC_TR_ST_Pos) |  // Десятки секунд (0 -> 00)
               (0x0 << RTC_TR_SU_Pos));  // Одиниці секунд (0 -> 00)

    // 8. Встановлення дати у форматі BCD (01/04/2024, понеділок)
    MODIFY_REG(RTC->DR,
               RTC_DR_YT_Msk | RTC_DR_YU_Msk | RTC_DR_MT_Msk | RTC_DR_MU_Msk | RTC_DR_DT_Msk | RTC_DR_DU_Msk | RTC_DR_WDU_Msk,
               (0x2 << RTC_DR_YT_Pos) |  // Десятки років (2 -> 2024)
               (0x4 << RTC_DR_YU_Pos) |  // Одиниці років (4 -> 2024)
               (0x0 << RTC_DR_MT_Pos) |  // Десятки місяців (0 -> квітень)
               (0x4 << RTC_DR_MU_Pos) |  // Одиниці місяців (4 -> квітень)
               (0x0 << RTC_DR_DT_Pos) |  // Десятки днів (0 -> 01)
               (0x1 << RTC_DR_DU_Pos) |  // Одиниці днів (1 -> 01)
               (0x1 << RTC_DR_WDU_Pos)); // День тижня (1 -> понеділок)

    // 9. Вихід з режиму ініціалізації
    CLEAR_BIT(RTC->ISR, RTC_ISR_INIT); // Скидання режиму ініціалізації

    // 10. Увімкнення захисту запису RTC
    RTC->WPR = 0xFE; // Ключ захисту запису 1
    RTC->WPR = 0x64; // Ключ захисту запису 2
}

void TIM2_Init(void)
{
    // Увімкнення тактування GPIOA (для PA15, як PWM вихід)
    RCC->IOPENR |= RCC_IOPENR_IOPAEN;
    // Налаштування піна 15 на режим альтернативної функції
    CONFIGURE_GPIO(GPIOA, 15, 0b10, 0, 0b11); // Пін Buzzer
    // Налаштування альтернативної функції AF1 для піна 15
    MODIFY_REG(GPIOA->AFR[1], GPIO_AFRH_AFSEL15_Msk, 0b0101 << GPIO_AFRH_AFSEL15_Pos);

    // Увімкнення тактування TIM2
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    CLEAR_BIT(TIM2->CR1, TIM_CR1_CEN); // Вимкнення таймера TIM2

    // Налаштування таймера 2
    // Встановлення різних параметрів конфігурації
    CLEAR_BIT(TIM2->CR1, TIM_CR1_UDIS); // Генерувати подію Update
    CLEAR_BIT(TIM2->CR1, TIM_CR1_URS); // Генерувати переривання
    CLEAR_BIT(TIM2->CR1, TIM_CR1_OPM); // One pulse mode off (Лічильник не зупиняється при оновленні)
    CLEAR_BIT(TIM2->CR1, TIM_CR1_DIR); // Лічильник збільшується
    MODIFY_REG(TIM2->CR1, TIM_CR1_CMS_Msk, 0b00 << TIM_CR1_CMS_Pos); // Вирівнювання по краю
    SET_BIT(TIM2->CR1, TIM_CR1_ARPE); // Автоматичне завантаження перезавантаження
    MODIFY_REG(TIM2->CR1, TIM_CR1_CKD_Msk, 0b00 << TIM_CR1_CKD_Pos); // Попереднє ділення вимкнено

    // Налаштування переривань
    SET_BIT(TIM2->DIER, TIM_DIER_UIE); // Включення переривань по оновленню

    // Встановлення значень для таймера
    TIM2->PSC = 32 - 1; // Переддільник на 32
    TIM2->ARR = 10000 - 1; // Автоматичне завантаження на 10000

    NVIC_EnableIRQ(TIM2_IRQn); // Дозвіл переривань по таймеру 2

    // Налаштування ШИМ (Канал 1)
    MODIFY_REG(TIM2->CCMR1, TIM_CCMR1_CC1S_Msk, 0b00 << TIM_CCMR1_CC1S_Pos); // Канал CC1 налаштований як вихід
    CLEAR_BIT(TIM2->CCMR1, TIM_CCMR1_OC1FE); // Швидкий режим вимкнено
    SET_BIT(TIM2->CCMR1, TIM_CCMR1_OC1PE); // Попереднє завантаження увімкнено
    MODIFY_REG(TIM2->CCMR1, TIM_CCMR1_OC1M_Msk, 0b110 << TIM_CCMR1_OC1M_Pos); // Режим PWM 1
    CLEAR_BIT(TIM2->CCMR1, TIM_CCMR1_OC1CE); // OC1Ref не впливає на вхід ETRF

    // Запуск ШИМ
    SET_BIT(TIM2->CCER, TIM_CCER_CC1E); // OC1 сигнал виводиться на відповідний вихідний пін
    CLEAR_BIT(TIM21->CCER, TIM_CCER_CC1P); // OC1 активний високий

    CLEAR_BIT(TIM2->CR1, TIM_CR1_CEN); // Вимкнення таймера TIM2
}

void TIM21_Init(void)
{
    // Увімкнення тактування GPIOB (для PB5, як PWM вихід)
    RCC->IOPENR |= RCC_IOPENR_IOPBEN;
    // Налаштування піна 5 на режим альтернативної функції
    CONFIGURE_GPIO(GPIOB, 5, 0b10, 0, 0b11); // Пін mainLED
    // Налаштування альтернативної функції AF1 для піна 5
    MODIFY_REG(GPIOB->AFR[0], GPIO_AFRL_AFSEL5_Msk, 0b0101 << GPIO_AFRL_AFSEL5_Pos);

    // Увімкнення тактування TIM21
    RCC->APB2ENR |= RCC_APB2ENR_TIM21EN;
    CLEAR_BIT(TIM21->CR1, TIM_CR1_CEN); // Вимкнення таймера TIM21

    // Налаштування таймера 21
    // Встановлення різних параметрів конфігурації
    CLEAR_BIT(TIM21->CR1, TIM_CR1_UDIS); // Генерувати подію Update
    CLEAR_BIT(TIM21->CR1, TIM_CR1_URS); // Генерувати переривання
    CLEAR_BIT(TIM21->CR1, TIM_CR1_OPM); // One pulse mode off (Лічильник не зупиняється при оновленні)
    CLEAR_BIT(TIM21->CR1, TIM_CR1_DIR); // Лічильник збільшується
    MODIFY_REG(TIM21->CR1, TIM_CR1_CMS_Msk, 0b00 << TIM_CR1_CMS_Pos); // Вирівнювання по краю
    SET_BIT(TIM21->CR1, TIM_CR1_ARPE); // Автоматичне завантаження перезавантаження
    MODIFY_REG(TIM21->CR1, TIM_CR1_CKD_Msk, 0b00 << TIM_CR1_CKD_Pos); // Попереднє ділення вимкнено

    // Налаштування переривань
    SET_BIT(TIM21->DIER, TIM_DIER_UIE); // Включення переривань по оновленню

    // Встановлення значень для таймера
    TIM21->PSC = 32 - 1; // Переддільник на 32
    TIM21->ARR = 10000 - 1; // Автоматичне завантаження на 10000

    NVIC_EnableIRQ(TIM21_IRQn); // Дозвіл переривань по таймеру 21

    // Налаштування ШИМ (Канал 1)
    MODIFY_REG(TIM21->CCMR1, TIM_CCMR1_CC1S_Msk, 0b00 << TIM_CCMR1_CC1S_Pos); // Канал CC1 налаштований як вихід
    CLEAR_BIT(TIM21->CCMR1, TIM_CCMR1_OC1FE); // Швидкий режим вимкнено
    SET_BIT(TIM21->CCMR1, TIM_CCMR1_OC1PE); // Попереднє завантаження увімкнено
    MODIFY_REG(TIM21->CCMR1, TIM_CCMR1_OC1M_Msk, 0b110 << TIM_CCMR1_OC1M_Pos); // Режим PWM 1
    CLEAR_BIT(TIM21->CCMR1, TIM_CCMR1_OC1CE); // OC1Ref не впливає на вхід ETRF

    // Запуск ШИМ
    SET_BIT(TIM21->CCER, TIM_CCER_CC1E); // OC1 сигнал виводиться на відповідний вихідний пін
    SET_BIT(TIM21->CCER, TIM_CCER_CC1P); // OC1 активний високий

    CLEAR_BIT(TIM21->CR1, TIM_CR1_CEN); // Вимкнення таймера TIM21
}


void GPIO_Init(void) {
    // Включення тактування портів A, B, C
    RCC->IOPENR |= RCC_IOPENR_IOPAEN | RCC_IOPENR_IOPBEN | RCC_IOPENR_IOPCEN;
    // Увімкнення тактування для портів A, B, C

    // Налаштування світлодіодів (виводи PA, PB)
    CONFIGURE_GPIO(GPIOA, 7, 0b01, 0, 0b00);  // Світлодіод LEDa
    CONFIGURE_GPIO(GPIOB, 1, 0b01, 0, 0b00);  // Світлодіод LEDb
    CONFIGURE_GPIO(GPIOA, 6, 0b01, 0, 0b00);  // Світлодіод LEDc
    CONFIGURE_GPIO(GPIOA, 5, 0b01, 0, 0b00);  // Світлодіод LEDd
    CONFIGURE_GPIO(GPIOA, 11, 0b01, 0, 0b00);  // Світлодіод LEDe
    CONFIGURE_GPIO(GPIOA, 9, 0b01, 0, 0b00);  // Світлодіод LEDf
    CONFIGURE_GPIO(GPIOB, 0, 0b01, 0, 0b00);  // Світлодіод LEDg
    CONFIGURE_GPIO(GPIOB, 3, 0b01, 0, 0b00);  // Світлодіод LEDdp
    CONFIGURE_GPIO(GPIOA, 3, 0b01, 0, 0b00);  // Світлодіод LEDD1
    CONFIGURE_GPIO(GPIOA, 4, 0b01, 0, 0b00);  // Світлодіод LEDD2
    CONFIGURE_GPIO(GPIOA, 12, 0b01, 0, 0b00);  // Світлодіод LEDD3
    CONFIGURE_GPIO(GPIOB, 4, 0b01, 0, 0b00);  // Світлодіод LEDD4
    CONFIGURE_GPIO(GPIOA, 10, 0b01, 0, 0b00);  // Світлодіод LEDl1l2
    CONFIGURE_GPIO(GPIOA, 8, 0b01, 0, 0b00);  // Світлодіод LEDalarm
    CONFIGURE_GPIO(GPIOC, 15, 0b01, 0, 0b00);  // Вивід pinEN
    // Налаштування GPIO для світлодіодів

    // Налаштування кнопок із EXTI
    CONFIGURE_GPIO(GPIOA, 0, 0b00, 0, 0b11); // Кнопка decrement
    CONFIGURE_EXTI(0, 0b000, 0, 1);          // EXTI для кнопки decrement
    CONFIGURE_GPIO(GPIOA, 1, 0b00, 0, 0b11); // Кнопка enter
    CONFIGURE_EXTI(1, 0b000, 0, 1);          // EXTI для кнопки enter
    CONFIGURE_GPIO(GPIOA, 2, 0b00, 0, 0b11); // Кнопка increment
    CONFIGURE_EXTI(2, 0b000, 0, 1);          // EXTI для кнопки increment
    CONFIGURE_GPIO(GPIOB, 9, 0b00, 0, 0b11); // Кнопка pwr
    CONFIGURE_EXTI(9, 0b000, 2, 3);          // EXTI для кнопки pwr із обробкою по зростанню

    // Включення переривання
    NVIC_EnableIRQ(EXTI0_1_IRQn);
    NVIC_EnableIRQ(EXTI2_3_IRQn);
    NVIC_EnableIRQ(EXTI4_15_IRQn);
    // Увімкнення переривань для ліній EXTI
}

void LowPowerMode(uint8_t status) {
    if (status) {
        lowpowerModeStatus = true;
        // Встановлюємо режим STOP з RTC працюючим у нормальному режимі
        //pinEN_OFF();
        //RCC->APB1ENR &= ~RCC_APB1ENR_TIM2EN;
        //RCC->APB1ENR &= ~RCC_APB2ENR_TIM21EN;
        //RCC->IOPENR &= ~RCC_IOPENR_IOPAEN | ~RCC_IOPENR_IOPBEN | ~RCC_IOPENR_IOPCEN;
        //RCC->IOPENR |= RCC_IOPENR_IOPBEN | RCC_IOPENR_IOPCEN;
        //CONFIGURE_GPIO(GPIOC, 15, 0b01, 0, 0b11); // pinEN
        //CONFIGURE_GPIO(GPIOB, 9, 0b00, 0, 0b11); // pwr
        //CONFIGURE_EXTI(9, 0b000, 2, 3);         // EXTI для кнопки pwr із обробкою по зростанню

        //PWR->CR |= PWR_CR_LPDS;  // Увімкнення режиму низького енергоспоживання
        //SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;  // Налаштування контролера для глибокого сну

        //__WFI();  // Очікування на переривання для виходу з режиму STOP
    } else {
        lowpowerModeStatus = false;
        //pinEN_ON();
        //GPIO_Init();
        //TIM2_Init();
        //TIM21_Init();
        //SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;  // Вимкнення режиму глибокого сну
        //PWR->CR &= ~PWR_CR_LPDS;  // Відновлення нормального режиму живлення
    }
}

void Delay_ms(uint16_t Miliseconds) {
    Delay_counter_ms = Miliseconds;
    while (Delay_counter_ms != 0) {
        Delay_counter_ms--;
    }
    // Проста функція затримки для очікування певної кількості мілісекунд
}

double pow2_24(double b) {
    // FastMagicPOW
    union { double d; long long i; } u = { 1000000 * b };
    u.i = (long long)(2.24 * u.i - 5712498484330472700L);
    return 3.64182e-14 * u.d + 3.68723e-8 * b + 3.97089e-13;
    // Приблизне обчислення 2.24 за допомогою ряду трансформацій
}

// double pow2_24(double c) {
//     double c_6 = c * c;
//     c_6 *= c_6 * c_6;
//     // Обчислення c^6

//     union { double d; long long i; } u = { 1000000 * c };
//     u.i = (long long)(3501208748460612300L + 0.24 * u.i);
//     // Використання union для точнішого обчислення

//     double x = 0.0363 * u.d + 2e-5;
//     // Початкове значення x

//     for (int i = 0; i < 5; i++) {
//         double x_24 = x * x;
//         x_24 *= x_24;
//         x_24 *= x_24;
//         x_24 *= x_24 * x_24;
//         // Обчислення x^24

//         x = x + 0.04 * (c_6 / x_24 - x);
//         // Оновлення значення x
//     }

//     return c * c * x;
//     // Повертає кінцеве значення
// }

char intToChar(uint8_t num) {
    switch (num) {
        case 0: return '0';
        case 1: return '1';
        case 2: return '2';
        case 3: return '3';
        case 4: return '4';
        case 5: return '5';
        case 6: return '6';
        case 7: return '7';
        case 8: return '8';
        case 9: return '9';
        default: return '?'; // Повертаємо '?' для невідомого числа
    }
    // Перетворює число від 0 до 9 на символ
}

void writeCHARSEG(char CHAR, uint8_t seg) {
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
    // Вмикає відповідний сегмент

    switch (CHAR) {
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
    // Вмикає світлодіоди для відповідного символу
}

uint16_t map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    // Масштабування значення x з одного діапазону на інший
}

void pwmFP7103() {
    if (menu[18].value) { // *16 P_3.0 Alarm_Status
        timeWakeUp = menu[11].value * 3600 + menu[12].value * 60 + 1; // *11 P_1.0 Hour_Rise; *12 P_1.1 Minute_Rise
        timeNow = hoursDecimal() * 3600 + minutesDecimal() * 60 + secondsDecimal() + 1;
        if (menu[14].value * 60 >= (timeWakeUp - timeNow)) { // *14 P_2.0 Period_Rising
            // *15 P_2.1 ɣ_Coefient_Rising
            pinEN_ON();
            SET_BIT(TIM21->CR1, TIM_CR1_CEN); // Запуск таймера
            TIM21->CCR1 = (uint16_t)((TIM21->ARR + 1) * pow2_24(map(((double)timeNow / timeWakeUp),
                    (uint16_t)(((double)(timeWakeUp - menu[14].value * 60) / timeWakeUp)),
                    1,
                    0,
                    1)));
        }
        if (!(menu[14].value * 60 >= (timeWakeUp - timeNow)) && (flagDecrementButtonLong || flagIncrementButtonLong)) {
            pinEN_OFF();
            CLEAR_BIT(TIM21->CR1, TIM_CR1_CEN); // Призупинення таймера
            TIM21->CCR1 = 0;
        }
    } else {
        pinEN_OFF();
        CLEAR_BIT(TIM21->CR1, TIM_CR1_CEN); // Призупинення таймера
        TIM21->CCR1 = 0;
    }
    // Управління підключенням і відключенням таймера, залежно від налаштувань меню і статусу будильника
}

void testMainLamp(void) {
    writeCHARSEG(' ', 0);
    writeCHARSEG(' ', 1);
    writeCHARSEG(' ', 2);
    writeCHARSEG(' ', 3);
    pinEN_ON();
    SET_BIT(TIM21->CR1, TIM_CR1_CEN); // Запуск таймера

    // *15 P_2.2 ɣ_Coefient_Rising
    while (SysTimer_ms % 10000 != 9999) {
        (SysTimer_ms % 2000 < 1000) ? LEDl1l2_ON() : LEDl1l2_OFF();
        if (SysTimer_ms % 10000 < 1000) TIM21->CCR1 = 0;
        if (SysTimer_ms % 10000 == 3000) TIM21->CCR1 = 3000;
        if (SysTimer_ms % 10000 == 6000) TIM21->CCR1 = 6000;
        if (SysTimer_ms % 10000 == 9000) TIM21->CCR1 = 9000;
    }
    // Тестування основної лампи з різними значеннями CCR1 таймера

    pinEN_OFF();
    CLEAR_BIT(TIM21->CR1, TIM_CR1_CEN); // Зупинення таймера
    TIM21->CCR1 = 0;
}

uint8_t hoursDecimal() {
    uint8_t hoursBCD = ((READ_BIT(RTC->TR, RTC_TR_HT | RTC_TR_HU)) >> RTC_TR_HU_Pos) & 0xFF;
    // Витягує години у форматі BCD

    return ((hoursBCD >> 4) & 0xF) * 10 + (hoursBCD & 0xF);
    // Перетворює BCD у десятковий формат
}

uint8_t minutesDecimal() {
    uint8_t minutesBCD = ((READ_BIT(RTC->TR, RTC_TR_MNT | RTC_TR_MNU)) >> RTC_TR_MNU_Pos) & 0xFF;
    // Витягує хвилини у форматі BCD

    return ((minutesBCD >> 4) & 0xF) * 10 + (minutesBCD & 0xF);
    // Перетворює BCD у десятковий формат
}

uint8_t secondsDecimal() {
    uint8_t secondsBCD = ((READ_BIT(RTC->TR, RTC_TR_ST | RTC_TR_SU)) >> RTC_TR_SU_Pos) & 0xFF;
    // Витягує секунди у форматі BCD

    return ((secondsBCD >> 4) & 0xF) * 10 + (secondsBCD & 0xF);
    // Перетворює BCD у десятковий формат
}

uint8_t Clock() {
    uint8_t i = 0;
    interaptTIMDebounce();

    tmpClock[0] = intToChar(hoursDecimal() / 10);
    if (tmpClock[0] == '0') i = 1;
    tmpClock[1] = intToChar(hoursDecimal() % 10);
    tmpClock[2] = intToChar(minutesDecimal() / 10);
    tmpClock[3] = intToChar(minutesDecimal() % 10);

    if (flagDecrementButton || flagEnterButton || flagIncrementButton) switchONDisplay = true;
    if (SysTimer_ms % menu[24].value == menu[26].value - 1) switchONDisplay = false;
    if ((hoursDecimal() > 5 && hoursDecimal() < 22) || switchONDisplay || menu[24].value == 0) {
        (menu[18].value == 1) ? LEDalarm_ON() : LEDalarm_OFF();

        if (SysTimer_ms % 4 == 0 && i == 0) writeCHARSEG(tmpClock[0], 0);
        if (SysTimer_ms % 4 == 1) writeCHARSEG(tmpClock[1], 1);
        if (SysTimer_ms % 4 == 2) writeCHARSEG(tmpClock[2], 2);
        if (SysTimer_ms % 4 == 3) writeCHARSEG(tmpClock[3], 3);

        (SysTimer_ms % 2000 <= 1000) ? LEDl1l2_ON() : LEDl1l2_OFF();

        pwmFP7103();
    }
    return (flagEnterButtonLong) ? 0 : 1;
    // Управління відображенням часу, перевірка статусів кнопок та керування світлодіодами
}

void setTimeNow() {
    RTC->WPR = 0xCA; // Step 1
    RTC->WPR = 0x53; // Step 2
    // Відключення захисту запису RTC

    SET_BIT(RTC->ISR, RTC_ISR_INIT);
    while (!(READ_BIT(RTC->ISR, RTC_ISR_INITF))) {
    }
    // Входження у режим ініціалізації

    MODIFY_REG(RTC->TR,
        RTC_TR_HT_Msk | RTC_TR_HU_Msk | RTC_TR_MNT_Msk | RTC_TR_MNU_Msk | RTC_TR_ST_Msk | RTC_TR_SU_Msk,
        (menu[2].value / 10 << RTC_TR_HT_Pos) | // Десятки годин
        (menu[2].value % 10 << RTC_TR_HU_Pos) | // Одиниці годин
        (menu[3].value / 10 << RTC_TR_MNT_Pos) | // Десятки хвилин
        (menu[3].value % 10 << RTC_TR_MNU_Pos) | // Одиниці хвилин
        (menu[4].value / 10 << RTC_TR_ST_Pos) | // Десятки секунд
        (menu[4].value % 10 << RTC_TR_SU_Pos)); // Одиниці секунд
    MODIFY_REG(RTC->DR,
        RTC_DR_YT_Msk | RTC_DR_YU_Msk | RTC_DR_MT_Msk | RTC_DR_MU_Msk | RTC_DR_DT_Msk | RTC_DR_DU_Msk | RTC_DR_WDU_Msk,
        (menu[7].value / 10 << RTC_DR_YT_Pos) | // Десятки років
        (menu[7].value % 10 << RTC_DR_YU_Pos) | // Одиниці років
        (menu[6].value / 10 << RTC_DR_MT_Pos) | // Десятки місяців
        (menu[6].value % 10 << RTC_DR_MU_Pos) | // Одиниці місяців
        (menu[5].value / 10 << RTC_DR_DT_Pos) | // Десятки днів
        (menu[5].value % 10 << RTC_DR_DU_Pos) | // Одиниці днів
        (menu[8].value << RTC_DR_WDU_Pos));     // День тижня
    // Оновлення значень дати та часу у регістрах RTC

    CLEAR_BIT(RTC->ISR, RTC_ISR_INIT);
    // Вихід з режиму ініціалізації

    RTC->WPR = 0xFE; // Відключення доступу на запис до регістрів RTC
    RTC->WPR = 0x64; // Відновлення зах
}

char *setActualMenu(int8_t v, int8_t h) {
    // Функція для встановлення актуального меню на основі вертикальних (v) і горизонтальних (h) переміщень

    if (v != 0) { // Рухаємося по вертикалі
        if (v == -1) { // Команда ВГОРУ (скасування)
            if (isParamEditMode) { // Якщо параметр у режимі редагування, то скасовуємо зміни
                isParamEditMode = false;
            } else { // Якщо пункт меню не у режимі редагування, переміщаємося до батька
                if (menu[actualIndex].parentid > 0) { // Якщо є куди переміщатися вгору (parentid > 0)
                    actualIndex = getMenuIndexByID(menu[actualIndex].parentid);
                    // Встановлюємо актуальний індекс на індекс батьківського елемента
                }
            }
        } else { // Якщо команда ВНИЗ - входу/редагування
            if (menu[actualIndex].isParam && !isParamEditMode) { // Якщо не в режимі редагування, то ...
                isParamEditMode = true; // Переходимо в режим редагування параметра
                tmpVal = menu[actualIndex].value; // Тимчасовій змінній присвоюємо актуальне значення параметра
            } else if (menu[actualIndex].isParam && isParamEditMode) { // Якщо в режимі редагування
                menu[actualIndex].value = tmpVal; // Зберігаємо задане значення
                isParamEditMode = false; // І виходимо з режиму редагування
            } else {
                bool nochild = true; // Прапорець, чи є дочірні елементи
                for (uint8_t i = 0; i < menuArraySize; i++) {
                    if (menu[i].parentid == menu[actualIndex].id) {
                        actualIndex = i; // Якщо є, робимо перший попавшийся актуальним елементом
                        nochild = false; // Потомки є
                        break; // Виходимо з for
                    }
                }
                if (nochild) { // Якщо ж потомків немає, воспринимаємо як команду
                    switch (menu[actualIndex].id) {
                        case 9: //  *9 P_0.7 Set
                            setTimeNow();
                            break;
                        case 16: // *16 P_2.2 Test lamp
                            testMainLamp();
                            break;
                        case 22: // *22 P_3.4 Alarm_Melody_test
                            SET_BIT(TIM2->CR1, TIM_CR1_CEN); // Запуск таймера
                            while (!(SysTimer_ms % 10000 == 9999)) {
                                StartMusic(menu[21].value); // *21 P_3.3 Alarm_Melody
                            }
                            CLEAR_BIT(TIM2->CR1, TIM_CR1_CEN); // Зупинка таймера
                            break;
                        case 30:
                            while (Clock()); // *29 P__6 Clock(StartWork)
                            break;
                        default:
                            break;
                    }
                }
            }
        }
    }

    if (h != 0) { // Якщо горизонтальна навігація
        if (isParamEditMode) { // У режимі редагування параметра
            tmpVal += h; // Змінюємо його значення і ...
            // ... контролюємо, щоб воно залишилося в заданому діапазоні
            if (tmpVal > menu[actualIndex]._max)
                tmpVal = menu[actualIndex]._min;
            if (tmpVal < menu[actualIndex]._min)
                tmpVal = menu[actualIndex]._max;
        } else { // Якщо режим редагування не активний, навігація серед дочірніх елементів одного батька
            actualIndex = getNearMenuIndexByID(menu[actualIndex].parentid,
                                               menu[actualIndex].id, h);
            // Встановлюємо актуальний індекс на основі батьківського ID, поточного ID та напрямку руху
        }
    }

    // Відображаємо інформацію
    if (isParamEditMode) {
        tmpV[0] = ((tmpVal / 1000) % 10 != 0) ? intToChar((tmpVal / 1000) % 10) : ' ';
        tmpV[1] = ((tmpVal / 100) % 10 != 0 || (tmpVal / 1000) % 10 != 0) ? intToChar((tmpVal / 100) % 10) : ' ';
        tmpV[2] = ((tmpVal / 10) % 10 != 0 || (tmpVal / 100) % 10 != 0) ? intToChar((tmpVal / 10) % 10) : ' ';
        tmpV[3] = intToChar(tmpVal % 10);
        // Перетворюємо значення параметра у символи для відображення
        return tmpV;
    } else {
        return menu[actualIndex]._name;
        // Повертаємо ім'я актуального меню
    }
}

uint8_t getMenuIndexByID(int8_t id) {
    // Функція отримання індексу пункту меню за його ID
    for (uint8_t i = 0; i < menuArraySize; i++) {
        if (menu[i].id == id) {
            return i; // Знаходимо і повертаємо індекс пункту меню
        }
    }
    return -1; // Якщо пункт меню не знайдено, повертаємо -1
}

uint8_t getNearMenuIndexByID(int8_t parentid, int8_t id, int8_t side) {
    // Функція отримання індексу пункту меню наступного або попереднього від актуального
    int8_t prevID = -1; // Змінна для зберігання індексу попереднього елемента
    int8_t nextID = -1; // Змінна для зберігання індексу наступного елемента
    int8_t actualID = -1; // Змінна для зберігання індексу актуального елемента

    int8_t firstID = -1; // Змінна для зберігання індексу першого елемента
    int8_t lastID = -1; // Змінна для зберігання індексу останнього елемента

    for (uint8_t i = 0; i < menuArraySize; i++) {
        if (menu[i].parentid == parentid) { // Перебираємо всі елементи з одним батьківським ID
            if (firstID == -1)
                firstID = i; // Запам'ятовуємо перший елемент списку

            if (menu[i].id == id) {
                actualID = i; // Запам'ятовуємо актуальний елемент списку
            } else {
                if (actualID == -1) { // Якщо зустрівся елемент до актуального, робимо його попереднім
                    prevID = i;
                } else if (actualID != -1 && nextID == -1) { // Якщо зустрівся елемент після актуального, робимо його наступним
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

void StartMusic(uint16_t melody) {
    TIM21->ARR = 99;
    TIM21->CCR1 = 49;
    // Налаштування таймера для відтворення музики
}

void interaptTIMDebounce(void) {
    // Обробка дебаунсу для кнопок

    // *26 P_5.1 debounceTime
    if (((SysTimer_ms - DecrementButtonDebounce) > menu[28].value) && (READ_BIT(EXTI->IMR, EXTI_IMR_IM0) == 0)) {
        SET_BIT(EXTI->IMR, EXTI_IMR_IM0);
        // Перевіряємо, чи минув час дебаунсу для кнопки Decrement і чи переривання не увімкнено, якщо так - увімкнемо переривання
    }

    if (((SysTimer_ms - EnterButtonDebounce) > menu[28].value) && (READ_BIT(EXTI->IMR, EXTI_IMR_IM1) == 0)) {
        SET_BIT(EXTI->IMR, EXTI_IMR_IM1);
        // Перевіряємо, чи минув час дебаунсу для кнопки Enter і чи переривання не увімкнено, якщо так - увімкнемо переривання
    }

    if (((SysTimer_ms - IncrementButtonDebounce) > menu[28].value) && (READ_BIT(EXTI->IMR, EXTI_IMR_IM2) == 0)) {
        SET_BIT(EXTI->IMR, EXTI_IMR_IM2);
        // Перевіряємо, чи минув час дебаунсу для кнопки Increment і чи переривання не увімкнено, якщо так - увімкнемо переривання
    }

    if ((flagDecrementButtonDown == 0) && (flagEnterButtonDown == 0) && (flagIncrementButtonDown == 0) && SysTimer_ms % 10000 == 9999) {
        SysTimer_ms = 0;
        // Якщо жодна кнопка не натиснута і системний таймер досягає певного значення, то скидаємо системний таймер
    }
}

/* Handlers--------------------------------------------------------*/
void SysTick_Handler(void) {
    SysTimer_ms++;
    // Збільшуємо системний таймер на 1 мілісекунду

    if (Delay_counter_ms) {
        Delay_counter_ms--;
        // Якщо лічильник затримки не дорівнює нулю, зменшуємо його
    }
}

void EXTI0_1_IRQHandler(void) {
    // Обробник переривань для кнопок EXTI0 і EXTI1

    // Перевірка для DecrementButton (EXTI3)
    if (EXTI->PR & EXTI_PR_PR0) {
        CLEAR_BIT(EXTI->IMR, EXTI_IMR_IM0);
        // Вимикаємо маску переривання для лінії 0

        DecrementButtonDebounce = SysTimer_ms;
        // Зберігаємо час debounce для DecrementButton

        if (!flagDecrementButtonDown) {
            timeDecrementButtonDown = SysTimer_ms;
            // Зберігаємо час натискання кнопки Decrement

            flagDecrementButtonDown = true;
            // Встановлюємо прапорець натискання кнопки Decrement

            CLEAR_BIT(EXTI->FTSR, EXTI_FTSR_FT0);
            SET_BIT(EXTI->RTSR, EXTI_RTSR_RT0);
            // Змінюємо налаштування тригера з падіння на зростання для відстеження відпускання
        } else {
            flagDecrementButtonDown = false;
            // Скидаємо прапорець натискання кнопки Decrement

            CLEAR_BIT(EXTI->RTSR, EXTI_RTSR_RT0);
            SET_BIT(EXTI->FTSR, EXTI_FTSR_FT0);
            // Змінюємо налаштування тригера зі зростання на падіння для відстеження наступного натискання

            // Перевіряємо, чи була кнопка натиснута довго
            if (SysTimer_ms - timeDecrementButtonDown > 4 * menu[29].value) {
                flagDecrementButton = false;
                flagDecrementButtonLong = true;
                // Встановлюємо прапорець довгого натискання
            } else {
                flagDecrementButtonLong = false;
                flagDecrementButton = true;
                // Встановлюємо прапорець короткого натискання
            }
        }

        EXTI->PR = EXTI_PR_PR0;
        // Скидаємо прапорець переривання на лінії 0
    }

    // Перевірка для EnterButton (EXTI1)
    if (EXTI->PR & EXTI_PR_PR1) {
        CLEAR_BIT(EXTI->IMR, EXTI_IMR_IM1);
        // Вимикаємо маску переривання для лінії 1

        EnterButtonDebounce = SysTimer_ms;
        // Зберігаємо час debounce для EnterButton

        if (!flagEnterButtonDown) {
            timeEnterButtonDown = SysTimer_ms;
            // Зберігаємо час натискання кнопки Enter

            flagEnterButtonDown = true;
            // Встановлюємо прапорець натискання кнопки Enter

            CLEAR_BIT(EXTI->FTSR, EXTI_FTSR_FT1);
            SET_BIT(EXTI->RTSR, EXTI_RTSR_RT1);
            // Змінюємо налаштування тригера з падіння на зростання для відстеження відпускання
        } else {
            flagEnterButtonDown = false;
            // Скидаємо прапорець натискання кнопки Enter

            CLEAR_BIT(EXTI->RTSR, EXTI_RTSR_RT1);
            SET_BIT(EXTI->FTSR, EXTI_FTSR_FT1);
            // Змінюємо налаштування тригера зі зростання на падіння для відстеження наступного натискання

            // Перевіряємо, чи була кнопка натиснута довго
            if (SysTimer_ms - timeEnterButtonDown > 4 * menu[29].value) {
                flagEnterButton = false;
                flagEnterButtonLong = true;
                // Встановлюємо прапорець довгого натискання
            } else {
                flagEnterButtonLong = false;
                flagEnterButton = true;
                // Встановлюємо прапорець короткого натискання
            }
        }

        EXTI->PR = EXTI_PR_PR1;
        // Скидаємо прапорець переривання на лінії 1
    }
}

void EXTI2_3_IRQHandler(void) {
    // Обробник переривань для ліній EXTI2 та EXTI3

    // Перевірка, чи було переривання від лінії EXTI2
    if (EXTI->PR & EXTI_PR_PR2) {
        // Забороняємо переривання для даної лінії, поки не завершимо обробку
        CLEAR_BIT(EXTI->IMR, EXTI_IMR_IM2);
        IncrementButtonDebounce = SysTimer_ms;
        // Зберігаємо час debounce для IncrementButton

        if (!flagIncrementButtonDown) {
            // Початок натискання
            timeIncrementButtonDown = SysTimer_ms;
            // Зберігаємо час натискання кнопки Increment

            flagIncrementButtonDown = true;
            // Встановлюємо прапорець натискання кнопки Increment

            // Переводимо переривання на спадаючий фронт для відстеження відпускання
            CLEAR_BIT(EXTI->FTSR, EXTI_FTSR_FT2); // Включаємо переривання по спадаючому фронту
            SET_BIT(EXTI->RTSR, EXTI_RTSR_RT2);   // Вимикаємо переривання по зростаючому фронту
        } else {
            // Кнопка відпущена
            // Скидаємо прапорець натискання
            flagIncrementButtonDown = false;

            // Переводимо переривання на зростаючий фронт для відстеження наступного натискання
            CLEAR_BIT(EXTI->RTSR, EXTI_RTSR_RT2); // Включаємо переривання по зростаючому фронту
            SET_BIT(EXTI->FTSR, EXTI_FTSR_FT2);   // Вимикаємо переривання по спадаючому фронту

            // Обробка короткого та довгого натискання
            if (SysTimer_ms - timeIncrementButtonDown > 4 * menu[29].value) { // *27 P_5.2 timeButtonLongPressed
                // Довге натискання
                flagIncrementButtonLong = true;
                flagIncrementButton = false;
            } else {
                // Коротке натискання
                flagIncrementButton = true;
                flagIncrementButtonLong = false;
            }
        }
        // Скидаємо прапорець EXTI2
        EXTI->PR = EXTI_PR_PR2;
    }
}

void EXTI4_15_IRQHandler(void) {
    // Обробник переривань для ліній EXTI4 - EXTI15

    // Перевіряємо, чи було переривання від лінії EXTI9
    if (EXTI->PR & EXTI_PR_PR9) {
        // Забороняємо переривання для лінії EXTI9
        EXTI->IMR &= ~EXTI_IMR_IM9;

        // Перевірка стану піну 9 (припускаємо, що сигнал 0 – активний)
        if (BUTTON_PRESSED(9, GPIOB) == 0) { // Якщо сигнал 0
            // Переводимо переривання на зростаючий фронт (чекаємо на сигнал 1)
            EXTI->RTSR |= EXTI_RTSR_RT9;  // Увімкнення переривання на зростаючий фронт
            EXTI->FTSR &= ~EXTI_FTSR_FT9; // Вимкнення переривання на спадний фронт

            // Входимо в режим мінімального енергоспоживання
            LowPowerMode(ENTER);
        } else { // Якщо сигнал 1 (відновлення сигналу)
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

void TIM2_IRQHandler(void) {
    // Обробник переривань для таймера 2
    if (READ_BIT(TIM2->SR, TIM_SR_UIF)) CLEAR_BIT(TIM2->SR, TIM_SR_UIF);
    // Якщо встановлений прапорець переривання, скидаємо його
}

void TIM21_IRQHandler(void) {
    // Обробник переривань для таймера 21
    if (READ_BIT(TIM21->SR, TIM_SR_UIF)) CLEAR_BIT(TIM21->SR, TIM_SR_UIF);
    // Якщо встановлений прапорець переривання, скидаємо його
}

void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* Користувач може додати свою реалізацію для повідомлення про стан помилки HAL */
    __disable_irq();
    while (1) {
        // Зациклюємося у випадку помилки, вимикаємо переривання
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Повідомляє ім'я вихідного файлу та номер рядка,
 *         де виникла помилка assert_param.
 * @param  file: вказівник на ім'я вихідного файлу
 * @param  line: номер рядка, де виникла помилка assert_param
 * @retval None
 */
void assert_failed(uint8_t *file, uint8_t line) {
    /* USER CODE BEGIN 6 */
    /* Користувач може додати свою реалізацію для повідомлення імені файлу та номера рядка,
       наприклад: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
