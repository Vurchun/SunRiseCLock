/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
#include <stdbool.h>            // Підключаємо бібліотеку для використання булевих значень
#include "math.h"               // Підключаємо бібліотеку для математичних операцій


/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Макрос для налаштування GPIO
#define CONFIGURE_GPIO(PORT, PIN, MODE, TYPE, SPEED)     \
    MODIFY_REG(PORT->MODER, GPIO_MODER_MODE##PIN##_Msk, MODE << GPIO_MODER_MODE##PIN##_Pos); \
    MODIFY_REG(PORT->OTYPER, GPIO_OTYPER_OT_##PIN, TYPE << GPIO_OTYPER_OT_##PIN); \
    MODIFY_REG(PORT->OSPEEDR, GPIO_OSPEEDER_OSPEED##PIN##_Msk, SPEED << GPIO_OSPEEDER_OSPEED##PIN##_Pos);

// Макрос для налаштування EXTI (зовнішні переривання)
#define CONFIGURE_EXTI(PIN, PORT_SOURCE, CR, LINE, EDGE)     \
    MODIFY_REG(SYSCFG->EXTICR[CR], SYSCFG_EXTICR##LINE##_EXTI##PIN##_Msk, PORT_SOURCE << SYSCFG_EXTICR##LINE##_EXTI##PIN##_Pos); \
    SET_BIT(EXTI->IMR, EXTI_IMR_IM##PIN); \
    SET_BIT(EXTI->FTSR, EXTI_FTSR_FT##PIN); \
    if (EDGE == 1) SET_BIT(EXTI->RTSR, EXTI_RTSR_RT##PIN);

// Макроси для вмикання/вимикання різних світлодіодів (LEDs)
#define LEDa_ON()        GPIOA->BSRR = GPIO_BSRR_BS_7
#define LEDa_OFF()       GPIOA->BSRR = GPIO_BSRR_BR_7
#define LEDb_ON()        GPIOB->BSRR = GPIO_BSRR_BS_1
#define LEDb_OFF()       GPIOB->BSRR = GPIO_BSRR_BR_1
#define LEDc_ON()        GPIOA->BSRR = GPIO_BSRR_BS_6
#define LEDc_OFF()       GPIOA->BSRR = GPIO_BSRR_BR_6
#define LEDd_ON()        GPIOA->BSRR = GPIO_BSRR_BS_5
#define LEDd_OFF()       GPIOA->BSRR = GPIO_BSRR_BR_5
#define LEDe_ON()        GPIOA->BSRR = GPIO_BSRR_BS_11
#define LEDe_OFF()       GPIOA->BSRR = GPIO_BSRR_BR_11
#define LEDf_ON()        GPIOA->BSRR = GPIO_BSRR_BS_9
#define LEDf_OFF()       GPIOA->BSRR = GPIO_BSRR_BR_9
#define LEDg_ON()        GPIOB->BSRR = GPIO_BSRR_BS_0
#define LEDg_OFF()       GPIOB->BSRR = GPIO_BSRR_BR_0
#define LEDdp_ON()       GPIOB->BSRR = GPIO_BSRR_BS_3
#define LEDdp_OFF()      GPIOB->BSRR = GPIO_BSRR_BR_3

// Макроси для керування дисплеями
#define LEDD1_OFF()      GPIOA->BSRR = GPIO_BSRR_BR_3
#define LEDD1_ON()       GPIOA->BSRR = GPIO_BSRR_BS_3
#define LEDD2_OFF()      GPIOA->BSRR = GPIO_BSRR_BR_4
#define LEDD2_ON()       GPIOA->BSRR = GPIO_BSRR_BS_4
#define LEDD3_OFF()      GPIOA->BSRR = GPIO_BSRR_BR_12
#define LEDD3_ON()       GPIOA->BSRR = GPIO_BSRR_BS_12
#define LEDD4_OFF()      GPIOB->BSRR = GPIO_BSRR_BR_4
#define LEDD4_ON()       GPIOB->BSRR = GPIO_BSRR_BS_4

// Макроси для додаткових індикаторів
#define LEDl1l2_ON()     GPIOA->BSRR = GPIO_BSRR_BS_10
#define LEDl1l2_OFF()    GPIOA->BSRR = GPIO_BSRR_BR_10
#define LEDalarm_ON()    GPIOA->BSRR = GPIO_BSRR_BS_8
#define LEDalarm_OFF()   GPIOA->BSRR = GPIO_BSRR_BR_8

// Макроси для керування додатковими пинами
#define pinEN_ON()       GPIOC->BSRR = GPIO_BSRR_BS_15
#define pinEN_OFF()      GPIOC->BSRR = GPIO_BSRR_BR_15

#define SYSCLK 32000000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* PM */
// Немає користувацьких макросів
/* END PM */

/* Private variables ---------------------------------------------------------*/
bool flagDecrementButton;       				// Прапорець для натискання кнопки зменшення
bool flagEnterButton;           				// Прапорець для натискання кнопки підтвердження
bool flagIncrementButton;       				// Прапорець для натискання кнопки збільшення
bool flagDecrementButtonLong;   				// Прапорець для довгого утримання кнопки зменшення
bool flagEnterButtonLong;       				// Прапорець для довгого утримання кнопки підтвердження
bool flagIncrementButtonLong;   				// Прапорець для довгого утримання кнопки збільшення
bool flagDecrementButtonDown;  					// Було натискання кнопки
bool flagEnterButtonDown;  						// Було натискання кнопки
bool flagIncrementButtonDown;  					// Було натискання кнопки

unsigned int timeButtonLongPressed = 675; 		// Довге утримання кнопки після 1,5 секунд
unsigned int timeButtonPressed = 175; 			// Довге утримання кнопки після 1,5 секунд
unsigned int timeDecrementButtonDown = 0;  		// Змінна, що зберігає час натискання кнопки
unsigned int timeEnterButtonDown = 0;  			// Змінна, що зберігає час натискання кнопки
unsigned int timeIncrementButtonDown = 0;  		// Змінна, що зберігає час натискання кнопки


int menuArraySize = 26;         				// Встановлюємо розмір масиву меню
int actualIndex = 0;            				// Поточний індекс меню
bool isParamEditMode = false;   				// Прапорець режиму редагування параметра
int tmpVValue = 0;              				// Тимчасова змінна для зберігання параметра

volatile uint32_t SysTimer_ms = 0;  			// Системний таймер (аналог HAL_GetTick)
volatile uint32_t Delay_counter_ms = 0;  		// Лічильник для затримки

int pwmcount = 0;               				// Лічильник PWM
int CounterTIM2 = 0;            				// Лічильник таймера 2
int CounterTIM21 = 0;           				// Лічильник таймера 21

// Структура меню
struct strMenu {
    int id;         							// Унікальний ідентифікаційний індекс ID
    int parentid;   							// ID батька (вкладеність)
    bool isParam;   							// Чи є пункт змінним параметром
    char _name[4];  							// Назва пункту меню
    int value;      							// Поточне значення параметра
    int _min;       							// Мінімально можливе значення
    int _max;       							// Максимально можливе значення
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
 *24 	P__5		Clock(StartWork)
 */
struct strMenu menu[] = {                       // Встановлюємо пункти меню
	{0, -1,    false,	"PPPP",		0, 0, 	0},
	//-----------------------------------------------------------------------
	{1, 0,     false,	"P__0",		0, 0, 	0},
	{2, 1,     true,	"P_00",		0, 0, 	24},
	{3, 1,     true,	"P_01", 	0, 0, 	59},
	{4, 1,     true,	"P_02",		0, 0, 	59},
	{5, 1,     true,	"P_03", 	0, 0, 	31},
	{6, 1,     true,	"P_04",		0, 0, 	12},
	{7, 1,     true,	"P_05", 	0, 0, 	99},
	{8, 1,     true,	"P_06",		0, 1, 	7},
	{9, 1,     false, 	"P_07", 	0, 0, 	0},
	//-----------------------------------------------------------------------
	{10, 0,    false, 	"P__1", 	0, 0, 	0},
	{11, 10,   true,	"P_10", 	0, 23, 	0},
	{12, 10,   true,	"P_11", 	0, 59, 	0},
	//-----------------------------------------------------------------------
	{13, 0,    false, 	"P__2", 	0, 0, 	0},
	{14, 13,   true,	"P_20", 	0, 999, 60},
	{15, 13,   true,	"P_21", 	0, 272, 224},
	//-----------------------------------------------------------------------
	{16, 0,    false, 	"P__3", 	0, 0, 	0},
	{17, 16,   true,	"P_30", 	0, 1, 	1},
	{18, 16,   true,	"P_31", 	0, 0, 	24},
	{19, 16,   true,	"P_32", 	0, 0, 	59},
	{20, 16,   true,	"P_33", 	0, 7, 	0},
	{21, 16,   false,	"P_34", 	0, 0, 	0},
	//-----------------------------------------------------------------------
	{22, 0,    false, 	"P__4", 	0, 0, 	1},
	{23, 22,   true,	"P_40", 	0, 3, 	0},
	{24, 22,   true,	"P_41", 	0, 1, 	1},
	//-----------------------------------------------------------------------
	{25, 0,    false, 	"P__5", 	0, 0, 	0}
	//-----------------------------------------------------------------------
};

/* Sound/Buzzer variables ---------------------------------------------------------*/
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
/* END S/BV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void GPIO_Init(void);
void LPUART1_UART_Init(void);
void RTC_Init(void);
void TIM2_Init(void);
void TIM21_Init(void);
/*Handlers*/
void Error_Handler(void);
void assert_failed(uint8_t *file, uint32_t line);
void EXTI0_1_IRQHandler (void);
void EXTI2_3_IRQHandler (void);
void EXTI4_15_IRQHandler (void);
void LPTIM1_IRQHandler (void);
void TIM2_IRQHandler (void);
void TIM21_IRQHandler (void);
void LPUART1_IRQHandler (void);
void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);

/* USER CODE BEGIN PFP */
void Delay_ms(uint32_t Milliseconds);
double custom_pow(double a, double x);
int custom_floor(double x);
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
void pwmFP7103();
/*----------------------------------------------------------------------------*/
void setTimeNow();
int Clock();
/*----------------------------------------------------------------------------*/
char intToChar(int num);
void writeCHARSEG(char CHAR, int seg);
char* setActualMenu(int v, int h);
int getMenuIndexByID(int id);
int getNearMenuIndexByID(int parentid, int id, int side);
/*----------------------------------------------------------------------------*/
void StartMusic(int melody);
void sound (int freq, int time_ms);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
int main(void) {
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
//	HAL_Init();

	SystemClock_Config();
	GPIO_Init();
	LPUART1_UART_Init();
	RTC_Init();
	TIM2_Init();
	TIM21_Init();

	writeCHARSEG(' ', ' ');
	pinEN_OFF();

	int vmenu = 0; // Змінна, що зберігає дію по вертикалі 1 - вхід в меню, -1 - вихід з меню
	int hmenu = 0; // Змінна, що зберігає дію по горизонталі 1 - вправо, -1 - вліво
	char* tmpValue;

	while(1){
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
	}
}

void SystemClock_Config(void) {
	// Налаштування PWR CR для регулювання напруги
	MODIFY_REG(PWR->CR, PWR_CR_VOS_Msk, 0b01 << PWR_CR_VOS_Pos);

	// Вимкнення HSI16DIV
	CLEAR_BIT(RCC->CR, RCC_CR_HSIDIVEN);

	// Вимкнення MSI
	CLEAR_BIT(RCC->CR, RCC_CR_MSION);

	// Якщо джерело системної частоти не HSI16
	if ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_HSI) {
		// Увімкнення HSI16
		SET_BIT(RCC->CR, RCC_CR_HSION);

		// Очікування стабілізації HSI16
		while (!(RCC->CR & RCC_CR_HSIRDY)) {
		}

		// Перемикання системної тактової частоти на HSI16
		MODIFY_REG(RCC->CFGR, RCC_CFGR_SW_Msk, RCC_CFGR_SW_HSI);

		// Очікування завершення перемикання
		while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_HSI) {
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
	while (!(RCC->CR & RCC_CR_PLLRDY)) {
	}

	// Перемикання системної частоти на PLL
	MODIFY_REG(RCC->CFGR, RCC_CFGR_SW_Msk, RCC_CFGR_SW_PLL);

	// Очікування завершення перемикання
	while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_PLL) {
	}
}

void LPUART1_UART_Init(void) {
	// Увімкнення тактування GPIOB
	RCC->IOPENR |= RCC_IOPENR_IOPBEN;
	// Настроить пин PB6/PB7 на режим альтернативной функции
	CONFIGURE_GPIO(GPIOB, 6, 0b10, 0, 0b11);  // TX
	CONFIGURE_GPIO(GPIOB, 7, 0b10, 0, 0b11);  // RX
	// Настроить альтернативную функцию AFR для пина PB6/PB7
	MODIFY_REG(GPIOB->AFR[0], GPIO_AFRL_AFSEL6_Msk, 0b0110 << GPIO_AFRL_AFSEL6_Pos);
	MODIFY_REG(GPIOB->AFR[0], GPIO_AFRL_AFSEL7_Msk, 0b0110 << GPIO_AFRL_AFSEL7_Pos);
	
    RCC->APB1ENR |= RCC_APB1ENR_LPUART1EN;// Включаємо тактування для LPUART1
    LPUART1->CR1 &= ~USART_CR1_UE; // Скидаємо налаштування LPUART1 перед конфігуруванням // Вимикаємо UART для конфігурації
    // Встановлюємо швидкість передачі (Baud rate) = 115200
    // Для LPUART1 швидкість передачі обчислюється за формулою: baud = (ClockFreq) / (PRESC * (USARTDIV+1))
    // У даному випадку PRESC = 1 (немає переддільника), тому:
    // Baud rate = (System clock) / (USARTDIV+1), де USARTDIV = (System clock / Baud) - 1
    uint32_t uartdiv = (SYSCLK / 115200) - 1;
    LPUART1->BRR = uartdiv;
    LPUART1->CR1 |= USART_CR1_M1; // Конфігуруємо довжину слова - 7 біт (M1=1, M0=0)
    LPUART1->CR2 &= ~USART_CR2_STOP; // Налаштовуємо стоп-біти - 1 стоп-біт (SBK=0)
    LPUART1->CR1 &= ~USART_CR1_PCE; // Встановлюємо парність - без парності (PCE=0)
    LPUART1->CR1 |= USART_CR1_RE | USART_CR1_TE;// Увімкнення режиму прийому та передачі (RE=1, TE=1)
    LPUART1->CR1 |= USART_CR1_UE;// Увімкнення LPUART1

}

void RTC_Init(void) {
    // 1. Enable power and backup domain access
    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN);
    SET_BIT(PWR->CR, PWR_CR_DBP);

    // 2. Enable LSE Oscillator
    SET_BIT(RCC->CSR, RCC_CSR_LSEON);
    while (!(READ_BIT(RCC->CSR, RCC_CSR_LSERDY))) {}

    // 3. Set LSE as RTC clock source and enable RTC
    MODIFY_REG(RCC->CSR, RCC_CSR_RTCSEL_Msk, 0b01 << RCC_CSR_RTCSEL_Pos);  // LSE selected as RTC clock
    SET_BIT(RCC->CSR, RCC_CSR_RTCEN);

    // 4. Disable RTC write protection
    RTC->WPR = 0xCA;  // Step 1
    RTC->WPR = 0x53;  // Step 2

    // 5. Enter initialization mode
    SET_BIT(RTC->ISR, RTC_ISR_INIT);
    while (!(READ_BIT(RTC->ISR, RTC_ISR_INITF))) {}

    // 6. Set the time in BCD format (17:36:00)
    MODIFY_REG(RTC->TR,
               RTC_TR_HT_Msk | RTC_TR_HU_Msk | RTC_TR_MNT_Msk | RTC_TR_MNU_Msk | RTC_TR_ST_Msk | RTC_TR_SU_Msk,
               (0x1 << RTC_TR_HT_Pos)  |   // Hour tens (1 -> 17)
               (0x7 << RTC_TR_HU_Pos)  |   // Hour units (7 -> 17)
               (0x3 << RTC_TR_MNT_Pos) |   // Minute tens (3 -> 36)
               (0x6 << RTC_TR_MNU_Pos) |   // Minute units (6 -> 36)
               (0x0 << RTC_TR_ST_Pos)  |   // Second tens (0 -> 00)
               (0x0 << RTC_TR_SU_Pos));    // Second units (0 -> 00)

    // 7. Set the date in BCD format (01/02/2024, Monday)
    MODIFY_REG(RTC->DR,
               RTC_DR_YT_Msk | RTC_DR_YU_Msk | RTC_DR_MT_Msk | RTC_DR_MU_Msk | RTC_DR_DT_Msk | RTC_DR_DU_Msk | RTC_DR_WDU_Msk,
               (0x2 << RTC_DR_YT_Pos)  |  // Year tens (2 -> 2024)
               (0x4 << RTC_DR_YU_Pos)  |  // Year units (4 -> 2024)
               (0x0 << RTC_DR_MT_Pos)  |  // Month tens (1 -> April)
               (0x4 << RTC_DR_MU_Pos)  |  // Month units (0 -> April)
               (0x0 << RTC_DR_DT_Pos)  |  // Day tens (0 -> 01)
               (0x1 << RTC_DR_DU_Pos)  |  // Day units (1 -> 01)
               (0x2 << RTC_DR_WDU_Pos)); // Weekday (3 -> Monday)

    // 8. Exit initialization mode
    CLEAR_BIT(RTC->ISR, RTC_ISR_INIT);

    // 9. Re-enable RTC write protection  
	RTC->WPR = 0xFE; // Disable write access for RTC register
	RTC->WPR = 0x64; //				-||-
}

void TIM2_Init(void) {

	// Увімкнення тактування GPIOA (для PA15, як PWM вихід)
	RCC->IOPENR |= RCC_IOPENR_IOPAEN;
	// Настроить пин 15 на режим альтернативной функции
	CONFIGURE_GPIO(GPIOB, 15, 0b10, 0, 0b11);  // BuzzerPin
	// Настроить альтернативную функцию AF1 для пина 15
	MODIFY_REG(GPIOA->AFR[1], GPIO_AFRH_AFSEL15_Msk, 0b0101 << GPIO_AFRH_AFSEL15_Pos);

	// Увімкнення тактування TIM2
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	CLEAR_BIT(TIM2->CR1, TIM_CR1_CEN);

	/*Настройка таймера 2*/
		//TIMx control register 1 (TIMx_CR1)
		//SET_BIT(TIM2->CR1, TIM_CR1_CEN);  //Запуск таймера
		CLEAR_BIT(TIM2->CR1, TIM_CR1_UDIS);//Генерировать событие Update
		CLEAR_BIT(TIM2->CR1, TIM_CR1_URS);  //Генерировать прерывание
		CLEAR_BIT(TIM2->CR1, TIM_CR1_OPM); //One pulse mode off(Счетчик не останавливается при обновлении)
		CLEAR_BIT(TIM2->CR1, TIM_CR1_DIR);  //Считаем вверх
		MODIFY_REG(TIM2->CR1, TIM_CR1_CMS_Msk, 0b00 << TIM_CR1_CMS_Pos); //Выравнивание по краю
		SET_BIT(TIM2->CR1, TIM_CR1_ARPE); //Auto-reload preload enable
		MODIFY_REG(TIM2->CR1, TIM_CR1_CKD_Msk, 0b00 << TIM_CR1_CKD_Pos); //Предделение выключено

		/*Настройка прерываний*/
		//TIMx DMA/Interrupt enable register (TIMx_DIER)
		SET_BIT(TIM2->DIER, TIM_DIER_UIE);//Update interrupt enable

		//TIMx status register (TIMx_SR) - Статусные регистры

		TIM2->PSC = 3200 - 1;
		TIM2->ARR = 10000 - 1;

		NVIC_EnableIRQ(TIM2_IRQn); //Разрешить прерывания по таймеру 2
		/*Настройка шим(Канал 1)*/
		MODIFY_REG(TIM2->CCMR1, TIM_CCMR1_CC1S_Msk, 0b00 << TIM_CCMR1_CC1S_Pos); //CC1 channel is configured as output
		CLEAR_BIT(TIM2->CCMR1, TIM_CCMR1_OC1FE); //Fast mode disable
		SET_BIT(TIM2->CCMR1, TIM_CCMR1_OC1PE);  //Preload enable
		MODIFY_REG(TIM2->CCMR1, TIM_CCMR1_OC1M_Msk, 0b110 << TIM_CCMR1_OC1M_Pos); //PWM MODE 1
		CLEAR_BIT(TIM2->CCMR1, TIM_CCMR1_OC1CE); //OC1Ref is not affected by the ETRF input

		/*Запуск ШИМ*/
		//15.4.9 TIMx capture/compare enable register (TIMx_CCER)
		SET_BIT(TIM2->CCER, TIM_CCER_CC1E);//On - OC1 signal is output on the corresponding output pin.
		CLEAR_BIT(TIM21->CCER, TIM_CCER_CC1P); //OC1 active high.


		SET_BIT(TIM2->CR1, TIM_CR1_CEN);
}

void TIM21_Init(void) {
	// Увімкнення тактування GPIOB (для PB5, як PWM вихід)
	RCC->IOPENR |= RCC_IOPENR_IOPBEN;
	// Настроить пин 5 на режим альтернативной функции
	CONFIGURE_GPIO(GPIOB, 5, 0b10, 0, 0b11);  // mainLED
	// Настроить альтернативную функцию AF1 для пина 5
	MODIFY_REG(GPIOB->AFR[0], GPIO_AFRL_AFSEL5_Msk, 0b0101 << GPIO_AFRL_AFSEL5_Pos);

	// Увімкнення тактування TIM21
	RCC->APB2ENR |= RCC_APB2ENR_TIM21EN;
	CLEAR_BIT(TIM21->CR1, TIM_CR1_CEN);

	/*Настройка таймера 21*/
	//TIMx control register 1 (TIMx_CR1)
	//SET_BIT(TIM21->CR1, TIM_CR1_CEN);  //Запуск таймера
	CLEAR_BIT(TIM21->CR1, TIM_CR1_UDIS);//Генерировать событие Update
	CLEAR_BIT(TIM21->CR1, TIM_CR1_URS);  //Генерировать прерывание
	CLEAR_BIT(TIM21->CR1, TIM_CR1_OPM); //One pulse mode off(Счетчик не останавливается при обновлении)
	CLEAR_BIT(TIM21->CR1, TIM_CR1_DIR);  //Считаем вверх
	MODIFY_REG(TIM21->CR1, TIM_CR1_CMS_Msk, 0b00 << TIM_CR1_CMS_Pos); //Выравнивание по краю
	SET_BIT(TIM21->CR1, TIM_CR1_ARPE); //Auto-reload preload enable
	MODIFY_REG(TIM21->CR1, TIM_CR1_CKD_Msk, 0b00 << TIM_CR1_CKD_Pos); //Предделение выключено

	/*Настройка прерываний*/
	//TIMx DMA/Interrupt enable register (TIMx_DIER)
	SET_BIT(TIM21->DIER, TIM_DIER_UIE);//Update interrupt enable

	//TIMx status register (TIMx_SR) - Статусные регистры
	TIM21->PSC = 32 - 1;
	TIM21->ARR = 10000 - 1;

	NVIC_EnableIRQ(TIM21_IRQn); //Разрешить прерывания по таймеру 21

	/*Настройка шим(Канал 1)*/
	MODIFY_REG(TIM21->CCMR1, TIM_CCMR1_CC1S_Msk, 0b00 << TIM_CCMR1_CC1S_Pos); //CC1 channel is configured as output
	CLEAR_BIT(TIM21->CCMR1, TIM_CCMR1_OC1FE); //Fast mode disable
	SET_BIT(TIM21->CCMR1, TIM_CCMR1_OC1PE);  //Preload enable
	MODIFY_REG(TIM21->CCMR1, TIM_CCMR1_OC1M_Msk, 0b110 << TIM_CCMR1_OC1M_Pos); //PWM MODE 1
	CLEAR_BIT(TIM21->CCMR1, TIM_CCMR1_OC1CE); //OC1Ref is not affected by the ETRF input

	/*Запуск ШИМ*/
	// TIMx capture/compare enable register (TIMx_CCER)
	SET_BIT(TIM21->CCER, TIM_CCER_CC1E);//On - OC1 signal is output on the corresponding output pin.
	SET_BIT(TIM21->CCER, TIM_CCER_CC1P); //OC1 active high.

	SET_BIT(TIM21->CR1, TIM_CR1_CEN);
}

void GPIO_Init(void) {
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
	CONFIGURE_GPIO(GPIOA, 0, 0b00, 0, 0b11);  // decrement
	CONFIGURE_EXTI(0, 0b000, 0, 1, 0);           // EXTI для decrement

	CONFIGURE_GPIO(GPIOA, 1, 0b00, 0, 0b11);  // enter
	CONFIGURE_EXTI(1, 0b000, 0, 1, 0);           // EXTI для enter

	CONFIGURE_GPIO(GPIOA, 2, 0b00, 0, 0b11);  // increment
	CONFIGURE_EXTI(2, 0b000, 0, 1, 0);           // EXTI для increment

	CONFIGURE_GPIO(GPIOB, 9, 0b00, 0, 0b11);  // pwr
	CONFIGURE_EXTI(9, 0b000, 2, 3, 1);   // EXTI для pwr з обробкою по зростанню
}

void Delay_ms(uint32_t Milliseconds) {
	Delay_counter_ms = Milliseconds;
	while (Delay_counter_ms != 0);
}

int custom_floor(double x) {
    int result = (int)x;  // Приведення до int обрізає дробову частину
    if (x < 0 && x != result) {
        result--;  // Якщо x від'ємне і не ціле, округляємо до меншого
    }
    return result;
}

double custom_pow(double a, double x)
{
    // Разделяем x на целую и дробную части
    int intPart = (int)x; // целая часть x
    double fracPart = x - intPart;     // дробная часть x

    // Возведение a в целую степень
    double result = 1.0;
    for (int i = 0; i < intPart; ++i) {
        result *= a;
    }

    // Аппроксимация дробной части (для x между 2 и 3 можно ограничиться линейной аппроксимацией)
    double fractionalMultiplier = 1.0;
    if (fracPart > 0.0) {
        fractionalMultiplier = 1.0 + fracPart * (a - 1.0); // Пример аппроксимации для дробной части
    }

    return result * fractionalMultiplier;
}

char intToChar(int num) {
	switch (num) {
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
		return '?';  // Повертаємо '?' для невідомого числа
	}
}

void writeCHARSEG(char CHAR, int seg) {
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

void pwmFP7103() {
	if (menu[16].value) {										//*16 		P_3.0	Alarm_Status
		int timeWakeUp 	= menu[10].value 	* 3600				//*10 		P_1.0	Hour_Rise
						+ menu[11].value	* 60;				//*11 		P_1.1	Minute_Rise
		int timeNow 	= (READ_BIT(RTC->TR,RTC_TR_HT)	*10+READ_BIT(RTC->TR,RTC_TR_HU))	* 3600
						+ (READ_BIT(RTC->TR,RTC_TR_MNT)	*10+READ_BIT(RTC->TR,RTC_TR_MNU))	* 60
						+ (READ_BIT(RTC->TR,RTC_TR_ST)	*10+READ_BIT(RTC->TR,RTC_TR_SU))	;
		if(menu[13].value * 60 >= timeWakeUp - timeNow){		// *13 		P_2.0	Period_Rising
			pinEN_ON();
			SET_BIT(TIM21->CR1, TIM_CR1_CEN);  					//Запуск таймера
			TIM21->CCR1 = custom_floor(TIM2->ARR * custom_pow((1 - timeNow / timeWakeUp), menu[14].value/100));
																// *14 		P_2.1	ɣ_Coefient_Rising
		}
	} else {
			pinEN_OFF();
			CLEAR_BIT(TIM21->CR1, TIM_CR1_CEN);  				//Призупинення таймера
			TIM21->CCR1 = 0;
		}
}

int Clock(){
	char tmpClock[4]={};
	int j = 0;
	tmpClock[0] = READ_BIT(RTC->TR,RTC_TR_HT);
	if (tmpClock[0] == 0){j=1;}
	tmpClock[1] = READ_BIT(RTC->TR,RTC_TR_HU);
	tmpClock[2] = READ_BIT(RTC->TR,RTC_TR_MNT);
	tmpClock[3] = READ_BIT(RTC->TR,RTC_TR_MNU);

	if(READ_BIT(RTC->TR,RTC_TR_HT)*10+READ_BIT(RTC->TR,RTC_TR_HU) > 5 && READ_BIT(RTC->TR,RTC_TR_HT)*10+READ_BIT(RTC->TR,RTC_TR_HU) < 22 
	|| flagDecrementButton || flagEnterButton || flagIncrementButton 
	|| flagDecrementButtonLong || flagEnterButtonLong || flagIncrementButtonLong)
	{
		for(int i = 0 + j; i<4;i++){
		 writeCHARSEG(tmpClock[i], i);
		 Delay_ms(50);
	 }
	}
	return flagDecrementButtonLong&&flagIncrementButtonLong?0:1;
}

void setTimeNow(){
    MODIFY_REG(RTC->TR,
               RTC_TR_HT_Msk | RTC_TR_HU_Msk | RTC_TR_MNT_Msk | RTC_TR_MNU_Msk | RTC_TR_ST_Msk | RTC_TR_SU_Msk,
               (menu[1].value/10 << RTC_TR_HT_Pos)  |   // Hour tens (1 -> 17)
               (menu[1].value%10 << RTC_TR_HU_Pos)  |   // Hour units (7 -> 17)
               (menu[2].value/10 << RTC_TR_MNT_Pos) |   // Minute tens (3 -> 36)
               (menu[2].value%10 << RTC_TR_MNU_Pos) |   // Minute units (6 -> 36)
               (menu[3].value/10 << RTC_TR_ST_Pos)  |   // Second tens (0 -> 00)
               (menu[3].value%10 << RTC_TR_SU_Pos));    // Second units (0 -> 00)
    MODIFY_REG(RTC->DR,
               RTC_DR_YT_Msk | RTC_DR_YU_Msk | RTC_DR_MT_Msk | RTC_DR_MU_Msk | RTC_DR_DT_Msk | RTC_DR_DU_Msk | RTC_DR_WDU_Msk,
               (menu[6].value/10 << RTC_DR_YT_Pos)  |  // Year tens (2 -> 24)
               (menu[6].value%10 << RTC_DR_YU_Pos)  |  // Year units (4 -> 24)
               (menu[5].value/10 << RTC_DR_MT_Pos)  |  // Month tens (1 -> April)
               (menu[5].value%10 << RTC_DR_MU_Pos)  |  // Month units (0 -> April)
               (menu[4].value/10 << RTC_DR_DT_Pos)  |  // Day tens (0 -> 01)
               (menu[4].value%10 << RTC_DR_DU_Pos)  |  // Day units (1 -> 01)
               (menu[7].value << RTC_DR_WDU_Pos)); // Weekday (3 -> Monday)
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
						StartMusic(menu[19].value);// *19 		P_3.3	Alarm_Melody
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
		TIM2->ARR = SYSCLK / TIM2->PSC / freq;
		TIM2->CCR1 = TIM2->ARR / 2;
	}
	else {
		TIM2->ARR = 1000;
		TIM2->CCR1 = 0;
	}
	TIM2->CNT=0;

	sound_time = ((SYSCLK / TIM2->PSC / TIM2->ARR) * time_ms ) / 1000;
	sound_counter = 0;
	SET_BIT(TIM2->CR1, TIM_CR1_CEN);  //Запуск таймера;
}
/* Handlers--------------------------------------------------------*/
void SysTick_Handler(void) {
	SysTimer_ms++;
	if (Delay_counter_ms) {
		Delay_counter_ms--;
	}
}

void EXTI0_1_IRQHandler(void){
    if (EXTI->PR & EXTI_PR_PR0){				// Перевірка, чи було переривання від лінії EXTI 0
        EXTI->PR = EXTI_PR_PR0;					// Скидаємо прапорець EXTI 0
        if (flagDecrementButtonDown){			// Обробка кнопки decrement
            if ((/*HAL_GetTick()*/ - timeDecrementButtonDown) > timeButtonLongPressed)
            {
                flagDecrementButtonLong = true;
            }
            else if ((/**HAL_GetTick()*/ - timeDecrementButtonDown) > timeButtonPressed)
            {
                flagDecrementButton = true;
            }
            flagDecrementButtonDown = false;
        }else{
            timeDecrementButtonDown = /*HAL_GetTick()*/0;
            flagDecrementButtonDown = true;
        }
    }
    if (EXTI->PR & EXTI_PR_PR1){				// Перевірка, чи було переривання від лінії EXTI 1
		EXTI->PR = EXTI_PR_PR1;					// Скидаємо прапорець EXTI 1 
        if (flagEnterButtonDown){				// Обробка кнопки enter
            if ((/*HAL_GetTick()*/ - timeEnterButtonDown) > timeButtonLongPressed){
                flagEnterButtonLong = true;
            }
            else if ((/*HAL_GetTick()*/ - timeEnterButtonDown) > timeButtonPressed){
                flagEnterButton = true;
            }
            flagEnterButtonDown = false;
        }else{
            timeEnterButtonDown = /*HAL_GetTick()*/0;
            flagEnterButtonDown = true;
        }
    }
}

void EXTI2_3_IRQHandler(void)
{
    // Перевірка, чи було переривання від лінії EXTI 2
    if (EXTI->PR & EXTI_PR_PR2)
    {
        // Скидаємо прапорець EXTI 2
        EXTI->PR = EXTI_PR_PR2;

        // Обробка кнопки increment
        if (flagIncrementButtonDown)
        {
            if ((/*HAL_GetTick()*/ - timeIncrementButtonDown) > timeButtonLongPressed)
            {
                flagIncrementButtonLong = true;
            }
            else if ((/*HAL_GetTick()*/ - timeIncrementButtonDown) > timeButtonPressed)
            {
                flagIncrementButton = true;
            }
            flagIncrementButtonDown = false;
        }
        else
        {
            timeIncrementButtonDown = /*HAL_GetTick()*/0;
            flagIncrementButtonDown = true;
        }
    }
}

void EXTI4_15_IRQHandler(void)
{
    // Перевірка, чи було переривання від лінії EXTI 9
    if (EXTI->PR & EXTI_PR_PR9)
    {
        // Скидаємо прапорець EXTI 9
        EXTI->PR = EXTI_PR_PR9;

        // Обробка подій, пов'язаних з EXTI 9
        // Код обробки може бути доданий тут
    }
}

void TIM2_IRQHandler(void) {
	if (READ_BIT(TIM2->SR, TIM_SR_UIF)) {
//		CounterTIM2++;
		CLEAR_BIT(TIM2->SR, TIM_SR_UIF);  //Сбросим флаг прерывания
	}


}

void TIM21_IRQHandler(void) {
	if (READ_BIT(TIM21->SR, TIM_SR_UIF)) {
//		CounterTIM2++;
		CLEAR_BIT(TIM21->SR, TIM_SR_UIF);  //Сбросим флаг прерывания
	}
}

void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

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
