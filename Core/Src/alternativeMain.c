/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>            // Підключаємо бібліотеку для використання булевих значень
#include "math.h"               // Підключаємо бібліотеку для математичних операцій
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Немає користувацьких визначень структур
/* USER CODE END PTD */

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
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
// Немає користувацьких макросів
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef hlpuart1;    // Структура для UART
RTC_HandleTypeDef hrtc;         // Структура для годинника реального часу
TIM_HandleTypeDef htim2;        // Структура для таймера 2
TIM_HandleTypeDef htim21;       // Структура для таймера 21

/* USER CODE BEGIN PV */
// Користувацькі змінні
bool flagDecrementButton;       // Прапорець для натискання кнопки зменшення
bool flagEnterButton;           // Прапорець для натискання кнопки підтвердження
bool flagIncrementButton;       // Прапорець для натискання кнопки збільшення
bool flagDecrementButtonLong;   // Прапорець для довгого утримання кнопки зменшення
bool flagEnterButtonLong;       // Прапорець для довгого утримання кнопки підтвердження
bool flagIncrementButtonLong;   // Прапорець для довгого утримання кнопки збільшення

int menuArraySize = 23;         // Встановлюємо розмір масиву меню
int actualIndex = 0;            // Поточний індекс меню
bool isParamEditMode = false;   // Прапорець режиму редагування параметра
int tmpVValue = 0;              // Тимчасова змінна для зберігання параметра

volatile uint32_t SysTimer_ms = 0;  // Системний таймер (аналог HAL_GetTick)
volatile uint32_t Delay_counter_ms = 0;  // Лічильник для затримки

int pwmcount = 0;               // Лічильник PWM
int CounterTIM2 = 0;            // Лічильник таймера 2
int CounterTIM21 = 0;           // Лічильник таймера 21

// Структура меню
struct strMenu {
    int id;         // Унікальний ідентифікаційний індекс ID
    int parentid;   // ID батька (вкладеність)
    bool isParam;   // Чи є пункт змінним параметром
    char _name[4];  // Назва пункту меню
    int value;      // Поточне значення параметра
    int _min;       // Мінімально можливе значення
    int _max;       // Максимально можливе значення
};
/* PPPP
 * P__0			Time_Now
 * 		P_0.0	Hour_Now
 * 		P_0.1	Minute_Now
 *		P_0.2	Seconds_Now
 * 		P_0.3	Day_Now
 * 		P_0.4	Month_Now
 * 		P_0.5	Year_Now
 * 		P_0.6	WeekDay_Nows
 * 		P_0.7	Set
 * P__1			Time_Rise
 * 		P_1.0	Hour_Rise
 * 		P_1.1	Minute_Rise
 * P__2			Rising_Parametrs
 * 		P_2.0	Period_Rising
 * 		P_2.1	ɣ_Coefient_Rising
 * P__3			Alarm_Parametrs
 * 		P_3.0	Alarm_Status
 * 		P_3.1	Alarm_Melody
 * 		P_3.2	Alarm_Melody_test
 * P__4			Menu_Parametrs
 * 		P_4.0	Numbers_Change_Style
 * 		P_4.1	Menu_Night_Mode
 * P__5			Clock(StartWork)
 */
struct strMenu menu[] = {                         // Встановлюємо пункти меню
	  {0, -1,    false,	"PPPP",		0, 0, 	0},
	  //-----------------------------------------------------------------------
	  {1, 0,     false,	"P__0",		0, 0, 	0},
	  {2, 1,     true,	"P_00",		0, 2, 	0},
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
/* USER CODE END PV */

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
void SVC_Handler(void)
void DebugMon_Handler(void)
void PendSV_Handler(void);

/* USER CODE BEGIN PFP */
void Delay_ms(uint32_t Milliseconds);
double custom_pow(double a, double x);
double custom_floor(double x);
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
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	GPIO_Init();
	LPUART1_UART_Init();
	RTC_Init();
	TIM2_Init();
	TIM21_Init();
	/* USER CODE BEGIN 2 */
	writeCHARSEG(' ', ' ');
	pinEN_OFF();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
//		pinEN_ON();
//		TIM21->CCR1=CounterTIM21;
		TIM2->ARR=CounterTIM2;
		TIM2->CCR1=(TIM2->ARR)/2;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 * 
 * #Working
 */
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

/**
 * @brief LPUART1 Initialization Function
 * @param None
 * @retval None
 * 
 * #Development
 */
void LPUART1_UART_Init(void) {
//
//  /* USER CODE BEGIN LPUART1_Init 0 */
//
//  /* USER CODE END LPUART1_Init 0 */
//
//  /* USER CODE BEGIN LPUART1_Init 1 */
//
//  /* USER CODE END LPUART1_Init 1 */
//  hlpuart1.Instance = LPUART1;
//  hlpuart1.Init.BaudRate = 209700;
//  hlpuart1.Init.WordLength = UART_WORDLENGTH_7B;
//  hlpuart1.Init.StopBits = UART_STOPBITS_1;
//  hlpuart1.Init.Parity = UART_PARITY_NONE;
//  hlpuart1.Init.Mode = UART_MODE_TX_RX;
//  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
//  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
//  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN LPUART1_Init 2 */
//
//  /* USER CODE END LPUART1_Init 2 */

}

/**
 * @brief RTC Initialization Function with Date and Time setting
 * @param None
 * @retval None
 * 
 * #Development
 */
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
    RTC->WPR = 0xFF;  // Enable write protection again
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 * 
 * #Working
 * 
 */
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

/**
 * @brief TIM21 Initialization Function
 * @param None
 * @retval None
 * 
 * #Working
 */
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

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 * 
 * #Working
 */
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

	/* USER CODE BEGIN GPIO_Init_2 */
	/* USER CODE END GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void Delay_ms(uint32_t Milliseconds) {
	Delay_counter_ms = Milliseconds;
	while (Delay_counter_ms != 0);
}

int custom_floor(double x) {
    int32_t result = (int)x;  // Приведення до int обрізає дробову частину
    if (x < 0 && x != (double)result) {
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
	if (menu[11].value) {
		int timeWakeUp 	= menu[6].value 	* 3600
					+ menu[7].value	* 60;
		int timeNow 	= sTime.Hours 		* 3600
					+ sTime.Minutes		* 60
					+ sTime.Seconds;
		if(menu[9].value * 60 >= timeWakeUp - timeNow){
			pinEN_ON();
			//TIM_Cmd(TIM21, ENABLE);
			TIM21->CCR1 = (int16_t) (65535 * pow((1 - timeNow / timeWakeUp), 2.24));
		}
	} else {
			pinEN_OFF();
			//TIM_Cmd(TIM21, DISABLE);
			TIM21->CCR1 = 0;
		}
}

/* Handlers--------------------------------------------------------*/

void SysTick_Handler(void) {

	SysTimer_ms++;

	if (Delay_counter_ms) {
		Delay_counter_ms--;
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

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
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
