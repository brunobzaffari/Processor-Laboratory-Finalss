/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// Estado do Cursor
#define CURSOR_OFF 0x0C   // Apagado
#define CURSOR_ON 0x0E    // Ligado
#define CURSOR_BLINK 0x0F // Piscante

// Estado dos pinos de Controle...
#define RS_0 GPIOA -> BRR =  1<<9 //PA9
#define RS_1 GPIOA -> BSRR = 1<<9 //PA9
#define EN_0 GPIOC -> BRR =  1<<7 //PC7
#define EN_1 GPIOC -> BSRR = 1<<7 //PC7

// Estado dos pinos do Barramento do LCD...
#define D7_0 GPIOA -> BRR  = 1<<8 //PA8
#define D7_1 GPIOA -> BSRR = 1<<8 //PA8

#define D6_0 GPIOB -> BRR  = 1<<10 //PB10
#define D6_1 GPIOB -> BSRR = 1<<10 //PB10

#define D5_0 GPIOB -> BRR  = 1<<4 //PB4
#define D5_1 GPIOB -> BSRR = 1<<4 //PB4

#define D4_0 GPIOB -> BRR  = 1<<5 //PB5
#define D4_1 GPIOB -> BSRR = 1<<5 //PB5

// Para usarmos o terminal
#define NO_LCD 1
#define NA_SERIAL 2
//RELES
#define BUZZER_ON	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 1)
#define BUZZER_OFF	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 0)
// ------ Controle ----------
volatile int corrente = 2;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void udelay(void);
void delayus(int tempo);
void lcd_wrcom4 (uint8_t com4);
void lcd_wrcom(uint8_t com);
void lcd_wrchar(char ch);
void lcd_init(uint8_t cursor);
void lcd_wrstr(char *str);
void lcd_wr2dig(uint8_t valor);
void lcd_senddata(uint8_t data);
void lcd_clear(void);
void lcd_progchar(uint8_t n);
void lcd_goto(uint8_t x, uint8_t y);
int __io_putschar(int ch);
int fputc(int ch, FILE * f);
//-------------Ajuste e funcionamento do alarme-------------//
void ajuste_hora(void);
void ajuste_data(void);
void mostrahoras(int x);
//-------------Ajuste e funcionamento do alarme-------------//
void desliga(void);
void horas_alarme(void);
void alarme(void);
void acinona_se(void);
void ajuste_hora_inicial(void);
void ajuste_hora_final(void);
void delay(uint16_t us);
//---------------------------
void save_pw_alarme(void);
void save_pw_in(void);
//---------------------------
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// ----------- Variaveis globais --------------------
uint8_t  hora, min, seg;
RTC_TimeTypeDef relogio;
RTC_DateTypeDef calendario;
TIM_OC_InitTypeDef sConfig = {0};
char senha_alarme[5]= "3872", senha_user[5] = {0};
volatile HAL_StatusTypeDef ret;
int ret_error=0;
int erro = 0;
int ok_alarme=0;// Para conferir se esta na faixa de acionamento do alarme
int abertura=0;// Verifica se abriu dentro da faixa de acionamento do alarme
int INVASAO=0;// Caso ok_alarme e abertura == 1... Aciona o alarme
uint8_t h_init, h_end, m_init, m_end;

char AONDE=NO_LCD;
// ----------- Variaveis globais --------------------

int __io_putchar(int ch){
	if (AONDE == NO_LCD){
		if (ch != '\n') lcd_wrchar(ch);
	}
	if (AONDE == NA_SERIAL){
			HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 100);
		}
	return ch;
}

// --------------------------------Interrupção de GPIO-------------
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
//	if((GPIO_Pin == GPIO_PIN_1)){
//		delay(100);
//		if((GPIOC->IDR & (1<<1))==0){corrente = 0;}// Entra cartao
//		else{corrente = 1;}// Sai cartao
//	}

	if((GPIO_Pin == GPIO_PIN_2))corrente =4;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
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
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_RTC_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Init(&huart2);
   HAL_RTC_Init(&hrtc);
   HAL_RTC_WaitForSynchro(&hrtc);
   HAL_TIM_Base_Init(&htim1);
   HAL_TIM_Base_Start(&htim1);
   lcd_init(CURSOR_OFF);
   lcd_clear();
   HAL_Delay(1);
 //   ----------------------Msg inicial--------------------------------
   	AONDE=NA_SERIAL;
   	printf("\rBUZZER\r\n");

 //  ----------------------Msg inicial--------------------------------

 //  ----------------------Ajusta hora do rtc e do alarme----------------------
   	ajuste_hora();
   	HAL_Delay(1500);
   	mostrahoras(NA_SERIAL);
   	ajuste_hora_inicial();
   	ajuste_hora_final();

   	AONDE= NO_LCD;
   	lcd_clear();
   	lcd_goto(0,0);
   	printf("Init %02d:%02d\n", m_init, h_init);
   	lcd_goto(1,0);
   	printf("End %02d:%02d\n", m_end, h_end);
   	HAL_Delay(2000);
//	lcd_clear();
 //  ----------------------Ajusta hora do alarme----------------------
	AONDE= NA_SERIAL;
	printf("\rDigite uma senha de 4 digitos para desligar o alarme\r\n");
	save_pw_alarme();
	HAL_Delay(100);
	printf("\rDigite uma senha de 4 digitos para entrar\r\n");
	save_pw_in();

//  ----------------------Ajusta hora do alarme----------------------
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(corrente == 2){
		 // printf("\rC\n\r");
		acinona_se();// Muda a variavel pra 1 se estiver dentro do tempo de acionamento
		 // printf("\r2\n\r");
		alarme();// Liga o rele se tiver  alarme_ok e abertura ==1
		 // printf("\r3\n\r");
		mostrahoras(NO_LCD);
		AONDE=NO_LCD;
		lcd_goto(0,0);
		if((GPIOC->IDR & (1<<2))== 0){
		  printf("CLOSED(%d)| OK(%d)\n", abertura, ok_alarme);
		}
		else{
		  printf("OPENED(%d)| OK(%d)\n", abertura, ok_alarme);
		}
		lcd_goto(1,0);
		printf("INV(%d)\n", INVASAO);
	  }
	  if(corrente == 3){//DESACIONA O ALARME
		  AONDE=NO_LCD;
		  lcd_clear();
		  lcd_goto(0,0);
		  //printf("OPEN(%d)\n", abertura);
		  lcd_goto(1,0);
		  //printf("INV(%d)\n", INVASAO);
		  desliga();// Espera a senha certa pra desacionar e se desaciona volta pro estado 2
	  }
	  if(corrente == 4){// A interrupcao deixa ele aqui Aqui verifica se abriu a porta na faixa da hora
		HAL_Delay(50);
		//printf("\rx\n\r");
		if(ok_alarme == 1){// Porta aberta e dentro da hora do alarme
			if((GPIOC->IDR & (1<<2))== 0) abertura = 1;
			corrente = 2;
		}
		else {
			corrente = 2;
		}
		if(INVASAO==1) corrente = 3;// Caso esta aqui por engano
		AONDE = NO_LCD;
		lcd_goto(1, 9);
		printf("4\n");
		lcd_clear();
	  }
		HAL_Delay(1000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_RTC;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 1;
  sDate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 47;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC3 PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA8 PA9 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void save_pw_alarme(void){
	char senha[5];
	int k=0;
	for (k=0; k<4; k++){//Digita a senha
		do{
			erro = HAL_UART_Receive(&huart2, (uint8_t*)&senha[k], 1, 10);
		}while(erro != HAL_UART_ERROR_NONE);
		HAL_UART_Transmit(&huart2, (uint8_t*)&senha[k], 1, 2);
		if((k+1)==4)senha[4]='\0';
	}
	HAL_Delay(3);
	strcpy(senha_alarme, senha);
	printf("\rSenha alarme definida: %s\r\n", senha_alarme);
}
void save_pw_in(void){
	char senha[5];
	int k=0;
	for (k=0; k<4; k++){//Digita a senha
		do{
			erro = HAL_UART_Receive(&huart2, (uint8_t*)&senha[k], 1, 10);
		}while(erro != HAL_UART_ERROR_NONE);
		HAL_UART_Transmit(&huart2, (uint8_t*)&senha[k], 1, 2);
		if((k+1)==4)senha[4]='\0';
	}
	HAL_Delay(3);
	strcpy(senha_user, senha);
	printf("\rSenha user definida: %s\r\n", senha_user);
}
// --------------------------------Delay---------------------------
void delay(uint16_t us){
    uint16_t start = __HAL_TIM_GET_COUNTER(&htim1); // Lê o valor atual do contador
    while ((__HAL_TIM_GET_COUNTER(&htim1) - start) < us); // Aguarda até que a diferença atinja 'us'
}
// --------------------------------------- HORAS AJUSTES--------------------------------
void ajuste_hora(void){// CONFIGURAÇÃO DA HORA E MINUTOS
	char ch=0, u=0, d=0, h=0, m=0;
	AONDE = NO_LCD;
	lcd_goto(0,0);
	printf("CONFIG. HORAS:\n");
	HAL_Delay(1500);
	lcd_clear();
	printf("Digite a hora:\n");
	AONDE = NA_SERIAL;
	printf("\n");
	printf("---------------------------------------------\n");
	printf("\rConfiguração das horas do RTC:\r\n");
	do{// AJUSTE DO HORA
		ch=0;u=0; d=0; h=0;
		AONDE = NA_SERIAL;
		printf("\rDigite a hora [00-23]:\r\n");
		do{
			erro = HAL_UART_Receive(&huart2, (uint8_t*)&ch, 1, 10);
		} while (erro != HAL_UART_ERROR_NONE);
		HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 2); // eco na serial
		d = ch-'0'; // Converte ASCII em DECIMAL
		do{
			erro = HAL_UART_Receive(&huart2, (uint8_t*)&ch, 1, 10);
		} while (erro != HAL_UART_ERROR_NONE);
		HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 2);
		u = ch-'0';
		h = 10 * d + u;
		if(h>23)printf("\rHora inválida, digite novamente!\r\n");
		HAL_Delay(3);
		printf("\r\n");
	}while(h>23);
	AONDE = NO_LCD;
	lcd_clear();
	lcd_goto(0,0);
	printf("Horas config.: %d\n", h);
	HAL_Delay(3);
	lcd_clear();
	lcd_goto(0,0);
	printf("Digite a min:\n");
	do{// AJUSTE DO MINUTO
		ch=0;u=0; d=0; m=0;
		AONDE = NA_SERIAL;
		printf("\rDigite a minutos [00-59]: \r\n");
		do{
			erro = HAL_UART_Receive(&huart2, (uint8_t*)&ch, 1, 10);
		} while (erro != HAL_UART_ERROR_NONE);
		HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 2);
		d = ch-'0'; // Converte ASCII em DECIMAL
		do{
			erro = HAL_UART_Receive(&huart2, (uint8_t*)&ch, 1, 10);
		} while (erro != HAL_UART_ERROR_NONE);
		HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 2);
		u = ch-'0';
		m = 10 * d + u;
		if(m>59)printf("\rMinutos inválido, digite novamente!\r\n");
		HAL_Delay(3);
		printf("\r\n");
	}while(m>59);
	hora = (uint8_t)h;
	min = (uint8_t)m;
	seg = 0;
	HAL_Delay(1500);
	relogio.Hours = hora;
	relogio.Minutes = min;
	relogio.Seconds = seg;
	HAL_RTC_SetTime(&hrtc, &relogio, RTC_FORMAT_BIN);
	calendario.Year = 23;
	calendario.Month = 12;
	calendario.Date = 12;
	HAL_RTC_SetDate(&hrtc, &calendario, RTC_FORMAT_BIN);
	HAL_RTC_WaitForSynchro(&hrtc);
	HAL_Delay(1500);

}
void ajuste_hora_inicial(void){// CONFIGURAÇÃO DA HORA E MINUTOS
	char ch=0, u=0, d=0;
	char h=0, m=0;
	AONDE = NO_LCD;
	lcd_goto(0,0);
	lcd_clear();
	printf("Digite a hora:\n");
	AONDE = NA_SERIAL;
	printf("\r---------------------------------------------\r\n");
	printf("\rConfiguração do horário inicial do alarme:\r\n");
	do{// AJUSTE DO HORA
		ch=0;u=0; d=0; h=0;
		AONDE = NA_SERIAL;
		printf("\rDigite a hora inicial [00-23]:\r\n");
		do{
			erro = HAL_UART_Receive(&huart2, (uint8_t*)&ch, 1, 10);
		} while (erro != HAL_UART_ERROR_NONE);
		HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 2); // eco na serial
		d = ch-'0'; // Converte ASCII em DECIMAL
		do{
			erro = HAL_UART_Receive(&huart2, (uint8_t*)&ch, 1, 10);
		} while (erro != HAL_UART_ERROR_NONE);
		HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 2);
		u = ch-'0';
		h = 10 * d + u;
		if(h>23)printf("\rHora inválida, digite novamente!\r\n");
		HAL_Delay(30);
		printf("\r\n");
	}while(h>23);
	h_init = h;
	AONDE = NO_LCD;
	lcd_clear();
	lcd_goto(0,0);
	printf("Horas config: %d\n", h);
	HAL_Delay(1000);
	lcd_clear();
	lcd_goto(0,0);
	printf("Digite a min:\n");
	do{// AJUSTE DO MINUTO
		ch=0;u=0; d=0; m=0;
		AONDE = NA_SERIAL;
		printf("\rDigite a minutos [00-59]:\r\n");
		do{
			erro = HAL_UART_Receive(&huart2, (uint8_t*)&ch, 1, 10);
		} while (erro != HAL_UART_ERROR_NONE);
		HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 2);
		d = ch-'0'; // Converte ASCII em DECIMAL
		do{
			erro = HAL_UART_Receive(&huart2, (uint8_t*)&ch, 1, 10);
		} while (erro != HAL_UART_ERROR_NONE);
		HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 2);
		u = ch-'0';
		m = 10 * d + u;
		if(m>59)printf("\rMinutos inválido, digite novamente!\r\n");
		HAL_Delay(30);
		printf("\r\n");
	}while(m>59);
	m_init = m;
	AONDE = NO_LCD;
	lcd_clear();
	lcd_goto(0,0);
	printf("Min init conf:%d\n", m_init);
	HAL_Delay(30);
	lcd_clear();
	AONDE = NA_SERIAL;
	printf("\n");
	printf("\rDef init %02d:%02d\r\n", h_init, m_init);
	printf("\r---------------------------------------------\r\n");
}
void ajuste_hora_final(void){// CONFIGURAÇÃO DA HORA E MINUTOS
	char ch=0, u=0, d=0, h=0, m=0;
	AONDE = NO_LCD;
	lcd_goto(0,0);
	lcd_clear();
	printf("Digite a hora:\n");
	AONDE = NA_SERIAL;
	printf("\n");
	printf("\r---------------------------------------------\r\n");
	printf("\rConfiguração das horas finais do alarme:\r\n");
	do{// AJUSTE DO HORA
		ch=0;u=0; d=0; h=0;
		AONDE = NA_SERIAL;
		printf("\rDigite a hora final[00-23]:\r\n");
		do{
			erro = HAL_UART_Receive(&huart2, (uint8_t*)&ch, 1, 10);
		} while (erro != HAL_UART_ERROR_NONE);
		HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 2); // eco na serial
		d = ch-'0'; // Converte ASCII em DECIMAL
		do{
			erro = HAL_UART_Receive(&huart2, (uint8_t*)&ch, 1, 10);
		} while (erro != HAL_UART_ERROR_NONE);
		HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 2);
		u = ch-'0';
		h = 10 * d + u;
		if(h>23)printf("\rHora inválida, digite novamente!\r\n");
		HAL_Delay(30);
		printf("\r\n");
	}while(h>23);
	h_end = h;
	AONDE = NO_LCD;
	lcd_clear();
	lcd_goto(0,0);
	printf("Horas config! %d\n", h_end);
	HAL_Delay(600);
	lcd_clear();
	lcd_goto(0,0);
	printf("Digite a min:\n");
	do{// AJUSTE DO MINUTO
		ch=0;u=0; d=0; m=0;
		AONDE = NA_SERIAL;
		printf("\rDigite a minutos [00-59]:\r\n");
		do{
			erro = HAL_UART_Receive(&huart2, (uint8_t*)&ch, 1, 10);
		} while (erro != HAL_UART_ERROR_NONE);
		HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 2);
		d = ch-'0'; // Converte ASCII em DECIMAL
		do{
			erro = HAL_UART_Receive(&huart2, (uint8_t*)&ch, 1, 10);
		} while (erro != HAL_UART_ERROR_NONE);
		HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 2);
		u = ch-'0';
		m = 10 * d + u;
		if(m>59)printf("\rMinutos inválido, digite novamente!\r\n");
		HAL_Delay(30);
		printf("\r\n");
	}while(m>59);
	m_end = m;
	AONDE = NO_LCD;
	lcd_clear();
	lcd_goto(0,0);
	printf("Min end conf:%d\n", m_end);
	HAL_Delay(600);
	lcd_clear();
	AONDE = NA_SERIAL;
	printf("\n");
	printf("\rDef end %02d:%02d\r\n", h_end, m_end);
	printf("\r---------------------------------------------\r\n");
}
void mostrahoras(int x){// Mostra a hora na serial ou no LCD
	AONDE=x;
	HAL_RTC_GetTime(&hrtc, &relogio, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &calendario, RTC_FORMAT_BIN);

	hora = relogio.Hours;
	min  = relogio.Minutes;
	seg  = relogio.Seconds;
	lcd_goto(1 ,8);
	if(AONDE==NA_SERIAL) printf("\r%02d:%02d:%02d\r\n", hora, min, seg);
	else printf("%02d:%02d:%02d\n", hora, min, seg);
}
// ----------------------------------- BUZZER ------------------------------------------
void acinona_se(void){// Aviasa que asiona o alarme se entrar no periodo definido do alarme
	HAL_RTC_GetTime(&hrtc, &relogio, RTC_FORMAT_BIN);
	hora = relogio.Hours;
	min  = relogio.Minutes;
	seg = relogio.Seconds;

	if(h_init<=h_end){//10:06 e 15:01 ou 10:00 e 10:10
		if((hora>=h_init)&&(hora<=h_end)){
			if(h_init==h_end){//10:00 e 10:10
				if((min>=m_init)&&(min<=m_end))
				ok_alarme = 1;
			}
			else if(((hora>h_init)||(hora<h_end))|| //10:06 e 15:01
				((hora == h_init)&&(min>=m_init))||
				((hora == h_end)&&(min<=m_end))){
				ok_alarme = 1;
			}
		}
	}

	else if(h_init>h_end){//22:00 e 8:00
		if((hora>=h_init)||(hora<=h_end)){
			if(((hora==h_init)&&(min>=m_init))||((hora==h_end)&&(min<=m_end))||(hora<h_end)||(hora>h_init)){
				ok_alarme = 1;
			}
		}
	}

	else ok_alarme = 0;
}

void alarme(void){
	if(ok_alarme && abertura){
		HAL_RTC_GetTime(&hrtc, &relogio, RTC_FORMAT_BIN);
		hora = relogio.Hours;
		min  = relogio.Minutes;
		seg  = relogio.Seconds;
		AONDE= NA_SERIAL;
		printf("\rInvasão registrada ás %02d:%02d:%02d\r\n", hora, min, seg);
		lcd_clear();
		BUZZER_ON;
		INVASAO=1;

		corrente = 3;// para ficar ativo ate que alguem digite a senha;
	}
}
void desliga(void){
	char senha_dig[5];
	AONDE = NO_LCD;
	lcd_clear();
	lcd_goto(0,3);
	printf("***ALERTA !!!\n");
	lcd_goto(1,3);
	printf("***INVASAO\n");
	BUZZER_ON;
	while(1){
		AONDE= NA_SERIAL;
		printf("\rDigite a senha de 4 dig para desligar o alarme\r\n");
		for(int k=0; k<4; k++){//Digita a senha
			do{
				erro = HAL_UART_Receive(&huart2, (uint8_t*)&senha_dig[k], 1, 10);
			}while(erro != HAL_UART_ERROR_NONE);
			HAL_UART_Transmit(&huart2, (uint8_t*)&senha_dig[k], 1, 2);
			if((k+1)==4)senha_dig[4]='\0';
		}
		acinona_se();//Verifica se atingiu m_end:h_end -> TIMEOUT
		if(strcmp(senha_dig, senha_alarme)==0){// Caso a senha seja a mesma que esta na senha_alarme desativa alarme
			BUZZER_OFF;
			INVASAO=0;
			abertura = 0;
			corrente = 2;
			AONDE=NO_LCD;
			lcd_clear();
			lcd_goto(0,0);
			printf("ALARME OFF\n");
			HAL_RTC_GetTime(&hrtc, &relogio, RTC_FORMAT_BIN);
			hora = relogio.Hours;
			min  = relogio.Minutes;
			seg  = relogio.Seconds;
			AONDE= NA_SERIAL;
			printf("\rDesligado ás %02d:%02d:%02d\r\n", hora, min, seg);
			HAL_Delay(1000);
			break;
		}
		else if(ok_alarme==0){//Atingiu m_end:h_end
			BUZZER_OFF;
			INVASAO=0;
			abertura = 0;
			corrente = 2;
			HAL_RTC_GetTime(&hrtc, &relogio, RTC_FORMAT_BIN);
			hora = relogio.Hours;
			min  = relogio.Minutes;
			seg  = relogio.Seconds;
			AONDE= NA_SERIAL;
			printf("\rTIMEOUT ás %02d:%02d:%02d\r\n", hora, min, seg);
			AONDE=NO_LCD;
			lcd_clear();
			lcd_goto(0,0);
			printf("TIMEOUT\n");
			corrente = 2;
			HAL_Delay(1000);
			break;
		}
		else{// Caso erre a senha pergunta de novo
			memset(senha_dig, 0, sizeof(senha_dig));
			AONDE= NA_SERIAL;
			printf("\rSenha incorreta!\r\n");
			HAL_Delay(50);
		}
	}
}

//--------------------------------LCD----------------------------
void lcd_backlight (uint8_t light){
	if(light == 0){
		GPIOB -> BRR = 1<<3;
	} else {
		GPIOB -> BSRR= 1<<3;
	}
}
void lcd_init(uint8_t cursor){
	lcd_wrcom4(3);
	lcd_wrcom4(3);
	lcd_wrcom4(3);
	lcd_wrcom4(2);
	lcd_wrcom(0x28);
	lcd_wrcom(cursor);
	lcd_wrcom(0x06);
	lcd_wrcom(0x01);
}
void lcd_wrcom4(uint8_t com4){
	lcd_senddata(com4); //D4...d0
	RS_0;
	EN_1;
	delayus(5);
	EN_0;
	HAL_Delay(5);
}
void lcd_wrcom(uint8_t com){
	lcd_senddata(com>>4); //0000D7...D4
	RS_0;
	EN_1;
	delayus(5);
	EN_0;
	delayus(5);

	lcd_senddata(com & 0x0F); //0000D3...d0
	EN_1;
	delayus(5);
	EN_0;
	HAL_Delay(5);
}
void lcd_clear(void){
	lcd_wrcom(0x01);
}
//goto para 16x2
void lcd_goto(uint8_t x, uint8_t y){
	uint8_t com = 0x80;
	if (x==0 && y<16) com = 0x80 + y;
	else if (x==1 && y<16) com = 0xC0 + y;
	else com = 0x80;
	lcd_wrcom(com);
}
void lcd_wrchar(char ch){
	lcd_senddata(ch>>4); //D7...D4
	RS_1;
	EN_1;
	delayus(5);
	EN_0;
	delayus(5);

	lcd_senddata(ch & 0x0F); //D3...D0
	RS_1;
	EN_1;
	delayus(5);
	EN_0;
	HAL_Delay(5);
}

void lcd_wrstr(char *str){
	while(*str) lcd_wrchar(*str++);
}

void udelay(void){
	int tempo = 7;
	while(tempo--);
}

void delayus(int tempo){
	while(tempo--) udelay();
}

void lcd_wr2dig(uint8_t valor){
	lcd_wrchar(valor/10 + '0'); // ou +48 -> dezena
	lcd_wrchar(valor%10 + '0'); // ou +48 -> unidade
}

void lcd_senddata(uint8_t data){
	if((data & (1<<3))==0) D7_0; else D7_1;
	if((data & (1<<2))==0) D6_0; else D6_1;
	if((data & (1<<1))==0) D5_0; else D5_1;
	if((data & (1<<0))==0) D4_0; else D4_1;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
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
