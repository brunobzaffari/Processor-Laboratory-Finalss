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

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

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

// Para usarmos o DHT22
#define DHT22_PORT GPIOA
#define DHT22_PIN GPIO_PIN_1
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
//--------------------------------DHT----------------------------
//void udelay2(void);
//void delay(int tempo);
void delay(uint16_t us);
void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void DHT22_init(void);
void DHT22_Start(void);
uint8_t DHT22_Check_Response(void);
uint8_t DHT22_Read(void);
int DHT22(float *Temperature, float *Humidity);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
HAL_StatusTypeDef o;
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
//--------------------------------DHT----------------------------
void delay(uint16_t us){
    uint16_t start = __HAL_TIM_GET_COUNTER(&htim1); // Lê o valor atual do contador
    while ((__HAL_TIM_GET_COUNTER(&htim1) - start) < us); // Aguarda até que a diferença atinja 'us'
}

void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin){
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}
void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin){
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void DHT22_init(void){
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	HAL_GPIO_DeInit(DHT22_PORT, DHT22_PIN);
	GPIO_InitStruct.Pin = DHT22_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(DHT22_PORT, &GPIO_InitStruct);
	HAL_Delay(1000);
	HAL_GPIO_WritePin(DHT22_PORT, DHT22_PIN, GPIO_PIN_SET);// pull the pin high
}
void DHT22_Start(void){
	Set_Pin_Output(DHT22_PORT, DHT22_PIN);// set the pin as output
	HAL_GPIO_WritePin(DHT22_PORT, DHT22_PIN, 0);// pull the pin low
	delay(1500);// wait for > 1ms
	HAL_GPIO_WritePin(DHT22_PORT, DHT22_PIN, 1);// pull the pin high
	delay(30);   // wait for 30us
	Set_Pin_Input(DHT22_PORT, DHT22_PIN);// set as input
}

uint8_t DHT22_Check_Response(void){
	uint8_t Response = 0;
//	AONDE=NO_LCD;
//	lcd_goto(0,0);
//	printf("4\n");
	delay(40);  // wait for 40us
//	printf("5\n");
	if (!(HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN))){// if the pin is low
//		printf("6\n");
		delay(80);// wait for 80us
//		printf("7\n");
		if ((HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN))) Response = 1;// if the pin is high, response is ok
		else Response = -1;
//			printf("8\n");
	}

//	pMillis = HAL_GetTick();
//	cMillis = HAL_GetTick();
//	printf("9\n");
	while((HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN)));// wait for the pin to go low
//		cMillis = HAL_GetTick();
//	printf("10\n");
	return Response;
}
uint8_t DHT22_Read(void){
	uint8_t i,j;
//	AONDE=NO_LCD;
//	lcd_goto(0,0);
	for (j=0;j<8;j++){
		//pMillis = HAL_GetTick();
		//cMillis = HAL_GetTick();
//		printf("3\n");
		while(!(HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN)));// wait for the pin to go high
//		printf("2\n");	//cMillis = HAL_GetTick();
		delay(30);// wait for 40 us
//		printf("3\n");
		if (!(HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN))) i&= ~(1<<(7-j));// if the pin is lOW(write 0)
		else i|= (1<<(7-j));// if the pin is high, write 1
//		printf("4\n");
		while((HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN)));// wait for the pin to go low
			//cMillis = HAL_GetTick();
//		printf("5\n");
	}
//	printf("6\n");
	return i;
}

//int DHT22(float *Temperature, float *Humidity, uint8_t *t1, uint8_t *t2, uint8_t *r1, uint8_t *r2, uint16_t *t, uint16_t *r){
int DHT22(float *Temperature, float *Humidity){
	uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2;
	uint16_t SUM, CHECK, TEMP, RH;
	//float Temperature, Humidity;// Debug
	////////////////////////////////////////////////////////////////////////
	HAL_GPIO_WritePin(DHT22_PORT, DHT22_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DHT22_PORT, DHT22_PIN, GPIO_PIN_SET);
	////////////////////////////////////////////////////////////////////////
	AONDE = NO_LCD;
	lcd_goto(0,0);
	//printf("1\n");
	DHT22_Start();
	//printf("2\n");
	if(DHT22_Check_Response()){
		//printf("3\n");
		Rh_byte1 = DHT22_Read();
		//printf("4\n");
		Rh_byte2 = DHT22_Read();
		//printf("5\n");
		Temp_byte1 = DHT22_Read();
		//printf("6\n");
		Temp_byte2 = DHT22_Read();
		//printf("7\n");
		SUM = DHT22_Read();
		//printf("8\n");
		CHECK = Rh_byte1 + Rh_byte2 +  Temp_byte1 + Temp_byte2;
		if (CHECK == SUM){
			//printf("9\n");
			TEMP=((Temp_byte1 << 7 ) | Temp_byte2);
			RH =((Rh_byte1 << 8 ) | Rh_byte2);
			*Temperature=(float)TEMP/(10.0);
			*Humidity=(float)RH/(10.0);
			return 1;
		}
	}
	return 0;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	float Temp = 0, Humi = 0;
	int x;
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
    DHT22_init();
    HAL_Delay(1000);

    AONDE = NA_SERIAL;
    printf("\r Estou imprimindo na serial \n\r");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	    x = DHT22(&Temp, &Humi);
		AONDE = NO_LCD;
		lcd_goto(0,0);
		printf("%.1f%cC\n", Temp, 223);
		lcd_goto(1,0);
		printf("RH %.1f\n", Humi);
		if(x ==0)DHT22_init();
		HAL_Delay(2000);
//	if(DHT22(&Temp, &Humi)){
//		AONDE = NO_LCD;
//		lcd_goto(0,0);
//		printf("%.1f%cC\n", Temp, 223);
//		lcd_goto(1,0);
//		printf("RH %.2f\n", Humi);
//	}
//	else {
//		DHT22_init();

		//printf("\rInicializando novamente\n\r");
//	}

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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|LD2_Pin|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 LD2_Pin PA8 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|LD2_Pin|GPIO_PIN_8|GPIO_PIN_9;
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

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
//---------------------------------------------------------------

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
