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

//PARA O USO DA UART
#define NO_LCD 1
#define NA_SERIAL 2
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
//-------FUNCOES PADRAO PARA O FUNCIONAMENTO DO LCD----------//
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
//----------------Delay-------------------------
void delay(uint16_t us);
//----------------Debug-------------------------
void debugI2c(void);
void Scan_I2C_Address(I2C_HandleTypeDef *hi2c);
void print_mem(void);
//----------------Basicas Memoria-------------------------
void clean_mem(void);
void save_in_mem(int *pos_in, char *str);
void indentificador_de_mem_principal(HAL_StatusTypeDef ret);
void le_uart(char *str);
int senha_alarme_func(void);// Recupera a senha pra desligar o alarme
//----------------Memoria-------------------------
void check_mem_load(void);
int save_logs(void);
int verify_acess(char *nome, char *senha);// Se precionado o botao, esige que a pessoa ponha seu nome e usuario pra liberar a entrada
int busca_nome_mp(int *pos, char *nome_mp);
void concede_acesso(void);

//Variavel de controle de maquina de estados
volatile int corrente = 1;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
TIM_OC_InitTypeDef sConfig = {0};

int ret_error=0;
int erro = 0;

char AONDE=NO_LCD;
char senha_mp[5] = {0};
char senha_alarme[5] = {0};
char nome_mp[5] = {0};

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
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if((GPIO_Pin == GPIO_PIN_1)){
		delay(100);
		if((GPIOC->IDR & (1<<1))==0){corrente = 0;}// Pressiona Botao pra digitar log e senha e entrar
		if((GPIOC->IDR & (1<<1))!=0){corrente = 1;}// Des pressiona Botao
	}
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
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  //--------------------------Inits----------------------------------
 	HAL_RTC_WaitForSynchro(&hrtc);
 	HAL_TIM_Base_Init(&htim1);
 	HAL_TIM_Base_Start(&htim1);
   	HAL_UART_Init(&huart2);
   	HAL_I2C_Init(&hi2c1);
   	HAL_RTC_Init(&hrtc);
	lcd_init(CURSOR_OFF);

//	HAL_Delay(10);
	debugI2c();
//	print_mem();
// 	delay(1000);
// 	AONDE=NA_SERIAL;
// 	printf("\rALARME RESIDENCIAL\r\n");
// 	check_mem_load();
 	AONDE = NA_SERIAL;
 	print_mem();
// 	printf("%s\r\n", senha_alarme);
// 	print_mem();
 	Scan_I2C_Address(&hi2c1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48
                              |RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_RTC;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  htim1.Init.Prescaler = 0;
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
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : B1_Pin PC1 */
  GPIO_InitStruct.Pin = B1_Pin|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
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

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//------------------------------------------ FUNCAO DELAY---------------------------------------------------------
void delay(uint16_t us){
    uint16_t start = __HAL_TIM_GET_COUNTER(&htim1); // Lê o valor atual do contador
    while ((__HAL_TIM_GET_COUNTER(&htim1) - start) < us); // Aguarda até que a diferença atinja 'us'
}

//------------------------------------------ FUNCOES DE DEBUG ---------------------------------------------------------
void Scan_I2C_Address(I2C_HandleTypeDef *hi2c){
    uint8_t address;
    HAL_StatusTypeDef res;
    printf("Iniciando a varredura do barramento I2C...\n");

    for(address = 1; address < 128; address++)
    {
        res = HAL_I2C_IsDeviceReady(hi2c, (uint16_t)(address << 1), 10, 100);
        if(res == HAL_OK)
        {
            printf("\rDispositivo encontrado no endereço I2C: 0x%X\n", address);
            return;
        }
    }
    printf("\rDispositivo não encontrado\n");

}

void debugI2c(void){/*                          DEBUG                                        */
	char t = 0;
	AONDE= NA_SERIAL;
	HAL_StatusTypeDef ret;
	for(uint8_t k=0; k<25; k++){
		//t = t + 1;
		ret =HAL_I2C_Mem_Write(&hi2c1, 0xA0, k, 1, (uint8_t*)&t, 1, 1000);
		//HAL_Delay(100);
		if ( ret != HAL_OK ) {
			printf(" nao consegue trensferir\r\n");
		}
		else {
			printf(" consegue trensferir\r\n");
		}
		//pw_in[k]= ch;
		//printf("%c\r\n", t);
		//printf("%c\r\n",(char)pw_in[k]);
		//if((k+1)==4)pw_in[4]='\0';
	}
	t = 0;
	printf("----------------------------------------------------\r\n");
	for(uint8_t k=0; k<26; k++){
		ret = HAL_I2C_Mem_Read(&hi2c1, 0xA1, k, 1, (uint8_t*)&t, 1, 1000);
		//HAL_Delay(1);
		//pw_in[k]= ch;
		if ( ret != HAL_OK ) {
			printf(" nao consegue ler\r\n");
		}
		else {
			printf(" consegue ler\r\n");
		}
		printf("%c\r\n", t);
		//printf("%c\r\n",(char)pw_in[k]);
		//if((k+1)==4)pw_in[4]='\0';
	}
	/*                                   DEBUG                           */
}
void print_mem(void){
	char t = 0;
	HAL_StatusTypeDef ret;
	uint8_t k=0;
	AONDE = NA_SERIAL;
	for(uint8_t k=0; k<33; k++){
		ret = HAL_I2C_Mem_Read(&hi2c1, 0xA1, k, 1, (uint8_t*)&t, 1, 1000);
//		if ( ret != HAL_OK ) {
//			printf(" nao consegue ler\r\n");
//		}
//		else {
//			printf(" consegue ler\r\n");
//		}
		 if(ret == HAL_ERROR)   printf("|%d|%c,%dHAL_ERROR\n\r", k, t,(int)t);
		 if(ret == HAL_BUSY)    printf("|%d|%c,%dHAL_BUSY\n\r", k, t,(int)t);
		 if(ret ==HAL_TIMEOUT)  printf("|%d|%c,%dHAL_TIMEOUT\n\r", k, t,(int)t);
		 if(ret == HAL_OK) printf("|%d|%c,%dHAL_OK\n\r", k, t,(int)t);
		//printf("%c\r\n",(char)pw_in[k]);
		//if((k+1)==4)pw_in[4]='\0'; printf("|%d|%c %d", k, t,(int)t);
	}
//	while(ret){
//		ret = HAL_I2C_Mem_Read(&hi2c1, 0xA1, k, 1, (uint8_t*)&t, 1, 1000);
//		HAL_Delay(1);
////		if ( ret != HAL_OK ) {
////			printf(" nao consegue ler\r\n");
////		}
////		else {
////			printf(" consegue ler\r\n");
////		}
//		 if(ret == HAL_ERROR)   printf("|%d|%c,%d HAL_ERROR\n\r", k, t,(int)t);
//		 if(ret == HAL_BUSY)    printf("|%d|%c,%d HAL_BUSY\n\r", k, t,(int)t);
//		 if(ret ==HAL_TIMEOUT)  printf("|%d|%c,%d HAL_TIMEOUT\n\r", k, t,(int)t);
//		 if(ret == HAL_OK) printf("|%d|%c,%dHAL_OK\n\r", k, t,(int)t);
//		//printf("%c\r\n",(char)pw_in[k]);
//		//if((k+1)==4)pw_in[4]='\0';
//		 k+=1;
//	}
}
//------------------------------------------ FUNCOES BASICAS DE MEMORIA ---------------------------------------------------------
void clean_mem(void){
	char t = 0;
	AONDE= NA_SERIAL;
	HAL_StatusTypeDef ret;
	for(uint8_t k=0; k<33; k++){
		//t = t + 1;
		ret =HAL_I2C_Mem_Write(&hi2c1, 0xA0, k, 1, (uint8_t*)&t, 1, 1000);
		HAL_Delay(10);
		if ( ret != HAL_OK ) {
			//printf(" nao consegue trensferir\r\n");
		}
		else {
			//printf(" consegue trensferir\r\n");
		}

	}
	printf("----------------------------------------------------\r\n");
	for(uint8_t k=0; k<33; k++){
		ret = HAL_I2C_Mem_Read(&hi2c1, 0xA1, k, 1, (uint8_t*)&t, 1, 1000);
		//HAL_Delay(1);
		//pw_in[k]= ch;
		if ( ret != HAL_OK ) {
			//printf(" nao consegue ler\r\n");
		}
		else {
			//printf(" consegue ler\r\n");
		}
		printf("|%d|%c %d\n\r", k, t,(int)t);
	}

}

void indentificador_de_mem_principal(HAL_StatusTypeDef ret){// Verifica se a memoria principal esta presente
	if(ret != HAL_OK){
		ret_error =1;
	}
}
void le_uart(char *str){// Le a string digitada pelo usuario, com tamanho de 4 caracteres
	char ch=0;
	int i=0;
	char str2[5];
	for(i=0;i<4;i++){
		do{
			erro = HAL_UART_Receive(&huart2, (uint8_t*)&ch, 1, 10);
		} while (erro != HAL_UART_ERROR_NONE);
		HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 2);//ECO
		str[i]=ch;
		ch=0;
	}
	str[4]='\0';
	strcpy(str, str2);
	AONDE = NA_SERIAL;
	printf("\r\n\r");
}
void save_in_mem(int *pos_in, char *str){// Salva na memoria principal
	HAL_StatusTypeDef ret;
	int pos= *pos_in;
	int pos2 =pos;
	AONDE=NA_SERIAL;
	int ch;
	printf("%d\n\r", pos);
	for(int k=0; k<4; k++){// Salva na memoria
		ch = str[k];
		ret = HAL_I2C_Mem_Write(&hi2c1, 0xA0, pos, 1, (uint8_t*)&ch, 1, 1000);
		HAL_Delay(100);
		pos += 1;
		printf("|%d|%c %d\n\r", k, str[k],(int)str[k]);
	}
	char confere[5]={0};
	for(int k=0; k<4; k++){
		ret = HAL_I2C_Mem_Read(&hi2c1, 0xA1, pos2, 1, (uint8_t*)&confere[k], 1, 1000);
		HAL_Delay(100);
		pos += 1;
		if ( ret != HAL_OK ) {
			//printf(" nao consegue ler\r\n");
		}
		else {
			//printf(" consegue ler\r\n");
		}
		printf("|%d|%c %d\n\r", k, confere[k],confere[k]);
	}

	*pos_in = pos;// Retorna a posicao atual da memoria principal
}
int senha_alarme_func(void){// Salva a senha do alarme da memória principal para a memória do programa
	AONDE=NA_SERIAL;
	char senha[5]={0};
	int pos = 0x07;
	HAL_StatusTypeDef ret;
	for(int i = 0; i<4; i++){// Salva o nome presente no cartão!
		ret = HAL_I2C_Mem_Read(&hi2c1, 0xA1, pos + i, 1, (uint8_t*)&senha[i], 1, 1000);
		//HAL_Delay(100);
//		if (ret != HAL_OK ){
//			return 2;// Retorna pro display, a função setornara no código principal para a variável corrente
//		}
	}
	senha[4] = '\0';
	strcpy(senha_alarme, senha);// 'senha_alarme' é uma variavel globar para salvar a senha para desligar o alarme
	return 0; // Talvez mudar isso
}
//------------------------------------------ FUNCOES BASICAS DE MEMORIA ---------------------------------------------------------
//------------------------------------------ FUNCOES DE MEMORIA---------------------------------------------------------
void check_mem_load(void){// Verifica se a memoria esta carregada ou vazia
	uint8_t CH=0;
	AONDE=NA_SERIAL;
	HAL_StatusTypeDef ret;
	HAL_Delay(1);
	ret = HAL_I2C_Mem_Read(&hi2c1, 0xA1, 1, 1, (uint8_t*)&CH, 1, 100);
	if( ret!= HAL_OK ){
		printf(" Memoria comprometida\r\n");
			return;
	}
	else{
		if(CH == 0x1B){
			printf("Memoria carregada\r\n");
		}
		else{
			printf("Memoria vazia\r\n");
			clean_mem();
			save_logs();
		}
		senha_alarme_func();
	}
}
int save_logs(void){// Salva os usuários e suas senhas na memória principal, com uma estrutura de partição
	AONDE=NA_SERIAL;
	HAL_StatusTypeDef ret;
  /*//------------ Com o formato --------\\
	// |-Pos-| ---- Dados da Particao ----|
	// | *0* | (0x1B-Indica se esta cheia)|
	// | *1* | (--0xA0-Indica o nome 0--) |
	// |02-05| (---Nome--4 Bytes--------) |
	// | *6* | (--0xB0-Indica a senha 0-) |
	// |07-10| (--Senha--4 Bytes--------) |
	// | 11* | (--0xA1-Indica o nome 1--) |
	// |12-15| (---Nome--4 Bytes--------) |
	// | 16* | (--0xB1-Indica a senha 1-) |
	// |17-20| (--Senha--4 Bytes--------) |
	// | 21* | (--0xA2-Indica o nome 2--) |
	// |22-25| (---Nome--4 Bytes--------) |
	// | 26* | (--0xB2-Indica a senha 2-) |
	// |27-30| (--Senha--4 Bytes--------) |
	// ...................................|*/
	char nome[5]={0};
	char senhax[5]={0};
	int pos=2;
	AONDE = NA_SERIAL;
	uint8_t particao = 0xA0; // O primeiro se refere a se é nome(A) ou senha(B) e o segundo se refere user sendo '0' o usuario coringa;
	ret_error = 0;
	HAL_Delay(100);
	/////////////////////////////////////////////////////////////////////////////////////
	printf("Escrava o nome do Usuario Coringa Com 4 Letras\r\n");
	memset(nome, 0, sizeof(nome));// Limpa a variavel
	le_uart(nome);// Le a string digitada pelo usuario, com tamanho de 4 caracteres
	particao = 0xA0;// Indica que o nome 0 esta a seguir
	ret = HAL_I2C_Mem_Write(&hi2c1, 0xA0, pos, 1, (uint8_t*)&particao, 1, 1000);
	HAL_Delay(100);
	pos+=1;
	save_in_mem(&pos, nome);// Salva na memoria principal a parrticao 0xA0
	indentificador_de_mem_principal(ret);// Verifica se a memoria principal esta presente
	/*---------------------------------------------------------------------------------*/
	printf("Escrava o senha alarme Com 4 Letras, para desativar os alarmes\r\n");
	memset(senhax, 0, sizeof(senhax));// Limpa a variavel
	le_uart(senhax);// Le a string digitada pelo usuario, com tamanho de 4, mas somente aceita numeros e ignora outros chars
	particao = 0xB0;// Indica que a senha 0 esta a seguir
	ret = HAL_I2C_Mem_Write(&hi2c1, 0xA0, pos, 1, (uint8_t*)&particao, 1, 1000);
	HAL_Delay(100);
	pos+=1;
	save_in_mem(&pos, senhax);// Salva na memoria principal a parrticao 0xB0
	indentificador_de_mem_principal(ret);// Verifica se a memoria principal esta presente
	/////////////////////////////////////////////////////////////////////////////////////


	return 0;
}
int verify_acess(char *nome, char *senha){
	AONDE=NA_SERIAL;
	HAL_StatusTypeDef ret;
	//////////////////////////////////////////////////////////////////////
	char nome_user[5]={0};// Variavel para salvar o nome lido do teclado
	char senha_userr[5]={0};
	AONDE=NA_SERIAL;
	printf("Digite o nome de usuario com 4 characteres \r\n");
	le_uart(nome_user);//PEDE AO USER O NOME E SENHA
	printf("Digite a senha com 4 characteres \r\n");
	le_uart(senha_userr);
	//////////////////////////////////////////////////////////////////////
	//Agora vai tentar procurar esse nome na memoria principal
	char nome_mp[5]={0};
	int pos = 0;
	while(1){
		memset(nome_mp, 0, sizeof(nome_mp));
		busca_nome_mp(&pos,nome_mp);// Busca o nome na memoria principal
		if((strcmp(nome_mp, nome_user)==0)){// Se o nome lido do cartao for igual ao nome que esta na memoria principal
			printf("Nome encontrado\r\n");
			////////////////////////////////////////////////////////////////////////
			//Agora vai salvar a senha principal na variavel senha_mp
			printf("Verifincado senha\r\n");

			char senha[5]={0};
			for(int i = 0; i<4; i++){
				ret = HAL_I2C_Mem_Read(&hi2c1, 0xA1, pos +i, 1, (uint8_t*)&senha[i], 1, 1000);
				if (ret != HAL_OK ){
					printf(" Erro em ler a senha, tente  mais tarde\r\n");
					return 2;// Retorna pro display, a função setornara no código principal para a variável corrente
				}
			}
			senha[4] = '\0';// Senha recuuperada da memoria principal com sucesso
			if((strcmp(senha_userr, senha))==0){
					AONDE = NA_SERIAL;
					printf("\rEntrada user %s\r\n", nome_mp);
					//catraca();
					lcd_clear();
					AONDE = NO_LCD;
					lcd_goto(0,0);
					printf("Entrada user\n");
					lcd_goto(1,0);
					printf("%s\n", nome_mp);
					HAL_Delay(1500);
				}
				else{
					AONDE = NA_SERIAL;
					printf("\rAcesso negado!\r\n");
					lcd_clear();
					AONDE = NO_LCD;
					lcd_goto(0,0);
					printf("Acesso negado!\n");
					HAL_Delay(1000);
				}
				lcd_clear();
				corrente = 2;

			// 'senha' é uma variavel para salvar a senha referente ao nome que estava na mp,
			// para depois conferir com a senha que o user digitou para liberer a catraca
		}
		////////////////////////////////////////////////////////////////////////
		else printf("Procurando\r\n");
		if( pos > 18 ){//procurrar ate a posicao 18, no caso ele vai somar mais 5 e vai dar 23
			printf("Nome não encontrado na Memoria principal\r\n");
			return 2;
		}
	}
}
int busca_nome_mp(int *pos_out, char *nome_mp){// Busca o nome na memoria principal
	AONDE=NA_SERIAL;
	uint8_t ch;
	char nome[5]={0};
	int pos = *pos_out;
	HAL_StatusTypeDef ret;
	do{
		HAL_I2C_Mem_Read(&hi2c1, 0xA1, pos , 1, (uint8_t*)&ch, 1, 1000);
		pos = pos +1;
	}while(ch!=0xA0||ch!=0xA1||ch!=0xA2);
	if(ch==0xA0||ch==0xA1||ch==0xA2){
		for(int i = 0; i<4; i++){// Salva o nome presente no cartão!
			ret = HAL_I2C_Mem_Read(&hi2c1, 0xA1, pos +i, 1, (uint8_t*)&nome[i], 1, 1000);
			//HAL_Delay(100);
			if (ret != HAL_OK ){
				printf(" Erro em ler o nome, retire o cartao\r\n");
				return 2;// Retorna pro display, a função setornara no código principal para a variável corrente
			}
		}
		nome[4] = '\0';
	}
	*pos_out = pos + 5;// Soma 5 para ir para a proxima posicao
	strcpy(nome_mp, nome);// Salva o nome na variavel global
	return 5;// Sera decidido depois
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
