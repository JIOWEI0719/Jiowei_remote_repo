/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "oled.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc3;

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
char message[20];

char ActualValue_ch[20];
char SetPoint_ch[20];

uint8_t state=0;

float Time_Stable=0.0f;

//---------------------------
float output=0.0;
float Stable_Time=0.0;

char Stable_Time_ch[20];

uint8_t Stable_Cnt=0;
uint32_t Stable_Time_Cnt=0;
uint8_t Stabled=0;
uint8_t Printed=0;

uint8_t rxBuffer[30];
/* PID_stucture_definetion*/
typedef struct {
    float SetPoint;     
    float ActualValue;  
    float Err;          
    float ErrLast;      
    float Kp, Ki, Kd;   
    float Integral;     
    float Output;       
} PID_TypeDef;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM8_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */
/*int fputc(int ch, FILE *f)
{
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
	return (ch);
}*/

void UART_SendString_huart1(char *str)
{
	unsigned int k=0;
	do{
		HAL_UART_Transmit(&huart1,(uint8_t *)(str + k) ,1,1000);
		k++;
	}while (*(str + k)!='\0');
}

short GetEncoder(void)
{
	short CNT=0;
	short counter1=__HAL_TIM_GET_COUNTER(&htim8);//read the counter1
	HAL_Delay(100);
	short counter2=__HAL_TIM_GET_COUNTER(&htim8);//read the counter2
	if(counter2-counter1>0)//make sure that we can get a positive speed for calculations
		CNT= counter2-counter1;
	else
		CNT= counter1-counter2;
	counter1=counter2=0;
	__HAL_TIM_GET_COUNTER(&htim8)=0;
	return CNT;
}

char* GetSpeed(PID_TypeDef* pid)
{
	pid->ActualValue=((float)GetEncoder())*60.0f*10.0f/(13.0f*4.0f*30.0f);
	
 	snprintf(ActualValue_ch,sizeof(ActualValue_ch),"%f",pid->ActualValue);
	snprintf(message,sizeof(message),"%.2f,%.2f\n",pid->ActualValue,pid->SetPoint);
	snprintf(SetPoint_ch,sizeof(SetPoint_ch),"%.2f",pid->SetPoint);
	
	UART_SendString_huart1(message);
	return ActualValue_ch;
}

void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd)
{
    pid->SetPoint = 30.0;
    pid->ActualValue = 0.0;
    pid->Err = 0.0;
    pid->ErrLast = 0.0;
    pid->Integral = 0.0;
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->Output = 0.0;
}


float PID_Calculate(PID_TypeDef *pid)
{
    pid->Err = pid->SetPoint - pid->ActualValue;
    pid->Integral += pid->Err;
    if (pid->Integral > 1000) pid->Integral = 1000;
    if (pid->Integral < -1000) pid->Integral = -1000;

	
    pid->Output = pid->Kp * pid->Err + 
                  pid->Ki * pid->Integral + 
                  pid->Kd * (pid->Err - pid->ErrLast);

    pid->ErrLast = pid->Err;
    
    return pid->Output;
}


float string_to_float(uint8_t *str) {
    int sign = 1;
    float integer = 0, fraction = 0.1;
    int decimal_found = 0;

    if (*str == '-') {
        return -1.0;//Invalid Input
    }

    while (*str != '\n'&&*str != ',') {
        if (*str == '.') {
            decimal_found = 1;
            str++;
            continue;
        }
        if (*str >= '0' && *str <= '9') {
            if (!decimal_found) {
                integer = integer * 10 + (*str - '0');
            } else {
                integer += (*str - '0') * fraction;
                fraction *= 0.1f;
            }
        } else {
            break; 
        }
        str++;
    }
    return sign * integer;
}
uint8_t* USART_Receive_Pointer=NULL;
float Time_Buffer=0.0;
char Time_Buffer_ch[20];
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
PID_TypeDef speedPID;
/*void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) 
    {
       Set_DA(curv);
    }
}*/
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
 PID_Init(&speedPID,0.003,0.10,0.001);  // Kp, Ki, Kd
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
  MX_DMA_Init();
  MX_DAC_Init();
  MX_TIM3_Init();
  MX_SPI1_Init();
  MX_ADC3_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  OLED_DisPlay_On();
  OLED_Init(); 
 
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start_IT(&htim7);
	//-------------PWM Start---------------------//
	HAL_TIM_Base_Start_IT(&htim3);//PWM Output
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	//-------------TIM8_Encoder_Start------------//
	HAL_TIM_Encoder_Start(&htim8,TIM_CHANNEL_ALL);
	
	MX_USART1_UART_Init();
  while (1)
  {		
		OLED_Refresh();
		
		HAL_UART_Receive_IT(&huart1,rxBuffer,sizeof(rxBuffer));
		if(rxBuffer[0]!='\0')
		{
			USART_Receive_Pointer=rxBuffer;
			speedPID.Kp=string_to_float(USART_Receive_Pointer);
			USART_Receive_Pointer++;
			speedPID.Ki=string_to_float(USART_Receive_Pointer);
			USART_Receive_Pointer++;
			speedPID.Kd=string_to_float(USART_Receive_Pointer);
		}
		//-------------------RCC&SPD------------------//
		//
		OLED_ShowString(10,0,(uint8_t *)"RCC&SPD:",8,1);
		if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_3)==GPIO_PIN_RESET)
		{
			HAL_Delay(20);
			if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_3)==GPIO_PIN_RESET)//PE3 KEY0 +5%
			{
				Stabled=0; 
				Stable_Cnt=0;
				Stable_Time_Cnt=0;
				Stable_Time=0.0;
				//--------------------------
				speedPID.SetPoint+=30.0f;
				if(speedPID.SetPoint>=550.0f)
					speedPID.SetPoint=550.0f;
			}
		}
		if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_2)==GPIO_PIN_RESET)
		{
			HAL_Delay(20);
			if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_2)==GPIO_PIN_RESET)//PE2 KEY1 -5%
			{
				Stabled=0; 
				Stable_Cnt=0;
				Stable_Time_Cnt=0;
				Printed=0;
				Stable_Time=0.0;
				//--------------------------
				speedPID.SetPoint-=30.0f;
				if(speedPID.SetPoint<30.0f)
					speedPID.SetPoint=30.0f;
			}
		}
		
		OLED_ShowNum(57,0,speedPID.Output,2,8,1);//Now PWM
		OLED_ShowString(78,0,(uint8_t*)SetPoint_ch,8,1);//Set Speed
		//-------------------Motor Direction-----------//
		//
		OLED_ShowString(10,12,(uint8_t *)"Direction:",8,1);
		
		if (state==0)
			OLED_ShowString(70,12,(uint8_t *)"Stop",8,1);
		else if(state==1)
			OLED_ShowString(70,12,(uint8_t *)"-   ",8,1);
		else if(state==2)
			OLED_ShowString(70,12,(uint8_t *)"+   ",8,1);

		if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_1)==GPIO_PIN_RESET)//KEY_2 - direction
		{
			HAL_Delay(15);
			if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_1)==GPIO_PIN_RESET)
			{
				Stabled=0; 
				Stable_Cnt=0;
				Stable_Time_Cnt=0;
				Printed=0;
				Stable_Time=0.0;
				//--------------------------
				OLED_ShowString(70,12,(uint8_t *)"-   ",8,1);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);//AIN1 1
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_RESET);//AIN2 0
				state=1;
			}
		}
		else if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_0)==GPIO_PIN_RESET)//KEY_3 + direction
		{
			HAL_Delay(15);
			if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_0)==GPIO_PIN_RESET)
			{
				Stabled=0; 
				Stable_Cnt=0;
				Stable_Time_Cnt=0;
				Printed=0;
				Stable_Time=0.0;
				//--------------------------
				OLED_ShowString(70,12,(uint8_t *)"+   ",8,1);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);//AIN1 0
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET);//AIN2 1
				state=2;
			}
		}
		else if(HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_15)==GPIO_PIN_RESET)//KEY_4 stop
		{
			HAL_Delay(15);
			if(HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_15)==GPIO_PIN_RESET)
			{
				Stabled=0; 
				Stable_Cnt=0;
				Stable_Time_Cnt=0;
				Printed=0;
				Stable_Time=0.0;
				//--------------------------
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);//AIN1 0
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_RESET);//AIN2 0
				OLED_ShowString(70,12,(uint8_t *)"Stop",8,1);
				state=0;
			}				
		}
		//-----------------Motor Get Speed Checking----------------//
		//
		OLED_ShowString(10,24,(uint8_t *)"Real Speed(r/min):",8,1);
		
		OLED_ShowString(10,36,(uint8_t *)GetSpeed(&speedPID),8,1);
	
		//-----------------Checking the Time to be stable-------------------------//
		//
		OLED_ShowString(10,48,(uint8_t *)"Acc_Tim(s):",8,1);
		snprintf(Stable_Time_ch,sizeof(Stable_Time_ch),"%.2f",Stable_Time);
		if(Stabled)
		{
			if(!Printed)
			{
				OLED_ShowString(78,48,(uint8_t *)Stable_Time_ch,8,1);
				Time_Buffer=string_to_float((uint8_t *)Stable_Time_ch);
				snprintf(Time_Buffer_ch,sizeof(Time_Buffer_ch),"%.2f",Time_Buffer);
				Printed=1;
			}
			if(Printed)
			{
				OLED_ShowString(78,48,(uint8_t *)Time_Buffer_ch,8,1);
			}
		}
		else
		{
			
		}
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 15999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 4199;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 4999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 4199;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim8, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OLED_RES_GPIO_Port, OLED_RES_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, AIN1_Pin|AIN2_Pin|OLED_DC_Pin|OLED_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : KEY_1_Pin KEY_0_Pin KEY_3_Pin KEY_2_Pin */
  GPIO_InitStruct.Pin = KEY_1_Pin|KEY_0_Pin|KEY_3_Pin|KEY_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : OLED_RES_Pin */
  GPIO_InitStruct.Pin = OLED_RES_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OLED_RES_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY_4_Pin */
  GPIO_InitStruct.Pin = KEY_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(KEY_4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : AIN1_Pin AIN2_Pin OLED_DC_Pin OLED_CS_Pin */
  GPIO_InitStruct.Pin = AIN1_Pin|AIN2_Pin|OLED_DC_Pin|OLED_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/*float output=0.0;
float Stable_Time=0.0;

uint8_t Stable_Cnt=0;
uint32_t Stable_Time_Cnt=0;
uint8_t Stabled=0;*/

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
 { 
	 if (htim->Instance == TIM6) 
	 {
		 if(state!=0)
		 { 
				if(state==1&&speedPID.SetPoint<0)
					speedPID.SetPoint=-speedPID.SetPoint;
				output=PID_Calculate(&speedPID);
			 if(output>0)
			 {
					if(output>100) output=100;
					if(output<0) output=0;
			 }
			 else{
					output=-output;
					if(output>100) output=100;
					if(output<0) output=0;
			 }
			 speedPID.ActualValue=(uint8_t)output;

			 __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_2,(int)speedPID.ActualValue);
		}
	 }
//---------------------------Stable Time Checking----------------------------------------
	 if (htim->Instance == TIM7) 
	 {
		 if(state!=0&&Stabled!=1&&Printed!=1)
		 {
				 Stable_Time_Cnt++;
				 if(speedPID.ActualValue<1.05f*speedPID.SetPoint&&
						speedPID.ActualValue>0.95f*speedPID.SetPoint)
					{
					Stable_Cnt++;
					}
				 if(Stable_Cnt==10)
				 {
					Stable_Time=0.1*Stable_Time_Cnt;
					Stabled=1;
				 }
		 }
		 else 
		 {
					Stabled=0; 
					Stable_Cnt=0;
					Stable_Time_Cnt=0;
					Stable_Time=0.0;
		}
	 }
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
