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
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define V_BUF_LEN 8U
#define I_BUF_LEN 8U

#define ADC1_VOLT_CHANNEL ADC_CHANNEL_1
#define ADC2_CURR_CHANNEL ADC_CHANNEL_3

// LPF
#define SAMPLING_FREQ 250000.0f
#define CUTOFF_FREQ   1000.0f
#define PI            3.14159265359f
#define LPF_ALPHA     ( (2.0f * PI * CUTOFF_FREQ) / (SAMPLING_FREQ + 2.0f * PI * CUTOFF_FREQ) )
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

FDCAN_HandleTypeDef hfdcan2;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint16_t v_buf[V_BUF_LEN]; // ADC1(전압) DMA 버퍼
uint16_t i_buf[I_BUF_LEN]; // ADC2(전류) DMA 버퍼

// 전압 클램핑 ADC 코드
uint16_t high_th  = 2731; // 43.5V
uint16_t low_th = 2712; // 42.5V

volatile uint8_t gpio_gate_on = 0; // GPIO 제어 핀 상태
volatile uint8_t pwm_on = 0; // PWM 제어 핀 상태, 현재 미사용

// 디바운싱 설정
static uint8_t debounce_count = 4;  // 연속된 샘플 개수 (변경 가능)
static uint8_t debounce_high_cnt = 0;  // high 임계값 초과 카운터
static uint8_t debounce_low_cnt = 0;   // low 임계값 미만 카운터


// 필터된 값
static volatile float filtered_v_code = 0.0f;
static volatile float filtered_i_code = 0.0f;


// MA 필터된 값
static volatile float v_ma = 0.0f;
static volatile float i_ma = 0.0f;


/* CAN-BUS */
FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8];
uint8_t RxData[8];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_FDCAN2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
void ADC_StartAll(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// ADC 초기 설정
void ADC_StartAll(void) {
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_DIFFERENTIAL_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)v_buf, V_BUF_LEN);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)i_buf, I_BUF_LEN);
}

//GPIO 히스테리시스 제어용 (최적화: 직접 레지스터 접근)
static inline void GPIO_GateOn(void) {
    GPIOA->BSRR = GPIO_PIN_9;  // 직접 레지스터 쓰기 (HAL 방식보다 빠름)
    gpio_gate_on = 1;
//    high_th  = 2731; // 43.5V
}
static inline void GPIO_GateOff(void) {
    GPIOA->BSRR = (uint32_t)GPIO_PIN_9 << 16U;  // Reset 비트
    gpio_gate_on = 0;
}

// GPIO 제어핀 상태
static inline uint8_t GPIO_InstantBit(void) {
    return (HAL_GPIO_ReadPin(Gate_GPIO_Port, Gate_Pin) == GPIO_PIN_SET) ? 1u : 0u;
}

// adc값->mV
static inline int16_t Code_to_VmV(uint16_t code) {
    float v_bus = ((float) code - 2048.0) * 3.3f * 40.0f / 2047.0f;
    int32_t mV = (int32_t)(v_bus * 1000.0f + 0.5f);
    if (mV < 0) mV = 0;
    return mV;
}

// adc값->mA
static inline int32_t Code_to_ImA(uint16_t code) {
    float i = (float) code * 3.3f / 4095.0f;
    float I = (i - 1.65f) / 0.066f;
    return (int32_t) (I * 1000.0f);
}

// 가장 최신 VI 샘플
static void ReadLatest(uint16_t* v_code, uint16_t* i_code) {
    // DMA 잔량(CNDTR)로 현재 쓰기 위치 계산
    uint32_t pos_v = V_BUF_LEN - (uint32_t)(hadc1.DMA_Handle->Instance->CNDTR);
    uint32_t pos_i = I_BUF_LEN - (uint32_t)(hadc2.DMA_Handle->Instance->CNDTR);
    pos_v %= V_BUF_LEN; pos_i %= I_BUF_LEN;

    // 두 DMA의 진행이 미세하게 다를 수 있으니 더 작은 쪽을 기준
    uint32_t pos_min = (pos_v < pos_i) ? pos_v : pos_i;

    // 직전에 완료된 인덱스 = pos_min - 1 (링 보정)
    uint32_t idx = (pos_min == 0) ? (V_BUF_LEN - 1) : (pos_min - 1);
    *v_code = v_buf[idx];
    *i_code = i_buf[idx];
}

/* ==== UART 명령 ==== */
#define RX_BUFSZ 16
static uint8_t  rx_byte; // Interrupt로 받은 1바이트
static char     rx_line[RX_BUFSZ]; // trim된 데이터 문자열
static uint16_t rx_len = 0; // 커서

// 초기화
void UART_Q_Init(void) {
    rx_len = 0;
    HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
}

// 문자 전송
static inline void UART_SendStr(const char* s) {
    HAL_UART_Transmit(&huart2, (uint8_t*)s, (uint16_t)strlen(s), 20);
}

// 전송받은 문자열 핸들링
static void UART_HandleLine(const char* line_in) {
    // '\r' 제거 및 공백 트림
    char line[RX_BUFSZ];
    size_t L = strnlen(line_in, sizeof(line)-1); // 현재 길이
    while (L && (line_in[L-1]=='\r' || line_in[L-1]==' ' || line_in[L-1]=='\t')) --L; // 공백이나 '\r' 이면 트림
    memcpy(line, line_in, L); line[L] = '\0'; // 문자열 끝에 NULL

    if (strcmp(line, "Q") == 0) { // Q 명령: 현재 V, I, GPIO상태, PWM상태, PWM추기 출력

        // 최신 동시 샘플 추출 (전류만 사용)
        uint16_t v_code, i_code;
        ReadLatest(&v_code, &i_code);

        // 물리량 환산 (전압/전류 모두 필터링된 값 사용)
        int32_t v_mV = Code_to_VmV((uint16_t)(filtered_v_code + 0.5f));
        int32_t i_mA = Code_to_ImA((uint16_t)(filtered_i_code + 0.5f));

        // 히스테리시스 gate 핀 상태
        uint8_t gpio   = GPIO_InstantBit();

        // PWM 채널 미사용으로 인한 0 전송
        uint8_t pwm_on = 0;
        int duty_ppt = 0;

        // 한 줄 응답
        char msg[84];
        int n = snprintf(msg, sizeof(msg), "D,%ld,%ld,%u,%u,%d\n",
                         (long)v_mV, (long)i_mA, (unsigned)gpio, (unsigned)pwm_on, duty_ppt);
        if (n > 0) HAL_UART_Transmit(&huart2, (uint8_t*)msg, (uint16_t)n, 20);
    }
    else {
        UART_SendStr("ERR\n");
        return; //"Q" 아니면 리턴
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_FDCAN2_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  ADC_StartAll();
  // BrakePWM_Start();
  // HAL_TIM_Base_Start_IT(&htim7); // PI 제어 사용 안 함
  UART_Q_Init();
  
  // LPF 상태 변수 초기화 (첫 ADC 값으로)
  HAL_Delay(10); // ADC 안정화 대기 (더 길게)
  
  // 여러 샘플 평균으로 초기화 (안정적)
  float sum_v = 0.0f, sum_i = 0.0f;
  for(int n = 0; n < 10; n++) {
      sum_v += (float)HAL_ADC_GetValue(&hadc1);
      sum_i += (float)HAL_ADC_GetValue(&hadc2);
      HAL_Delay(1);
  }
  filtered_v_code = sum_v / 10.0f;
  filtered_i_code = sum_i / 10.0f;
  
  // 로컬 static 변수는 TIM6 콜백 시작 시 자동 초기화됨 (0.0f)
  // 첫 몇 샘플 동안 빠르게 수렴함
  
  // TIM6 인터럽트 시작 (초기화 완료 후)
  HAL_TIM_Base_Start_IT(&htim6);

  /* FDCAN Start */
  if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK) {
      Error_Handler();
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
      Error_Handler();
  }

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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T6_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.OversamplingMode = ENABLE;
  hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_4;
  hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_2;
  hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc1.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_DIFFERENTIAL_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T6_TRGO;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc2.Init.OversamplingMode = ENABLE;
  hadc2.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_4;
  hadc2.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_2;
  hadc2.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc2.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief FDCAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN2_Init(void)
{

  /* USER CODE BEGIN FDCAN2_Init 0 */

  /* USER CODE END FDCAN2_Init 0 */

  /* USER CODE BEGIN FDCAN2_Init 1 */

  /* USER CODE END FDCAN2_Init 1 */
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = DISABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 17;
  hfdcan2.Init.NominalSyncJumpWidth = 1;
  hfdcan2.Init.NominalTimeSeg1 = 7;
  hfdcan2.Init.NominalTimeSeg2 = 2;
  hfdcan2.Init.DataPrescaler = 1;
  hfdcan2.Init.DataSyncJumpWidth = 1;
  hfdcan2.Init.DataTimeSeg1 = 1;
  hfdcan2.Init.DataTimeSeg2 = 1;
  hfdcan2.Init.StdFiltersNbr = 0;
  hfdcan2.Init.ExtFiltersNbr = 0;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */

  
  FDCAN_FilterTypeDef sFilterConfig;

  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_DUAL;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x101;
  sFilterConfig.FilterID2 = 0x7FF;

  if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK)
  {
      Error_Handler();
  }
  /* USER CODE END FDCAN2_Init 2 */

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
  htim6.Init.Prescaler = 15;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 84;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  huart2.Init.BaudRate = 921600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Gate_GPIO_Port, Gate_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_TIM1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Gate_Pin */
  GPIO_InitStruct.Pin = Gate_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Gate_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief  FDCAN Tx, 응답 전송
  * @note   ID:0x100, DLC:8
  */
volatile uint32_t cantest = 0;
void CAN_Send_Status_Response(void) {

    // 전압, 전류
    uint16_t voltage_mv = (uint16_t)Code_to_VmV((uint16_t)(filtered_v_code + 0.5f));
    uint16_t current_ma = (uint16_t)Code_to_ImA((uint16_t)(filtered_i_code + 0.5f));
    uint16_t duty_permille = 0; // PWM 안써서 0

    // 핀 상태
    uint8_t pin_status = 0;
//    pin_status |= (PWM_InstantBit() & 0x01) << 1;    // Bit 1: PWM 핀: 미사용으로 인한 0
    pin_status |= (GPIO_InstantBit() & 0x01) << 0;   // Bit 0: 히스테리시스 핀

    // CAN Tx 메시지 설정
    TxHeader.Identifier = 0x100;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    // 데이터 패킹 (Big-Endian)
    TxData[0] = (voltage_mv >> 8) & 0xFF;
    TxData[1] = voltage_mv & 0xFF;
    TxData[2] = (current_ma >> 8) & 0xFF;
    TxData[3] = current_ma & 0xFF;
    TxData[4] = (duty_permille >> 8) & 0xFF;
    TxData[5] = duty_permille & 0xFF;
    TxData[6] = pin_status;
    TxData[7] = 0x00; // Reserved

    // 메시지 전송
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, TxData) != HAL_OK) {
        // 전송 실패
        Error_Handler();
    }
    cantest++;
}

/**
  * @brief  FDCAN Rx FIFO 0 수신 콜백
  * @note   ID 0x101 메시지 수신 시 호출됨
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
    if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
        // FIFO 0에서 메시지 수신
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK) {
            Error_Handler();
        }

        // 요청 메시지인지 확인 (ID: 0x101, DLC: 1, Data: 0x01)
        if (RxHeader.Identifier == 0x101 && RxHeader.DataLength == FDCAN_DLC_BYTES_1 && RxData[0] == 0x01) {
            CAN_Send_Status_Response();
        }
    }
}

/**
 * @brief 반핑퐁 함수
 * @note MCU 부담 감소, MA 계산(디버그용)
 */
void HAL_DMA_XferHalfCpltCallback(DMA_HandleTypeDef *hdma) {
    if (hdma == hadc1.DMA_Handle) {
        // v_buf[0 .. V_BUF_LEN/2-1] 처리
        v_ma = v_buf[V_BUF_LEN/2 - 1];
    }
    else if (hdma == hadc2.DMA_Handle) {
        // i_buf[0 .. I_BUF_LEN/2-1] 처리
        i_ma = i_buf[I_BUF_LEN/2 - 1];
    }
}

void HAL_DMA_XferCpltCallback(DMA_HandleTypeDef *hdma) {
    if (hdma == hadc1.DMA_Handle) {
        // v_buf[V_BUF_LEN/2 .. V_BUF_LEN-1] 처리
        v_ma = v_buf[V_BUF_LEN - 1];
    }
    else if (hdma == hadc2.DMA_Handle) {
        // i_buf[I_BUF_LEN/2 .. I_BUF_LEN-1] 처리
        i_ma = i_buf[I_BUF_LEN - 1];
    }
}

/**
 * @brief TIM_period 콜백함수
 * @note TIM6: LPF 및 히스테리시스 제어
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM6) {
        // 125kHz로 실행되는 LPF 및 히스테리시스 제어
        // HAL 함수를 사용하여 ADC 값 읽기 (125kHz 빠른 샘플링)
        uint16_t v = (uint16_t) HAL_ADC_GetValue(&hadc1);
        uint16_t i = (uint16_t) HAL_ADC_GetValue(&hadc2);

        // LPF 적용 (float, 전역 변수 직접 사용)
        filtered_v_code += LPF_ALPHA * ((float)v - filtered_v_code);
        filtered_i_code += LPF_ALPHA * ((float)i - filtered_i_code);

        // 히스테리시스 제어 (디바운싱 적용)
//#define FILTERED_CONTROL
#define RAW_CONTROL

#ifdef RAW_CONTROL
        // 디바운싱: 연속된 N개 샘플이 임계값을 넘어야 동작
        if (!gpio_gate_on) {
            if (v > high_th) {
                debounce_high_cnt++;
                debounce_low_cnt = 0;  // 반대쪽 카운터 리셋
                
                if (debounce_high_cnt >= debounce_count) {
                    GPIO_GateOn();
                    debounce_high_cnt = 0;  // 카운터 리셋
                }
            } else {
                debounce_high_cnt = 0;  // 조건 불만족 시 리셋
            }
        }
        else {  // gpio_gate_on == 1
            if (v < low_th) {
                debounce_low_cnt++;
                debounce_high_cnt = 0;  // 반대쪽 카운터 리셋
                
                if (debounce_low_cnt >= debounce_count) {
                    GPIO_GateOff();
                    debounce_low_cnt = 0;  // 카운터 리셋
                }
            } else {
                debounce_low_cnt = 0;  // 조건 불만족 시 리셋
            }
        }
#endif
#ifdef FILTERED_CONTROL
        if (!gpio_gate_on && filtered_v_code > high_th) {
            GPIO_GateOn();
        }
        else if (gpio_gate_on && filtered_v_code < low_th) {
            GPIO_GateOff();
        }
#endif
    }
}

/**
 * @brief UART 수신 콜백함수
 * @note USART2: UART 모니터링부
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        uint8_t b = rx_byte; // 받은 1바이트 데이터
        if (b == '\n' || rx_len >= (RX_BUFSZ - 1)) { // EOL 또는 배열 끝
            rx_line[rx_len] = '\0'; // '\0' 문자열 끝 처리
            if (rx_len) UART_HandleLine(rx_line); // 해당 버퍼의 문자열 처리
            rx_len = 0; // 커서 초기화
        } else {
            if (b != '\r') rx_line[rx_len++] = (char)b; // '\r'이 아닌 글자면 수신버퍼에 저장 후 커서 1 증가
        }
        HAL_UART_Receive_IT(&huart2, &rx_byte, 1); // 계속 수신
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
#ifdef USE_FULL_ASSERT
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
