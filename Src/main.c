/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dfsdm.h"
#include "dma.h"
#include "sai.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "wav_example.h"
#include "cs43l22.h"
#include "stm32l476g_discovery.h"
#include "opus.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define FRAME_SIZE              160
#define SaturaLH(N, L, H) (((N)<(L))?(L):(((N)>(H))?(H):(N)))

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint32_t pause_cnt = 0;

unsigned char micr_gain = 50;

unsigned short device_id = 0x01;
unsigned short gate_id = 0xFE;
unsigned short point_to_point_tmr = 0x00;
unsigned char to_id = 0xFF;

//static unsigned long t = 0;

#define BUF_LENGTH	3

int32_t		RecBuf[2][FRAME_SIZE];
int16_t		micr_data[2][FRAME_SIZE];
int16_t		PlayBuf[BUF_LENGTH][FRAME_SIZE*2];
char microphone_encoded_data[2][256];
unsigned char encoded_micr_ready_buf_num = 0;
int8_t encoded_length = 0;
uint8_t can_pckt_length = 0;
int16_t	audio_stream[1024];
uint32_t	wav_pos = 0;
uint32_t frame_num = 0;
uint32_t cur_frame = 0;

uint32_t							DmaRecHalfBuffCplt  = 0;
uint32_t    						DmaRecBuffCplt      = 0;

extern DFSDM_Filter_HandleTypeDef 	hdfsdm1_filter0;
extern SAI_HandleTypeDef 			hsai_BlockA1;
AUDIO_DrvTypeDef					*audio_drv;

extern CAN_HandleTypeDef hcan1;
static CAN_TxHeaderTypeDef   TxHeader;
static uint32_t              TxMailbox=0;
static uint8_t               TxData[8];
static CAN_RxHeaderTypeDef   RxHeader;
static uint8_t               RxData[8];
static uint8_t				 can_frame[20];
static uint8_t				 can_frame_id[256][20];
static uint8_t				 frame_ready = 0;

static uint32_t i;
static uint16_t led_tmr=0;
static uint8_t buf_num = 0;
static int res = 0;

OpusDecoder *dec;
OpusEncoder *enc;

static int error = 0;

volatile unsigned short sys_tmr = 0;
unsigned long tmp=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void delay_ms(unsigned short value) {
	sys_tmr=0;
	while(sys_tmr<(unsigned long)value*10) HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
}

static void play(void) {
	uint8_t buf_num = 0;
	if(frame_num>=1) {
		if(frame_num-cur_frame!=1) {
			cur_frame = frame_num-1;
		}
		buf_num = cur_frame%BUF_LENGTH;
		audio_drv->Play(AUDIO_I2C_ADDRESS, (uint16_t *) &PlayBuf[buf_num][0], FRAME_SIZE*2);
		HAL_SAI_Transmit_DMA(&hsai_BlockA1, (uint8_t *) &PlayBuf[buf_num][0], FRAME_SIZE*2);
		cur_frame++;
    }
}

static void initCANFilter() {
	CAN_FilterTypeDef  sFilterConfig;

	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;

	HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
}

static void send_full_frame(uint8_t len, uint8_t *ptr) {
	uint8_t i=len;
	uint8_t cur_pckt = 1;
	uint8_t pckt_cnt = 0;

	while(i) {
		pckt_cnt++;
		if(i<=6) {i=0;}
		else i-=7;
	}
	i=1;

	while(len>0) {
		TxHeader.StdId = device_id;
		TxHeader.ExtId = 0;
		TxHeader.RTR = CAN_RTR_DATA;
		TxHeader.IDE = CAN_ID_STD;
		TxHeader.TransmitGlobalTime = DISABLE;
		if(len<=6) { // last packet
			TxHeader.DLC = 2+len;
			TxData[0] = (cur_pckt&0x0F)|((pckt_cnt&0x0F)<<4); // current packet number and packets cnt
			TxData[1] = to_id;
			for(i=0;i<len;i++) TxData[i+2] = ptr[(cur_pckt-1)*7+i];
			while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {delay_ms(1);}
			HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
			cur_pckt++;
			len=0;
		}else {
			TxHeader.DLC = 0x08;
			TxData[0] = (cur_pckt&0x0F)|((pckt_cnt&0x0F)<<4); // current packet number and packets cnt
			for(i=0;i<7;i++) TxData[i+1] = ptr[(cur_pckt-1)*7+i];
			while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {delay_ms(1);}
			HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
			cur_pckt++;
			len-=7;
		}
	}
	//HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port,GREEN_LED_Pin);
}

static void encode_work() {
  if(DmaRecHalfBuffCplt == 1) {

	  for(i=0;i<FRAME_SIZE;i++) {
		  micr_data[0][i] = (int16_t)((uint16_t)wav_ex[wav_pos+1]<<8 | wav_ex[wav_pos])*(int32_t)micr_gain/100;
		  wav_pos+=2;if(wav_pos>=sizeof(wav_ex)) wav_pos=0;
	  }

	  //HAL_GPIO_WritePin(GREEN_LED_GPIO_Port,GREEN_LED_Pin,GPIO_PIN_SET);
	  encoded_length = opus_encode(enc, (opus_int16*)&micr_data[0][0], FRAME_SIZE,(unsigned char*) &microphone_encoded_data[0][0], 256);

	  //HAL_GPIO_WritePin(GREEN_LED_GPIO_Port,GREEN_LED_Pin,GPIO_PIN_RESET);
	  encoded_micr_ready_buf_num = 1;

	  DmaRecHalfBuffCplt  = 0;
  }
  if(DmaRecBuffCplt == 1)
  {
	  //for(i=0;i<FRAME_SIZE;i++) {micr_data[1][i] = SaturaLH((RecBuf[1][i] >>8), -32768, 32767)*(int32_t)micr_gain/100;}
	  for(i=0;i<FRAME_SIZE;i++) {
		  micr_data[1][i] = (int16_t)((uint16_t)wav_ex[wav_pos+1]<<8 | wav_ex[wav_pos])*(int32_t)micr_gain/100;
		  wav_pos+=2;if(wav_pos>=sizeof(wav_ex)) wav_pos=0;
	  }
	  //HAL_GPIO_WritePin(GREEN_LED_GPIO_Port,GREEN_LED_Pin,GPIO_PIN_SET);
	  encoded_length = opus_encode(enc, (opus_int16*)&micr_data[1][0], FRAME_SIZE,(unsigned char*) &microphone_encoded_data[1][0], 256);
	  //HAL_GPIO_WritePin(GREEN_LED_GPIO_Port,GREEN_LED_Pin,GPIO_PIN_RESET);
	  encoded_micr_ready_buf_num = 2;

	  DmaRecBuffCplt  = 0;
  }
}

static void decode_work() {
  if(frame_ready) {
	  pause_cnt = 0;
	  frame_ready=0;
	  res = opus_decode(dec,(unsigned char*)&can_frame[0],can_pckt_length,&audio_stream[0],1024,0);
	  if(res==160) HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port,GREEN_LED_Pin);
	  buf_num = frame_num%BUF_LENGTH;

	  for(i=0;i<FRAME_SIZE;i++) {
		  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET || res<=0) {
			  PlayBuf[buf_num][2*(FRAME_SIZE-1-i)] = 0;
			  PlayBuf[buf_num][2*(FRAME_SIZE-1-i)+1] = 0;
		  }else {
			  PlayBuf[buf_num][2*(FRAME_SIZE-1-i)] = audio_stream[FRAME_SIZE-1-i];
			  PlayBuf[buf_num][2*(FRAME_SIZE-1-i)+1] = PlayBuf[buf_num][2*(FRAME_SIZE-1-i)];
		  }

	  }
	  frame_num++;
	  //HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port,GREEN_LED_Pin);
  }else {
	  pause_cnt++;
	  if(pause_cnt>=30) {
		  pause_cnt = 0;
		  buf_num = frame_num%BUF_LENGTH;
		  for(i=0;i<FRAME_SIZE;i++) {
			  PlayBuf[buf_num][2*(FRAME_SIZE-1-i)] = 0;
			  PlayBuf[buf_num][2*(FRAME_SIZE-1-i)+1] = 0;
		  }
		  frame_num++;
	  }
  }
}

static void can_work() {
  static unsigned char button_state = 0;
  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)!=button_state) {
	  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) HAL_CAN_DeactivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	  else HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  }
  if(encoded_micr_ready_buf_num) {
	  if(encoded_length>0) {
		  //HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port,GREEN_LED_Pin);
		  can_pckt_length = encoded_length;
		  for(i=0;i<can_pckt_length;i++) can_frame[i] = microphone_encoded_data[encoded_micr_ready_buf_num-1][i];
		  frame_ready = 1;

	  }
	  /*if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) {
		  if(encoded_length>0) {
			  send_full_frame(encoded_length,(unsigned char*)&microphone_encoded_data[encoded_micr_ready_buf_num-1]);
		  }
	  }*/
	  encoded_micr_ready_buf_num=0;
  }
  button_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);

  if(point_to_point_tmr) point_to_point_tmr--;
  else to_id = 0xFF; // send data to all
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
  MX_DFSDM1_Init();
  MX_SAI1_Init();
  MX_CAN1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  enc = opus_encoder_create(8000, 1, OPUS_APPLICATION_VOIP , &error);
  opus_encoder_ctl(enc, OPUS_SET_COMPLEXITY(1));
  opus_encoder_ctl(enc, OPUS_SET_BITRATE(8000));
  dec = opus_decoder_create(8000,1,&error);

  HAL_TIM_Base_Start_IT(&htim1);

  if(CS43L22_ID != cs43l22_drv.ReadID(AUDIO_I2C_ADDRESS))  {Error_Handler();}
  audio_drv = &cs43l22_drv;
  audio_drv->Reset(AUDIO_I2C_ADDRESS);
  if(0 != audio_drv->Init(AUDIO_I2C_ADDRESS, OUTPUT_DEVICE_HEADPHONE, 60, AUDIO_FREQUENCY_8K)) { Error_Handler();}

  HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0, (int32_t*)&RecBuf[0][0], FRAME_SIZE*2);

  initCANFilter();
  HAL_CAN_Start(&hcan1);
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
      Error_Handler();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  decode_work();
	  encode_work();
	  can_work();



	  delay_ms(1);
	  led_tmr++;
	  if(led_tmr>=500) {
		  led_tmr=0;
		  //HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port,GREEN_LED_Pin);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_7;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_SAI1|RCC_PERIPHCLK_DFSDM1;
  PeriphClkInit.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI1;
  PeriphClkInit.Dfsdm1ClockSelection = RCC_DFSDM1CLKSOURCE_PCLK;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 2;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 43;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void SysTick_Handler(void) {

}

void HAL_DFSDM_FilterRegConvHalfCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
  DmaRecHalfBuffCplt = 1;

  	  /*for(i=0;i<FRAME_SIZE;i++) {
  		  PlayBuf[frame_num%BUF_LENGTH][2*i] = (int16_t)((uint16_t)wav_ex[t+1]<<8 | wav_ex[t])*(int32_t)micr_gain/100;
  	  	  PlayBuf[frame_num%BUF_LENGTH][2*i+1] = PlayBuf[frame_num%BUF_LENGTH][2*i];
  	  	  t+=2;
  	  	  if(t>=sizeof(wav_ex)) t=0;
  	  }
  	  frame_num++;*/
  	  play();
}

void HAL_DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
	/*for(i=0;i<FRAME_SIZE;i++) {
	  PlayBuf[frame_num%BUF_LENGTH][2*i] = (int16_t)((uint16_t)wav_ex[t+1]<<8 | wav_ex[t])*(int32_t)micr_gain/100;
	  PlayBuf[frame_num%BUF_LENGTH][2*i+1] = PlayBuf[frame_num%BUF_LENGTH][2*i];
	  t+=2;
	  if(t>=sizeof(wav_ex)) t=0;
  }
  frame_num++;*/
  play();
  DmaRecBuffCplt = 1;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	static uint8_t i = 0;
		static uint8_t cur_num = 0;
		static uint8_t cnt = 0;
		if(HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0)) {

		  if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {

			  cur_num = RxData[0] & 0x0F;
			  cnt = RxData[0] >> 4;
			  if(cur_num==cnt && frame_ready==0) {

				  if(RxData[1]==device_id) {
					  to_id = RxHeader.StdId;
					  point_to_point_tmr = 3000;
					  for(i=0;i<RxHeader.DLC-2;i++) can_frame_id[RxHeader.StdId][(cur_num-1)*7+i]=RxData[i+2];
					  can_pckt_length = (cnt-1)*7+RxHeader.DLC-2;
					  for(i=0;i<can_pckt_length;i++) can_frame[i]=can_frame_id[RxHeader.StdId][i];
					  frame_ready=1;
				  }else if(RxData[1]==0xFF) {
					  for(i=0;i<RxHeader.DLC-2;i++) can_frame_id[RxHeader.StdId][(cur_num-1)*7+i]=RxData[i+2];
					  can_pckt_length = (cnt-1)*7+RxHeader.DLC-2;
					  for(i=0;i<can_pckt_length;i++) can_frame[i]=can_frame_id[RxHeader.StdId][i];
					  to_id = 0xFF;
					  frame_ready=1;
				  }
			  }else {
				  for(i=0;i<7;i++) can_frame_id[RxHeader.StdId][(cur_num-1)*7+i]=RxData[i+1];
			  }
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
