/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     

#include <speex/speex.h>
#include "wav_example.h"
#include "cs43l22.h"
#include "stm32l476g_discovery.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define FRAME_SIZE              160
#define ENCODED_FRAME_SIZE      20
#define SaturaLH(N, L, H) (((N)<(L))?(L):(((N)>(H))?(H):(N)))

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

unsigned char micr_gain = 50;

#define BUF_LENGTH	3

int32_t		RecBuf[2][FRAME_SIZE];
int16_t		micr_data[2][FRAME_SIZE];
int16_t		PlayBuf[BUF_LENGTH][FRAME_SIZE*2];
char microphone_encoded_data[2][ENCODED_FRAME_SIZE];
unsigned char encoded_micr_ready_buf_num = 0;
int16_t	audio_stream[FRAME_SIZE];
uint32_t	wav_pos = 0;
uint32_t frame_num = 0;
uint32_t cur_frame = 0;

uint32_t							DmaRecHalfBuffCplt  = 0;
uint32_t    						DmaRecBuffCplt      = 0;
extern AUDIO_DrvTypeDef					*audio_drv;

extern SAI_HandleTypeDef 			hsai_BlockA1;

extern SpeexBits bits;/* Holds bits so they can be read and written by the Speex routines */
extern void *enc_state;
extern void *dec_state;

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId decodeTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

static void play(void) {
	uint8_t buf_num = 0;
	if(frame_num>=1) {
		if(frame_num-cur_frame!=1) {
			HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port,GREEN_LED_Pin);
			cur_frame = frame_num-1;
		}
		buf_num = cur_frame%BUF_LENGTH;
		audio_drv->Play(AUDIO_I2C_ADDRESS, (uint16_t *) &PlayBuf[buf_num][0], FRAME_SIZE*2);
		HAL_SAI_Transmit_DMA(&hsai_BlockA1, (uint8_t *) &PlayBuf[buf_num][0], FRAME_SIZE*2);
		cur_frame++;
    }
}

static void encode_frame(int16_t *input, char *output) {
	speex_bits_reset(&bits);
	speex_encode_int(enc_state,input,&bits);
	speex_bits_write(&bits, output, ENCODED_FRAME_SIZE);
}

static void decode_frame(char *input, int16_t *output) {
	speex_bits_read_from(&bits, input, ENCODED_FRAME_SIZE);
	speex_decode_int(dec_state, &bits, output);
}
   
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartDecodeTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 512);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of decodeTask */
  osThreadDef(decodeTask, StartDecodeTask, osPriorityIdle, 0, 512);
  decodeTaskHandle = osThreadCreate(osThread(decodeTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */

  uint32_t i;
  uint8_t buf_num = 0;
  for(;;)
  {
	  	  if(DmaRecHalfBuffCplt == 1) {
	  		  //for(i=0;i<FRAME_SIZE;i++) {micr_data[0][i] = SaturaLH((RecBuf[0][i] >>8), -32768, 32767)*(int32_t)micr_gain/100;}
	  		  for(i=0;i<FRAME_SIZE;i++) {
	  			  micr_data[0][i] = (int16_t)((uint16_t)wav_ex[wav_pos]<<8 | wav_ex[wav_pos+1])*(int32_t)micr_gain/100;
	  			  wav_pos+=2;if(wav_pos>=EX_SIZE) wav_pos=0;
	  		  }

	  		  encode_frame(&micr_data[0][0],&microphone_encoded_data[0][0]);
	  		  encoded_micr_ready_buf_num = 1;

	  		  DmaRecHalfBuffCplt  = 0;
	  		  play();

	  	  }
	  	  if(DmaRecBuffCplt == 1)
	  	  {
	  		  //for(i=0;i<FRAME_SIZE;i++) {micr_data[1][i] = SaturaLH((RecBuf[1][i] >>8), -32768, 32767)*(int32_t)micr_gain/100;}
	  		  for(i=0;i<FRAME_SIZE;i++) {
	  			  micr_data[1][i] = (int16_t)((uint16_t)wav_ex[wav_pos]<<8 | wav_ex[wav_pos+1])*(int32_t)micr_gain/100;
	  			  wav_pos+=2;if(wav_pos>=EX_SIZE) wav_pos=0;
	  		  }

	  		  encode_frame(&micr_data[1][0],&microphone_encoded_data[1][0]);
	  		  encoded_micr_ready_buf_num = 2;

	  		  DmaRecBuffCplt  = 0;
	  		  play();
	  	  }
	  	  osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartDecodeTask */
/**
* @brief Function implementing the decodeTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDecodeTask */
void StartDecodeTask(void const * argument)
{
  /* USER CODE BEGIN StartDecodeTask */
  /* Infinite loop */
	uint32_t i;
	  uint8_t buf_num = 0;
  for(;;)
  {
	  if(encoded_micr_ready_buf_num) {
		  decode_frame(&microphone_encoded_data[encoded_micr_ready_buf_num-1][0],&audio_stream[0]);
		  encoded_micr_ready_buf_num=0;
		  buf_num = frame_num%BUF_LENGTH;
		  for(i=0;i<FRAME_SIZE;i++) {
			  PlayBuf[buf_num][2*(FRAME_SIZE-1-i)] = audio_stream[FRAME_SIZE-1-i];
			  PlayBuf[buf_num][2*(FRAME_SIZE-1-i)+1] = PlayBuf[buf_num][2*(FRAME_SIZE-1-i)];
		  }
		  frame_num++;

	  }
    osDelay(1);
  }
  /* USER CODE END StartDecodeTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void HAL_DFSDM_FilterRegConvHalfCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
  DmaRecHalfBuffCplt = 1;
}

void HAL_DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
  DmaRecBuffCplt = 1;
  //HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port,GREEN_LED_Pin);
}
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
