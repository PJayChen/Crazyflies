/**
 *    ||          ____  _ __                           
 * +------+      / __ )(_) /_______________ _____  ___ 
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * uart.c - uart CRTP link and raw access functions
 */
#include <string.h>
#include <stdio.h>

/*ST includes */
#include "stm32f10x.h"

/*FreeRtos includes*/
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#include "uart.h"
#include "crtp.h"
//#include "cfassert.h"
#include "nvicconf.h"
#include "config.h"

#include "debug.h"

#include "crc.h"

#define UART_DATA_TIMEOUT_MS 1000
#define UART_DATA_TIMEOUT_TICKS (UART_DATA_TIMEOUT_MS / portTICK_RATE_MS)
//#define CRTP_START_BYTE 0x26   //ggm定义
#define CRTP_FIRST_BYTE 0x55
#define CRTP_SECOND_BYTE 0xAA
#define CCR_ENABLE_SET  ((uint32_t)0x00000001)

static bool isInit = false;

xSemaphoreHandle waitUntilSendDone = NULL;
static uint8_t outBuffer[64];
static uint8_t dataIndex;
static uint8_t dataSize;
static uint8_t crcIndex = 0;

//static enum { notSentSecondStart, sentSecondStart} txState;
static xQueueHandle packetDelivery;
static xQueueHandle uartDataDelivery;


void uartRxTask(void *param);

void uartInit(void)
{

  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
	

  /* Enable GPIO and USART clock */
//  RCC_APB2PeriphClockCmd(UART_GPIO_PERIF, ENABLE);
//  RCC_APB1PeriphClockCmd(UART_PERIF, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

  /* Configure USART Rx as input floating */
  GPIO_InitStructure.GPIO_Pin   = UART_GPIO_RX;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
  GPIO_Init(UART_GPIO_PORT, &GPIO_InitStructure);
/* Configure USART Tx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin   = UART_GPIO_TX;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
  GPIO_Init(UART_GPIO_PORT, &GPIO_InitStructure);

  USART_InitStructure.USART_BaudRate            = 115200;
  USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;

  USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits            = USART_StopBits_1;
  USART_InitStructure.USART_Parity              = USART_Parity_No ;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_Init(UART_TYPE, &USART_InitStructure);


  // Configure Tx buffer empty interrupt
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

//  USART_ITConfig(UART_TYPE, USART_IT_RXNE, ENABLE);

  vSemaphoreCreateBinary(waitUntilSendDone);

  xTaskCreate(uartRxTask, "UART-Rx",
              configMINIMAL_STACK_SIZE, NULL, /*priority*/2, NULL);

  packetDelivery = xQueueCreate(2, sizeof(CRTPPacket));
  uartDataDelivery = xQueueCreate(100, sizeof(uint8_t));

  //Enable it
  USART_Cmd(UART_TYPE, ENABLE);
  
  isInit = true;
}

bool uartTest(void)
{
  return isInit;
}

void uartRxTask(void *param)
{
  enum {waitForFirstStart, waitForSecondStart,
        waitForPort, waitForSize, waitForData, waitForCRC } rxState;
	portTickType lastWakeTime2;
	lastWakeTime2 = xTaskGetTickCount ();
  uint8_t c;
  uint8_t dataIndex1 = 0;
  uint8_t crc = 0;
  CRTPPacket p;
  rxState = waitForFirstStart;
  uint8_t counter = 0;
  while(1)
  {
    
		p.header = 0x21;
		p.data[0]=0x00;
		p.data[1]=0x06;
		xQueueSend(packetDelivery, &p, 0);
		vTaskDelayUntil(&lastWakeTime2, M2T(100));
		/*************************************************************************************
		if (xQueueReceive(uartDataDelivery, &c, UART_DATA_TIMEOUT_TICKS) == pdTRUE)
    {
      //counter++;
      if (counter > 4)
        ledSetRed(1);
			55 AA 04 00 06 60 C6
      switch(rxState)
      {
        case waitForFirstStart:
          rxState = (c == CRTP_FIRST_BYTE) ? waitForSecondStart : waitForFirstStart;
          break;
        case waitForSecondStart:
          rxState = (c == CRTP_SECOND_BYTE) ? waitForSize : waitForFirstStart;
          break;
        case waitForPort:
          p.header = c;
          crc = c;
          rxState = waitForSize;
          break;
        case waitForSize:
          if (c < CRTP_MAX_DATA_SIZE)
          {
            p.size = c;
            dataIndex1 = 0;
            rxState =waitForData;
          }
          else
          {
            rxState = waitForFirstStart;
          }
          break;
        case waitForData:
          p.data[dataIndex1] = c;
          dataIndex1++;
          if (dataIndex1 == p.size)
          {
						///////////////////////////////////
						if(p.data[1]==0x06)
						{
							p.header = 0x21;
							xQueueSend(packetDelivery, &p, 0);
							rxState = waitForFirstStart;
						}
						///////////////////////////////////
            rxState = waitForFirstStart;
          }
          break;
        default:
//          ASSERT(0);
          break;
					
      }
			
    }
    else
    {
      // Timeout
      rxState = waitForFirstStart;
    }
		*****************************************************************/
  }
}

static int uartReceiveCRTPPacket(CRTPPacket *p)
{
  if (xQueueReceive(packetDelivery, p, portMAX_DELAY) == pdTRUE)
  {
    return 0;
  }

  return -1;
}

static portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
static uint8_t rxDataInterrupt;

void uartIsr(void)
{
	if (USART_GetITStatus(UART_TYPE, USART_IT_TXE))
  {
    if (dataIndex < dataSize)
    {
      USART_SendData(UART_TYPE, outBuffer[dataIndex] & 0xFF);
      dataIndex++;
			/**********************************************************************************
      if (dataIndex < dataSize - 1 && dataIndex > 1)
      {
        outBuffer[crcIndex] = (outBuffer[crcIndex] + outBuffer[dataIndex]) % 0xFF;
      }
			***********************************************************************************/
    }
    else
    {
      USART_ITConfig(UART_TYPE, USART_IT_TXE, DISABLE);
      xHigherPriorityTaskWoken = pdFALSE;
      xSemaphoreGiveFromISR(waitUntilSendDone, &xHigherPriorityTaskWoken);
    }
  }
  USART_ClearITPendingBit(UART_TYPE, USART_IT_TXE);
	/**********************************************
  if (USART_GetITStatus(UART_TYPE, USART_IT_RXNE))
  {
		rxDataInterrupt = USART_ReceiveData(UART_TYPE) & 0xFF;
    xQueueSendFromISR(uartDataDelivery, &rxDataInterrupt, &xHigherPriorityTaskWoken);
  }
	********************************************/
//	if(xHigherPriorityTaskWoken)
//	portYIELD();
}

static int uartSendCRTPPacket(CRTPPacket *p)
{
  uint16_t crc;
	xSemaphoreTake(waitUntilSendDone, portMAX_DELAY);
	uint8_t buf[] = {0x00,0x01};
//	uint8_t buf1[] = {0x34,0x1E,0x32,0x3F,0xD5,0xD7,0xDF,0xBB,0x8A,0xF9,0x22,0x3B,0x2B,0xDD,0x37,0x3F};
	outBuffer[0] = CRTP_FIRST_BYTE;
  outBuffer[1] = CRTP_SECOND_BYTE;
  outBuffer[2] = p->header;
	/////////////////////////////////////////
	outBuffer[3] = 0x00;
	outBuffer[4] = 0x01;
	/////////////////////////////////////////
//  outBuffer[3] = p->size;
	////////////////////////////////////////
	memcpy(&outBuffer[5], p->data, 16);
//	memcpy(&outBuffer[5], buf1, 16);
	////////////////////////////////////////
//  memcpy(&outBuffer[4], p->data, p->size);
  dataIndex = 1;
	dataSize = 23;
//  txState = notSentSecondStart;
//  dataSize = p->size + 5;
//  crcIndex = dataSize - 1;
//	crcIndex = 19;
//	outBuffer[19] = 0;
//  outBuffer[crcIndex] = 0;
	crc = math_crc16(0,buf,sizeof(buf));
	crc = math_crc16(crc,&outBuffer[5],16);
	outBuffer[21]=((uint8_t *)&crc)[1];
	outBuffer[22]=((uint8_t *)&crc)[0];
	outBuffer[23]=0;
  USART_SendData(UART_TYPE, outBuffer[0] & 0xFF);
  USART_ITConfig(UART_TYPE, USART_IT_TXE, ENABLE);
  return 0;
}

static int uartSetEnable(bool enable)
{
  return 0;
}

static struct crtpLinkOperations uartOp =
{
  .setEnable         = uartSetEnable,
  .sendPacket        = uartSendCRTPPacket,
  .receivePacket     = uartReceiveCRTPPacket,
};

struct crtpLinkOperations * uartGetLink()
{
  return &uartOp;
}

void uartSendData(uint32_t size, uint8_t* data)
{
#ifdef UART_OUTPUT_RAW_DATA_ONLY
  uint32_t i;

  for(i = 0; i < size; i++)
  {
    while (!(UART_TYPE->SR & USART_FLAG_TXE));
    UART_TYPE->DR = (data[i] & 0xFF);
  }
#endif
}

int uartPutchar(int ch)
{
    uartSendData(1, (uint8_t *)&ch);
    
    return (unsigned char)ch;
}

int fputc(int ch, FILE *f)
{
/* 将Printf内容发往串口 */
  USART_SendData(USART1, (unsigned char) ch);
  while (!(USART1->SR & USART_FLAG_TXE));
 
  return (ch);
}

