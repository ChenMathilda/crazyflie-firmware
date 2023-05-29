/*-----------------------------------------------------------------------------
 Copyright (C) 2020-2021 ETH Zurich, Switzerland, University of Bologna, Italy.
 All rights reserved.

 File:    main.c
 Author:  Vlad Niculescu      <vladn@iis.ee.ethz.ch>
 Date:    15.03.2021
-------------------------------------------------------------------------------*/

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"
#include "FreeRTOS.h"
#include "task.h"
#include "debug.h"
#include "log.h"
#include "uart1_dma.h"
#include "uart1_dma_task.h"

#define DEBUG_MODULE "UART"
#include "debug.h"
#define DMASIZE 5 * 2
uint8_t aideckRxDMA[DMASIZE];

volatile uint8_t dma_flag = 0;
uint16_t log_counter = 0;
static uint8_t flag;
typedef struct
{
	float steer;
	float coll;
	float sign;
	bool refresh;
} dma_data_t;
dma_data_t dma_data = {0.0, 0.0, 0.5, false};

#define histSize 5
typedef struct
{
	float steer_ctl[histSize];
	float coll_ctl[histSize];
	float sign_ctl[histSize];
	bool refresh;
	uint8_t index;
} latest3_data_t;
latest3_data_t latest3_data = {{0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, {0.5f, 0.5f, 0.5f, 0.5f, 0.5f}, 0};
float calAveOfhistSize(float *data)
{
	float sum = 0.0;
	for (int i = 0; i < histSize; i++)
		sum += data[i];
	return (sum / histSize);
}
float calLatesthistSizeSignal(float *data)
{
	float sum = 0.0;
	for (int i = 0; i < histSize; i++)
		sum += data[i];

	if (sum < 1)
		return -1.0f;
	else if (sum >= 1 && sum <= 4)
		return 0.0f;
	else
		return 1.0f;
}
static void processHistoryData(float *steerAngle, float *collision, float *signFromation)
{
	latest3_data.steer_ctl[latest3_data.index] = *steerAngle;
	latest3_data.coll_ctl[latest3_data.index] = *collision;
	latest3_data.sign_ctl[latest3_data.index] = *signFromation;
	latest3_data.index = (latest3_data.index + 1) % histSize;
	*steerAngle = calAveOfhistSize(latest3_data.steer_ctl);
	*collision = calAveOfhistSize(latest3_data.coll_ctl);
	*signFromation = calLatesthistSizeSignal(latest3_data.sign_ctl);
}
bool uwbGetUartInfo(float *steer, float *coll, float *sign)
{
	// DEBUG_PRINT("dma_data.refresh:%d\n", dma_data.refresh);
	if (dma_data.refresh == true)
	{
		processHistoryData(&dma_data.steer, &dma_data.coll, &dma_data.sign);
		*steer = dma_data.steer;
		*coll = dma_data.coll;
		*sign = dma_data.sign;
		// DEBUG_PRINT("uwb-steer:%.2f\tcollision:%.2f\tsign: %.2f\n", *steer, *coll, *sign);
		memset(&dma_data.refresh, 0, sizeof(dma_data.refresh));
		return true;
	}
	return false;	
}

static uint32_t uart_tick;
void uartRxTask(void *param)
{
	DEBUG_PRINT("DMA Rx Task! \n");
	USART_DMA_Start(115200, aideckRxDMA, DMASIZE);
	while (1)
	{
		vTaskDelay(M2T(100));
		if (dma_flag == 1 )
		{
			dma_flag = 0; // clear the flag
			for (int i = 0; i < DMASIZE / 2; i++)
			{
				if (aideckRxDMA[i] == 0x05 && aideckRxDMA[i + DMASIZE / 2 - 1] == 0x06)
				{
					dma_data.steer = ((float)aideckRxDMA[i + 1] / 100 - 1);
					dma_data.coll = (float)aideckRxDMA[i + 2] / 100;
					dma_data.sign = (float)aideckRxDMA[i + 3] / 100;
					dma_data.refresh = true;
					memset(aideckRxDMA, 0, DMASIZE * sizeof(uint8_t)); // clear the dma buffer
					// uart_tick = xTaskGetTickCount();
					// DEBUG_PRINT("tick:%d\n",uart_tick);
				}
			}
			
		}
	}
}

void __attribute__((used)) DMA1_Stream1_IRQHandler(void)
{
	DMA_ClearFlag(DMA1_Stream1, UART3_RX_DMA_ALL_FLAGS);
	dma_flag = 1;
}
