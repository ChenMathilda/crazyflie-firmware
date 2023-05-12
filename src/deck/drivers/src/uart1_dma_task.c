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
#define BUFFERSIZE 1
#define DMASIZE 5 * 2
uint8_t aideckRxBuffer[BUFFERSIZE];
uint8_t aideckRxDMA[DMASIZE];

volatile uint8_t dma_flag = 0;
uint8_t log_counter = 0;
static uint8_t flag;

typedef struct
{
	float steer;
	float coll;
	float sign;
	bool refresh;
} control_data_t;
control_data_t control_data = {0.0, 0.0, 0.5, false};

bool uwbGetUartInfo(float *steer, float *coll, float *sign)
{
	if (control_data.refresh == true)
	{
		*steer = control_data.steer;
		*coll = control_data.coll;
		*sign = control_data.sign;
		// DEBUG_PRINT("steer:%.2f \tcollision:%.2f \t  sign: %.2f\n", *steer, *coll, *sign);
		memset(&control_data.steer, 0, sizeof(control_data.steer));
		memset(&control_data.sign, 0.5, sizeof(control_data.sign));
		memset(&control_data.refresh, 0, sizeof(control_data.refresh));
		return (true);
	}
	else
	{
		return (false);
	}
}

void uartRxTask(void *param)
{
	DEBUG_PRINT("DMA Rx Task! \n");
	USART_DMA_Start(115200, aideckRxDMA, DMASIZE);
	while (1)
	{
		vTaskDelay(M2T(100));
		if (dma_flag == 1 /*&& control_data.refresh == false*/)
		{
			dma_flag = 0;								// clear the flag
			for (int i = 0; i < DMASIZE / 2; i++)
			{
				// DEBUG_PRINT("%x,%x\n", aideckRxDMA[i], aideckRxDMA[i + DMASIZE / 2 - 1]);
				if (aideckRxDMA[i] == 0x05 && aideckRxDMA[i + DMASIZE / 2 - 1] == 0x06)
				{
					control_data.steer = ((float)aideckRxDMA[i + 1] / 100 - 1);
					control_data.coll = (float)aideckRxDMA[i + 2] / 100;
					control_data.sign = (float)aideckRxDMA[i + 3] / 100;
					control_data.refresh = true;
					// DEBUG_PRINT("steer:%d\tcollision:%d\tsign:%d\n", aideckRxDMA[i + 1], aideckRxDMA[i + 2], aideckRxDMA[i + 3]);
					// DEBUG_PRINT("steer:%.2f\tcollision:%.2f\tsign: %.2f\n", control_data.steer, control_data.coll, control_data.sign);
					// break;
				}
			}
			memset(aideckRxDMA, 0, DMASIZE * sizeof(uint8_t)); // clear the dma buffer
		}
	}
}

void __attribute__((used)) DMA1_Stream1_IRQHandler(void)
{
	DMA_ClearFlag(DMA1_Stream1, UART3_RX_DMA_ALL_FLAGS);
	dma_flag = 1;
}
