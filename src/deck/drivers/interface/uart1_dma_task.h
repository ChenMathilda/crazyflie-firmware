#ifndef __UART1_DMA_TASK_H
#define __UART1_DMA_TASK_H

#ifdef __cplusplus
 extern "C" {
#endif 

#include "stm32f4xx.h"


#ifdef __cplusplus
}
#endif



void uartRxTask(void *param);
bool uwbGetUartInfo(float* steer, float* coll, float* sign);
#endif