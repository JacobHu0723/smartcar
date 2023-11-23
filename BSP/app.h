#ifndef __APP_H
#define __APP_H			  	 

#include "stdlib.h"	
#include "stdint.h"

void	ctrdeal(void);
void  sec_dealtask(void);
void deal_u1dat(uint8_t *buf,uint8_t len);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
#endif

