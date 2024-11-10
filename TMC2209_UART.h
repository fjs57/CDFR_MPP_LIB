/*
 * TMC2209_UART.h
 *
 *  Created on: Nov 2, 2024
 *      Author: franc
 */

#ifndef L_MPP_TMC2209_UART_H_
#define L_MPP_TMC2209_UART_H_

#include "main.h"

typedef struct
{
	UART_HandleTypeDef *huart;
}
TMC2209_UART_Descriptor_t;

typedef struct
{
	uint8_t node_addr;
}
TMC2209_UART_Config_t;

typedef struct
{
	HAL_StatusTypeDef error_code;
	uint32_t var;
}
TMC2209_UART_Data_t;

typedef struct
{
	TMC2209_UART_Descriptor_t *descriptor;
	TMC2209_UART_Config_t config;
	TMC2209_UART_Data_t data;
}
TMC2209_UART_Instance_t;

void TMC2209_UART_Init(TMC2209_UART_Instance_t *instance);

void TMC2209_UART_Read_CurrentSpeed(TMC2209_UART_Instance_t *instance);
void TMC2209_UART_Process(TMC2209_UART_Instance_t *instance);

#endif /* L_MPP_TMC2209_UART_H_ */
