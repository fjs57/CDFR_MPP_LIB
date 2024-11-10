/*
 * mpp.h
 *
 *  Created on: Nov 1, 2024
 *      Author: franc
 */

#ifndef STEP_GEN_H_
#define STEP_GEN_H_

#include "main.h"

typedef struct
{
	TIM_HandleTypeDef *htim;
	uint32_t tim_ch;
	uint32_t tim_peripheral_clk; // type RCC_PERIPHCLK_TIMXX with XX the id of timer
	GPIO_TypeDef *dir_port;
	uint32_t dir_pin;
	uint32_t tim_max_value;
}
STEP_GEN_Descriptor_t;

typedef struct
{
	uint32_t cycle_period; // in ms
	uint32_t frequency_acceleration; // in hz/s
}
STEP_GEN_Config_t;

typedef struct
{
	int32_t current_speed; // in Hz
	int32_t target_speed; // in Hz
	int64_t current_position; // in steps
	uint32_t last_execution;
	uint32_t acceleration_step;
}
STEP_GEN_Data_t;

typedef struct
{
	STEP_GEN_Descriptor_t *descriptor;
	STEP_GEN_Config_t config;
	STEP_GEN_Data_t data;
}
STEP_GEN_Instance_t;


void STEP_GEN_Init(STEP_GEN_Instance_t *instance);
void STEP_GEN_Process(STEP_GEN_Instance_t *instance);
void STEP_GEN_OnStepProcess(STEP_GEN_Instance_t *instance, TIM_HandleTypeDef *htim);

void STEP_GEN_SetTargetSpeed			(STEP_GEN_Instance_t *instance, int32_t 	new_target_speed		);
void STEP_GEN_SetCurrentPosition		(STEP_GEN_Instance_t *instance, int64_t 	new_current_position	);
void STEP_GEN_SetAcceleration			(STEP_GEN_Instance_t *instance, uint32_t 	new_acceleration		);
void STEP_GEN_SetCyclePeriod			(STEP_GEN_Instance_t *instance, uint32_t 	new_cycle_period		);

void STEP_GEN_ForceCurrentSpeed			(STEP_GEN_Instance_t *instance, int32_t 	new_current_speed		);

int32_t 	STEP_GEN_GetTargetSpeed		(STEP_GEN_Instance_t *instance);
int32_t 	STEP_GEN_GetCurrentSpeed	(STEP_GEN_Instance_t *instance);
int64_t 	STEP_GEN_GetCurrentPosition	(STEP_GEN_Instance_t *instance);
uint32_t	STEP_GEN_GetAcceleration	(STEP_GEN_Instance_t *instance);
uint32_t 	STEP_GEN_GetCyclePeriod		(STEP_GEN_Instance_t *instance);

#endif /* STEP_GEN_H_ */
