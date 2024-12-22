/*
 * mpp.h
 *
 *  Created on: Nov 1, 2024
 *      Author: franc
 */

#ifndef STEP_GEN_H_
#define STEP_GEN_H_

#include "main.h"
#include <stdbool.h>

typedef void (*STEP_GEN_Callback)();

typedef enum
{
	STEP_GEN_CTRL_NONE = 0,
	STEP_GEN_CTRL_SPEED,
	STEP_GEN_CTRL_POSITION,
	STEP_GEN_CTRL_MAX
}
STEP_GEN_ControlMode_t;

typedef enum
{
	STEP_GEN_STATE_INIT = 0,
	STEP_GEN_STATE_NEW_TARGET,
	STEP_GEN_STATE_AT_TARGET,
	STEP_GEN_STATE_RUN,
	STEP_GEN_STATE_DECELERATE,
	STEP_GEN_STATE_APPROACH,
	STEP_GEN_STATE_ERROR,
	STEP_GEN_STATE_INACTIVE
}
STEP_GEN_ControllerState_t;

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

	// position control settings
	uint32_t run_speed; // in hz
	uint32_t approach_speed; // in hz
	uint32_t approach_distance; // in steps
	STEP_GEN_ControlMode_t default_control_mode;
}
STEP_GEN_Config_t;

typedef struct
{
	int32_t current_speed; // in Hz
	int32_t target_speed; // in Hz
	int64_t current_position; // in steps
	int64_t target_position;
	uint32_t last_execution;
	uint32_t acceleration_step;
	STEP_GEN_ControlMode_t control_mode;
	STEP_GEN_ControllerState_t controller_state;
	int8_t target_dir;
	int8_t speed_dir;
	int64_t stop_distance;
	int64_t stop_position;
	STEP_GEN_Callback movement_finished_callback;
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
bool STEP_GEN_OnStepProcess(STEP_GEN_Instance_t *instance, TIM_HandleTypeDef *htim);

bool STEP_GEN_SetControlMode(STEP_GEN_Instance_t *instance, STEP_GEN_ControlMode_t new_mode);
STEP_GEN_ControlMode_t STEP_GEN_GetControlMode(STEP_GEN_Instance_t *instance);

void STEP_GEN_MovementFinished_SetCallback(STEP_GEN_Instance_t *instance, STEP_GEN_Callback callback);

void STEP_GEN_SetTargetSpeed			(STEP_GEN_Instance_t *instance, int32_t 	new_target_speed		);
void STEP_GEN_SetCurrentPosition		(STEP_GEN_Instance_t *instance, int64_t 	new_current_position	);
void STEP_GEN_SetTargetPosition			(STEP_GEN_Instance_t *instance, int64_t 	new_target_position		);
void STEP_GEN_SetAcceleration			(STEP_GEN_Instance_t *instance, uint32_t 	new_acceleration		);
void STEP_GEN_SetCyclePeriod			(STEP_GEN_Instance_t *instance, uint32_t 	new_cycle_period		);
void STEP_GEN_SetApproachDistance		(STEP_GEN_Instance_t *instance, int64_t     new_distance			);
void STEP_GEN_SetApproachSpeed			(STEP_GEN_Instance_t *instance, int32_t 	new_speed				);
void STEP_GEN_SetRunSpeed				(STEP_GEN_Instance_t *instance, int32_t		new_speed				);

void STEP_GEN_ForceCurrentSpeed			(STEP_GEN_Instance_t *instance, int32_t 	new_current_speed		);
void STEP_GEN_HardStop					(STEP_GEN_Instance_t *instance);

int32_t 	STEP_GEN_GetTargetSpeed		(STEP_GEN_Instance_t *instance);
int32_t 	STEP_GEN_GetCurrentSpeed	(STEP_GEN_Instance_t *instance);
int64_t 	STEP_GEN_GetCurrentPosition	(STEP_GEN_Instance_t *instance);
int64_t 	STEP_GEN_GetTargetPosition	(STEP_GEN_Instance_t *instance);
uint32_t	STEP_GEN_GetAcceleration	(STEP_GEN_Instance_t *instance);
uint32_t 	STEP_GEN_GetCyclePeriod		(STEP_GEN_Instance_t *instance);
int64_t 	STEP_GEN_GetApproachDistance(STEP_GEN_Instance_t *instance);
int32_t 	STEP_GEN_GetApproachSpeed	(STEP_GEN_Instance_t *instance);
int32_t 	STEP_GEN_GetRunSpeed		(STEP_GEN_Instance_t *instance);

#endif /* STEP_GEN_H_ */
