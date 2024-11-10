/*
 * M_MPP.h
 *
 *  Created on: Nov 3, 2024
 *      Author: franc
 */

#ifndef L_MPP_M_MPP_H_
#define L_MPP_M_MPP_H_

#include "main.h"
#include <stdbool.h>

typedef struct
{
	TIM_HandleTypeDef *htim;
	uint32_t tim_ch;
	uint32_t tim_peripheral_clk; // Hz
	uint32_t tim_max_value; // 65536 for 16 bits etc...

	GPIO_TypeDef *dir_port;
	uint32_t dir_pin;

	GPIO_TypeDef *en_port;
	uint32_t en_pin;

	UART_HandleTypeDef *huart;
}
M_MPP_Descriptor_t;

typedef struct
{
	uint8_t node_addr;

	uint32_t steps_per_rev;
	uint32_t microsteps_count;

	uint32_t enable_watchdog_timeout;
	bool enable_watchdog_activation_state;

	float acceleration; // in rad/s²
	uint32_t speed_update_period; // in ms
}
M_MPP_Config_t;

typedef struct
{
	uint32_t last_enable_watchdog_ping;
	bool enable_state;
}
M_MPP_Data_t;

typedef struct
{
	M_MPP_Descriptor_t *descriptor;
	M_MPP_Config_t config;
	M_MPP_Data_t data;
}
M_MPP_Instance_t;

void MPP_Init(M_MPP_Instance_t *instance);

void MPP_Process(M_MPP_Instance_t *instance);

bool MPP_OnPwmPulseFinishedProcess(M_MPP_Instance_t *instance, TIM_HandleTypeDef *htim);

void MPP_Enable(M_MPP_Instance_t *instance);
void MPP_Disable(M_MPP_Instance_t *instance);
bool MPP_GetEnableState(M_MPP_Instance_t *instance);

void MPP_EnableWatchdogActivate(M_MPP_Instance_t *instance);
void MPP_EnableWatchdogDeactivate(M_MPP_Instance_t *instance);
bool MPP_GetEnableWatchdogActivationState(M_MPP_Instance_t *instance);

void MPP_SetTargetSpeed(M_MPP_Instance_t *instance, float new_speed);
float MPP_GetTargetSpeed(M_MPP_Instance_t *instance);

void MPP_SetCurrentSpeed(M_MPP_Instance_t *instance, float new_speed);
float MPP_GetCurrentSpeed(M_MPP_Instance_t *instance);

void MPP_SetCurrentPosition(M_MPP_Instance_t *instance, double new_position);
double MPP_GetCurrentPosition(M_MPP_Instance_t *instance);

void MPP_SetStepPerRevolution(M_MPP_Instance_t *instance, uint32_t new_spr);
uint32_t MPP_GetStepPerRevolution(M_MPP_Instance_t *instance);

void MPP_SetMicrosteps(M_MPP_Instance_t *instance, uint32_t new_ustep);
uint32_t MPP_GetMicrosteps(M_MPP_Instance_t *instance);

void MPP_SetAcceleration(M_MPP_Instance_t *instance, float new_acc);
float MPP_GetAcceleration(M_MPP_Instance_t *instance);

void MPP_SetSpeedUpdatePeriod(M_MPP_Instance_t *instance, uint32_t new_period);
uint32_t MPP_GetSpeedUpdatePeriod(M_MPP_Instance_t *instance);

#endif /* L_MPP_M_MPP_H_ */
