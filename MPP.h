/*
 * MPP.h
 *
 *  Created on: Nov 3, 2024
 *      Author: franc
 */

#ifndef L_MPP_MPP_H_
#define L_MPP_MPP_H_

#include "main.h"
#include <stdbool.h>
#include "MPP_CFG.h"

#include "step_generator.h"
#ifdef MPP_CFG_USE_TMC
#include "TMC2209_UART.h"
#endif

#define MPP_2PI 6.283185307179586
#define MPP_PI 3.14159265359

typedef STEP_GEN_ControlMode_t MPP_ControlMode_t;

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
#ifdef MPP_CFG_USE_TMC
	UART_HandleTypeDef *huart;
#endif
}
MPP_Descriptor_t;

typedef struct
{
#ifdef MPP_CFG_USE_TMC
	uint8_t node_addr;
#endif

	uint32_t steps_per_rev;
	uint32_t microsteps_count;

	uint32_t enable_watchdog_timeout;
	bool enable_watchdog_activation_state;

	float acceleration; // in rad/s²
	uint32_t speed_update_period; // in ms

	double approach_distance; // in rad
	float approach_speed; // in rad/s
	float run_speed; // in rad/s

	MPP_ControlMode_t default_control_mode;
}
MPP_Config_t;

typedef struct
{
	uint32_t last_enable_watchdog_ping;
	bool enable_state;

	STEP_GEN_Descriptor_t sg_desc;
	STEP_GEN_Instance_t sg_inst;
#ifdef MPP_CFG_USE_TMC
	TMC2209_UART_Descriptor_t tmc_desc;
	TMC2209_UART_Instance_t tmc_inst;
#endif
}
MPP_Data_t;

typedef struct
{
	MPP_Descriptor_t *descriptor;
	MPP_Config_t config;
	MPP_Data_t data;
}
MPP_Instance_t;

void MPP_Init(MPP_Instance_t *instance);

void MPP_Process(MPP_Instance_t *instance);
void MPP_kHz_Process(MPP_Instance_t *instance);

bool MPP_OnPwmPulseFinishedProcess(MPP_Instance_t *instance, TIM_HandleTypeDef *htim);

void MPP_HardStop(MPP_Instance_t *instance);

void MPP_Enable(MPP_Instance_t *instance);
void MPP_Disable(MPP_Instance_t *instance);
bool MPP_GetEnableState(MPP_Instance_t *instance);

void MPP_EnableWatchdogActivate(MPP_Instance_t *instance);
void MPP_EnableWatchdogDeactivate(MPP_Instance_t *instance);
bool MPP_GetEnableWatchdogActivationState(MPP_Instance_t *instance);

bool MPP_SetControlMode(MPP_Instance_t *instance, MPP_ControlMode_t new_mode);
MPP_ControlMode_t MPP_GetControlMode(MPP_Instance_t *instance);

void MPP_SetTargetSpeed(MPP_Instance_t *instance, float new_speed);
float MPP_GetTargetSpeed(MPP_Instance_t *instance);

void MPP_SetTargetPosition(MPP_Instance_t *instance, double new_position);
double MPP_GetTargetPosition(MPP_Instance_t *instance);

void MPP_SetCurrentSpeed(MPP_Instance_t *instance, float new_speed);
float MPP_GetCurrentSpeed(MPP_Instance_t *instance);

void MPP_SetCurrentPosition(MPP_Instance_t *instance, double new_position);
double MPP_GetCurrentPosition(MPP_Instance_t *instance);

bool MPP_SetStepPerRevolution(MPP_Instance_t *instance, uint32_t new_spr);
uint32_t MPP_GetStepPerRevolution(MPP_Instance_t *instance);

bool MPP_SetMicrosteps(MPP_Instance_t *instance, uint32_t new_ustep);
uint32_t MPP_GetMicrosteps(MPP_Instance_t *instance);

void MPP_SetAcceleration(MPP_Instance_t *instance, float new_acc);
float MPP_GetAcceleration(MPP_Instance_t *instance);

void MPP_SetSpeedUpdatePeriod(MPP_Instance_t *instance, uint32_t new_period);
uint32_t MPP_GetSpeedUpdatePeriod(MPP_Instance_t *instance);

#endif /* L_MPP_MPP_H_ */
