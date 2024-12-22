/*
 * MPP.c
 *
 *  Created on: Nov 3, 2024
 *      Author: franc
 */
#include "MPP.h"
#include "MPP_CFG.h"

#define MPP_DESC instance->descriptor
#define MPP_CFG instance->config
#define MPP_DATA instance->data

#define MPP_SG_INST MPP_DATA.sg_inst
#define MPP_SG_PTR &(MPP_SG_INST)
#define MPP_SG_DESC MPP_DATA.sg_desc
#define MPP_SG_CFG MPP_SG_INST.config

#ifdef MPP_CFG_USE_TMC
#define MPP_TMC_INST MPP_DATA.tmc_inst
#define MPP_TMC_PTR &(MPP_TMC_INST)
#define MPP_TMC_DESC MPP_DATA.tmc_desc
#endif

//#define MPP_2PI 6.283185307179586


static bool MPP_ComputeEnableState(MPP_Instance_t *instance);
static int32_t MPP_RadPerSecToHz(MPP_Instance_t *instance, float rad_per_sec);
static float MPP_HzToRadPerSec(MPP_Instance_t *instance, int32_t hz);
static int64_t MPP_RadToSteps(MPP_Instance_t *instance, double rad);
static double MPP_StepsToRad(MPP_Instance_t *instance, int64_t steps);
static void MPP_UpdateStepGenAcc(MPP_Instance_t *instance);
static void MPP_WatchdogProcess(MPP_Instance_t *instance);
static void MPP_WriteEnablePin(MPP_Instance_t *instance);


static bool MPP_ComputeEnableState(MPP_Instance_t *instance)
{
	uint32_t time = HAL_GetTick();

	if ( ! MPP_DATA.enable_state )
		return 0; // if not enabled, return 0

	if ( MPP_CFG.enable_watchdog_activation_state )
	{
		// if enabled, and watch dog is active, return watch dog state
		return (time <= (MPP_DATA.last_enable_watchdog_ping + MPP_CFG.enable_watchdog_timeout) );
	}
	// else, it is enabled
	return 1;
}

static int32_t MPP_RadPerSecToHz(MPP_Instance_t *instance, float rad_per_sec)
{
	float tmp;
	tmp = rad_per_sec;
	tmp *= (float)(MPP_CFG.steps_per_rev * MPP_CFG.microsteps_count); // micro-step per turn
	tmp /= MPP_2PI; // rad per turn
	return (int32_t)tmp;
}

static float MPP_HzToRadPerSec(MPP_Instance_t *instance, int32_t hz)
{
	float tmp;
	tmp = hz;
	tmp *= MPP_2PI;
	tmp /= (float)(MPP_CFG.steps_per_rev * MPP_CFG.microsteps_count);
	return tmp;
}

static int64_t MPP_RadToSteps(MPP_Instance_t *instance, double rad)
{
	double tmp;
	tmp = rad;
	tmp *= (double)(MPP_CFG.steps_per_rev * MPP_CFG.microsteps_count);
	tmp /= MPP_2PI;
	return (int64_t)tmp;
}

static double MPP_StepsToRad(MPP_Instance_t *instance, int64_t steps)
{
	double tmp;
	tmp = steps;
	tmp *= MPP_2PI;
	tmp /= (double)(MPP_CFG.steps_per_rev * MPP_CFG.microsteps_count);
	return tmp;
}

static void MPP_UpdateStepGenAcc(MPP_Instance_t *instance)
{
	uint32_t acc;
	acc = (uint32_t)MPP_RadPerSecToHz(instance, MPP_CFG.acceleration);
	STEP_GEN_SetAcceleration(MPP_SG_PTR, acc);
}

static void MPP_WatchdogProcess(MPP_Instance_t *instance)
{
	MPP_DATA.enable_state = MPP_ComputeEnableState(instance);
}

static void MPP_WriteEnablePin(MPP_Instance_t *instance)
{
	HAL_GPIO_WritePin(MPP_DESC->en_port, MPP_DESC->en_pin, !MPP_DATA.enable_state);
}


void MPP_Init(MPP_Instance_t *instance)
{
	MPP_SG_DESC.dir_pin = MPP_DESC->dir_pin;
	MPP_SG_DESC.dir_port = MPP_DESC->dir_port;

	MPP_SG_DESC.htim = MPP_DESC->htim;
	MPP_SG_DESC.tim_ch = MPP_DESC-> tim_ch;
	MPP_SG_DESC.tim_peripheral_clk = MPP_DESC->tim_peripheral_clk;
	MPP_SG_DESC.tim_max_value = MPP_DESC->tim_max_value;

	MPP_SG_INST.descriptor = &(MPP_SG_DESC);

#ifdef MPP_CFG_USE_TMC
	MPP_TMC_INST.config.node_addr = MPP_CFG.node_addr;

	MPP_TMC_DESC.huart = MPP_DESC->huart;

	MPP_TMC_INST.descriptor = &(MPP_TMC_DESC);
#endif

	MPP_DATA.enable_state = 0;
	MPP_DATA.last_enable_watchdog_ping = 0;

	MPP_SG_CFG.run_speed = MPP_RadPerSecToHz(instance, MPP_CFG.run_speed);
	MPP_SG_CFG.approach_speed = MPP_RadPerSecToHz(instance, MPP_CFG.approach_speed);
	MPP_SG_CFG.approach_distance = MPP_RadToSteps(instance, MPP_CFG.approach_distance);
	MPP_SG_CFG.cycle_period = MPP_CFG_STEP_GEN_CYCLE_PERIOD;
	MPP_SG_CFG.default_control_mode = (STEP_GEN_ControlMode_t)MPP_CFG.default_control_mode;
	MPP_UpdateStepGenAcc(instance);

	STEP_GEN_Init(MPP_SG_PTR);

#ifdef MPP_CFG_USE_TMC
	TMC2209_UART_Init(MPP_TMC_PTR);
#endif

}

void MPP_Process(MPP_Instance_t *instance)
{
	MPP_WatchdogProcess(instance);
	MPP_WriteEnablePin(instance);
#ifdef MPP_CFG_USE_TMC
	TMC2209_UART_Process(MPP_TMC_PTR);
#endif
}

void MPP_kHz_Process(MPP_Instance_t *instance)
{
	STEP_GEN_Process(MPP_SG_PTR);
}

bool MPP_OnPwmPulseFinishedProcess(MPP_Instance_t *instance, TIM_HandleTypeDef *htim)
{
	return STEP_GEN_OnStepProcess(MPP_SG_PTR, htim);
}

void MPP_HardStop(MPP_Instance_t *instance)
{
	STEP_GEN_HardStop(MPP_SG_PTR);
}

void MPP_Enable(MPP_Instance_t *instance)
{
	MPP_DATA.enable_state = 1;
	MPP_DATA.last_enable_watchdog_ping = HAL_GetTick();
}

void MPP_Disable(MPP_Instance_t *instance)
{
	MPP_DATA.enable_state = 0;
}

bool MPP_GetEnableState(MPP_Instance_t *instance)
{
	return MPP_ComputeEnableState(instance);
}

void MPP_EnableWatchdogActivate(MPP_Instance_t *instance)
{
	MPP_CFG.enable_watchdog_activation_state = 1;
}

void MPP_EnableWatchdogDeactivate(MPP_Instance_t *instance)
{
	MPP_CFG.enable_watchdog_activation_state = 0;
}

bool MPP_GetEnableWatchdogActivationState(MPP_Instance_t *instance)
{
	return MPP_CFG.enable_watchdog_activation_state;
}

bool MPP_SetControlMode(MPP_Instance_t *instance, MPP_ControlMode_t new_mode)
{
	return STEP_GEN_SetControlMode(MPP_SG_PTR, new_mode);
}

MPP_ControlMode_t MPP_GetControlMode(MPP_Instance_t *instance)
{
	return STEP_GEN_GetControlMode(MPP_SG_PTR);
}

void MPP_SetTargetSpeed(MPP_Instance_t *instance, float new_speed)
{
	int32_t speed;
	speed = MPP_RadPerSecToHz(instance, new_speed);
	STEP_GEN_SetTargetSpeed(MPP_SG_PTR, speed);
}

float MPP_GetTargetSpeed(MPP_Instance_t *instance)
{
	int32_t speed_hz;
	speed_hz = STEP_GEN_GetTargetSpeed(MPP_SG_PTR);
	return MPP_HzToRadPerSec(instance, speed_hz);
}

void MPP_SetTargetPosition(MPP_Instance_t *instance, double new_position)
{
	int64_t position_step;
	position_step = MPP_RadToSteps(instance, new_position);
	STEP_GEN_SetTargetPosition(MPP_SG_PTR, position_step);
}

double MPP_GetTargetPosition(MPP_Instance_t *instance)
{
	int64_t position_step;
	position_step = STEP_GEN_GetTargetPosition(MPP_SG_PTR);
	return MPP_RadToSteps(instance, position_step);
}

void MPP_SetCurrentSpeed(MPP_Instance_t *instance, float new_speed)
{
	int32_t speed;
	speed = MPP_RadPerSecToHz(instance, new_speed);
	STEP_GEN_ForceCurrentSpeed(MPP_SG_PTR, speed);
}

float MPP_GetCurrentSpeed(MPP_Instance_t *instance)
{
	int32_t speed_hz;
	speed_hz = STEP_GEN_GetCurrentSpeed(MPP_SG_PTR);
	return MPP_HzToRadPerSec(instance, speed_hz);
}

void MPP_SetCurrentPosition(MPP_Instance_t *instance, double new_position)
{
	int64_t pos;
	pos = MPP_RadToSteps(instance, new_position);
	STEP_GEN_SetCurrentPosition(MPP_SG_PTR, pos);
}

double MPP_GetCurrentPosition(MPP_Instance_t *instance)
{
	int64_t pos;
	pos = STEP_GEN_GetCurrentPosition(MPP_SG_PTR);
	return MPP_StepsToRad(instance, pos);
}

bool MPP_SetStepPerRevolution(MPP_Instance_t *instance, uint32_t new_spr)
{
	if ( MPP_GetEnableState(instance) ) return 0; // refused if not disabled
	MPP_CFG.steps_per_rev = new_spr;
	MPP_UpdateStepGenAcc(instance);
	return 1;
}

uint32_t MPP_GetStepPerRevolution(MPP_Instance_t *instance)
{
	return MPP_CFG.steps_per_rev;
}

bool MPP_SetMicrosteps(MPP_Instance_t *instance, uint32_t new_ustep)
{
	if ( MPP_GetEnableState(instance) ) return 0; // refused if not disabled
	MPP_CFG.microsteps_count = new_ustep;
	MPP_UpdateStepGenAcc(instance);
	return 1;
}

uint32_t MPP_GetMicrosteps(MPP_Instance_t *instance)
{
	return MPP_CFG.microsteps_count;
}

void MPP_SetAcceleration(MPP_Instance_t *instance, float new_acc)
{
	MPP_CFG.acceleration = new_acc;
	MPP_UpdateStepGenAcc(instance);
}

float MPP_GetAcceleration(MPP_Instance_t *instance)
{
	return MPP_CFG.acceleration;
}

void MPP_SetSpeedUpdatePeriod(MPP_Instance_t *instance, uint32_t new_period)
{
	STEP_GEN_SetCyclePeriod(MPP_SG_PTR, new_period);
}

uint32_t MPP_GetSpeedUpdatePeriod(MPP_Instance_t *instance)
{
	return STEP_GEN_GetCyclePeriod(MPP_SG_PTR);
}

void MPP_SetApproachSpeed(MPP_Instance_t *instance, float new_speed)
{
	int32_t speed;
	speed = MPP_RadPerSecToHz(instance, new_speed);
	STEP_GEN_SetApproachSpeed(MPP_SG_PTR, speed);
}

float MPP_GetApproachSpeed(MPP_Instance_t *instance)
{
	int32_t speed;
	speed = STEP_GEN_GetApproachSpeed(MPP_SG_PTR);
	return MPP_HzToRadPerSec(instance, speed);
}

void MPP_SetApproachDistance(MPP_Instance_t *instance, double new_distance)
{
	int64_t distance;
	distance = MPP_RadToSteps(instance, new_distance);
	STEP_GEN_SetApproachDistance(MPP_SG_PTR, distance);
}

double MPP_GetApproachDistance(MPP_Instance_t *instance)
{
	int64_t distance;
	distance = STEP_GEN_GetApproachDistance(MPP_SG_PTR);
	return MPP_StepsToRad(instance, distance);
}

void MPP_SetRunSpeed(MPP_Instance_t *instance, float new_speed)
{
	int32_t speed;
	speed = MPP_RadPerSecToHz(instance, new_speed);
	STEP_GEN_SetRunSpeed(MPP_SG_PTR, speed);
}

float MPP_GetRunSpeed(MPP_Instance_t *instance)
{
	int32_t speed;
	speed = STEP_GEN_GetRunSpeed(MPP_SG_PTR);
	return MPP_HzToRadPerSec(instance, speed);
}
