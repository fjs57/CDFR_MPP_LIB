/*
 * M_MPP.c
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

#ifdef MPP_CFG_USE_TMC
#define MPP_TMC_INST MPP_DATA.tmc_inst
#define MPP_TMC_PTR &(MPP_TMC_INST)
#define MPP_TMC_DESC MPP_DATA.tmc_desc
#endif

#define MPP_2PI 6.283185307179586


static bool MPP_ComputeEnableState(M_MPP_Instance_t *instance);
static int32_t MPP_RadPerSecToHz(M_MPP_Instance_t *instance, float rad_per_sec);
static float MPP_HzToRadPerSec(M_MPP_Instance_t *instance, int32_t hz);
static int64_t MPP_RadToSteps(M_MPP_Instance_t *instance, double rad);
static double MPP_StepsToRad(M_MPP_Instance_t *instance, int64_t steps);
static void MPP_UpdateStepGenAcc(M_MPP_Instance_t *instance);
static void MPP_WatchdogProcess(M_MPP_Instance_t *instance);


static bool MPP_ComputeEnableState(M_MPP_Instance_t *instance)
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

static int32_t MPP_RadPerSecToHz(M_MPP_Instance_t *instance, float rad_per_sec)
{
	float tmp;
	tmp = rad_per_sec;
	tmp *= (float)(MPP_CFG.steps_per_rev * MPP_CFG.microsteps_count); // micro-step per turn
	tmp /= MPP_2PI; // rad per turn
	return (int32_t)tmp;
}

static float MPP_HzToRadPerSec(M_MPP_Instance_t *instance, int32_t hz)
{
	float tmp;
	tmp = hz;
	tmp *= MPP_2PI;
	tmp /= (float)(MPP_CFG.steps_per_rev * MPP_CFG.microsteps_count);
	return tmp;
}

static int64_t MPP_RadToSteps(M_MPP_Instance_t *instance, double rad)
{
	double tmp;
	tmp = rad;
	tmp *= (double)(MPP_CFG.steps_per_rev * MPP_CFG.microsteps_count);
	tmp /= MPP_2PI;
	return (int64_t)tmp;
}

static double MPP_StepsToRad(M_MPP_Instance_t *instance, int64_t steps)
{
	double tmp;
	tmp = steps;
	tmp *= MPP_2PI;
	tmp /= (double)(MPP_CFG.steps_per_rev * MPP_CFG.microsteps_count);
	return tmp;
}

static void MPP_UpdateStepGenAcc(M_MPP_Instance_t *instance)
{
	uint32_t acc;
	acc = (uint32_t)MPP_RadPerSecToHz(instance, MPP_CFG.acceleration);
	STEP_GEN_SetAcceleration(MPP_SG_PTR, acc);
}

static void MPP_WatchdogProcess(M_MPP_Instance_t *instance)
{
	MPP_DATA.enable_state = MPP_ComputeEnableState(instance);
}


void MPP_Init(M_MPP_Instance_t *instance)
{
	MPP_SG_INST.config.cycle_period = MPP_CFG_STEP_GEN_CYCLE_PERIOD;

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

	MPP_UpdateStepGenAcc(instance);

	STEP_GEN_Init(MPP_SG_PTR);

#ifdef MPP_CFG_USE_TMC
	TMC2209_UART_Init(MPP_TMC_PTR);
#endif

}

void MPP_Process(M_MPP_Instance_t *instance)
{
	MPP_WatchdogProcess(instance);
#ifdef MPP_CFG_USE_TMC
	TMC2209_UART_Process(MPP_TMC_PTR);
#endif
}

void MPP_kHz_Process(M_MPP_Instance_t *instance)
{
	STEP_GEN_Process(MPP_SG_PTR);
}

bool MPP_OnPwmPulseFinishedProcess(M_MPP_Instance_t *instance, TIM_HandleTypeDef *htim)
{
	return STEP_GEN_OnStepProcess(MPP_SG_PTR, htim);
}

void MPP_Enable(M_MPP_Instance_t *instance)
{
	MPP_DATA.enable_state = 1;
	MPP_DATA.last_enable_watchdog_ping = HAL_GetTick();
}

void MPP_Disable(M_MPP_Instance_t *instance)
{
	MPP_DATA.enable_state = 0;
}

bool MPP_GetEnableState(M_MPP_Instance_t *instance)
{
	return MPP_ComputeEnableState(instance);
}

void MPP_EnableWatchdogActivate(M_MPP_Instance_t *instance)
{
	MPP_CFG.enable_watchdog_activation_state = 1;
}

void MPP_EnableWatchdogDeactivate(M_MPP_Instance_t *instance)
{
	MPP_CFG.enable_watchdog_activation_state = 0;
}

bool MPP_GetEnableWatchdogActivationState(M_MPP_Instance_t *instance)
{
	return MPP_CFG.enable_watchdog_activation_state;
}


void MPP_SetTargetSpeed(M_MPP_Instance_t *instance, float new_speed)
{
	int32_t speed;
	speed = MPP_RadPerSecToHz(instance, new_speed);
	STEP_GEN_SetTargetSpeed(MPP_SG_PTR, speed);
}

float MPP_GetTargetSpeed(M_MPP_Instance_t *instance)
{
	int32_t speed_hz;
	speed_hz = STEP_GEN_GetTargetSpeed(MPP_SG_PTR);
	return MPP_HzToRadPerSec(instance, speed_hz);
}

void MPP_SetCurrentSpeed(M_MPP_Instance_t *instance, float new_speed)
{
	int32_t speed;
	speed = MPP_RadPerSecToHz(instance, new_speed);
	STEP_GEN_ForceCurrentSpeed(MPP_SG_PTR, speed);
}

float MPP_GetCurrentSpeed(M_MPP_Instance_t *instance)
{
	int32_t speed_hz;
	speed_hz = STEP_GEN_GetCurrentSpeed(MPP_SG_PTR);
	return MPP_HzToRadPerSec(instance, speed_hz);
}

void MPP_SetCurrentPosition(M_MPP_Instance_t *instance, double new_position)
{
	int64_t pos;
	pos = MPP_RadToSteps(instance, new_position);
	STEP_GEN_SetCurrentPosition(MPP_SG_PTR, pos);
}

double MPP_GetCurrentPosition(M_MPP_Instance_t *instance)
{
	int64_t pos;
	pos = STEP_GEN_GetCurrentPosition(MPP_SG_PTR);
	return MPP_StepsToRad(instance, pos);
}

bool MPP_SetStepPerRevolution(M_MPP_Instance_t *instance, uint32_t new_spr)
{
	if ( MPP_GetEnableState(instance) ) return 0; // refused if not disabled
	MPP_CFG.steps_per_rev = new_spr;
	MPP_UpdateStepGenAcc(instance);
	return 1;
}

uint32_t MPP_GetStepPerRevolution(M_MPP_Instance_t *instance)
{
	return MPP_CFG.steps_per_rev;
}

bool MPP_SetMicrosteps(M_MPP_Instance_t *instance, uint32_t new_ustep)
{
	if ( MPP_GetEnableState(instance) ) return 0; // refused if not disabled
	MPP_CFG.microsteps_count = new_ustep;
	MPP_UpdateStepGenAcc(instance);
	return 1;
}

uint32_t MPP_GetMicrosteps(M_MPP_Instance_t *instance)
{
	return MPP_CFG.microsteps_count;
}

void MPP_SetAcceleration(M_MPP_Instance_t *instance, float new_acc)
{
	MPP_CFG.acceleration = new_acc;
	MPP_UpdateStepGenAcc(instance);
}

float MPP_GetAcceleration(M_MPP_Instance_t *instance)
{
	return MPP_CFG.acceleration;
}

void MPP_SetSpeedUpdatePeriod(M_MPP_Instance_t *instance, uint32_t new_period)
{
	STEP_GEN_SetCyclePeriod(MPP_SG_PTR, new_period);
}

uint32_t MPP_GetSpeedUpdatePeriod(M_MPP_Instance_t *instance)
{
	return STEP_GEN_GetCyclePeriod(MPP_SG_PTR);
}

