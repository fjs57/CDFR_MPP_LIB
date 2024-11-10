/*
 * mpp.c
 *
 *  Created on: Nov 1, 2024
 *      Author: franc
 */

#include "step_generator.h"

#define STEP_GEN_CONFIG instance->config
#define STEP_GEN_DESCRIPTOR instance->descriptor
#define STEP_GEN_DATA instance->data

#define STEP_GEN_SIGN(var) ((var)>=0)?1:-1
#define STEP_GEN_ABS(var) ((var)>=0?(var):(-1)*(var))
#define STEP_GEN_MIN(A,B) (A<B?A:B)
#define STEP_GEN_MAX(A,B) (A>B?A:B)

static void STEP_GEN_TIM_Start(STEP_GEN_Instance_t *instance);
static void STEP_GEN_TIM_Stop(STEP_GEN_Instance_t *instance);
static void STEP_GEN_TIM_SetFrequency(STEP_GEN_Instance_t *instance);
static void STEP_GEN_GPIO_SetDir(STEP_GEN_Instance_t *instance);
static void STEP_GEN_CountSteps(STEP_GEN_Instance_t *instance);
static void STEP_GEN_ApplyTimings(STEP_GEN_Instance_t *instance, uint32_t prescaler, uint32_t period);
static void STEP_GEN_ComputeNextFrequency(STEP_GEN_Instance_t *instance);
static void STEP_GEN_ComputeAccelerationStep(STEP_GEN_Instance_t *instance);

/**
 * Starts the timer of this instance
 */
static void STEP_GEN_TIM_Start(STEP_GEN_Instance_t *instance)
{
	HAL_TIM_PWM_Start_IT(
			STEP_GEN_DESCRIPTOR->htim,
			STEP_GEN_DESCRIPTOR->tim_ch
	);
}

/**
 * Stops the timer of this instance
 */
static void STEP_GEN_TIM_Stop(STEP_GEN_Instance_t *instance){
	HAL_TIM_PWM_Stop_IT(
			STEP_GEN_DESCRIPTOR->htim,
			STEP_GEN_DESCRIPTOR->tim_ch
	);
}

/**
 * Applies the current frequency to the timer
 */
static void STEP_GEN_TIM_SetFrequency(STEP_GEN_Instance_t *instance)
{
	uint32_t divider, prescaler, period;
	uint8_t i;


	// if the speed is null, stops the timer
	if ( STEP_GEN_DATA.current_speed == 0)
	{
		STEP_GEN_TIM_Stop(instance);
		return;
	}

	// compute the division factor of the timer clock to generate the wanted speed
	divider = STEP_GEN_DESCRIPTOR->tim_peripheral_clk / STEP_GEN_ABS(STEP_GEN_DATA.current_speed);


	// iteratively try to find a pair of prescaler and period to match the division factor without overflowing registers
	// could be reduced to a single computation but not needed yet...
	for ( i = 0; i < 32; i += 4 )
	{
		period = (divider >> i);
		prescaler = divider/period;

		if (period <= STEP_GEN_DESCRIPTOR->tim_max_value)
		{
			break;
		}
	}


	STEP_GEN_ApplyTimings(instance, prescaler, period);
	STEP_GEN_TIM_Start(instance);
}

/**
 * Sets the DIR state according to the current speed sign
 */
static void STEP_GEN_GPIO_SetDir(STEP_GEN_Instance_t *instance)
{
	HAL_GPIO_WritePin(
		STEP_GEN_DESCRIPTOR->dir_port,
		STEP_GEN_DESCRIPTOR->dir_pin,
		(STEP_GEN_DATA.current_speed >= 0)
	);
}

/**
 * incre-/decrements step counter according to current speed sign
 */
static void STEP_GEN_CountSteps(STEP_GEN_Instance_t *instance)
{
	STEP_GEN_DATA.current_position += (STEP_GEN_DATA.current_speed>=0)?1:-1;
}

/**
 * Apply given timings to the intsance's timer
 */
static void STEP_GEN_ApplyTimings(STEP_GEN_Instance_t *instance, uint32_t prescaler, uint32_t period)
{
	__HAL_TIM_SET_PRESCALER(STEP_GEN_DESCRIPTOR->htim, prescaler-1);//minus one as psc and period are 0 based.
	__HAL_TIM_SET_AUTORELOAD(STEP_GEN_DESCRIPTOR->htim, period-1);
	__HAL_TIM_SET_COMPARE(STEP_GEN_DESCRIPTOR->htim, STEP_GEN_DESCRIPTOR->tim_ch, (period>>1));
}

/**
 * compute the frequency according to target and acceleration
 */
static void STEP_GEN_ComputeNextFrequency(STEP_GEN_Instance_t *instance)
{
	int32_t next_speed;
	if ( STEP_GEN_DATA.current_speed == STEP_GEN_DATA.target_speed )
	{
		next_speed = STEP_GEN_DATA.current_speed;
	}
	else
	if ( STEP_GEN_DATA.current_speed < STEP_GEN_DATA.target_speed )
	{
		next_speed = STEP_GEN_MIN(
			STEP_GEN_DATA.current_speed + (int32_t)STEP_GEN_DATA.acceleration_step,
			STEP_GEN_DATA.target_speed
		);
	}
	else
	if ( STEP_GEN_DATA.current_speed > STEP_GEN_DATA.target_speed )
	{
		next_speed = STEP_GEN_MAX(
			STEP_GEN_DATA.current_speed - (int32_t)STEP_GEN_DATA.acceleration_step,
			STEP_GEN_DATA.target_speed
		);
	}

	STEP_GEN_DATA.current_speed = next_speed;
}

/**
 * Compute the speed delta for the speed computation according to period and acceleration parameters
 */
static void STEP_GEN_ComputeAccelerationStep(STEP_GEN_Instance_t *instance)
{
	uint64_t step;
	step = (uint64_t)STEP_GEN_CONFIG.frequency_acceleration;
	step *= (uint64_t)STEP_GEN_CONFIG.cycle_period;
	step /= 1000;
	STEP_GEN_DATA.acceleration_step = (uint32_t)step;
}



/**
 * Initialize the instance
 */
void STEP_GEN_Init(STEP_GEN_Instance_t *instance)
{
	STEP_GEN_DATA.current_speed = 0;//Hz
	STEP_GEN_DATA.current_position = 0;
	STEP_GEN_DATA.last_execution = 0;
	STEP_GEN_DATA.target_speed = 0;
	STEP_GEN_ComputeAccelerationStep(instance);
}

/**
 * Process to be called in SysTickHandler, main loop or any interruption with at least 4x the cycle period duration
 */
void STEP_GEN_Process(STEP_GEN_Instance_t *instance)
{
	uint32_t tick = HAL_GetTick();
	if (tick < (STEP_GEN_DATA.last_execution + STEP_GEN_CONFIG.cycle_period))
	{
		return;
	}
	STEP_GEN_DATA.last_execution = tick;
	STEP_GEN_ComputeNextFrequency(instance);
	STEP_GEN_TIM_SetFrequency(instance);
}

/**
 * shall be called in the HAL_TIM_PWM_PulseFinishedCallback implementation
 */
bool STEP_GEN_OnStepProcess(STEP_GEN_Instance_t *instance, TIM_HandleTypeDef *htim)
{
	if (htim != STEP_GEN_DESCRIPTOR->htim)
	{
		return 0;
	}
	STEP_GEN_CountSteps(instance);
	STEP_GEN_GPIO_SetDir(instance);
	return 1;
}

void STEP_GEN_SetTargetSpeed				(STEP_GEN_Instance_t *instance, int32_t 	new_target_speed		){ STEP_GEN_DATA.target_speed = new_target_speed; }
void STEP_GEN_SetCurrentPosition			(STEP_GEN_Instance_t *instance, int64_t 	new_current_position	){ STEP_GEN_DATA.current_position = new_current_position; }
void STEP_GEN_SetAcceleration			(STEP_GEN_Instance_t *instance, uint32_t new_acceleration		)
{
	STEP_GEN_CONFIG.frequency_acceleration = new_acceleration;
	STEP_GEN_ComputeAccelerationStep(instance);
}
void STEP_GEN_SetCyclePeriod				(STEP_GEN_Instance_t *instance, uint32_t new_cycle_period		)
{
	STEP_GEN_CONFIG.cycle_period = new_cycle_period;
	STEP_GEN_ComputeAccelerationStep(instance);
}

void STEP_GEN_ForceCurrentSpeed			(STEP_GEN_Instance_t *instance, int32_t 	new_current_speed		)
{
	STEP_GEN_DATA.target_speed = new_current_speed;
	STEP_GEN_DATA.current_speed = new_current_speed;
}

int32_t 	STEP_GEN_GetTargetSpeed		(STEP_GEN_Instance_t *instance){ return STEP_GEN_DATA.target_speed; }
int32_t 	STEP_GEN_GetCurrentSpeed	(STEP_GEN_Instance_t *instance){ return STEP_GEN_DATA.current_speed; }
int64_t 	STEP_GEN_GetCurrentPosition	(STEP_GEN_Instance_t *instance){ return STEP_GEN_DATA.current_position; }
uint32_t	STEP_GEN_GetAcceleration	(STEP_GEN_Instance_t *instance){ return STEP_GEN_CONFIG.frequency_acceleration; }
uint32_t 	STEP_GEN_GetCyclePeriod		(STEP_GEN_Instance_t *instance){ return STEP_GEN_CONFIG.cycle_period; }
