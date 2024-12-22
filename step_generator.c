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

#define STEP_GEN_SIGN(var) (((var)>=0)?1:-1)
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
static void STEP_GEN_AtTargetAction(STEP_GEN_Instance_t *instance);
static void STEP_GEN_ControllerProcess(STEP_GEN_Instance_t *instance);
static STEP_GEN_ControllerState_t STEP_GEN_Controller_GetNextState(STEP_GEN_Instance_t *instance);
static void STEP_GEN_Controller_StateInitialization(STEP_GEN_Instance_t *instance);
static void STEP_GEN_Controller_StateExecution(STEP_GEN_Instance_t *instance);
static uint32_t STEP_GEN_DistanceToReachApproachSpeed(STEP_GEN_Instance_t *instance);
static void STEP_GEN_PositionMode_Init(STEP_GEN_Instance_t *instance);
static void STEP_GEN_SpeedMode_Init(STEP_GEN_Instance_t *instance);

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
	uint64_t divider, max_divider;
	uint32_t prescaler, period;
	uint8_t i;


	// if the speed is null, stops the timer
	if ( STEP_GEN_DATA.current_speed == 0)
	{
		STEP_GEN_TIM_Stop(instance);
		return;
	}

	// max divider value corresponds to the square of the register max value --> 2^64 in case of 32 bit registers
	max_divider = ( (uint64_t)(STEP_GEN_DESCRIPTOR->tim_max_value) )*( (uint64_t)(STEP_GEN_DESCRIPTOR->tim_max_value) );

	// compute the division factor of the timer clock to generate the wanted speed
	divider = (uint64_t)(STEP_GEN_DESCRIPTOR->tim_peripheral_clk) / (uint64_t)(STEP_GEN_ABS(STEP_GEN_DATA.current_speed));

	// limits the divider value to the maximum (case of low frequency)
	divider = STEP_GEN_MIN(divider, max_divider);

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
 * in/decrements step counter according to current speed sign
 */
static void STEP_GEN_CountSteps(STEP_GEN_Instance_t *instance)
{
	STEP_GEN_DATA.current_position += STEP_GEN_SIGN(STEP_GEN_DATA.current_speed);
}

/**
 * Apply given timings to the intsance's timer
 */
static void STEP_GEN_ApplyTimings(STEP_GEN_Instance_t *instance, uint32_t prescaler, uint32_t period)
{
	__HAL_TIM_SET_PRESCALER(STEP_GEN_DESCRIPTOR->htim, prescaler-1);//minus one as psc and period are 0 based.
	__HAL_TIM_SET_AUTORELOAD(STEP_GEN_DESCRIPTOR->htim, period-1);
	__HAL_TIM_SET_COMPARE(STEP_GEN_DESCRIPTOR->htim, STEP_GEN_DESCRIPTOR->tim_ch, (period>>1)); // half of period for 50% of duty cycle
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
 * Executed when target and current position are equal
 */
static void STEP_GEN_AtTargetAction(STEP_GEN_Instance_t *instance)
{
	// only executes if :
	//		position control enabled
	if ( STEP_GEN_DATA.control_mode != STEP_GEN_CTRL_POSITION ) return;
	// 		in approach state
	if ( STEP_GEN_DATA.controller_state != STEP_GEN_STATE_APPROACH ) return;
	//		if current position meets the target
	if ( STEP_GEN_DATA.current_position != STEP_GEN_DATA.target_position) return;

	// change the state to AT TARGET
	STEP_GEN_DATA.controller_state = STEP_GEN_STATE_AT_TARGET;

	// stops the motor
	STEP_GEN_DATA.current_speed = 0;
	STEP_GEN_DATA.target_speed = 0;
	STEP_GEN_TIM_SetFrequency(instance);
}

/**
 * Controls the speed according to position control
 */
static void STEP_GEN_ControllerProcess(STEP_GEN_Instance_t *instance)
{
	STEP_GEN_ControllerState_t current_state, next_state;

	// only executes if position control enabled else, sets the state to init and returns
	if ( STEP_GEN_DATA.control_mode != STEP_GEN_CTRL_POSITION )
	{
		STEP_GEN_DATA.controller_state = STEP_GEN_STATE_INIT;
		return;
	}

	STEP_GEN_DATA.speed_dir = STEP_GEN_SIGN(STEP_GEN_DATA.current_speed);
	STEP_GEN_DATA.target_dir = STEP_GEN_SIGN(STEP_GEN_DATA.target_position - STEP_GEN_DATA.current_position);
	STEP_GEN_DATA.stop_distance = STEP_GEN_DistanceToReachApproachSpeed(instance);
	STEP_GEN_DATA.stop_position = STEP_GEN_DATA.current_position + STEP_GEN_DATA.stop_distance * (int64_t)STEP_GEN_DATA.speed_dir;

	current_state = STEP_GEN_DATA.controller_state;

	next_state = STEP_GEN_Controller_GetNextState(instance);

	if ( next_state != current_state )
	{
		STEP_GEN_DATA.controller_state = next_state;
		STEP_GEN_Controller_StateInitialization(instance);
	}
	else
	{
		STEP_GEN_Controller_StateExecution(instance);
	}

}

/**
 * Computes the next state fro position controller
 */
static STEP_GEN_ControllerState_t STEP_GEN_Controller_GetNextState(STEP_GEN_Instance_t *instance)
{
	switch ( STEP_GEN_DATA.controller_state )
	{
	case STEP_GEN_STATE_INIT:
		// always go to AT_TARGET even if not matching to wait receiving new one
		return STEP_GEN_STATE_AT_TARGET;

	case STEP_GEN_STATE_AT_TARGET:
		// go to NEW_TARGET when a new target is received
		return STEP_GEN_STATE_AT_TARGET;

	case STEP_GEN_STATE_NEW_TARGET:
		// State used when a new target is received whatever the current configuration


		if ( STEP_GEN_ABS(STEP_GEN_DATA.stop_position - STEP_GEN_DATA.target_position) <= STEP_GEN_CONFIG.approach_distance )
		{
			// if the projected stop point is inside the approach envelope around the target, starts to decelerate
			return STEP_GEN_STATE_DECELERATE;
		}
		else
		if ( STEP_GEN_ABS(STEP_GEN_DATA.current_speed) <= STEP_GEN_CONFIG.approach_speed )
		{
			// else, if the speed is inferior to the approach
			return STEP_GEN_STATE_RUN;
		}
		else
		if ( STEP_GEN_DATA.speed_dir == STEP_GEN_DATA.target_dir )
		{
			return STEP_GEN_STATE_RUN;
		}
		else
		{
			return STEP_GEN_STATE_DECELERATE;
		}

	case STEP_GEN_STATE_RUN:
		// go to NEW_TARGET when a new target is received
		// go to decelerate if projected deceleration point is inside approach distance margin
		if ( STEP_GEN_ABS(STEP_GEN_DATA.stop_position - STEP_GEN_DATA.target_position) <= STEP_GEN_CONFIG.approach_distance )
		{
			return STEP_GEN_STATE_DECELERATE;
		}
		else
		{
			return STEP_GEN_STATE_RUN;
		}

	case STEP_GEN_STATE_DECELERATE:
		// go to approach if speed have reached the target
		if ( STEP_GEN_ABS(STEP_GEN_DATA.current_speed) <= STEP_GEN_CONFIG.approach_speed )
		{
			return STEP_GEN_STATE_APPROACH;
		}
		else
		{
			return STEP_GEN_STATE_DECELERATE;
		}

	case STEP_GEN_STATE_APPROACH:
		// go to NEW_TARGET when a new target is received
		// go to AT_TARGET when positions matches using the step interruption
		if ( STEP_GEN_ABS(STEP_GEN_DATA.stop_position - STEP_GEN_DATA.target_position) > STEP_GEN_CONFIG.approach_distance )
		{
			return STEP_GEN_STATE_RUN;
		}
		return STEP_GEN_STATE_APPROACH;

	default:
		// lost so go
		return STEP_GEN_STATE_ERROR;
	}

}

/**
 * Executes the initialization code for the position controller state
 */
static void STEP_GEN_Controller_StateInitialization(STEP_GEN_Instance_t *instance)
{
	switch ( STEP_GEN_DATA.controller_state )
	{
	case STEP_GEN_STATE_INIT:
		// NOP
		return;

	case STEP_GEN_STATE_AT_TARGET:
		// stops the motor
		STEP_GEN_DATA.current_speed = 0;
		STEP_GEN_DATA.target_speed = 0;
		return;

	case STEP_GEN_STATE_NEW_TARGET:
		// NOP
		return;

	case STEP_GEN_STATE_RUN:
		STEP_GEN_DATA.target_speed = STEP_GEN_CONFIG.run_speed * (int64_t)STEP_GEN_DATA.target_dir;
		return;

	case STEP_GEN_STATE_DECELERATE:
	case STEP_GEN_STATE_APPROACH:
		// target speed to approach speed
		STEP_GEN_DATA.target_speed = STEP_GEN_CONFIG.approach_speed *  (int64_t)STEP_GEN_DATA.target_dir;
		return;

	default:
		// stops the motor
		STEP_GEN_DATA.current_speed = 0;
		STEP_GEN_DATA.target_speed = 0;
		return;
	}
}

/**
 * Executes the action for position conroller state
 */
static void STEP_GEN_Controller_StateExecution(STEP_GEN_Instance_t *instance)
{
	switch ( STEP_GEN_DATA.controller_state )
	{
	case STEP_GEN_STATE_INIT:
		// NOP
		return;

	case STEP_GEN_STATE_AT_TARGET:
		//NOP
		return;

	case STEP_GEN_STATE_NEW_TARGET:
		// NOP
		return;

	case STEP_GEN_STATE_RUN:
		// NOP
		return;

	case STEP_GEN_STATE_DECELERATE:
		// NOP
		return;

	case STEP_GEN_STATE_APPROACH:
		// NOP
		return;

	default:
		// stops the motor
		STEP_GEN_DATA.current_speed = 0;
		STEP_GEN_DATA.target_speed = 0;
		return;
	}
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
 * Computes the distance to decelerate and reach approach speed
 */
static uint32_t STEP_GEN_DistanceToReachApproachSpeed(STEP_GEN_Instance_t *instance)
{
	uint32_t v_cur, v_app, acc;
	v_cur = (uint32_t)(STEP_GEN_ABS(STEP_GEN_DATA.current_speed));
	v_app = (uint32_t)(STEP_GEN_ABS(STEP_GEN_CONFIG.approach_speed));
	acc = (uint32_t)(STEP_GEN_CONFIG.frequency_acceleration);


#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wimplicit-function-declaration"
	// forbidden case
	if ( acc == 0 ) HardFault_Handler(); // ignore warning
#pragma GCC diagnostic pop


	// edge case, current speed inferior to approach speed, no deceleration needed
	if ( v_cur <= v_app ) return 0;

	// V²/2A
	return (v_cur*v_cur-v_app*v_app)/(acc);
}

/**
 * Initialize the system when going into position mode
 */
static void STEP_GEN_PositionMode_Init(STEP_GEN_Instance_t *instance)
{
	STEP_GEN_DATA.target_position = STEP_GEN_DATA.current_position;
	STEP_GEN_DATA.controller_state = STEP_GEN_STATE_AT_TARGET;
	STEP_GEN_DATA.control_mode = STEP_GEN_CTRL_POSITION;
}

/**
 * Initialize the system when going into speed mode
 */
static void STEP_GEN_SpeedMode_Init(STEP_GEN_Instance_t *instance)
{
	STEP_GEN_DATA.target_speed = 0;
	STEP_GEN_DATA.controller_state = STEP_GEN_STATE_INACTIVE;
	STEP_GEN_DATA.control_mode = STEP_GEN_CTRL_SPEED;
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
	STEP_GEN_DATA.target_position = 0;
	STEP_GEN_DATA.control_mode = STEP_GEN_CONFIG.default_control_mode;
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
	STEP_GEN_ControllerProcess(instance);
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
	STEP_GEN_AtTargetAction(instance);
	return 1;
}

bool STEP_GEN_SetControlMode(STEP_GEN_Instance_t *instance, STEP_GEN_ControlMode_t new_mode)
{
	if ( STEP_GEN_DATA.control_mode == new_mode ) return 1;

	if ( new_mode >= STEP_GEN_CTRL_MAX ) return 0;

	if ( STEP_GEN_DATA.current_speed != 0 ) return 0;

	switch ( new_mode )
	{
	case STEP_GEN_CTRL_POSITION:
		STEP_GEN_PositionMode_Init(instance);
		break;
	case STEP_GEN_CTRL_SPEED:
		STEP_GEN_SpeedMode_Init(instance);
		break;
	default:
		STEP_GEN_DATA.control_mode = STEP_GEN_CTRL_NONE;
		break;
	}

	return 1;
}

/**
 * Sets a new target speed. Refuses if not in speed control mode
 */
void STEP_GEN_SetTargetSpeed			(STEP_GEN_Instance_t *instance, int32_t new_target_speed		)
{
	if ( STEP_GEN_DATA.control_mode != STEP_GEN_CTRL_SPEED ) return;
	STEP_GEN_DATA.target_speed = new_target_speed;
}

/**
 * Sets a new targt position. Refuses if not in position control mode
 */
void STEP_GEN_SetTargetPosition			(STEP_GEN_Instance_t *instance, int64_t new_target_position		)
{
	if ( STEP_GEN_DATA.control_mode != STEP_GEN_CTRL_POSITION ) return;
	STEP_GEN_DATA.target_position = new_target_position;
	STEP_GEN_DATA.controller_state = STEP_GEN_STATE_NEW_TARGET;
}


void STEP_GEN_SetCurrentPosition		(STEP_GEN_Instance_t *instance, int64_t new_current_position	)
{
	if ( STEP_GEN_DATA.control_mode == STEP_GEN_CTRL_POSITION )
	{
		if ( STEP_GEN_DATA.controller_state != STEP_GEN_STATE_AT_TARGET )
		{
			return;
		}
		STEP_GEN_DATA.target_position = new_current_position;
	}
	STEP_GEN_DATA.current_position = new_current_position;
}

/**
 * Sets the acceleration parameter. If in POISTION mode, restarts the algo to avoid side effects
 */
void STEP_GEN_SetAcceleration			(STEP_GEN_Instance_t *instance, uint32_t new_acceleration		)
{
	STEP_GEN_CONFIG.frequency_acceleration = new_acceleration;
	STEP_GEN_ComputeAccelerationStep(instance);
	if ( STEP_GEN_DATA.control_mode == STEP_GEN_CTRL_POSITION )
	{
		STEP_GEN_DATA.controller_state = STEP_GEN_STATE_NEW_TARGET;
	}
}

/**
 * Sets the speed and position control controllers periods
 */
void STEP_GEN_SetCyclePeriod			(STEP_GEN_Instance_t *instance, uint32_t new_cycle_period		)
{
	STEP_GEN_CONFIG.cycle_period = new_cycle_period;
	STEP_GEN_ComputeAccelerationStep(instance);
}

/**
 * Used to force a speed. Refuses if not SPEED control
 */
void STEP_GEN_ForceCurrentSpeed			(STEP_GEN_Instance_t *instance, int32_t new_current_speed		)
{
	if ( STEP_GEN_DATA.control_mode != STEP_GEN_CTRL_SPEED ) return;
	STEP_GEN_DATA.target_speed = new_current_speed;
	STEP_GEN_DATA.current_speed = new_current_speed;
}

/**
 * Performs a hard stop of the motor. If in POSITIOn mode, consider to be at its target
 */
void STEP_GEN_HardStop					(STEP_GEN_Instance_t *instance)
{
	STEP_GEN_DATA.target_speed = 0;
	STEP_GEN_DATA.current_speed = 0;
	STEP_GEN_DATA.target_position = STEP_GEN_DATA.current_position;
//	STEP_GEN_DATA.control_mode = STEP_GEN_CTRL_NONE;
	STEP_GEN_TIM_SetFrequency(instance);

	if ( STEP_GEN_DATA.control_mode == STEP_GEN_CTRL_POSITION )
	{
		STEP_GEN_DATA.controller_state = STEP_GEN_STATE_AT_TARGET;
	}
}

STEP_GEN_ControlMode_t 	STEP_GEN_GetControlMode		(STEP_GEN_Instance_t *instance){ return STEP_GEN_DATA.control_mode; }
int32_t 				STEP_GEN_GetTargetSpeed		(STEP_GEN_Instance_t *instance){ return STEP_GEN_DATA.target_speed; }
int32_t 				STEP_GEN_GetCurrentSpeed	(STEP_GEN_Instance_t *instance){ return STEP_GEN_DATA.current_speed; }
int64_t 				STEP_GEN_GetCurrentPosition	(STEP_GEN_Instance_t *instance){ return STEP_GEN_DATA.current_position; }
uint32_t				STEP_GEN_GetAcceleration	(STEP_GEN_Instance_t *instance){ return STEP_GEN_CONFIG.frequency_acceleration; }
uint32_t 				STEP_GEN_GetCyclePeriod		(STEP_GEN_Instance_t *instance){ return STEP_GEN_CONFIG.cycle_period; }
