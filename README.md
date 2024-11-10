The MPP library is used to control DIR and STEP pins of a motor driver in speed mode only

Configuration needs in IOC File :
- 1 Timer with 1 channel in PWM generation mode, and OC IRQ active
- 1 GPIO


Exemple of Instance :
```c

STEP_GEN_Descriptor_t mpp_desc = {
	.dir_pin = DIR_Pin,
	.dir_port = DIR_GPIO_Port,
	.htim = &htim1,
	.tim_ch = TIM_CHANNEL_1,
	.tim_peripheral_clk = 80000000,
	.tim_max_value = 0x0000FFFF
};


STEP_GEN_Instance_t mpp_inst = {
	.config = {
		.cycle_period = 10,
		.frequency_acceleration = 1000
	},
	.descriptor = &mpp_desc
};

...

void main()
{
...
  /* USER CODE BEGIN 2 */
  STEP_GEN_Init(&mpp_inst);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  STEP_GEN_Process(&mpp_inst);
	/* USER CODE END 3 */
  }
}

```

Exemple of HAL_TIM_PWM_PulseFinishedCallback implementation :
```c

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	STEP_GEN_OnStepProcess(&mpp_inst, htim);
}

```