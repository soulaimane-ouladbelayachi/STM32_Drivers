
#include "motor.h"

#define MOTOR_1_PIN GPIO_PIN_11
#define MOTOR_2_PIN GPIO_PIN_10
#define MOTOR_3_PIN GPIO_PIN_9
#define MOTOR_4_PIN GPIO_PIN_8
#define MOTOR_PORT GPIOA


TIM_HandleTypeDef htim1;

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);


void motorsInit(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
  RCC_ClkInitTypeDef    clkconfig;
  uint32_t              uwTimclock, uwAPB2Prescaler = 0U;
  uint32_t              uwPrescalerValue = 0U;
  uint32_t              pFLatency;

  /* Get clock configuration */
  HAL_RCC_GetClockConfig(&clkconfig, &pFLatency);

  /* Get APB2 prescaler */
  uwAPB2Prescaler = clkconfig.APB2CLKDivider;

  /* Compute TIM1 clock */
  if (uwAPB2Prescaler == RCC_HCLK_DIV1)
  {
    uwTimclock = HAL_RCC_GetPCLK2Freq();
  }
  else
  {
    uwTimclock = 2*HAL_RCC_GetPCLK2Freq();
  }

  /* Compute the prescaler value to have TIM1 counter clock equal to 1MHz */
  uwPrescalerValue = (uint32_t) ((uwTimclock / 1000000U) - 1U);

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = uwPrescalerValue;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = MOTOR_OUTPUT_PERIOD;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = MOTOR_LOW_VAL_US;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim1);

}

int motorsStart()
{


  // Set all the outputs to low, should already be done but just in case
  __HAL_TIM_SET_COMPARE(&htim1, MOTOR_FRONT_LEFT, MOTOR_LOW_VAL_US);
  __HAL_TIM_SET_COMPARE(&htim1, MOTOR_FRONT_RIGHT, MOTOR_LOW_VAL_US);
  __HAL_TIM_SET_COMPARE(&htim1, MOTOR_BACK_LEFT, MOTOR_LOW_VAL_US);
  __HAL_TIM_SET_COMPARE(&htim1, MOTOR_BACK_RIGHT, MOTOR_LOW_VAL_US);

  if (HAL_TIM_PWM_Start(&htim1, MOTOR_FRONT_LEFT) != HAL_OK)
  {
      /* PWM Generation Error */
      DEBUG_PRINT("Failed to start motor 1\n");

  }
  if (HAL_TIM_PWM_Start(&htim1, MOTOR_FRONT_RIGHT) != HAL_OK)
  {
      /* PWM Generation Error */
      DEBUG_PRINT("Failed to start motor 1\n");

  }
  if (HAL_TIM_PWM_Start(&htim1, MOTOR_BACK_LEFT) != HAL_OK)
  {
      /* PWM Generation Error */
      DEBUG_PRINT("Failed to start motor 1\n");

  }
  /* Start channel 4 */
  if (HAL_TIM_PWM_Start(&htim1, MOTOR_BACK_RIGHT) != HAL_OK)
  {
      /* PWM Generation Error */
      DEBUG_PRINT("Failed to start motor 1\n");

  }

}

void motorsDeinit()
{
    if (HAL_TIM_PWM_Stop(&htim1, MOTOR_FRONT_LEFT) != HAL_OK)
    {
        DEBUG_PRINT("Failed to de init motors\n");

    }
    if (HAL_TIM_PWM_Stop(&htim1, MOTOR_FRONT_RIGHT) != HAL_OK)
    {
        DEBUG_PRINT("Failed to de init motors\n");

    }
    if (HAL_TIM_PWM_Stop(&htim1, MOTOR_BACK_LEFT) != HAL_OK)
    {
        DEBUG_PRINT("Failed to de init motors\n");

    }
    if (HAL_TIM_PWM_Stop(&htim1, MOTOR_BACK_RIGHT) != HAL_OK)
    {
        DEBUG_PRINT("Failed to de init motors\n");

    }


}

void motorsStop()
{
  __HAL_TIM_SET_COMPARE(&htim1, MOTOR_FRONT_LEFT, MOTOR_LOW_VAL_US);
  __HAL_TIM_SET_COMPARE(&htim1, MOTOR_FRONT_RIGHT, MOTOR_LOW_VAL_US);
  __HAL_TIM_SET_COMPARE(&htim1, MOTOR_BACK_LEFT, MOTOR_LOW_VAL_US);
  __HAL_TIM_SET_COMPARE(&htim1, MOTOR_BACK_RIGHT, MOTOR_LOW_VAL_US);

}

void setMotor(MotorNum motor, uint32_t val)
{


    if (val < MOTOR_LOW_VAL_US)
    {
        val = MOTOR_LOW_VAL_US;
        rc = FC_ERROR;
    } else if (val > MOTOR_HIGH_VAL_US)
    {
        val = MOTOR_LOW_VAL_US;
        rc = FC_ERROR;
    }

    __HAL_TIM_SET_COMPARE(&htim1, motor, val);


}

