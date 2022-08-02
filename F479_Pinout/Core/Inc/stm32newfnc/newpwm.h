#include "stm32f4xx_hal.h"
#ifndef _NEWPWM_H_
#define _NEWPWM_H_

class NEWPWM
{
private:
    TIM_HandleTypeDef _tim;
    TIM_TypeDef *_port;
    uint32_t _Period;
    uint32_t _channel;
    uint8_t is_begun;

public:
    NEWPWM(TIM_TypeDef *timport, uint32_t Period = 255)
    {
        is_begun = 0;
        _port = timport;
        _tim.Instance = _port;
        _tim.Init.Prescaler = 1;
        _tim.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
        _tim.Init.Period = Period;
        _tim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
        _tim.Init.RepetitionCounter = 0;
        _tim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    }
    TIM_HandleTypeDef *getHandleTypeDef() { return &_tim; }

    void set_Period(uint32_t period){
        _Period  = period;
    }
    void run_ch_pwm(uint32_t channel, uint16_t pwm)
    {
        if (channel>TIM_CHANNEL_4)
        {
            return; //error
        }
        
        HAL_TIM_PWM_Stop(&_tim,channel);
        switch (channel)
        {
        case TIM_CHANNEL_1: _port->CCR1 = pwm; break;
        case TIM_CHANNEL_2: _port->CCR2 = pwm; break;
        case TIM_CHANNEL_3: _port->CCR3 = pwm; break;
        case TIM_CHANNEL_4: _port->CCR4 = pwm; break;   
        }
        HAL_TIM_PWM_Start(&_tim,channel);

    }
    uint8_t get_is_begun() { return is_begun;}
    void begin()
    {
        is_begun =1;
        TIM_ClockConfigTypeDef sClockSourceConfig = {0};
        TIM_MasterConfigTypeDef sMasterConfig = {0};
        TIM_OC_InitTypeDef sConfigOC = {0};
        TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
        if (HAL_TIM_Base_Init(&_tim) != HAL_OK)
        {
            // Error_Handler();
        }
        sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
        if (HAL_TIM_ConfigClockSource(&_tim, &sClockSourceConfig) != HAL_OK)
        {
            // Error_Handler();
        }
        if (HAL_TIM_PWM_Init(&_tim) != HAL_OK)
        {
            // Error_Handler();
        }
        sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
        sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
        if (HAL_TIMEx_MasterConfigSynchronization(&_tim, &sMasterConfig) != HAL_OK)
        {
            // Error_Handler();
        }
        sConfigOC.OCMode = TIM_OCMODE_PWM1;
        sConfigOC.Pulse = 0;
        sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
        sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
        sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
        sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
        sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
        if (HAL_TIM_PWM_ConfigChannel(&_tim, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
        {
            // Error_Handler();
        }
        if (HAL_TIM_PWM_ConfigChannel(&_tim, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
        {
            // Error_Handler();
        }
        if (HAL_TIM_PWM_ConfigChannel(&_tim, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
        {
            // Error_Handler();
        }
        sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
        sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
        sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
        sBreakDeadTimeConfig.DeadTime = 0;
        sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
        sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
        sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
        if (HAL_TIMEx_ConfigBreakDeadTime(&_tim, &sBreakDeadTimeConfig) != HAL_OK)
        {
            // Error_Handler();
        }
        /* USER CODE BEGIN TIM1_Init 2 */

        /* USER CODE END TIM1_Init 2 */
        HAL_TIM_MspPostInit(&_tim);
    }
    
};
#endif
