#pragma once

#include "L0_Platform/stm32f10x/stm32f10x.h"
#include "L1_Peripheral/pwm.hpp"


namespace sjsu::stm32f10x
{
/// An implementation of the sjsu::Pwm peripheral for the stm32f10x family of
/// microcontrollers.
///
/// @warning - Changing the frequency of 1 PWM channel, changes the frequency
/// for all other PWM channels.
class Pwm final : public sjsu::Pwm
{
public:
    void ConfigureFrequency(units::frequency::hertz_t frequency) override
    {
        
    }

    void SetDutyCycle(float duty_cycle) override
    {

    }

    float GetDutyCycle() override
    {

    }

    void ModuleInitialize()
    {
        // Auto Reload Register is not used
        TIM1->ARR = 0;

        // Setting duty cycle to be 75%
        TIM1->CCR1 = 49152;

        // Setting channel to output (reset value)
        TIM1->CCMR1 &= ~(3 << 8);

        // Setting output to PWM type 1
        TIM1->CCMR1 &= ~(7 << 4);
        TIM1->CCMR1 |= (6 << 4);

        // Enable time
        TIM1->CCER |= (1 << 0);

        // Enable timer
        TIM1->CR1 |= (1 << 0);
    }
};
}  // namespace sjsu::stm32f10x