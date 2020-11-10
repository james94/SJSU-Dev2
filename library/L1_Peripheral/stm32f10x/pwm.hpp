#pragma once

#include "L0_Platform/stm32f10x/stm32f10x.h"
#include "L1_Peripheral/stm32f10x/pin.hpp"
#include "L1_Peripheral/pwm.hpp"
#include "L1_Peripheral/stm32f10x/system_controller.hpp"
#include "utility/log.hpp"


namespace sjsu::stm32f10x
{
/// An implementation of the sjsu::Pwm peripheral for the stm32f10x family of
/// microcontrollers.
class Pwm final : public sjsu::Pwm
{
public:    
    void ConfigureFrequency(units::frequency::hertz_t frequency) override
    {
        // To configure frequency, we have to set the ARR register
        auto & system  = SystemController::GetPlatformController();
        //auto frequency = system.GetClockRate(Peripherals::kSystemTimer.device_id);

        // This means that whenever TIM->CNT hits this value, it will start over from 0
    }

    void SetDutyCycle(float duty_cycle) override
    {
        // We set duty cycle by adjusting TIMx->CCRx
        TIM1->CCR1 = (float) (duty_cycle * TIM1->ARR);
    }

    float GetDutyCycle() override
    {
        return (float) (TIM1->ARR / TIM1->CCR1);
    }

    void ModuleEnable(bool enable = true) override
    {

    }

    void ModuleInitialize()
    {
        RCC->APB2ENR |= (1 << 2);           // enable port a clock
        RCC->APB1ENR |= (1 << 0);           // enable clock for timer 2

        /* Porta Pin 1 &2 as alternate function output Push pull 50 MHz */
        auto output = Pin('A', 1);
        output.ConfigureFunction(1);

        TIM2->ARR = 65535;                      // Set Auto reload value
        TIM2->PSC = 21;                         // Set Prescalar value

        TIM2->CCMR1  = 0x00007800; 
        TIM2->EGR |= (1 << 0);

        TIM2->CCER = 0x00000010;  

        TIM2->CR1 |= (1 << 7);      // Auto preload ENABLE
        TIM2->CR1 |= (1 << 0);       // ENABLE Timer counter   


        // // Configure GPIO first
        // // We use Port C, pin 13
        // auto output_pin = Pin('C', 13);
        // output_pin.ConfigureFunction(1);

        // // Then we configure the timer
        // // Should give 75% duty cycle
        // TIM1->ARR = 65530;
        // TIM1->CCR1 = 45555;

        // // Set prescaler to zero
        // TIM1->PSC = 0;

        // // Setting output to PWM type 1
        // TIM1->CCMR1 &= ~(7 << 4);
        // TIM1->CCMR1 |= (6 << 4);

        // // This makes it so that the value in CCRx will only be updated at
        // // an update event
        // TIM1->CCMR1 |= (1 << 3);

        // // Auto Reload preload enable
        // TIM1->CR1 |= (1 << 7);

        // // Connect clock to timer
        // RCC->APB2ENR |= (1 << 11);

        // // Enable AFIO
        // RCC->APB2ENR |= (1 << 0);

        // // Enable clock for GPIO port C
        // RCC->APB2ENR |= (1 << 4);

        // // Enable timer pin
        // TIM1->CCER |= (1 << 0);

        // // Set MOE pin to connect timr to GPIO
        // TIM1->BDTR |= (1 << 15);

        // // Enable counter itself
        // TIM1->CR1 |= (1 << 0);

        // // This initializes all registers (UG bit)
        // TIM1->EGR |= (1 << 0);

        // sjsu::LogInfo("Value of ARR: %b\n", TIM1->ARR);

        while(true) {
            TIM2->CCR2  = 32768;
        }
    }
};
}  // namespace sjsu::stm32f10x