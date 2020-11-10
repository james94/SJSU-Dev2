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

    // Timers 1-5 and 8-14 can be used for PWM
    struct Timer_t {
      // Reference to the pin
      sjsu::Pin & pin;

      // Specifying which of the 12 timers it is
      uint8_t timer : 4;

      // Holds the address to the STM PWM peripheral
      TIM_TypeDef * registers;
    };

    struct Timer {
     private:
      inline static sjsu::stm32f10x::Pin pin_a_0 = sjsu::stm32f10x::Pin('A', 0);
      inline static sjsu::stm32f10x::Pin pin_a_2 = sjsu::stm32f10x::Pin('A', 2);
      inline static sjsu::stm32f10x::Pin pin_a_6 = sjsu::stm32f10x::Pin('A', 6);
      inline static sjsu::stm32f10x::Pin pin_a_7 = sjsu::stm32f10x::Pin('A', 7);
      inline static sjsu::stm32f10x::Pin pin_a_8 = sjsu::stm32f10x::Pin('A', 8);
      inline static sjsu::stm32f10x::Pin pin_b_6 = sjsu::stm32f10x::Pin('B', 6);
      inline static sjsu::stm32f10x::Pin pin_b_8 = sjsu::stm32f10x::Pin('B', 8);
      inline static sjsu::stm32f10x::Pin pin_b_9 = sjsu::stm32f10x::Pin('B', 9);

     public:
      inline static const Timer_t kTimer1 = {
        .pin = pin_a_8,
        .timer = 1,
        .registers = TIM1
      };

      inline static const Timer_t kTimer2 = {
        .pin = pin_a_0,
        .timer = 2,
        .registers = TIM2
      };

      inline static const Timer_t kTimer3 = {
        .pin = pin_a_6,
        .timer = 3,
        .registers = TIM3
      };

      inline static const Timer_t kTimer4 = {
        .pin = pin_b_6,
        .timer = 4,
        .registers = TIM4
      };

      inline static const Timer_t kTimer9 = {
        .pin = pin_a_2,
        .timer = 9,
        .registers = TIM9
      };

      inline static const Timer_t kTimer10 = {
        .pin = pin_b_8,
        .timer = 10,
        .registers = TIM10
      };

      inline static const Timer_t kTimer11 = {
        .pin = pin_b_9,
        .timer = 11,
        .registers = TIM11
      };

      inline static const Timer_t kTimer13 = {
        .pin = pin_a_6,
        .timer = 13,
        .registers = TIM12
      };

      inline static const Timer_t kTimer14 = {
        .pin = pin_a_7,
        .timer = 14,
        .registers = TIM14
      };
    };

    /// Constructor for a STM32F10x PWM channel.
    ///
    /// @param timer - Reference to a const timer description for this
    ///        instance of the PWM driver.
    explicit constexpr Pwm(const Timer_t & timer) : timer_(timer) {}
    

    void ConfigureFrequency(units::frequency::hertz_t frequency) override
    {
        // To configure frequency, we have to set the ARR register
        auto & system  = SystemController::GetPlatformController();
        //auto frequency = system.GetClockRate(Peripherals::kSystemTimer.device_id);

        // This means that whenever TIM->CNT hits this value, it will start over from 0
    }

    void SetDutyCycle(float duty_cycle) override
    {
      // Clamp the duty cycle to make sure it's in the right range
      const float kClampedDutyCycle = std::clamp(duty_cycle, 0.0f, 1.0f);

      // We set duty cycle by adjusting TIMx->CCRx
      timer_.registers->CCR1 = static_cast<float>(kClampedDutyCycle * timer_.registers->ARR);
    }

    float GetDutyCycle() override
    {
      return static_cast<float>(timer_.registers->ARR / timer_.registers->CCR1);
    }

    void ModuleEnable(bool enable = true) override
    {
    }

    void ModuleInitialize()
    {
        RCC->APB2ENR |= (1 << 2);           // enable port a clock
        RCC->APB1ENR |= (1 << 0);           // enable clock for timer 2

        auto output = Pin('A', 2);
        output.ConfigureFunction(1);

        TIM2->ARR = 65535;                      // Set Auto reload value
        TIM2->PSC = 21;                         // Set Prescalar value

        // Using output compare 2
        //TIM2->CCMR1  = 0x00007800;
        TIM2->CCMR2 |= (7 << 4);
        TIM2->CCMR2 |= (1 << 3);

        TIM2->EGR |= (1 << 0);

        //TIM2->CCER = 0x00000010;  
        TIM2->CCER |= (1 << 8);

        TIM2->CR1 |= (1 << 7);      // Auto preload ENABLE
        TIM2->CR1 |= (1 << 0);      // ENABLE Timer counter   


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
            TIM2->CCR3  = 32768;
        }
    }
  const Timer_t & timer_;
};
}  // namespace sjsu::stm32f10x