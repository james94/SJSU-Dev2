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
  static constexpr auto kOutputCompareMode = bit::MaskFromRange(4, 6);
  static constexpr auto kOutputComparePreloadEnable = bit::MaskFromRange(3);

  static constexpr auto kOutputChannelEnable = bit::MaskFromRange(0);

  static constexpr auto kAutoReloadPreloadEnable = bit::MaskFromRange(7);
  static constexpr auto kTimerEnable = bit::MaskFromRange(0);
  static constexpr auto kUpdateGeneration = bit::MaskFromRange(0);

  struct PortEnable {
    static constexpr auto kPortAClockEnable = bit::MaskFromRange(2);
    static constexpr auto kPortBClockEnable = bit::MaskFromRange(3);
  };

  struct TimerClockEnable {
    static constexpr auto kTimer1ClockEnable = bit::MaskFromRange(11);
    static constexpr auto kTimer2ClockEnable = bit::MaskFromRange(0);
    static constexpr auto kTimer3ClockEnable = bit::MaskFromRange(1);
    static constexpr auto kTimer4ClockEnable = bit::MaskFromRange(2);
    static constexpr auto kTimer9ClockEnable = bit::MaskFromRange(19);
    static constexpr auto kTimer10ClockEnable = bit::MaskFromRange(20);
    static constexpr auto kTimer11ClockEnable = bit::MaskFromRange(21);
    static constexpr auto kTimer14ClockEnable = bit::MaskFromRange(8);
  };

  // Timers 1-5 and 8-14 can be used for PWM
  struct Timer_t {
    // Reference to the pin
    sjsu::Pin & pin;

    // Specifying which of the 12 timers it is
    uint8_t timer : 4;

    // Holds the address to the STM PWM peripheral
    TIM_TypeDef * registers;

    // Peripheral ID for each timer
    sjsu::SystemController::ResourceID id;
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
      .registers = TIM1,
      .id = SystemController::Peripherals::kTimer1
    };

    inline static const Timer_t kTimer2 = {
      .pin = pin_a_0,
      .timer = 2,
      .registers = TIM2,
      .id = SystemController::Peripherals::kTimer2
    };

    inline static const Timer_t kTimer3 = {
      .pin = pin_a_6,
      .timer = 3,
      .registers = TIM3,
      .id = SystemController::Peripherals::kTimer3
    };

    inline static const Timer_t kTimer4 = {
      .pin = pin_b_6,
      .timer = 4,
      .registers = TIM4,
      .id = SystemController::Peripherals::kTimer4
    };

    inline static const Timer_t kTimer9 = {
      .pin = pin_a_2,
      .timer = 9,
      .registers = TIM9,
      .id = SystemController::Peripherals::kTimer9
    };

    inline static const Timer_t kTimer10 = {
      .pin = pin_b_8,
      .timer = 10,
      .registers = TIM10,
      .id = SystemController::Peripherals::kTimer10
    };

    inline static const Timer_t kTimer11 = {
      .pin = pin_b_9,
      .timer = 11,
      .registers = TIM11,
      .id = SystemController::Peripherals::kTimer11
    };

    inline static const Timer_t kTimer14 = {
      .pin = pin_a_7,
      .timer = 14,
      .registers = TIM14,
      .id = SystemController::Peripherals::kTimer14
    };
  }; // Timer

  /// Constructor for a STM32F10x PWM channel.
  ///
  /// @param timer - Reference to a const timer description for this
  ///        instance of the PWM driver.
  explicit constexpr Pwm(const Timer_t & timer) : timer_(timer) {}
  
  void ConfigureFrequency(units::frequency::hertz_t frequency) override
  {
    auto & system  = SystemController::GetPlatformController();
    auto kPeripheralFrequency = system.GetClockRate(timer_.id);

    auto period = 1 / frequency;
    uint32_t product = kPeripheralFrequency * period;

    timer_.registers->PSC = GetPrescalarValue(product);
    timer_.registers->ARR = product / (timer_.registers->PSC + 1);

    timer_.registers->EGR = bit::Set(timer_.registers->EGR, kUpdateGeneration);

  }

  void SetDutyCycle(float duty_cycle) override
  {
    // Clamp the duty cycle to make sure it's in the right range
    const float kClampedDutyCycle = std::clamp(duty_cycle, 0.0f, 1.0f);

    // The output signal is active for as long as the value in TIMx->CNT is
    // less than the value in TIMx->CCR1 and inactive for the opposite. So
    // setting TIMx->CCR to 80% of TIMx->ARR will result in the signal being
    // active only 20% of the time. This is why we do 1 - duty_cycle.
    timer_.registers->CCR1 = static_cast<uint16_t>((1 - kClampedDutyCycle) * timer_.registers->ARR);
  }

  float GetDutyCycle() override
  {
    return static_cast<float>(timer_.registers->CCR1 / timer_.registers->ARR);
  }

  void ModuleEnable(bool enable = true) override
  {
  }

  void ModuleInitialize() override
  {
    const auto port_enable = timer_.pin.GetPort() == 'A' ? PortEnable::kPortAClockEnable : PortEnable::kPortBClockEnable;
    RCC->APB2ENR = bit::Set(RCC->APB2ENR, port_enable);

    EnableTimer();

    // Set pin to alternative function to support PWM
    timer_.pin.ConfigureFunction(1);

    // Here we seet the PWM mode and the preload enable
    timer_.registers->CCMR1 = bit::Insert(timer_.registers->CCMR1, static_cast<uint16_t>(7), kOutputCompareMode);
    timer_.registers->CCMR1 = bit::Set(timer_.registers->CCMR1, kOutputComparePreloadEnable);

    // This updates information in registers
    // todo: make this better
    timer_.registers->EGR = bit::Set(timer_.registers->EGR, kUpdateGeneration);

    // Enables the output channel (This driver only uses channel 1)
    timer_.registers->CCER = bit::Set(timer_.registers->CCER, kOutputChannelEnable);

    // Setting auto reload preload enable and enables timer
    timer_.registers->CR1 = bit::Set(timer_.registers->CR1, kAutoReloadPreloadEnable);
    timer_.registers->CR1 = bit::Set(timer_.registers->CR1, kTimerEnable);
  }
    
 private:
  void EnableTimer()
  {
    switch (timer_.timer)
    {
    case 1:
      RCC->APB2ENR = bit::Set(RCC->APB2ENR, TimerClockEnable::kTimer1ClockEnable);
      break;
    case 2:
      RCC->APB1ENR = bit::Set(RCC->APB1ENR, TimerClockEnable::kTimer2ClockEnable);
      break;
    case 3:
      RCC->APB1ENR = bit::Set(RCC->APB1ENR, TimerClockEnable::kTimer3ClockEnable);
      break;
    case 4:
      RCC->APB1ENR = bit::Set(RCC->APB1ENR, TimerClockEnable::kTimer4ClockEnable);
      break;
    case 9:
      RCC->APB2ENR = bit::Set(RCC->APB2ENR, TimerClockEnable::kTimer9ClockEnable);
      break;
    case 10:
      RCC->APB2ENR = bit::Set(RCC->APB2ENR, TimerClockEnable::kTimer10ClockEnable);
      break;
    case 11:
      RCC->APB2ENR = bit::Set(RCC->APB2ENR, TimerClockEnable::kTimer11ClockEnable);
      break;
    case 14:
      RCC->APB1ENR = bit::Set(RCC->APB1ENR, TimerClockEnable::kTimer14ClockEnable);    
    default:
      break;
    }
  }

  uint16_t GetPrescalarValue(uint32_t product)
  {
    uint16_t prescalar = 0;
    uint16_t kMax16Bits = ~0;

    uint32_t arr;

    // To select a prescalar and divider value, we use this equation:
    //      (PSC+1)(ARR+1) = (EventTime)(ClkFreq)
    // product is the value (EventTime)(ClkFreq)
    //
    // We want ARR to be a larger value for a more precise duty cycle
    // so we increment prescalar until ARR in the value (PSC+1)(ARR+1)
    // is less than 2^16
    do {
      prescalar++;
      arr = product / prescalar;
    } while (arr > kMax16Bits);
    
    return prescalar - 1;
  }

  const Timer_t & timer_;
};
}  // namespace sjsu::stm32f10x