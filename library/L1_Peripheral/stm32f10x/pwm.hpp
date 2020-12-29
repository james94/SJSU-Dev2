#pragma once

#include "L0_Platform/stm32f10x/stm32f10x.h"
#include "L1_Peripheral/pwm.hpp"
#include "L1_Peripheral/stm32f10x/pin.hpp"
#include "L1_Peripheral/stm32f10x/system_controller.hpp"
#include "utility/log.hpp"

namespace sjsu::stm32f10x
{
/// An implementation of the sjsu::Pwm peripheral for the stm32f10x family of
/// microcontrollers.
class Pwm final : public sjsu::Pwm
{
 public:
  // We use pins spread out across Timers 1 - 5
  struct Channel_t
  {
    // Reference to the pin
    sjsu::Pin & pin;

    // Holds the address to the corresponding Timer peripheral
    TIM_TypeDef * registers;

    // Holds the channel (1 - 4) which corresponds to the pin
    uint8_t channel;

    // Peripheral ID for the timer
    sjsu::SystemController::ResourceID id;
  };

  /// Constructor for a STM32F10x PWM channel.
  ///
  /// @param timer - Reference to a const timer description for this
  ///        instance of the PWM driver.
  explicit constexpr Pwm(const Channel_t & timer) : pwm_pin_(timer) {}

  void ConfigureFrequency(units::frequency::hertz_t frequency) override
  {
    static constexpr auto kUpdateGeneration = bit::MaskFromRange(0);

    auto & system                   = SystemController::GetPlatformController();
    const auto kPeripheralFrequency = system.GetClockRate(pwm_pin_.id);

    auto period      = 1 / frequency;
    uint32_t product = kPeripheralFrequency * period;

    pwm_pin_.registers->PSC = GetPrescalarValue(product);
    pwm_pin_.registers->ARR =
        static_cast<uint16_t>(product / (pwm_pin_.registers->PSC + 1));

    bit::Register(&pwm_pin_.registers->EGR).Set(kUpdateGeneration).Save();
  }

  void SetDutyCycle(float duty_cycle) override
  {
    // Clamp the duty cycle to make sure it's in the right range
    const float kClampedDutyCycle = std::clamp(duty_cycle, 0.0f, 1.0f);

    switch (pwm_pin_.channel)
    {
      case 1:
        pwm_pin_.registers->CCR1 =
            static_cast<uint16_t>(kClampedDutyCycle * pwm_pin_.registers->ARR);
        break;

      case 2:
        pwm_pin_.registers->CCR2 =
            static_cast<uint16_t>(kClampedDutyCycle * pwm_pin_.registers->ARR);
        break;

      case 3:
        pwm_pin_.registers->CCR3 =
            static_cast<uint16_t>(kClampedDutyCycle * pwm_pin_.registers->ARR);
        break;

      case 4:
        pwm_pin_.registers->CCR4 =
            static_cast<uint16_t>(kClampedDutyCycle * pwm_pin_.registers->ARR);
        break;
    }
  }

  float GetDutyCycle() override
  {
    float duty_cycle;

    switch (pwm_pin_.channel)
    {
      case 1:
        duty_cycle = static_cast<float>(pwm_pin_.registers->CCR1 /
                                        pwm_pin_.registers->ARR);
        break;
      case 2:
        duty_cycle = static_cast<float>(pwm_pin_.registers->CCR2 /
                                        pwm_pin_.registers->ARR);
        break;
      case 3:
        duty_cycle = static_cast<float>(pwm_pin_.registers->CCR3 /
                                        pwm_pin_.registers->ARR);
        break;
      case 4:
        duty_cycle = static_cast<float>(pwm_pin_.registers->CCR4 /
                                        pwm_pin_.registers->ARR);
        break;
    }

    return duty_cycle;
  }

  void ModuleEnable(bool enable = true) override
  {
    // Preload enable
    static constexpr auto kPreloadEnableLow  = bit::MaskFromRange(3);
    static constexpr auto kPreloadEnableHigh = bit::MaskFromRange(11);

    // Output channel enable
    static constexpr auto kChannel1OutputEnable = bit::MaskFromRange(0);
    static constexpr auto kChannel2OutputEnable = bit::MaskFromRange(4);
    static constexpr auto kChannel3OutputEnable = bit::MaskFromRange(8);
    static constexpr auto kChannel4OutputEnable = bit::MaskFromRange(12);

    // Main Output Enable (Timer 1 only)
    static constexpr auto kMainOutputEnable = bit::MaskFromRange(15);

    // Timer enable
    static constexpr auto kTimerEnable = bit::MaskFromRange(0);

    if (enable)
    {
      switch (pwm_pin_.channel)
      {
        case 1:
          bit::Register(&pwm_pin_.registers->CCMR1)
              .Set(kPreloadEnableLow)
              .Save();
          bit::Register(&pwm_pin_.registers->CCER)
              .Set(kChannel1OutputEnable)
              .Save();
          break;
        case 2:
          bit::Register(&pwm_pin_.registers->CCMR1)
              .Set(kPreloadEnableHigh)
              .Save();
          bit::Register(&pwm_pin_.registers->CCER)
              .Set(kChannel2OutputEnable)
              .Save();
          break;
        case 3:
          bit::Register(&pwm_pin_.registers->CCMR2)
              .Set(kPreloadEnableLow)
              .Save();
          bit::Register(&pwm_pin_.registers->CCER)
              .Set(kChannel3OutputEnable)
              .Save();
          break;
        case 4:
          bit::Register(&pwm_pin_.registers->CCMR2)
              .Set(kPreloadEnableHigh)
              .Save();
          bit::Register(&pwm_pin_.registers->CCER)
              .Set(kChannel4OutputEnable)
              .Save();
          break;
      }

      if (pwm_pin_.id == SystemController::Peripherals::kTimer1)
      {
        bit::Register(&pwm_pin_.registers->BDTR).Set(kMainOutputEnable).Save();
      }

      bit::Register(&pwm_pin_.registers->CR1).Set(kTimerEnable).Save();
    }
    else
    {
      // Preload enable
      switch (pwm_pin_.channel)
      {
        case 1:
          bit::Register(&pwm_pin_.registers->CCMR1)
              .Clear(kPreloadEnableLow)
              .Save();
          bit::Register(&pwm_pin_.registers->CCER)
              .Clear(kChannel1OutputEnable)
              .Save();
          break;
        case 2:
          bit::Register(&pwm_pin_.registers->CCMR1)
              .Clear(kPreloadEnableHigh)
              .Save();
          bit::Register(&pwm_pin_.registers->CCER)
              .Clear(kChannel2OutputEnable)
              .Save();
          break;
        case 3:
          bit::Register(&pwm_pin_.registers->CCMR2)
              .Clear(kPreloadEnableLow)
              .Save();
          bit::Register(&pwm_pin_.registers->CCER)
              .Clear(kChannel3OutputEnable)
              .Save();
          break;
        case 4:
          bit::Register(&pwm_pin_.registers->CCMR2)
              .Clear(kPreloadEnableHigh)
              .Save();
          bit::Register(&pwm_pin_.registers->CCER)
              .Clear(kChannel4OutputEnable)
              .Save();
          break;
      }

      if (pwm_pin_.id == SystemController::Peripherals::kTimer1)
      {
        bit::Register(&pwm_pin_.registers->BDTR)
            .Clear(kMainOutputEnable)
            .Save();
      }

      bit::Register(&pwm_pin_.registers->CR1).Clear(kTimerEnable).Save();
    }
  }

  void ModuleInitialize() override
  {
    SystemController::GetPlatformController().PowerUpPeripheral(pwm_pin_.id);

    // Set pin to alternative function to support PWM
    pwm_pin_.pin.ConfigureFunction(1);

    // We only use PWM mode 1 in this driver. The high and low are because
    // different channels have different locations where the mode is set
    static constexpr auto kPwmModeLow  = bit::MaskFromRange(4, 6);
    static constexpr auto kPwmModeHigh = bit::MaskFromRange(12, 14);
    static constexpr uint8_t kPwmMode1 = 0b110;

    switch (pwm_pin_.channel)
    {
      case 1:
        bit::Register(&pwm_pin_.registers->CCMR1)
            .Insert(kPwmMode1, kPwmModeLow)
            .Save();
        break;
      case 2:
        bit::Register(&pwm_pin_.registers->CCMR1)
            .Insert(kPwmMode1, kPwmModeHigh)
            .Save();
        break;
      case 3:
        bit::Register(&pwm_pin_.registers->CCMR2)
            .Insert(kPwmMode1, kPwmModeLow)
            .Save();
        break;
      case 4:
        bit::Register(&pwm_pin_.registers->CCMR2)
            .Insert(kPwmMode1, kPwmModeHigh)
            .Save();
        break;
    }
  }

 private:
  uint16_t GetPrescalarValue(uint32_t product)
  {
    uint16_t prescalar    = 0;
    uint16_t k_max16_bits = ~0;

    uint32_t arr;

    // To select a prescalar and divider value, we use this equation:
    //      (PSC+1)(ARR+1) = (EventTime)(ClkFreq)
    // product is the value (EventTime)(ClkFreq)
    //
    // We want ARR to be a larger value for a more precise duty cycle
    // so we increment prescalar until ARR in the value (PSC+1)(ARR+1)
    // is less than 2^16
    do
    {
      prescalar++;
      arr = product / prescalar;
    } while (arr > k_max16_bits);

    return --prescalar;
  }

  const Channel_t & pwm_pin_;
};

template <int peripheral>
struct GetPwm
{
  static_assert(1 <= peripheral && peripheral <= 4,
                "Peripheral template parameter must be between 1 and 4 as the "
                "stm32f10x uses timers 1 to 4 for PWM generation.");

  template <int channel>
  static Pwm & Channel()
  {
    static_assert(0 <= channel && channel <= 4,
                  "stm31f10x only supports PWM channels from 0 to 4.");

    constexpr std::pair<int, int> kPinValues = GetPinBasedOnChannel<channel>();
    static Pin pwm_pin(kPinValues.first, kPinValues.second);
    static const Pwm::Channel_t kChannelInfo = {
      .pin       = pwm_pin,
      .registers = GetRegisters(),
      .channel   = channel,
      .id        = GetPeripheralID(),
    };
    static Pwm pwm_channel(kChannelInfo);

    return pwm_channel;
  }

  template <int port, int pin>
  static Pwm & FromPin()
  {
    if constexpr (GetChannelBasedOnPin<port, pin>() != -1)
    {
      return Channel<GetChannelBasedOnPin<port, pin>()>();
    }
    else
    {
      static_assert(
          InvalidOption<port, pin>,
          "stm32f10x TIMER1 only supports PWM pins: PA8, PA9, PA10, PA11. "
          "stm32f10x TIMER2 only supports PWM pins: PA0, PA1, PA2, PA3. "
          "stm32f10x TIMER3 only supports PWM pins: PA6, PA7, PB0, PB1. "
          "stm32f10x TIMER4 only supports PWM pins: PB6, PB7, PB8, PB9.");
      // NOTE: this is simply here to appease the compiler. If this block is
      // hit, this code will not compile and thus Channel<0>() will never be
      // used in this way.
      return Channel<0>();
    }
  }

 private:
  constexpr static auto GetPinMap()
  {
    std::array<std::array<std::pair<int, int>, 4>, 4> pins = {
      std::array<std::pair<int, int>, 4>{
          std::make_pair('A', 8),
          std::make_pair('A', 9),
          std::make_pair('A', 10),
          std::make_pair('A', 11),  // TIMER 1
      },
      std::array<std::pair<int, int>, 4>{
          std::make_pair('A', 0),
          std::make_pair('A', 1),
          std::make_pair('A', 2),
          std::make_pair('A', 3),  // TIMER 2
      },
      std::array<std::pair<int, int>, 4>{
          std::make_pair('A', 6),
          std::make_pair('A', 7),
          std::make_pair('B', 0),
          std::make_pair('B', 1),  // TIMER 3
      },
      std::array<std::pair<int, int>, 4>{
          std::make_pair('B', 6),
          std::make_pair('B', 7),
          std::make_pair('B', 8),
          std::make_pair('B', 9),  // TIMER 4
      },
    };

    return pins[peripheral - 1];
  }

  constexpr static auto GetRegisters()
  {
    std::array<TIM_TypeDef *, 4> registers = { TIM1, TIM2, TIM3, TIM4 };
    return registers[peripheral];
  }

  constexpr static auto GetPeripheralID()
  {
    std::array<SystemController::ResourceID, 4> ids = {
      SystemController::Peripherals::kTimer1,
      SystemController::Peripherals::kTimer2,
      SystemController::Peripherals::kTimer3,
      SystemController::Peripherals::kTimer4,
    };
    return ids[peripheral];
  }

  template <int channel>
  constexpr static auto & GetPinBasedOnChannel()
  {
    return GetPinMap()[channel];
  }

  template <int port, int pin>
  constexpr static int GetChannelBasedOnPin()
  {
    for (size_t i = 0; i < GetPinMap().size(); i++)
    {
      if (port == GetPinMap()[i].first && pin == GetPinMap()[i].second)
      {
        return i;
      }
    }
    return -1;
  }
};
}  // namespace sjsu::stm32f10x
