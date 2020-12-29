#pragma once

#include <cstdint>

#include "L0_Platform/lpc40xx/LPC40xx.h"
#include "L1_Peripheral/adc.hpp"
#include "L1_Peripheral/lpc40xx/pin.hpp"
#include "L1_Peripheral/lpc40xx/system_controller.hpp"
#include "utility/bit.hpp"
#include "utility/error_handling.hpp"
#include "utility/log.hpp"
#include "utility/units.hpp"

namespace sjsu
{
namespace lpc40xx
{
/// ADC driver for the LPC40xx and LPC17xx microcontrollers.
class Adc final : public sjsu::Adc
{
 public:
  /// Namespace containing the bitmask objects that are used to manipulate the
  /// lpc40xx ADC Control register.
  struct Control  // NOLINT
  {
    /// In burst mode, sets the ADC channels to be automatically converted.
    /// It bit position represents 1 channel with this 8 channel ADC.
    /// In software mode, this should hold only a single 1 for the single
    /// channel to be converted.
    static constexpr bit::Mask kChannelSelect = bit::MaskFromRange(0, 7);

    /// Sets the channel's clock divider. Potentially saving power if clock is
    /// reduced further.
    static constexpr bit::Mask kClockDivider = bit::MaskFromRange(8, 15);

    /// Enable Burst Mode for the ADC. See BurstMode() method of this class to
    /// learn more about what it is and how it works.
    static constexpr bit::Mask kBurstEnable = bit::MaskFromRange(16);

    /// Power on the ADC
    static constexpr bit::Mask kPowerEnable = bit::MaskFromRange(21);

    /// In order to start a conversion a start code must be inserted into this
    /// bit location.
    static constexpr bit::Mask kStartCode = bit::MaskFromRange(24, 26);

    /// Not used in this driver, but allows the use of an external pins to
    /// trigger a conversion. This flag indicates if rising or falling edges
    /// trigger the conversion.
    /// 1 = falling, 0 = rising.
    static constexpr bit::Mask kStartEdge = bit::MaskFromRange(27);
  };

  /// Namespace containing the bitmask objects that are used to manipulate the
  /// lpc40xx ADC Global Data register.
  struct DataRegister  // NOLINT
  {
    /// Result mask holds the latest result from the last ADC that was converted
    static constexpr bit::Mask kResult = bit::MaskFromRange(4, 15);

    /// Converted channel mask indicates which channel was converted in the
    /// latest conversion.
    static constexpr bit::Mask kConvertedChannel = bit::MaskFromRange(24, 26);

    /// Holds whether or not the ADC overran its conversion.
    static constexpr bit::Mask kOverrun = bit::MaskFromRange(30);

    /// Indicates when the ADC conversion is complete.
    static constexpr bit::Mask kDone = bit::MaskFromRange(31);
  };

  /// Structure that defines a channel's pin, pin's function code and channel
  /// number.
  ///
  /// Usage:
  ///
  /// ```
  /// sjsu::lpc40xx::Pin adc_pin(/* adc port number */, /* adc pin number */);
  /// const sjsu::lpc40xx::Adc::Channel_t kCustomChannelX = {
  ///   .adc_pin      = adc_pin,
  ///   .channel      = /* insert correct channel here */,
  ///   .pin_function = 0b101,
  /// };
  /// sjsu::lpc40xx::Adc channelX_adc(kCustomChannelX);
  /// ```
  struct Channel_t
  {
    /// Reference to the pin associated with the adc channel.
    sjsu::Pin & adc_pin;

    /// Channel number
    uint8_t channel;

    /// Which function code selects the ADC function for the pin, specified in
    /// the adc_pin field.
    uint8_t pin_function;
  };

  /// The default and highest frequency that the ADC can operate at.
  static constexpr units::frequency::hertz_t kClockFrequency = 1_MHz;

  /// A pointer holding the address to the LPC40xx ADC peripheral.
  /// This variable is a dependency injection point for unit testing thus it is
  /// public and mutable. This is needed to perform the "test by side effect"
  /// technique for this class.
  inline static LPC_ADC_TypeDef * adc_base = LPC_ADC;

  /// Number of active bits of the ADC. The ADC is a 12-bit ADC meaning that the
  /// largest value it can have is 2^12 = 4096
  static constexpr uint8_t kActiveBits = 12;

  /// @param channel Passed channel descriptor object. See Channel_t and
  ///        Channel documentation for more details about how to use this.
  /// @param reference_voltage The ADC reference voltage in microvolts.
  explicit constexpr Adc(const Channel_t & channel,
                         units::voltage::microvolt_t reference_voltage = 3.3_V)
      : channel_(channel), kReferenceVoltage(reference_voltage)
  {
  }

  void ModuleInitialize() override
  {
    sjsu::SystemController::GetPlatformController().PowerUpPeripheral(
        sjsu::lpc40xx::SystemController::Peripherals::kAdc);

    // It is required for proper operation of analog pins for the LPC40xx that
    // the pins be floating.
    channel_.adc_pin.Initialize();
    channel_.adc_pin.ConfigureFunction(channel_.pin_function);
    channel_.adc_pin.ConfigureFloating();
    channel_.adc_pin.ConfigureAsAnalogMode(true);
    channel_.adc_pin.Enable();

    const auto kPeripheralFrequency =
        sjsu::SystemController::GetPlatformController().GetClockRate(
            sjsu::lpc40xx::SystemController::Peripherals::kAdc);

    const uint32_t kClockDivider = kPeripheralFrequency / kClockFrequency;

    bit::Register(&adc_base->CR)
        .Insert(kClockDivider, Control::kClockDivider)
        .Set(Control::kBurstEnable)
        .Set(Control::kPowerEnable)
        .Save();
  }

  void ModuleEnable(bool enable = true) override
  {
    if (enable)
    {
      bit::Register(&adc_base->CR)
          .Set(bit::MaskFromRange(channel_.channel))
          .Save();
    }
    else
    {
      bit::Register(&adc_base->CR)
          .Clear(bit::MaskFromRange(channel_.channel))
          .Save();
    }
  }

  uint32_t Read() override
  {
    return bit::Extract(adc_base->DR[channel_.channel], DataRegister::kResult);
  }

  uint8_t GetActiveBits() override
  {
    return kActiveBits;
  }

  units::voltage::microvolt_t ReferenceVoltage() override
  {
    return kReferenceVoltage;
  }

 private:
  const Channel_t & channel_;
  const units::voltage::microvolt_t kReferenceVoltage;
};

template <int peripheral>
struct GetAdc
{
  static_assert(peripheral == 0,
                "LPC40xx only has 1 ADC peripheral block, thus peripheral can "
                "only be the value 0.");

  enum AdcMode : uint8_t
  {
    kCh0123Pins = 0b001,
    kCh4567Pins = 0b011
  };

  template <int channel>
  static auto & GetChannel()
  {
    static_assert(0 <= channel && channel <= 7,
                  "LPC40xx only supports ADC channels from ADC0 to ADC7.");

    if constexpr (channel == 0)
    {
      return adc0;
    }
    else if constexpr (channel == 1)
    {
      return adc1;
    }
    else if constexpr (channel == 2)
    {
      return adc2;
    }
    else if constexpr (channel == 3)
    {
      return adc3;
    }
    else if constexpr (channel == 4)
    {
      return adc4;
    }
    else if constexpr (channel == 5)
    {
      return adc5;
    }
    else if constexpr (channel == 6)
    {
      return adc6;
    }
    else if constexpr (channel == 7)
    {
      return adc7;
    }
    else
    {
      return GetInactive<Adc>();
    }
  }

  template <int port, int pin>
  static auto & GetChannelByPin()
  {
    if constexpr (port == 0 && pin == 23)
    {
      return adc0;
    }
    else if constexpr (port == 0 && pin == 24)
    {
      return adc1;
    }
    else if constexpr (port == 0 && pin == 25)
    {
      return adc2;
    }
    else if constexpr (port == 0 && pin == 26)
    {
      return adc3;
    }
    else if constexpr (port == 1 && pin == 30)
    {
      return adc4;
    }
    else if constexpr (port == 1 && pin == 31)
    {
      return adc5;
    }
    else if constexpr (port == 0 && pin == 12)
    {
      return adc6;
    }
    else if constexpr (port == 0 && pin == 13)
    {
      return adc7;
    }
    else
    {
      static_assert(InvalidOption<port, pin>::value,
                    "LPC40xx only supports ADC pins P0[23], P0[24], P0[25], "
                    "P0[26], P1[30], P1[31], P0[12], P0[13].");
      return GetInactive<Adc>();
    }
  }

 private:
  inline static auto adc_pin_channel0 = Pin(0, 23);
  /// Predefined channel information for channel 0.
  /// Pass this to the lpc17xx::Adc class to utilize adc channel0.
  inline static const Adc::Channel_t kChannel0 = {
    .adc_pin      = adc_pin_channel0,
    .channel      = 0,
    .pin_function = AdcMode::kCh0123Pins,
  };

  inline static auto adc_pin_channel1 = Pin(0, 24);
  /// Predefined channel information for channel 1.
  /// Pass this to the lpc17xx::Adc class to utilize adc channel1.
  inline static const Adc::Channel_t kChannel1 = {
    .adc_pin      = adc_pin_channel1,
    .channel      = 1,
    .pin_function = AdcMode::kCh0123Pins,
  };

  inline static auto adc_pin_channel2 = Pin(0, 25);
  /// Predefined channel information for channel 2.
  /// Pass this to the lpc17xx::Adc class to utilize adc channel2.
  inline static const Adc::Channel_t kChannel2 = {
    .adc_pin      = adc_pin_channel2,
    .channel      = 2,
    .pin_function = AdcMode::kCh0123Pins,
  };

  inline static auto adc_pin_channel3 = Pin(0, 26);
  /// Predefined channel information for channel 3.
  /// Pass this to the lpc17xx::Adc class to utilize adc channel3.
  inline static const Adc::Channel_t kChannel3 = {
    .adc_pin      = adc_pin_channel3,
    .channel      = 3,
    .pin_function = AdcMode::kCh0123Pins,
  };

  inline static auto adc_pin_channel4 = Pin(1, 30);
  /// Predefined channel information for channel 4.
  /// Pass this to the lpc17xx::Adc class to utilize adc channel4.
  inline static const Adc::Channel_t kChannel4 = {
    .adc_pin      = adc_pin_channel4,
    .channel      = 4,
    .pin_function = AdcMode::kCh4567Pins,
  };

  inline static auto adc_pin_channel5 = Pin(1, 31);
  /// Predefined channel information for channel 5.
  /// Pass this to the lpc17xx::Adc class to utilize adc channel5.
  inline static const Adc::Channel_t kChannel5 = {
    .adc_pin      = adc_pin_channel5,
    .channel      = 5,
    .pin_function = AdcMode::kCh4567Pins,
  };

  inline static auto adc_pin_channel6 = Pin(0, 12);
  /// Predefined channel information for channel 6.
  /// Pass this to the lpc17xx::Adc class to utilize adc channel6.
  inline static const Adc::Channel_t kChannel6 = {
    .adc_pin      = adc_pin_channel6,
    .channel      = 6,
    .pin_function = AdcMode::kCh4567Pins,
  };

  inline static auto adc_pin_channel7 = Pin(0, 13);
  /// Predefined channel information for channel 7.
  /// Pass this to the lpc17xx::Adc class to utilize adc channel7.
  inline static const Adc::Channel_t kChannel7 = {
    .adc_pin      = adc_pin_channel7,
    .channel      = 7,
    .pin_function = AdcMode::kCh4567Pins,
  };

  inline static Adc adc0 = Adc(kChannel0);
  inline static Adc adc1 = Adc(kChannel1);
  inline static Adc adc2 = Adc(kChannel2);
  inline static Adc adc3 = Adc(kChannel3);
  inline static Adc adc4 = Adc(kChannel4);
  inline static Adc adc5 = Adc(kChannel5);
  inline static Adc adc6 = Adc(kChannel6);
  inline static Adc adc7 = Adc(kChannel7);
};
template <int peripheral>
struct GetAdc2
{
  static_assert(peripheral == 0,
                "LPC40xx only has 1 ADC peripheral block, thus peripheral can "
                "only be the value 0.");

  template <int channel>
  static Adc & Channel()
  {
    static_assert(0 <= channel && channel <= 7,
                  "LPC40xx only supports ADC channels from ADC0 to ADC7.");

    constexpr std::pair<int, int> kPinValues = GetPinBasedOnChannel<channel>();

    static Pin adc_pin(kPinValues.first, kPinValues.second);
    static const Adc::Channel_t kChannelInfo = {
      .adc_pin      = adc_pin,
      .channel      = channel,
      .pin_function = GetPinFunctionCodeBasedOnChannel<channel>(),
    };
    static Adc adc_channel(kChannelInfo);

    return adc_channel;
  }

  template <int port, int pin>
  static Adc & FromPin()
  {
    static_assert(GetChannelBasedOnPin<port, pin>() != -1,
                  "LPC40xx only supports ADC pins P0[23], P0[24], P0[25], "
                  "P0[26], P1[30], P1[31], P0[12], P0[13].");
    return GetChannel<GetChannelBasedOnPin<port, pin>()>();
  }

 private:
  enum AdcMode : uint8_t
  {
    kCh0123Pins = 0b001,
    kCh4567Pins = 0b011
  };

  constexpr static auto GetPinMap()
  {
    std::array<std::pair<int, int>, 8> pins = {
      std::make_pair(0, 23),  // channel0
      std::make_pair(0, 24),  // channel1
      std::make_pair(0, 25),  // channel2
      std::make_pair(0, 26),  // channel3
      std::make_pair(1, 30),  // channel4
      std::make_pair(1, 31),  // channel5
      std::make_pair(0, 12),  // channel6
      std::make_pair(0, 13),  // channel7
    };

    return pins;
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

  template <int channel>
  constexpr static auto GetPinFunctionCodeBasedOnChannel()
  {
    if constexpr (0 <= channel && channel <= 3)
    {
      return AdcMode::kCh0123Pins;
    }
    else
    {
      return AdcMode::kCh4567Pins;
    }
  }
};
}  // namespace lpc40xx
}  // namespace sjsu
