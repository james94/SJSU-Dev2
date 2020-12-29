#pragma once

#include "L1_Peripheral/lpc40xx/gpio.hpp"

namespace sjsu
{
namespace lpc17xx
{
using sjsu::lpc40xx::Gpio;

template<uint8_t port, uint8_t pin>
auto & GetGpio()
{
  static Gpio gpio(port, pin);
  return gpio;
}
}  // namespace lpc17xx
}  // namespace sjsu
