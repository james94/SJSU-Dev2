#pragma once



namespace sjsu
{
namespace stm32f10x
{
/// An implementation of the sjsu::Pwm peripheral for the stm32f10x family of
/// microcontrollers.
///
/// @warning - Changing the frequency of 1 PWM channel, changes the frequency
/// for all other PWM channels.
class Pwm final : public sjsu::Pwm
{

}
}  // namespace stm32f10x
}  // namespace sjsu