#include "L1_Peripheral/stm32f10x/pwm.hpp"
#include "utility/log.hpp"

int main()
{
  sjsu::stm32f10x::Pwm pwm(sjsu::stm32f10x::Pwm::Timer::kTimer4);
  pwm.Initialize();
  pwm.ConfigureFrequency(500_Hz);
  pwm.SetDutyCycle(0.35f);
  while(true);
  return 0;
}
