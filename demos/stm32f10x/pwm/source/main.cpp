#include "L1_Peripheral/stm32f10x/pwm.hpp"
#include "utility/log.hpp"

int main()
{
  sjsu::stm32f10x::Pwm pwm(sjsu::stm32f10x::Pwm::Timer::kTimer1);
  pwm.Initialize();
  pwm.ConfigureFrequency(1_kHz);
  pwm.SetDutyCycle(0.2f);
  while(true);
  return 0;
}
