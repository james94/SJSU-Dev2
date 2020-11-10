#include "L1_Peripheral/stm32f10x/pwm.hpp"
#include "utility/log.hpp"

int main()
{
  sjsu::stm32f10x::Pwm pwm;
  pwm.ModuleInitialize();
  return 0;
}
