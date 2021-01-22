#include "L1_Peripheral/hardware_counter.hpp"
#include "L1_Peripheral/lpc40xx/gpio.hpp"
#include "L2_HAL/sensors/signal/frequency_counter.hpp"
#include "utility/log.hpp"
#include "utility/time.hpp"

int main()
{
  sjsu::LogInfo("Gpio Frequency Counter Application Starting...");

  // Create a GPIO to read from. This can be any GPIO that supports GPIO
  // interrupts. On the LPC40xx, that would be pins on port 0 and port 2.
  //
  // Using the SJTwo button 3 for the demo, but feel free to change this to some
  // other gpio port and pin number. The best option is a pin connected to a
  // function generator or PWM signal to verify that the frequency counter is
  // operating as expected.
  sjsu::lpc40xx::Gpio & gpio = sjsu::lpc40xx::GetGpio<0, 29>();

  // Pass the GPIO above into the gpio counter to be controlled by it.
  // The second parameter allows you to change which event triggers a count.
  // In this case, we want to trigger on rising and falling edges of the pin.
  // Note that this
  sjsu::GpioCounter counter(gpio, sjsu::Gpio::Edge::kRising);

  // Pass the gpio HardwareCounter object to the frequency counter object to be
  // used to calculate the frequency of the signal on that pin.
  sjsu::FrequencyCounter frequency_counter(&counter);

  // Initialize the hardware.
  frequency_counter.Initialize();

  sjsu::LogInfo(
      "With every rising edge of pin P%u.%u, the counter will increase and its "
      "value will be printed to stdout.",
      gpio.GetPin().GetPort(),
      gpio.GetPin().GetPin());

  sjsu::LogInfo(
      "The more rising edges per second on that pin will result in a higher "
      "frequency that is calculated by the frequency counter.");

  while (true)
  {
    // Using printf here to reduce the latency between each
    auto frequency = frequency_counter.GetFrequency();
    sjsu::LogInfo("Freq = %f\n", frequency.to<double>());
    sjsu::Delay(1000ms);
  }
  return 0;
}
