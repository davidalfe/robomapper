#include <chrono>
#include <thread>

#include "UltrasonicSensor.hpp"

namespace mapper
{
namespace see
{

uint16_t UltrasonicSensor::read()
{
  digitalWrite(triggerPin_, HIGH);
  std::this_thread::sleep_for(std::chrono::microseconds(TRIGGER_DURATION_US));
  digitalWrite(triggerPin_, LOW);

  while(digitalRead(echoPin_) == LOW);

  auto t1 = std::chrono::system_clock::now();
  while(digitalRead(echoPin_) == HIGH);
  auto t2 = std::chrono::system_clock::now();

  auto pulseWidth = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);

  double cm = pulseWidth.count() / 58.0;

  if(cm < MIN_DISTANCE_CM || cm > MAX_DISTANCE_CM)
  {
    cm = 0.0;
  }

  std::this_thread::sleep_for(std::chrono::microseconds(WAIT_AFTER_MEASURING_US));

  return static_cast<uint16_t>(cm);
}

} // namespace see
} // namespace mapper
