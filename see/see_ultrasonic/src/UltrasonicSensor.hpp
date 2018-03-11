#ifndef __mapper_see_UltrasonicSensor__
#define __mapper_see_UltrasonicSensor__

#include <cstdint>

#include <wiringPi.h>

namespace mapper
{
namespace see
{

constexpr uint8_t MIN_DISTANCE_CM = 2;
constexpr uint16_t MAX_DISTANCE_CM = 400;

constexpr uint8_t TRIGGER_DURATION_US = 10;
constexpr uint8_t WAIT_AFTER_MEASURING_US = 60;


class UltrasonicSensor
{
public:
  UltrasonicSensor(const uint16_t triggerPin, const uint16_t echoPin)
    : triggerPin_{triggerPin}, echoPin_{echoPin}
  {
    pinMode(triggerPin_, OUTPUT);
    pinMode(echoPin_, INPUT);
  }

  uint16_t read();

private:
    const uint16_t triggerPin_;
    const uint16_t echoPin_;
};

} // namespace see
} // namespace mapper

#endif // __mapper_see_UltrasonicSensor__
