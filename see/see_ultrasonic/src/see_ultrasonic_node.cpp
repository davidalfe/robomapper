#include <ros/ros.h>

#include <wiringPi.h>

#include "see_ultrasonic/UltrasonicEvent.h"

#include "Kalman.hpp"
#include "UltrasonicSensor.hpp"

// TODO: Move this to configuration arguments
namespace {
constexpr uint8_t PUBLISH_FREQUENCY = 10;

constexpr uint8_t SENSOR_CENTER_TRG_PIN = 0;
constexpr uint8_t SENSOR_CENTER_ECH_PIN = 2;
constexpr uint8_t SENSOR_RIGHT_TRG_PIN = 1;
constexpr uint8_t SENSOR_RIGHT_ECH_PIN = 4;
constexpr uint8_t SENSOR_LEFT_TRG_PIN = 21;
constexpr uint8_t SENSOR_LEFT_ECH_PIN = 22;

const std::string SENSOR_BASE_TOPIC{"/sensors/ultrasonic"};
const std::string SENSOR_RAW_TOPIC{SENSOR_BASE_TOPIC + "/raw"};
const std::string SENSOR_FILTERED_TOPIC{SENSOR_BASE_TOPIC + "/filtered"};
constexpr uint8_t SENSOR_MAX_QUEUE{10};

constexpr double KALMAN_INITIAL_PROCESS_NOISE_COVARIANCE = 0.125;
constexpr double KALMAN_INITIAL_MEASUREMENT_NOISE_COVARIANCE = 32;
constexpr double KALMAN_INITIAL_ESTIMATION_ERROR_COVARIANCE = 1023;

mapper::see::Kalman kalmanCenter{
  KALMAN_INITIAL_PROCESS_NOISE_COVARIANCE,
  KALMAN_INITIAL_MEASUREMENT_NOISE_COVARIANCE,
  KALMAN_INITIAL_ESTIMATION_ERROR_COVARIANCE,
  0
};

mapper::see::Kalman kalmanRight{
  KALMAN_INITIAL_PROCESS_NOISE_COVARIANCE,
  KALMAN_INITIAL_MEASUREMENT_NOISE_COVARIANCE,
  KALMAN_INITIAL_ESTIMATION_ERROR_COVARIANCE,
  0
};

mapper::see::Kalman kalmanLeft{
  KALMAN_INITIAL_PROCESS_NOISE_COVARIANCE,
  KALMAN_INITIAL_MEASUREMENT_NOISE_COVARIANCE,
  KALMAN_INITIAL_ESTIMATION_ERROR_COVARIANCE,
  0
};

/*
 * Returns the filtered value applying the passed Kalman filter. In case that the
 * sensor returns an erroneus value (0), the previous value is considered as ref.
 */
double getFilteredValue(mapper::see::Kalman& filter, double value) {
  return (value != 0)
      ? filter.getFilteredValue(value)
          : filter.getFilteredValue(filter.getPreviousValue());
}

see_ultrasonic::UltrasonicEvent filter(const see_ultrasonic::UltrasonicEvent& event)
{
  see_ultrasonic::UltrasonicEvent filteredEvent;

  filteredEvent.center = getFilteredValue(kalmanCenter, event.center);
  filteredEvent.right = getFilteredValue(kalmanRight, event.right);
  filteredEvent.left = getFilteredValue(kalmanLeft, event.left);

  return filteredEvent;
}


} // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "see_ultrasonic");
  wiringPiSetup();

  mapper::see::UltrasonicSensor sensorCenter{SENSOR_CENTER_TRG_PIN, SENSOR_CENTER_ECH_PIN};
  mapper::see::UltrasonicSensor sensorRight{SENSOR_RIGHT_TRG_PIN, SENSOR_RIGHT_ECH_PIN};
  mapper::see::UltrasonicSensor sensorLeft{SENSOR_LEFT_TRG_PIN, SENSOR_LEFT_ECH_PIN};

  ros::NodeHandle node;

  ros::Publisher sensorRawPublisher =
      node.advertise<see_ultrasonic::UltrasonicEvent>(SENSOR_RAW_TOPIC, SENSOR_MAX_QUEUE);
  ros::Publisher sensorFilteredPublisher =
      node.advertise<see_ultrasonic::UltrasonicEvent>(SENSOR_FILTERED_TOPIC, SENSOR_MAX_QUEUE);


  ros::Rate loop_rate(PUBLISH_FREQUENCY);

  while(ros::ok())
  {
    see_ultrasonic::UltrasonicEvent event;

    event.center = sensorCenter.read();
    event.right = sensorRight.read();
    event.left = sensorLeft.read();

    auto filteredEvent = filter(event);

    sensorRawPublisher.publish(event);
    sensorFilteredPublisher.publish(filteredEvent);

    ros::spinOnce();

    loop_rate.sleep();
  }
}
