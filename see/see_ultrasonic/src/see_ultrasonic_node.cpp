#include <ros/ros.h>

#include <wiringPi.h>

#include "see_ultrasonic/UltrasonicEvent.h"

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

const std::string SENSOR_TOPIC{"/sensors/ultrasonic"};
constexpr uint8_t SENSOR_MAX_QUEUE{10};
} // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "see_ultrasonic");
  wiringPiSetup();

  mapper::see::UltrasonicSensor sensorCenter{SENSOR_CENTER_TRG_PIN, SENSOR_CENTER_ECH_PIN};
  mapper::see::UltrasonicSensor sensorRight{SENSOR_RIGHT_TRG_PIN, SENSOR_RIGHT_ECH_PIN};
  mapper::see::UltrasonicSensor sensorLeft{SENSOR_LEFT_TRG_PIN, SENSOR_LEFT_ECH_PIN};

  ros::NodeHandle node;

  ros::Publisher sensorPublisher =
      node.advertise<see_ultrasonic::UltrasonicEvent>(SENSOR_TOPIC, SENSOR_MAX_QUEUE);

  ros::Rate loop_rate(PUBLISH_FREQUENCY);

  while(ros::ok())
  {
    see_ultrasonic::UltrasonicEvent event;

    event.center = sensorCenter.read();
    event.right = sensorRight.read();
    event.left = sensorLeft.read();

    sensorPublisher.publish(event);

    ros::spinOnce();

    loop_rate.sleep();
  }
}
