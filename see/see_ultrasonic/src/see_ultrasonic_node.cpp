#include <chrono>
#include <iostream>
#include <string>
#include <thread>

//#include <ros/ros.h>

#include <wiringPi.h>

#include "UltrasonicSensor.hpp"

constexpr double MAX_DISTANCE = 400.0;

int main(int argc, char** argv)
{
  wiringPiSetup();

  mapper::see::UltrasonicSensor sensor1{0, 2};
  mapper::see::UltrasonicSensor sensor2{1, 4};
  mapper::see::UltrasonicSensor sensor3{21, 22};

  while(true)
  {
    auto out1 = sensor1.read();
    auto out2 = sensor2.read();
    auto out3 = sensor3.read();

    std::cout << "Measures: 1: " << out1 << ", 2: " << out2 << ", 3: " << out3 << std::endl;

    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}
