cmake_minimum_required(VERSION 2.8.3)
project(see_ultrasonic)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED 
             COMPONENTS
             message_generation
             roscpp
             std_msgs
)

add_message_files(
  FILES
  UltrasonicEvent.msg
)

generate_messages()

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(${catkin_INCLUDE_DIRS})

add_subdirectory(src)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )
