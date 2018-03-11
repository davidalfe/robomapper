include(ExternalProject)

ExternalProject_Add(
  wiringPi-build
  GIT_REPOSITORY    "git://git.drogon.net/wiringPi"
  CONFIGURE_COMMAND ""
  BUILD_COMMAND     "<SOURCE_DIR>/build;debian"
  INSTALL_COMMAND   ""
  BUILD_IN_SOURCE   1
  EXCLUDE_FROM_ALL
)
