cmake_minimum_required (VERSION 3.10)
project (dvrk_config_jhu VERSION 2.2.1)

find_package (ament_cmake QUIET)
if (ament_cmake_FOUND)
  ament_package ()
endif ()

install (
  DIRECTORY jhu-dVRK jhu-daVinci jhu-dVRK-Si jhu-daVinci-Si
  DESTINATION share
  )
