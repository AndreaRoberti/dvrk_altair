#
# (C) Copyright 2011-2023 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 3.10)
project (sawKeyboard VERSION 1.4.0)

# Core libraries
set (REQUIRED_CISST_LIBRARIES cisstCommon
                              cisstVector
                              cisstOSAbstraction
                              cisstMultiTask
                              cisstParameterTypes)

find_package (cisst 1.2.0 COMPONENTS ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  # catkin/ROS paths
  cisst_set_output_path ()

  # create/configure file for find_package (sawKeyboard)
  set (sawKeyboard_INCLUDE_DIR
       "${sawKeyboard_SOURCE_DIR}/include"
       "${sawKeyboard_BINARY_DIR}/include")
  set (sawKeyboard_LIBRARY_DIR "${LIBRARY_OUTPUT_PATH}")
  set (sawKeyboard_LIBRARIES sawKeyboard)

  # add all config files for this component
  cisst_add_config_files (sawKeyboard)

  include_directories (BEFORE ${sawKeyboard_INCLUDE_DIR})
  set (sawKeyboard_HEADER_DIR "${sawKeyboard_SOURCE_DIR}/include/sawKeyboard")
  add_library (sawKeyboard
               code/mtsKeyboard.cpp
               ${sawKeyboard_HEADER_DIR}/sawKeyboardExport.h
               ${sawKeyboard_HEADER_DIR}/mtsKeyboard.h
               ${sawKeyboard_BINARY_DIR}/include/sawKeyboard/sawKeyboardRevision.h)

  set_target_properties (sawKeyboard PROPERTIES
                         VERSION ${sawKeyboard_VERSION}
                         COMPONENT sawKeyboard
                         FOLDER "sawKeyboard")

  cisst_target_link_libraries (sawKeyboard ${REQUIRED_CISST_LIBRARIES})

  # Install target for headers and library
  install (DIRECTORY "${sawKeyboard_SOURCE_DIR}/include/sawKeyboard"
                     "${sawKeyboard_BINARY_DIR}/include/sawKeyboard"
           DESTINATION include
           COMPONENT sawKeyboard-dev
           PATTERN .svn EXCLUDE)

  install (TARGETS sawKeyboard COMPONENT sawKeyboard
           RUNTIME DESTINATION bin
           LIBRARY DESTINATION lib
           ARCHIVE DESTINATION lib)

else (cisst_FOUND_AS_REQUIRED)
  message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires ${REQUIRED_CISST_LIBRARIES}")
endif (cisst_FOUND_AS_REQUIRED)
