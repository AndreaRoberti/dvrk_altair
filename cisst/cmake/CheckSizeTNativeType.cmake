#
# Author(s):  Anton Deguet
# Created on: 2011
#
# (C) Copyright 2011-2019 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

function (check_size_t_native_type VARIABLE)
  # make sure we don't test over and over
  if (${VARIABLE} MATCHES "^${VARIABLE}$")
    message (STATUS "Checking to see if size_t is a native type")
    # check against int and long long int
    set (SOURCE
         "#include <vector>
          char method(unsigned int p) {
            return 'u';
          }
          char method(unsigned long long int p) {
            return 'l';
          }
          char method(size_t p) {
            return 's';
          }
          int main(void) {}")

    file (WRITE
          "${CMAKE_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/CMakeTmp/test_size_t_int.cpp"
          "${SOURCE}\n")

    try_compile (RESULT_int
                 ${CMAKE_BINARY_DIR}
                 "${CMAKE_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/CMakeTmp/test_size_t_int.cpp"
                 OUTPUT_VARIABLE OUTPUT_int)

    # check against long int (ubuntu 18.04 default compiler apparently uses unsigned long int for size_t)
    set (SOURCE
         "#include <vector>
          char method(unsigned long int p) {
            return 'l';
          }
          char method(size_t p) {
            return 's';
          }
          int main(void) {}")

    file (WRITE
          "${CMAKE_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/CMakeTmp/test_size_t_long.cpp"
          "${SOURCE}\n")

    try_compile (RESULT_long
                 ${CMAKE_BINARY_DIR}
                 "${CMAKE_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/CMakeTmp/test_size_t_long.cpp"
                 OUTPUT_VARIABLE OUTPUT_long)

    set (VARIABLE RESULT_int AND RESULT_long)

    set (OUTPUT "${OUTPUT_int}\n${OUTPUT_long}")

    # report using message and log files
    if (${VARIABLE})
      message (STATUS "Checking to see if size_t is a native type - yes")
      file (APPEND ${CMAKE_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/CMakeOutput.log
            "Determining if size_t is a native type passed with "
            "the following output:\n${OUTPUT}\n\n")
    else (${VARIABLE})
      message (STATUS "Checking to see if size_t is a native type - no")
      file (APPEND ${CMAKE_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/CMakeError.log
            "Determining if size_t is a native type passed with "
            "the following output:\n${OUTPUT}\n\n")
    endif (${VARIABLE})

  endif (${VARIABLE} MATCHES "^${VARIABLE}$")

endfunction (check_size_t_native_type VARIABLE)
