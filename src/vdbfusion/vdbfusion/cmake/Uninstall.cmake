# Copyright Contributors to the OpenVDB Project
# SPDX-License-Identifier: MPL-2.0

cmake_minimum_required(VERSION 3.15)

# TODO: Fix this, why it doesn't take the build/ directory???
set(MANIFEST "${CMAKE_BINARY_DIR}/../../../install_manifest.txt")

if(NOT EXISTS ${MANIFEST})
  message(FATAL_ERROR "Cannot find install manifest: '${MANIFEST}'")
endif()

file(STRINGS ${MANIFEST} INSTALLED_FILES)
foreach(INSTALLED_FILE ${INSTALLED_FILES})
  if(EXISTS ${INSTALLED_FILE})
    message(STATUS "Uninstalling: ${INSTALLED_FILE}")
    exec_program(
      ${CMAKE_COMMAND} ARGS
      "-E remove ${INSTALLED_FILE}"
      OUTPUT_VARIABLE stdout
      RETURN_VALUE RESULT)

    if(NOT "${RESULT}" STREQUAL 0)
      message(FATAL_ERROR "Failed to remove file: '${INSTALLED_FILE}'.")
    endif()
  endif()
endforeach(INSTALLED_FILE)
