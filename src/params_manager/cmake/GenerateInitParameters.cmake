# CMake function code to call the init_parameters.cpp file generation script at build time.
#
# Roberto Masocco <robmasocco@gmail.com>
# Intelligent Systems Lab <isl.torvergata@gmail.com>
#
# May 8, 2023

function(generate_init_parameters)
  find_package(PythonInterp 3 REQUIRED)

  ament_index_has_resource(
    PARAMS_MANAGER_PREFIX
    packages
    params_manager)
  if(NOT PARAMS_MANAGER_PREFIX)
    message(FATAL_ERROR "Could not find params_manager package")
  endif()

  set(options "")
  set(oneValueArgs YAML_FILE OUT_FILE)
  set(multiValueArgs "")
  cmake_parse_arguments(
    GENERATE_INIT_PARAMETERS
    "${options}"
    "${oneValueArgs}"
    "${multiValueArgs}"
    ${ARGN})

  if(NOT GENERATE_INIT_PARAMETERS_YAML_FILE)
    message(FATAL_ERROR "YAML_FILE argument not provided")
  endif()
  if(NOT GENERATE_INIT_PARAMETERS_OUT_FILE)
    message(FATAL_ERROR "OUT_FILE argument not provided")
  endif()

  add_custom_command(
    OUTPUT "${GENERATE_INIT_PARAMETERS_OUT_FILE}"
    COMMAND ${PYTHON_EXECUTABLE}
      "${PARAMS_MANAGER_PREFIX}/share/params_manager/scripts/generate_init_parameters.py"
      "${GENERATE_INIT_PARAMETERS_YAML_FILE}"
      "${CMAKE_CURRENT_BINARY_DIR}/${GENERATE_INIT_PARAMETERS_OUT_FILE}"
    MAIN_DEPENDENCY "${GENERATE_INIT_PARAMETERS_YAML_FILE}"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    COMMENT "Generating parameters declaration source file ${GENERATE_INIT_PARAMETERS_OUT_FILE} from ${GENERATE_INIT_PARAMETERS_YAML_FILE}"
    VERBATIM
    USES_TERMINAL)
endfunction()
