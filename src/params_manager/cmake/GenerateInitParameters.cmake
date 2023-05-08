# CMake function code to call the init_parameters.cpp file generation script at build time.
#
# Roberto Masocco <robmasocco@gmail.com>
# Intelligent Systems Lab <isl.torvergata@gmail.com>
#
# May 8, 2023

function(generate_init_parameters yaml_file)
  find_package(PythonInterp 3 REQUIRED)

  message("Generating init_parameters.cpp source file...")

  ament_index_has_resource(
    PARAMS_MANAGER_PREFIX
    packages
    params_manager)
  if(NOT PARAMS_MANAGER_PREFIX)
    message(FATAL_ERROR "Could not find params_manager package")
  endif()

  execute_process(COMMAND ${PYTHON_EXECUTABLE}
    "${PARAMS_MANAGER_PREFIX}/share/params_manager/scripts/generate_init_parameters.py"
    "${yaml_file}"
    COMMAND_ERROR_IS_FATAL ANY)
endfunction()
