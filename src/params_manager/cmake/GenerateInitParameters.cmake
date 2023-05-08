# CMake function code to call the init_parameters.cpp file generation script at build time.
#
# Roberto Masocco <robmasocco@gmail.com>
# Intelligent Systems Lab <isl.torvergata@gmail.com>
#
# May 8, 2023

function(generate_init_parameters yaml_file out_file)
  find_package(PythonInterp 3 REQUIRED)

  ament_index_has_resource(
    PARAMS_MANAGER_PREFIX
    packages
    params_manager)
  if(NOT PARAMS_MANAGER_PREFIX)
    message(FATAL_ERROR "Could not find params_manager package")
  endif()

  add_custom_command(
    OUTPUT "${out_file}"
    COMMAND ${PYTHON_EXECUTABLE}
      "${PARAMS_MANAGER_PREFIX}/share/params_manager/scripts/generate_init_parameters.py"
      "${yaml_file}"
      "${CMAKE_CURRENT_BINARY_DIR}/${out_file}"
    MAIN_DEPENDENCY "${yaml_file}"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    COMMENT "Generating parameters declaration source file ${out_file} from ${yaml_file} ..."
    VERBATIM
    USES_TERMINAL)
endfunction()
