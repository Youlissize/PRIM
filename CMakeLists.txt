# ----------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.0)

set(BUILD_TYPE_RELEASE TRUE)
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)     
add_compile_options("$<$<CONFIG:RELEASE>:-O3>")

project(mainBis)

add_executable(
  mainBis
 	mainBis.cpp
	scene.cpp
	mesh.cpp
	fusingConstraint.cpp)

project(main)
add_executable(
  main
	main.cpp
	scene.cpp
	mesh.cpp
	constraint.cpp)


add_custom_command(TARGET mainBis
  POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:mainBis> ${CMAKE_CURRENT_SOURCE_DIR})


add_custom_command(TARGET main
  POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:main> ${CMAKE_CURRENT_SOURCE_DIR})
