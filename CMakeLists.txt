cmake_minimum_required(VERSION 3.20)
project(sim_corrida VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

option(SIM_WARNINGS_AS_ERRORS "Trata avisos como erro" ON)
option(SIM_SANITIZE "Compila com AddressSanitizer + UndefinedBehaviorSanitizer" OFF)
option(SIM_BUILD_TESTS "Compila os testes" ON)

# Opcoes de compilacao compartilhadas (avisos + sanitizadores).
add_library(sim_options INTERFACE)
if(MSVC)
  target_compile_options(sim_options INTERFACE /W4 $<$<BOOL:${SIM_WARNINGS_AS_ERRORS}>:/WX>)
else()
  target_compile_options(sim_options INTERFACE
    -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Wshadow
    $<$<BOOL:${SIM_WARNINGS_AS_ERRORS}>:-Werror>)
  if(SIM_SANITIZE)
    target_compile_options(sim_options INTERFACE
      -fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover=undefined)
    target_link_options(sim_options INTERFACE -fsanitize=address,undefined)
  endif()
endif()

# Firmware da ECU e codec CAN: C puro, sem alocacao dinamica.
add_library(sim_ecu STATIC
  src/can_frames.c
  src/ecu.c)
target_include_directories(sim_ecu PUBLIC include)
target_link_libraries(sim_ecu PUBLIC sim_options)

add_library(sim_physics STATIC
  src/broker_publisher.cpp
  src/can_bus.cpp
  src/driver.cpp
  src/ecu_loop.cpp
  src/engine.cpp
  src/gearbox.cpp
  src/track.cpp
  src/telemetry_client.cpp
  src/tyre.cpp
  src/vehicle.cpp)
target_include_directories(sim_physics PUBLIC include)
target_link_libraries(sim_physics PUBLIC sim_options sim_ecu)
if(WIN32)
  target_link_libraries(sim_physics PUBLIC ws2_32)
endif()

# Grava telemetria real no formato do servidor C# (usada para gerar o fixture dos testes dele).
add_executable(sim_telemetry_dump tools/telemetry_dump.cpp)
target_link_libraries(sim_telemetry_dump PRIVATE sim_physics)

if(SIM_BUILD_TESTS)
  enable_testing()
  find_package(GTest QUIET)
  if(NOT GTest_FOUND)
    include(FetchContent)
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_Declare(googletest
      URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.tar.gz)
    FetchContent_MakeAvailable(googletest)
  endif()

  add_executable(sim_tests
    tests/broker_publisher_test.cpp
    tests/can_bus_test.cpp
    tests/can_description_test.cpp
    tests/driver_test.cpp
    tests/ecu_test.cpp
    tests/engine_test.cpp
    tests/handling_test.cpp
    tests/telemetry_client_test.cpp
    tests/track_test.cpp
    tests/tyre_test.cpp
    tests/validation_test.cpp
    tests/vehicle_test.cpp)
  target_link_libraries(sim_tests PRIVATE sim_physics GTest::gtest_main)
  target_compile_definitions(sim_tests PRIVATE SIM_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  include(GoogleTest)
  gtest_discover_tests(sim_tests DISCOVERY_MODE PRE_TEST)
endif()
