cmake_minimum_required(VERSION 3.8)
project(crazyflie_simulation)



if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(crazyflie_interfaces REQUIRED)

# Build quadcopter_model library
add_subdirectory(quadcopter_model)

## Build the Custom Simulation Library
set(SIMULATION_SRC
  src/simulation/simulation.cpp
  src/simulation/controller.cpp
)
add_library(simulation_lib ${SIMULATION_SRC})
set_target_properties(simulation_lib PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(simulation_lib PUBLIC cxx_std_17) 
target_include_directories(simulation_lib PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
)

target_link_libraries(simulation_lib
  quadcopter_model
  Eigen3::Eigen
)


## Build the CRTP Driver Library
set(CRTP_DRIVER_SRC
  src/crtp_driver/console.cpp
  src/crtp_driver/generic_commander.cpp
  src/crtp_driver/hl_commander.cpp
  src/crtp_driver/localization.cpp
  src/crtp_driver/logging.cpp
  src/crtp_driver/logblock.cpp
  src/crtp_driver/parameters.cpp)
add_library(crtp_driver_lib ${CRTP_DRIVER_SRC})
set_target_properties(crtp_driver_lib PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(crtp_driver_lib PUBLIC cxx_std_17)
target_include_directories(crtp_driver_lib PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
)

target_link_libraries(crtp_driver_lib
  simulation_lib  
  Eigen3::Eigen)

ament_target_dependencies(crtp_driver_lib 
  rclcpp
  crazyflie_interfaces)

# Build the crazyflie node executable
add_executable(crazyflie src/crazyflie.cpp)

target_link_libraries(crazyflie
  simulation_lib
  crtp_driver_lib
)

ament_target_dependencies(crazyflie 
  rclcpp_lifecycle
  rclcpp
  crazyflie_interfaces)

target_include_directories(crazyflie PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(crazyflie PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17


## Finally build all nodes
install(TARGETS crazyflie 
  DESTINATION lib/${PROJECT_NAME})

ament_package()