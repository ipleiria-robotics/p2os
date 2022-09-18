cmake_minimum_required(VERSION 3.5)
project(p2os_driver)

if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif ()
# find dependencies
find_package(ament_cmake REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  "p2os_msgs"
  "std_msgs"
  "nav_msgs"
  "rclcpp"
  "rcutils"
  "geometry_msgs"
  "tf2"
  "tf2_ros"
  "tf2_msgs"
  "tf2_geometry_msgs"
  "generate_parameter_library"
)
foreach (Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach ()

generate_parameter_library(p2os_parameters
        src/p2os_parameters.yaml
        )

INCLUDE_DIRECTORIES(include)

## Declare a cpp executable
add_executable(p2os_driver
  src/p2osnode.cc
  src/p2os.cc
  src/kinecalc.cc
  src/packet.cc
  src/robot_params.cc
  src/sip.cc
  src/p2os_ptz.cpp
)
target_link_libraries(p2os_driver
        p2os_parameters
)

ament_target_dependencies(p2os_driver ${THIS_PACKAGE_INCLUDE_DEPENDS})

## Mark executables and/or libraries for installation
install(
  TARGETS p2os_driver
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
