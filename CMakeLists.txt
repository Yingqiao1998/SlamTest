cmake_minimum_required(VERSION 3.8)
project(slam)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_BUILD_TYPE Debug)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--allow-shlib-undefined")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")



if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(sim_msgs_DIR "${PROJECT_CURRENT_SOURCE_DIR}/../../install/sim_msgs/share/sim_msgs/cmake")
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(sim_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(mavros REQUIRED)
find_package(Ceres REQUIRED)
find_package(PkgConfig REQUIRED)

set(SLAM_COMPONENT_SRCS
  src/imm.cc
  src/locator.cc
  src/coordinate.cc
  src/location.cc
  src/slamNode.cc
  
  # Estimator files
  src/estimator/estimator.cpp
  src/estimator/parameters.cpp
  src/estimator/feature_manager.cpp
  
  # Feature tracker files
  src/featureTracker/feature_tracker.cpp
  
  # Initial files
  src/initial/initial_aligment.cpp
  src/initial/initial_ex_rotation.cpp
  src/initial/initial_sfm.cpp
  src/initial/solve_5pts.cpp
  
  # Factor files
  src/factor/marginalization_factor.cpp
  src/factor/pose_local_parameterization.cpp
  src/factor/projection_factor.cpp
  src/factor/projectionOneFrameTwoCamFactor.cpp
  src/factor/projectionTwoFrameOneCamFactor.cpp
  src/factor/projectionTwoFrameTwoCamFactor.cpp
  
  # Utility files
  src/utility/utility.cpp
  # src/utility/visualization.cpp
  # src/utility/CameraPoseVisualization.cpp
  
  # Camera models files
  src/camera_models/src/camera_models/Camera.cc
  src/camera_models/src/camera_models/CameraFactory.cc
  src/camera_models/src/camera_models/CostFunctionFactory.cc
  src/camera_models/src/camera_models/PinholeCamera.cc
  src/camera_models/src/camera_models/PinholeFullCamera.cc
  src/camera_models/src/camera_models/CataCamera.cc
  src/camera_models/src/camera_models/EquidistantCamera.cc
  src/camera_models/src/camera_models/ScaramuzzaCamera.cc)

set(SLAM_COMPONENT_DEPS
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  sim_msgs
  tf2
  tf2_geometry_msgs
  nav_msgs
  mavros_msgs
  mavros
  cv_bridge
  )

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include/slam
  ${CMAKE_CURRENT_SOURCE_DIR}/../domain_model/include
  ${EIGEN3_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/src/camera_models/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src/estimator
  ${CMAKE_CURRENT_SOURCE_DIR}/src/featureTracker
  ${CMAKE_CURRENT_SOURCE_DIR}/src/initial
  ${CMAKE_CURRENT_SOURCE_DIR}/src/factor
  ${CMAKE_CURRENT_SOURCE_DIR}/src/utility
  ${CERES_INCLUDE_DIRS})

link_directories()

set(node_plugins "")

add_library(slam_component SHARED
  ${SLAM_COMPONENT_SRCS})

target_compile_definitions(slam_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
target_link_libraries(slam_component
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES})
ament_target_dependencies(slam_component
  ${SLAM_COMPONENT_DEPS})

rclcpp_components_register_nodes(slam_component "slam_composition::SlamComponent")
set(node_plugins "${node_plugins}slam_composition::SlamComponent;$<TARGET_FILE:slam_component>\n")


install(TARGETS
  slam_component
)

install(
  DIRECTORY
  launch
  cfg
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
