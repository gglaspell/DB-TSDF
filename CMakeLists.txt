cmake_minimum_required(VERSION 3.8)
project(db_tsdf)

cmake_policy(SET CMP0074 NEW)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS  OFF) 
add_compile_options(-O3 -march=native -mtune=native -mavx -mavx2 -mfma -fopenmp-simd)
add_compile_options(-Wall -Wextra -Wno-unused-parameter -Wno-sign-compare)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)
find_package(Boost REQUIRED COMPONENTS thread chrono)
find_package(OpenMP REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_srvs REQUIRED)

find_package(VTK REQUIRED COMPONENTS 
  CommonCore
  CommonDataModel
  FiltersCore
  IOXML
  ImagingCore
  ImagingGeneral
  ImagingMorphological
)

function(add_db_tsdf_node target_name mask_bits)
  add_executable(${target_name} src/db_tsdf_node.cpp)
  target_compile_definitions(${target_name} PRIVATE DB_TSDF_MASK_BITS=${mask_bits})

  target_link_libraries(${target_name}
    rclcpp::rclcpp
    tf2::tf2
    tf2_ros::tf2_ros
    tf2_geometry_msgs::tf2_geometry_msgs
    sensor_msgs::sensor_msgs
    geometry_msgs::geometry_msgs
    pcl_conversions::pcl_conversions
    std_srvs::std_srvs
    ${PCL_LIBRARIES}
    Boost::thread
    Boost::chrono
    OpenMP::OpenMP_CXX
    VTK::CommonCore
    VTK::CommonDataModel
    VTK::FiltersCore
    VTK::IOXML
    VTK::ImagingCore
    VTK::ImagingGeneral
  )

  target_include_directories(${target_name} PRIVATE
    include
    ${Boost_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${VTK_INCLUDE_DIRS}
  )
endfunction()

# Keep the historical executable name as the default 16-bit implementation.
add_db_tsdf_node(db_tsdf_node 16)
add_db_tsdf_node(db_tsdf_node_32 32)

if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
  add_executable(db_tsdf_mask_test test/test_mask.cpp)
  target_include_directories(db_tsdf_mask_test PRIVATE include)
  ament_add_test(db_tsdf_mask_test
    COMMAND $<TARGET_FILE:db_tsdf_mask_test>
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  )
endif()

install(DIRECTORY 
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(TARGETS
  db_tsdf_node
  db_tsdf_node_32
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
