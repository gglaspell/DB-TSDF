cmake_minimum_required(VERSION 3.16)
project(db_tsdf VERSION 1.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
option(DB_TSDF_BUILD_ROS "Build ROS 2 node and launch tools" ON)
option(DB_TSDF_NATIVE "Optimize for this build machine (nonportable binaries)" OFF)
option(DB_TSDF_SANITIZERS "Enable address and undefined behavior sanitizers" OFF)
include(CTest)

find_package(Eigen3 REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io kdtree)
find_package(OpenMP REQUIRED)
find_package(VTK REQUIRED COMPONENTS CommonCore CommonDataModel FiltersCore
  IOXML IOGeometry ImagingCore ImagingGeneral)

add_library(db_tsdf_core INTERFACE)
target_include_directories(db_tsdf_core INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
  $<INSTALL_INTERFACE:include> ${PCL_INCLUDE_DIRS})
target_link_libraries(db_tsdf_core INTERFACE Eigen3::Eigen ${PCL_LIBRARIES}
  OpenMP::OpenMP_CXX OpenSSL::Crypto nlohmann_json::nlohmann_json VTK::CommonCore VTK::CommonDataModel VTK::FiltersCore
  VTK::IOXML VTK::IOGeometry VTK::ImagingCore VTK::ImagingGeneral)
target_compile_options(db_tsdf_core INTERFACE -Wall -Wextra -Wpedantic)
# Match the allocator ABI advertised by prebuilt PCL. ASan and -march=native
# can otherwise make Eigen choose a different allocator in our translation
# units, even though the linked PCL binary still uses ordinary malloc/free.
include(CheckCXXSourceCompiles)
set(CMAKE_REQUIRED_INCLUDES ${PCL_INCLUDE_DIRS})
check_cxx_source_compiles("#include <pcl/pcl_config.h>
#if !defined(PCL_USES_EIGEN_HANDMADE_ALIGNED_MALLOC) || PCL_USES_EIGEN_HANDMADE_ALIGNED_MALLOC
#error PCL does not advertise the system allocator
#endif
int main() { return 0; }" DB_TSDF_PCL_SYSTEM_MALLOC)
unset(CMAKE_REQUIRED_INCLUDES)
set(db_tsdf_alignment_flags "")
if(DB_TSDF_PCL_SYSTEM_MALLOC)
  target_compile_definitions(db_tsdf_core INTERFACE EIGEN_MAX_ALIGN_BYTES=16 EIGEN_MALLOC_ALREADY_ALIGNED=1)
  set(db_tsdf_alignment_flags " -DEIGEN_MAX_ALIGN_BYTES=16 -DEIGEN_MALLOC_ALREADY_ALIGNED=1")
endif()
file(GLOB_RECURSE provenance_files CONFIGURE_DEPENDS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
  "include/*.hpp" "src/*.cpp" "scripts/*.py" "config/*.yaml" "launch/*.py" "cmake/*.in")
list(APPEND provenance_files CMakeLists.txt package.xml)
list(SORT provenance_files)
set(source_fingerprints "")
foreach(source IN LISTS provenance_files)
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/${source}")
  file(SHA256 "${CMAKE_CURRENT_SOURCE_DIR}/${source}" fingerprint)
  string(APPEND source_fingerprints "${source}:${fingerprint}\n")
endforeach()
string(SHA256 source_sha256 "${source_fingerprints}")
execute_process(COMMAND git rev-parse HEAD WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  OUTPUT_VARIABLE git_revision OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
execute_process(COMMAND git status --porcelain WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  OUTPUT_VARIABLE git_status OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
if(git_status)
  set(DB_TSDF_GIT_DIRTY 1)
endif()
string(TOUPPER "${CMAKE_BUILD_TYPE}" build_type_upper)
set(db_tsdf_flags "${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_${build_type_upper}} -std=c++20 -fopenmp -Wall -Wextra -Wpedantic${db_tsdf_alignment_flags}")
set(DB_TSDF_BUILD_NATIVE ${DB_TSDF_NATIVE})
set(DB_TSDF_BUILD_SANITIZERS ${DB_TSDF_SANITIZERS})
if(DB_TSDF_NATIVE)
  string(APPEND db_tsdf_flags " -march=native")
endif()
if(DB_TSDF_SANITIZERS)
  string(APPEND db_tsdf_flags " -fsanitize=address,undefined -fno-omit-frame-pointer")
endif()
configure_file(cmake/build_info.hpp.in include/db_tsdf/build_info.hpp @ONLY)
target_compile_definitions(db_tsdf_core INTERFACE
  DB_TSDF_SOURCE_SHA256="${source_sha256}" DB_TSDF_GIT_REVISION="${git_revision}"
  DB_TSDF_BUILD_TYPE="${CMAKE_BUILD_TYPE}")
if(DB_TSDF_NATIVE)
  target_compile_options(db_tsdf_core INTERFACE -march=native)
endif()
if(DB_TSDF_SANITIZERS)
  target_compile_options(db_tsdf_core INTERFACE -fsanitize=address,undefined -fno-omit-frame-pointer)
  target_link_options(db_tsdf_core INTERFACE -fsanitize=address,undefined)
endif()

add_executable(db_tsdf_offline src/offline.cpp)
target_link_libraries(db_tsdf_offline PRIVATE db_tsdf_core)

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  foreach(test_name mask volume integration exports)
    add_executable(db_tsdf_${test_name}_test test/test_${test_name}.cpp)
    target_link_libraries(db_tsdf_${test_name}_test PRIVATE db_tsdf_core)
    add_test(NAME db_tsdf_${test_name}_test COMMAND db_tsdf_${test_name}_test)
    set_tests_properties(db_tsdf_${test_name}_test PROPERTIES TIMEOUT 90)
  endforeach()
  add_test(NAME db_tsdf_workflows_test COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_workflows.py $<TARGET_FILE:db_tsdf_offline>)
  set_tests_properties(db_tsdf_workflows_test PROPERTIES TIMEOUT 120)
endif()

if(DB_TSDF_BUILD_ROS)
  find_package(ament_cmake REQUIRED)
  foreach(dependency rclcpp tf2 tf2_ros tf2_geometry_msgs sensor_msgs geometry_msgs
      pcl_conversions std_srvs std_msgs rcl_interfaces)
    find_package(${dependency} REQUIRED)
  endforeach()
  function(add_db_tsdf_node target_name mask_bits)
    add_executable(${target_name} src/db_tsdf_node.cpp)
    target_compile_definitions(${target_name} PRIVATE DB_TSDF_MASK_BITS=${mask_bits})
    target_link_libraries(${target_name} db_tsdf_core)
    if(COMMAND ament_target_dependencies)
      ament_target_dependencies(${target_name} rclcpp tf2 tf2_ros tf2_geometry_msgs
        sensor_msgs geometry_msgs pcl_conversions std_srvs std_msgs rcl_interfaces)
    else()
      foreach(dependency rclcpp tf2 tf2_ros tf2_geometry_msgs sensor_msgs geometry_msgs
          pcl_conversions std_srvs std_msgs rcl_interfaces)
        target_link_libraries(${target_name} ${dependency}::${dependency})
      endforeach()
    endif()
    install(TARGETS ${target_name} DESTINATION lib/${PROJECT_NAME})
  endfunction()
  add_db_tsdf_node(db_tsdf_node 16)
  add_db_tsdf_node(db_tsdf_node_32 32)
  install(TARGETS db_tsdf_offline DESTINATION lib/${PROJECT_NAME})
  install(PROGRAMS scripts/replay.py scripts/benchmark.py DESTINATION lib/${PROJECT_NAME})
  install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
  install(DIRECTORY include/ DESTINATION include)
  ament_export_include_directories(include)
  ament_package()
else()
  install(TARGETS db_tsdf_offline DESTINATION bin)
endif()
