cmake_minimum_required(VERSION 3.8)
project(emcl2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3 -Wall -Wextra -Wpedantic)
endif()

option(ENABLE_SANITIZER "Build with AddressSanitizer and LeakSanitizer" OFF)
if(ENABLE_SANITIZER)
  add_compile_options(-fsanitize=address -fno-omit-frame-pointer)
  add_link_options(-fsanitize=address)
endif()

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(emcl2_core STATIC
  src/Mcl.cpp
  src/ExpResetMcl2.cpp
  src/Particle.cpp
  src/OdomModel.cpp
  src/Pose.cpp
  src/Scan.cpp
  src/LikelihoodFieldMap.cpp
)
# emcl2_core is linked into the shared component library, so it must be PIC.
set_target_properties(emcl2_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

find_package(OpenMP)
if(OpenMP_CXX_FOUND)
  target_link_libraries(emcl2_core OpenMP::OpenMP_CXX)
endif()

# The node is built as a shared library so it can be loaded into a component
# container, and the executable is a thin wrapper around it.
ament_auto_add_library(emcl2_component SHARED
  src/emcl2_node.cpp
)
target_link_libraries(emcl2_component emcl2_core)
rclcpp_components_register_nodes(emcl2_component "emcl2::EMcl2Node")

ament_auto_add_executable(emcl2_node
  src/main.cpp
)
target_link_libraries(emcl2_node emcl2_component)

option(BUILD_BENCHMARK "Build the micro benchmark" OFF)
if(BUILD_BENCHMARK)
  ament_auto_add_executable(bench_emcl2 bench/bench_emcl2.cpp)
  target_link_libraries(bench_emcl2 emcl2_core)
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_emcl2 test/test_emcl2.cpp)
  target_link_libraries(test_emcl2 emcl2_core)
endif()

install(PROGRAMS
  test/test.bash
  test/test_gui.bash
  test/drive_path.py
  test/pose_logger.py
  test/convergence_metrics.py
  test/post_recovery_eval.py
  test/run_localization_eval.sh
  test/composition_smoke_test.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  config
  launch
  maps
  rviz
  test
  worlds
)
