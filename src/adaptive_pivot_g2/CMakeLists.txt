cmake_minimum_required(VERSION 3.16)
project(adaptive_pivot_g2 VERSION 0.1.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/adaptive_search.cpp
  src/candidate_selection.cpp
  src/hybrid_selection.cpp
  src/path_conditioning.cpp
  src/path_optimization.cpp
  src/quintic_transition.cpp
  src/time_parameterization.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_quintic_transition test/test_quintic_transition.cpp)
  target_link_libraries(test_quintic_transition ${PROJECT_NAME})

  ament_add_gtest(test_time_parameterization test/test_time_parameterization.cpp)
  target_link_libraries(test_time_parameterization ${PROJECT_NAME})

  ament_add_gtest(test_candidate_selection test/test_candidate_selection.cpp)
  target_link_libraries(test_candidate_selection ${PROJECT_NAME})

  ament_add_gtest(test_hybrid_selection test/test_hybrid_selection.cpp)
  target_compile_features(test_hybrid_selection PRIVATE cxx_std_17)
  target_link_libraries(test_hybrid_selection ${PROJECT_NAME})

  ament_add_gtest(test_adaptive_optimization test/test_adaptive_optimization.cpp)
  target_compile_features(test_adaptive_optimization PRIVATE cxx_std_17)
  target_link_libraries(test_adaptive_optimization ${PROJECT_NAME})

  ament_add_gtest(test_path_conditioning test/test_path_conditioning.cpp)
  target_compile_features(test_path_conditioning PRIVATE cxx_std_17)
  target_link_libraries(test_path_conditioning ${PROJECT_NAME})
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_package()
