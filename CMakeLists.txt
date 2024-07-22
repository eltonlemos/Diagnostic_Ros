cmake_minimum_required(VERSION 3.8)
project(diagnostic_moog)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(diagnostic src/diagnostic.cpp)
target_include_directories(diagnostic PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(diagnostic PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS diagnostic
  DESTINATION lib/${PROJECT_NAME})

install( # launch descriptor
  FILES src/diagnostics.launch.py
  DESTINATION share/${PROJECT_NAME}
)
install( # example publisher
  PROGRAMS src/diagnostic.py
  DESTINATION lib/${PROJECT_NAME}
)
install( # example publisher
  PROGRAMS src/hz.py
  DESTINATION lib/${PROJECT_NAME}
)
install( # example aggregator configration
  FILES config/analyzers.yaml
  DESTINATION share/${PROJECT_NAME}
)
install( # example aggregator configration
  FILES config/topics.yaml
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
