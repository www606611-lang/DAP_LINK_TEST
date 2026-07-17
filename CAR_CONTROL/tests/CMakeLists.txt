cmake_minimum_required(VERSION 3.22)

project(car_control_host_tests C)

enable_testing()

add_executable(line_tracking_finish_policy_test
  "${CMAKE_CURRENT_SOURCE_DIR}/line_tracking_finish_policy_test.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/../app/line_tracking_finish_policy.c"
)
target_compile_features(line_tracking_finish_policy_test PRIVATE c_std_99)
target_include_directories(line_tracking_finish_policy_test PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/../app"
)

add_test(NAME line_tracking_finish_policy
  COMMAND line_tracking_finish_policy_test)
