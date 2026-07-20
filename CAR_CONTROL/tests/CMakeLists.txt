cmake_minimum_required(VERSION 3.22)

project(car_control_host_tests C)

enable_testing()

add_executable(line_tracking_finish_policy_test
  "${CMAKE_CURRENT_SOURCE_DIR}/line_tracking_finish_policy_test.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/../app/bringup/line_tracking_finish_policy.c"
)
target_compile_features(line_tracking_finish_policy_test PRIVATE c_std_99)
target_include_directories(line_tracking_finish_policy_test PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/../app/bringup"
)

add_test(NAME line_tracking_finish_policy
  COMMAND line_tracking_finish_policy_test)

add_executable(car_app_test
  "${CMAKE_CURRENT_SOURCE_DIR}/car_app_test.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/../app/car_app.c"
)
target_compile_features(car_app_test PRIVATE c_std_99)
target_include_directories(car_app_test PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/../app"
)

add_test(NAME car_app COMMAND car_app_test)

add_executable(line_follow_mission_test
  "${CMAKE_CURRENT_SOURCE_DIR}/line_follow_mission_test.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/../app/mission/line_follow_mission.c"
)
target_compile_features(line_follow_mission_test PRIVATE c_std_99)
target_include_directories(line_follow_mission_test PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/../app/mission"
  "${CMAKE_CURRENT_SOURCE_DIR}/../bsp"
  "${CMAKE_CURRENT_SOURCE_DIR}/../control"
  "${CMAKE_CURRENT_SOURCE_DIR}/../diagnostics"
  "${CMAKE_CURRENT_SOURCE_DIR}/../drivers/device/line_sensor"
)

add_test(NAME line_follow_mission COMMAND line_follow_mission_test)

add_executable(wheel_odometry_test
  "${CMAKE_CURRENT_SOURCE_DIR}/wheel_odometry_test.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/../control/wheel_odometry.c"
)
target_compile_features(wheel_odometry_test PRIVATE c_std_99)
target_include_directories(wheel_odometry_test PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/../control"
  "${CMAKE_CURRENT_SOURCE_DIR}/../drivers/mcu"
)

add_test(NAME wheel_odometry COMMAND wheel_odometry_test)

add_executable(tuning_codec_test
  "${CMAKE_CURRENT_SOURCE_DIR}/tuning_codec_test.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/../app/tuning/tuning_codec.c"
)
target_compile_features(tuning_codec_test PRIVATE c_std_99)
target_include_directories(tuning_codec_test PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/../app/tuning"
  "${CMAKE_CURRENT_SOURCE_DIR}/../app/bringup"
  "${CMAKE_CURRENT_SOURCE_DIR}/../control"
)

add_test(NAME tuning_codec COMMAND tuning_codec_test)

add_executable(k230_vision_link_test
  "${CMAKE_CURRENT_SOURCE_DIR}/k230_vision_link_test.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/../drivers/device/k230/k230_vision_link.c"
)
target_compile_features(k230_vision_link_test PRIVATE c_std_99)
target_include_directories(k230_vision_link_test PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/../drivers/device/k230"
  "${CMAKE_CURRENT_SOURCE_DIR}/../drivers/mcu"
)

add_test(NAME k230_vision_link COMMAND k230_vision_link_test)
