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

add_executable(wire_protocol_test
  "${CMAKE_CURRENT_SOURCE_DIR}/wire_protocol_test.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/../drivers/utility/wire_protocol.c"
)
target_compile_features(wire_protocol_test PRIVATE c_std_99)
target_include_directories(wire_protocol_test PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/../drivers/utility"
)
add_test(NAME wire_protocol COMMAND wire_protocol_test)

add_executable(chassis_radio_link_test
  "${CMAKE_CURRENT_SOURCE_DIR}/chassis_radio_link_test.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/../drivers/device/chassis_radio/chassis_radio_link.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/../drivers/utility/wire_protocol.c"
)
target_compile_features(chassis_radio_link_test PRIVATE c_std_99)
target_include_directories(chassis_radio_link_test PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/../drivers/device/chassis_radio"
  "${CMAKE_CURRENT_SOURCE_DIR}/../drivers/mcu"
  "${CMAKE_CURRENT_SOURCE_DIR}/../drivers/utility"
)
add_test(NAME chassis_radio_link COMMAND chassis_radio_link_test)

add_executable(electromagnet_test
  "${CMAKE_CURRENT_SOURCE_DIR}/electromagnet_test.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/../drivers/device/electromagnet/electromagnet.c"
)
target_compile_features(electromagnet_test PRIVATE c_std_99)
target_include_directories(electromagnet_test PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/../drivers/device/electromagnet"
  "${CMAKE_CURRENT_SOURCE_DIR}/../drivers/mcu"
)
add_test(NAME electromagnet COMMAND electromagnet_test)
