#include "board_button.h"
#include "board_gpio_irq.h"
#include "board_motor_safe.h"
#include "board_resources.h"
#include "board_wheel_drive.h"
#include "bluetooth_uart.h"
#include "car_app.h"
#include "car_display.h"
#include "control_supervisor.h"
#include "debug_snapshot.h"
#include "delay.h"
#include "encoder_input.h"
#include "firmware_update.h"
#include "heading_bringup_test.h"
#include "icm20948.h"
#include "jdy31_config.h"
#include "line_sensor_bringup.h"
#include "line_follow_mission.h"
#include "line_tracking_bringup_test.h"
#include "motion_supervisor.h"
#include "position_bringup_test.h"
#include "reset_diagnostics.h"
#include "runtime_metrics.h"
#include "speed_bringup_test.h"
#include "speed_tuning_console.h"
#include "st7789.h"
#include "system_watchdog.h"
#include "ti_msp_dl_config.h"
#include "wheel_heading_control.h"
#include "wheel_line_tracking_control.h"
#include "wheel_odometry.h"
#include "wheel_position_control.h"
#include "wheel_speed_control.h"
#include "wheel_yaw_control.h"
#include "yaw_bringup_test.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

static void app_process_car_action(
    const car_app_snapshot_t *snapshot);

int main(void)
{
    uint32_t displayed_period = UINT32_MAX;
    car_display_phase_t display_phase = CAR_DISPLAY_PHASE_HEADER;
    bool display_dirty = true;

    FirmwareUpdate_AppInit();
    SYSCFG_DL_init();
    SystemWatchdog_Init();
    ResetDiagnostics_Init();
    BoardMotorSafe_Init();
    BoardWheelDrive_Init();
    BoardButton_Init(delay_get_ms());
    ControlSupervisor_Init(ResetDiagnostics_IsSuspicious());
    ICM20948_Init(delay_get_ms());
    EncoderInput_Init(delay_get_ms());
    WheelOdometry_Init(delay_get_ms());
    EncoderInput_SetInverted(ENCODER_INPUT_0,
        BOARD_ENCODER_0_FORWARD_INVERTED != 0);
    EncoderInput_SetInverted(ENCODER_INPUT_1,
        BOARD_ENCODER_1_FORWARD_INVERTED != 0);
    BoardGpioIrq_Init();
    WheelSpeedControl_Init(delay_get_ms());
    WheelPositionControl_Init(delay_get_ms());
    WheelYawControl_Init(delay_get_ms());
    WheelHeadingControl_Init(delay_get_ms());
    WheelLineTrackingControl_Init(delay_get_ms());
    SpeedBringupTest_Init(ResetDiagnostics_IsSuspicious());
    PositionBringupTest_Init(ResetDiagnostics_IsSuspicious());
    YawBringupTest_Init(ResetDiagnostics_IsSuspicious());
    HeadingBringupTest_Init(ResetDiagnostics_IsSuspicious());
    LineSensorBringup_Init(delay_get_ms());
    LineTrackingBringupTest_Init(ResetDiagnostics_IsSuspicious());
    LineFollowMission_Init(ResetDiagnostics_IsSuspicious());
    MotionSupervisor_Init(ResetDiagnostics_IsSuspicious());
    CarApp_Init(ResetDiagnostics_IsSuspicious());
    BluetoothUart_Init();
    JDY31_ConfigInit(delay_get_ms(),
        CAR_JDY31_CONFIGURE_ON_BOOT != 0);
    if (!JDY31_ConfigIsExclusive()) {
        SpeedTuningConsole_Init();
    }

    ST7789_Init();
    CarDisplay_Init();
    CarDebugSnapshot_Update();
    AppRuntimeMetrics_Init(delay_get_ms());

    while (1) {
        uint32_t now_ms = delay_get_ms();
        uint32_t display_interval_ms = JDY31_ConfigIsExclusive() ?
            200U : CAR_DISPLAY_SLICE_INTERVAL_MS;
        uint32_t now_display_period = now_ms / display_interval_ms;
        bool pb21_press_event;
        bool pb4_press_event;
        bool pb5_press_event;
        bool pb21_release_event;
        bool pb4_release_event;
        bool pb5_release_event;
        bool any_release_event;
        car_app_inputs_t car_app_inputs;
        car_app_snapshot_t car_app_snapshot;

        AppRuntimeMetrics_RecordLoop(now_ms);
        BoardButton_Task(now_ms);
        pb21_press_event = BoardButton_GetPressEventId(
            BOARD_BUTTON_ID_PB21);
        pb4_press_event = BoardButton_GetPressEventId(
            BOARD_BUTTON_ID_SW2_PB4);
        pb5_press_event = BoardButton_GetPressEventId(
            BOARD_BUTTON_ID_SW1_PB5);
        pb21_release_event = BoardButton_GetReleaseEventId(
            BOARD_BUTTON_ID_PB21);
        pb4_release_event = BoardButton_GetReleaseEventId(
            BOARD_BUTTON_ID_SW2_PB4);
        pb5_release_event = BoardButton_GetReleaseEventId(
            BOARD_BUTTON_ID_SW1_PB5);
        any_release_event = pb21_release_event || pb4_release_event ||
            pb5_release_event;

        EncoderInput_Task(now_ms);
        WheelOdometry_Task(now_ms);
        ICM20948_Task(now_ms);
        LineSensorBringup_Task(now_ms);
        BluetoothUart_Task(now_ms);
        JDY31_ConfigTask(now_ms);
        if (!JDY31_ConfigIsExclusive()) {
            SpeedTuningConsole_Task(now_ms);
        }
        FirmwareUpdate_Task();
        ControlSupervisor_Task(now_ms);

        car_app_inputs.service_active = JDY31_ConfigIsExclusive() ||
            FirmwareUpdate_IsPending();
        car_app_inputs.speed_test_active =
            SpeedBringupTest_GetState() == SPEED_BRINGUP_TEST_RUNNING;
        car_app_inputs.position_test_active =
            PositionBringupTest_GetState() ==
                POSITION_BRINGUP_TEST_RUNNING;
        car_app_inputs.heading_test_active =
            HeadingBringupTest_IsActive();
        car_app_inputs.line_test_active =
            LineTrackingBringupTest_IsActive();
        car_app_inputs.line_mission_active =
            LineFollowMission_IsActive();
        car_app_inputs.motion_active = MotionSupervisor_IsActive();
        car_app_inputs.yaw_test_active = YawBringupTest_IsActive();
        car_app_inputs.pb21_press_event = pb21_press_event;
        car_app_inputs.pb4_press_event = pb4_press_event;
        car_app_inputs.pb5_press_event = pb5_press_event;
        CarApp_Step(&car_app_inputs);
        if (CarApp_GetSnapshot(&car_app_snapshot)) {
            app_process_car_action(&car_app_snapshot);
        }

        SpeedBringupTest_Task(now_ms, false);
        PositionBringupTest_Task(now_ms, false);
        YawBringupTest_Task(now_ms);
        HeadingBringupTest_Task(now_ms);
        LineTrackingBringupTest_Task(now_ms);
        LineFollowMission_Task(now_ms);
        MotionSupervisor_Task(now_ms);
        WheelPositionControl_Task(now_ms);
        WheelYawControl_Task(now_ms);
        WheelHeadingControl_Task(now_ms);
        WheelLineTrackingControl_Task(now_ms);
        WheelSpeedControl_Task(now_ms);

        if (pb21_press_event) {
            CarDebugSnapshot_RecordButtonPress(CAR_DEBUG_BUTTON_PB21);
            display_phase = CAR_DISPLAY_PHASE_FOOTER;
            display_dirty = true;
        }
        if (pb4_press_event) {
            CarDebugSnapshot_RecordButtonPress(CAR_DEBUG_BUTTON_PB4);
            display_phase = CAR_DISPLAY_PHASE_FOOTER;
            display_dirty = true;
        }
        if (pb5_press_event) {
            CarDebugSnapshot_RecordButtonPress(CAR_DEBUG_BUTTON_PB5);
            display_phase = CAR_DISPLAY_PHASE_FOOTER;
            display_dirty = true;
        }
        if (any_release_event) {
            display_phase = CAR_DISPLAY_PHASE_FOOTER;
            display_dirty = true;
        }
        if (now_display_period != displayed_period) {
            displayed_period = now_display_period;
            display_dirty = true;
        }

        CarDebugSnapshot_Update();

        if (display_dirty) {
            uint32_t display_started_ms = delay_get_ms();

            CarDisplay_Update(now_ms, display_phase);
            AppRuntimeMetrics_RecordDisplay(
                delay_get_ms() - display_started_ms);
            display_phase = (car_display_phase_t) (
                ((uint32_t) display_phase + 1U) %
                (uint32_t) CAR_DISPLAY_PHASE_COUNT);
            display_dirty = false;
        }

        SystemWatchdog_Kick();
        __WFI();
    }
}

static void app_process_car_action(
    const car_app_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return;
    }

    if (snapshot->action == CAR_APP_ACTION_STOP_ACTIVE) {
        switch (snapshot->active_workflow) {
            case CAR_APP_WORKFLOW_SPEED_TEST:
                SpeedBringupTest_RequestStop();
                break;
            case CAR_APP_WORKFLOW_POSITION_TEST:
                PositionBringupTest_RequestStop();
                break;
            case CAR_APP_WORKFLOW_HEADING_TEST:
                HeadingBringupTest_RequestStop();
                break;
            case CAR_APP_WORKFLOW_LINE_TEST:
                LineTrackingBringupTest_RequestStop();
                break;
            case CAR_APP_WORKFLOW_LINE_MISSION:
                LineFollowMission_RequestStop();
                break;
            case CAR_APP_WORKFLOW_MOTION:
                MotionSupervisor_RequestStop();
                break;
            case CAR_APP_WORKFLOW_YAW_TEST:
                YawBringupTest_RequestStop();
                break;
            default:
                break;
        }
        CarDebugSnapshot_SetButtonYawCommand(0);
    } else if ((snapshot->action == CAR_APP_ACTION_START_YAW) &&
        YawBringupTest_RequestTurn(
            (float) snapshot->yaw_command_mdeg / 1000.0f)) {
        CarDebugSnapshot_SetButtonYawCommand(
            snapshot->yaw_command_mdeg);
    }
}
