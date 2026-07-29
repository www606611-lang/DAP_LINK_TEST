#include "car_runtime.h"

#include "board_button.h"
#include "board_gpio_irq.h"
#include "board_motor_safe.h"
#include "board_resources.h"
#include "board_startup.h"
#include "board_wheel_drive.h"
#include "bluetooth_uart.h"
#include "car_app.h"
#include "car_display.h"
#include "chassis_radio_link.h"
#include "control_supervisor.h"
#include "debug_snapshot.h"
#include "delay.h"
#include "encoder_input.h"
#include "electromagnet.h"
#include "firmware_update.h"
#if CAR_ENABLE_BRINGUP
#include "heading_bringup_test.h"
#endif
#include "icm20948.h"
#include "jdy31_config.h"
#include "line_follow_mission.h"
#include "line_sensor.h"
#if CAR_ENABLE_BRINGUP
#include "line_tracking_bringup_test.h"
#endif
#include "motion_supervisor.h"
#if CAR_ENABLE_BRINGUP
#include "position_bringup_test.h"
#endif
#include "radio_uart.h"
#include "reset_diagnostics.h"
#include "runtime_metrics.h"
#if CAR_ENABLE_BRINGUP
#include "speed_bringup_test.h"
#endif
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
#if CAR_ENABLE_BRINGUP
#include "yaw_bringup_test.h"
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

static uint32_t g_displayed_period = UINT32_MAX;
static car_display_phase_t g_display_phase = CAR_DISPLAY_PHASE_HEADER;
static bool g_display_dirty = true;
static uint8_t g_display_priority_slot;
static uint8_t g_display_slow_index;
static bool g_display_idle_line_next = true;

static const car_display_phase_t g_display_slow_phases[] = {
    CAR_DISPLAY_PHASE_HEADER,
    CAR_DISPLAY_PHASE_SPEED,
    CAR_DISPLAY_PHASE_ENCODER,
    CAR_DISPLAY_PHASE_LINE,
    CAR_DISPLAY_PHASE_CONTROL,
    CAR_DISPLAY_PHASE_HEALTH,
    CAR_DISPLAY_PHASE_FOOTER
};

static void car_runtime_process_action(
    const car_app_snapshot_t *snapshot);
static car_display_phase_t car_runtime_next_display_phase(
    car_app_workflow_t workflow);

void CarRuntime_Init(void)
{
    uint32_t now_ms;
    bool suspicious_reset;

    FirmwareUpdate_AppInit();
    BoardStartup_Init();
    SystemWatchdog_Init();
    ResetDiagnostics_Init();
    suspicious_reset = ResetDiagnostics_IsSuspicious();
    BoardMotorSafe_Init();
    BoardWheelDrive_Init();
    Electromagnet_Init();
    now_ms = delay_get_ms();
    BoardButton_Init(now_ms);
    ControlSupervisor_Init(suspicious_reset);
    ICM20948_Init(now_ms);
    EncoderInput_Init(now_ms);
    WheelOdometry_Init(now_ms);
    EncoderInput_SetInverted(ENCODER_INPUT_0,
        BOARD_ENCODER_0_FORWARD_INVERTED != 0);
    EncoderInput_SetInverted(ENCODER_INPUT_1,
        BOARD_ENCODER_1_FORWARD_INVERTED != 0);
    BoardGpioIrq_Init();
    WheelSpeedControl_Init(now_ms);
    WheelPositionControl_Init(now_ms);
    WheelYawControl_Init(now_ms);
    WheelHeadingControl_Init(now_ms);
    WheelLineTrackingControl_Init(now_ms);
#if CAR_ENABLE_BRINGUP
    SpeedBringupTest_Init(suspicious_reset);
    PositionBringupTest_Init(suspicious_reset);
    YawBringupTest_Init(suspicious_reset);
    HeadingBringupTest_Init(suspicious_reset);
#endif
    LineSensor_Init(now_ms);
#if CAR_ENABLE_BRINGUP
    LineTrackingBringupTest_Init(suspicious_reset);
#endif
    LineFollowMission_Init(suspicious_reset);
    MotionSupervisor_Init(suspicious_reset);
    CarApp_Init(suspicious_reset);
    BluetoothUart_Init();
    RadioUart_Init();
    ChassisRadioLink_Init(now_ms);
    JDY31_ConfigInit(now_ms, CAR_JDY31_CONFIGURE_ON_BOOT != 0);
    if (!JDY31_ConfigIsExclusive()) {
        SpeedTuningConsole_Init();
    }
    ST7789_Init();
    CarDisplay_Init();
    CarDebugSnapshot_Update();
    AppRuntimeMetrics_Init(now_ms);
}

void CarRuntime_Step(void)
{
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
    car_app_inputs_t car_app_inputs = {0};
    car_app_snapshot_t car_app_snapshot;
    car_app_workflow_t active_workflow = CAR_APP_WORKFLOW_NONE;

    AppRuntimeMetrics_RecordLoop(now_ms);
    BoardButton_Task(now_ms);
    pb21_press_event = BoardButton_GetPressEventId(BOARD_BUTTON_ID_PB21);
    pb4_press_event = BoardButton_GetPressEventId(BOARD_BUTTON_ID_SW2_PB4);
    pb5_press_event = BoardButton_GetPressEventId(BOARD_BUTTON_ID_SW1_PB5);
    pb21_release_event =
        BoardButton_GetReleaseEventId(BOARD_BUTTON_ID_PB21);
    pb4_release_event =
        BoardButton_GetReleaseEventId(BOARD_BUTTON_ID_SW2_PB4);
    pb5_release_event =
        BoardButton_GetReleaseEventId(BOARD_BUTTON_ID_SW1_PB5);
    any_release_event = pb21_release_event || pb4_release_event ||
        pb5_release_event;

    EncoderInput_Task(now_ms);
    WheelOdometry_Task(now_ms);
    ICM20948_Task(now_ms);
    LineSensor_Task(now_ms);
    ChassisRadioLink_SetStatusFlags(
        BoardMotorSafe_IsHighImpedance() ?
            CHASSIS_RADIO_STATUS_HIGH_Z : 0U);
    ChassisRadioLink_Task(now_ms);
    BluetoothUart_Task(now_ms);
    Electromagnet_Task(now_ms);
    JDY31_ConfigTask(now_ms);
    if (!JDY31_ConfigIsExclusive() && !FirmwareUpdate_IsPending()) {
        SpeedTuningConsole_Task(now_ms);
    }
    FirmwareUpdate_Task();
    ControlSupervisor_Task(now_ms);

    car_app_inputs.service_active = JDY31_ConfigIsExclusive() ||
        FirmwareUpdate_IsPending();
#if CAR_ENABLE_BRINGUP
    car_app_inputs.speed_test_active =
        SpeedBringupTest_GetState() == SPEED_BRINGUP_TEST_RUNNING;
    car_app_inputs.position_test_active =
        PositionBringupTest_GetState() == POSITION_BRINGUP_TEST_RUNNING;
    car_app_inputs.heading_test_active = HeadingBringupTest_IsActive();
    car_app_inputs.line_test_active = LineTrackingBringupTest_IsActive();
#endif
    car_app_inputs.line_mission_active = LineFollowMission_IsActive();
    car_app_inputs.motion_active = MotionSupervisor_IsActive();
#if CAR_ENABLE_BRINGUP
    car_app_inputs.yaw_test_active = YawBringupTest_IsActive();
#endif
    car_app_inputs.pb21_press_event = pb21_press_event;
    car_app_inputs.pb4_press_event = pb4_press_event;
    car_app_inputs.pb5_press_event = pb5_press_event;
    CarApp_Step(&car_app_inputs);
    if (CarApp_GetSnapshot(&car_app_snapshot)) {
        active_workflow = car_app_snapshot.active_workflow;
        car_runtime_process_action(&car_app_snapshot);
    }

#if CAR_ENABLE_BRINGUP
    SpeedBringupTest_Task(now_ms, false);
    PositionBringupTest_Task(now_ms, false);
    YawBringupTest_Task(now_ms);
    HeadingBringupTest_Task(now_ms);
    LineTrackingBringupTest_Task(now_ms);
#endif
    LineFollowMission_Task(now_ms);
    MotionSupervisor_Task(now_ms);
    WheelPositionControl_Task(now_ms);
    WheelYawControl_Task(now_ms);
    WheelHeadingControl_Task(now_ms);
    WheelLineTrackingControl_Task(now_ms);
    WheelSpeedControl_Task(now_ms);

    if (pb21_press_event) {
        CarDebugSnapshot_RecordButtonPress(CAR_DEBUG_BUTTON_PB21);
        g_display_phase = CAR_DISPLAY_PHASE_FOOTER;
        g_display_dirty = true;
    }
    if (pb4_press_event) {
        CarDebugSnapshot_RecordButtonPress(CAR_DEBUG_BUTTON_PB4);
        g_display_phase = CAR_DISPLAY_PHASE_FOOTER;
        g_display_dirty = true;
    }
    if (pb5_press_event) {
        CarDebugSnapshot_RecordButtonPress(CAR_DEBUG_BUTTON_PB5);
        g_display_phase = CAR_DISPLAY_PHASE_FOOTER;
        g_display_dirty = true;
    }
    if (any_release_event) {
        g_display_phase = CAR_DISPLAY_PHASE_FOOTER;
        g_display_dirty = true;
    }
    if (now_display_period != g_displayed_period) {
        g_displayed_period = now_display_period;
        g_display_dirty = true;
    }

    CarDebugSnapshot_Update();
    if (g_display_dirty) {
        uint32_t display_started_ms = delay_get_ms();

        CarDisplay_Update(now_ms, g_display_phase);
        AppRuntimeMetrics_RecordDisplay(
            delay_get_ms() - display_started_ms);
        g_display_phase = JDY31_ConfigIsExclusive() ?
            CAR_DISPLAY_PHASE_HEADER :
            car_runtime_next_display_phase(active_workflow);
        g_display_dirty = false;
    }

    SystemWatchdog_Kick();
    __WFI();
}

static car_display_phase_t car_runtime_next_display_phase(
    car_app_workflow_t workflow)
{
    car_display_phase_t phase;

    if (g_display_priority_slot == 0U) {
        phase = CAR_DISPLAY_PHASE_ATTITUDE;
    } else if (g_display_priority_slot == 1U) {
        if ((workflow == CAR_APP_WORKFLOW_LINE_TEST) ||
            (workflow == CAR_APP_WORKFLOW_LINE_MISSION)) {
            phase = CAR_DISPLAY_PHASE_LINE;
        } else {
            phase = g_display_idle_line_next ?
                CAR_DISPLAY_PHASE_LINE : CAR_DISPLAY_PHASE_SPEED;
            g_display_idle_line_next = !g_display_idle_line_next;
        }
    } else {
        phase = g_display_slow_phases[g_display_slow_index];
        g_display_slow_index++;
        if (g_display_slow_index >=
            (sizeof(g_display_slow_phases) /
                sizeof(g_display_slow_phases[0]))) {
            g_display_slow_index = 0U;
        }
    }
    g_display_priority_slot++;
    if (g_display_priority_slot >= 3U) {
        g_display_priority_slot = 0U;
    }
    return phase;
}

static void car_runtime_process_action(
    const car_app_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return;
    }

    if (snapshot->action == CAR_APP_ACTION_STOP_ACTIVE) {
        switch (snapshot->active_workflow) {
#if CAR_ENABLE_BRINGUP
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
#endif
            case CAR_APP_WORKFLOW_LINE_MISSION:
                LineFollowMission_RequestStop();
                break;
            case CAR_APP_WORKFLOW_MOTION:
                MotionSupervisor_RequestStop();
                break;
#if CAR_ENABLE_BRINGUP
            case CAR_APP_WORKFLOW_YAW_TEST:
                YawBringupTest_RequestStop();
                break;
#endif
            default:
                break;
        }
    } else if (snapshot->action ==
        CAR_APP_ACTION_START_LINE_MISSION) {
        (void) LineFollowMission_RequestStart();
    }
}
