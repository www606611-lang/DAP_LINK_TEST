"""Accepted YOLOv8/KPU vision pipeline with bounded resource cleanup."""

import gc
import os
import sys
import time

import config

try:
    import image
    import nncase_runtime as nn
    import ulab.numpy as np
    from media.display import Display
    from media.media import MediaManager
    from media.sensor import CAM_CHN_ID_0, CAM_CHN_ID_1, Sensor

    CANMV_RUNTIME = True
except ImportError:
    CANMV_RUNTIME = False
    image = None
    nn = None
    np = None
    Display = None
    MediaManager = None
    Sensor = None
    CAM_CHN_ID_0 = 0
    CAM_CHN_ID_1 = 1


def align_up(value, align):
    return ((value + align - 1) // align) * align


def clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


def ticks_diff(now_ms, then_ms):
    runtime_ticks_diff = getattr(time, "ticks_diff", None)
    if runtime_ticks_diff is not None:
        return runtime_ticks_diff(now_ms, then_ms)
    return int(now_ms) - int(then_ms)


def print_exception(exc):
    printer = getattr(sys, "print_exception", None)
    if printer:
        printer(exc)
    else:
        print("Exception:", repr(exc))


def load_labels(path):
    labels = []
    try:
        with open(path, "r") as label_file:
            for line in label_file:
                name = line.strip()
                if name:
                    labels.append(name)
    except OSError:
        labels = ["target"]
    return labels if labels else ["target"]


def create_sensor():
    try:
        return Sensor(id=config.SENSOR_ID)
    except Exception as exc:
        print(
            "Sensor(id=%s) unavailable, use default: %s" %
            (config.SENSOR_ID, repr(exc))
        )
        return Sensor()


def focus_caps_text(sensor):
    try:
        return "caps=%s" % (sensor.focus_caps(),)
    except Exception as exc:
        return "caps unavailable: %s" % repr(exc)


def focus_pos_text(sensor):
    try:
        return "pos=%s" % (sensor.focus_pos(),)
    except Exception:
        return "pos=?"


def set_fixed_focus(sensor, stage):
    if not config.ENABLE_FIXED_FOCUS:
        return False
    try:
        result = sensor.focus_pos(config.FIXED_FOCUS_POS)
        time.sleep_ms(120)
        print(
            "focus %s target=%d ret=%s %s" %
            (stage, config.FIXED_FOCUS_POS, result, focus_pos_text(sensor))
        )
        return result is not False
    except Exception as exc:
        print("focus %s failed: %s" % (stage, repr(exc)))
        return False


def box_iou(box_a, box_b):
    x1 = max(box_a[0], box_b[0])
    y1 = max(box_a[1], box_b[1])
    x2 = min(box_a[2], box_b[2])
    y2 = min(box_a[3], box_b[3])
    width = max(0, x2 - x1)
    height = max(0, y2 - y1)
    intersection = width * height
    area_a = max(0, box_a[2] - box_a[0]) * max(0, box_a[3] - box_a[1])
    area_b = max(0, box_b[2] - box_b[0]) * max(0, box_b[3] - box_b[1])
    union = area_a + area_b - intersection
    if union <= 0:
        return 0
    return intersection / union


def nms_detections(detections, threshold):
    detections.sort(key=lambda item: item[4], reverse=True)
    kept = []
    for detection in detections:
        keep = True
        for existing in kept:
            if (
                detection[5] == existing[5]
                and box_iou(detection, existing) > threshold
            ):
                keep = False
                break
        if keep:
            kept.append(detection)
            if len(kept) >= config.MAX_BOXES:
                break
    return kept


def add_topk_detection(detections, detection, topk):
    if len(detections) < topk:
        detections.append(detection)
        return

    minimum_index = 0
    minimum_score = detections[0][4]
    for index in range(1, len(detections)):
        if detections[index][4] < minimum_score:
            minimum_score = detections[index][4]
            minimum_index = index
    if detection[4] > minimum_score:
        detections[minimum_index] = detection


def letterbox_param(input_size, output_size):
    input_width, input_height = input_size
    output_width, output_height = output_size
    ratio = min(output_width / input_width, output_height / input_height)
    resized_width = int(ratio * input_width)
    resized_height = int(ratio * input_height)
    pad_width = output_width - resized_width
    pad_height = output_height - resized_height
    left = int(pad_width / 2)
    right = pad_width - left
    top = int(pad_height / 2)
    bottom = pad_height - top
    return top, bottom, left, right, ratio


def map_target_center(center_x, center_y):
    cx = (
        int(center_x) * config.TARGET_COORD_WIDTH + (config.AI_WIDTH // 2)
    ) // config.AI_WIDTH
    cy = (
        int(center_y) * config.TARGET_COORD_HEIGHT + (config.AI_HEIGHT // 2)
    ) // config.AI_HEIGHT
    return (
        clamp(cx, 0, config.TARGET_COORD_WIDTH - 1),
        clamp(cy, 0, config.TARGET_COORD_HEIGHT - 1),
    )


def select_target(detections):
    if not detections:
        return None
    best = detections[0]
    for detection in detections[1:]:
        if detection[4] > best[4]:
            best = detection
    center_x = (int(best[0]) + int(best[2])) // 2
    center_y = (int(best[1]) + int(best[3])) // 2
    return map_target_center(center_x, center_y)


class TargetSelector:
    """Keep one detection through short confidence drops near image edges."""

    def __init__(
        self,
        acquire_confidence,
        hold_confidence,
        max_match_distance_px,
        forget_ms,
    ):
        self.acquire_confidence = float(acquire_confidence)
        self.hold_confidence = float(hold_confidence)
        self.max_match_distance_sq = float(max_match_distance_px) ** 2
        self.forget_ms = int(forget_ms)
        if not 0.0 <= self.hold_confidence <= self.acquire_confidence <= 1.0:
            raise ValueError("invalid target confidence hysteresis")
        if self.max_match_distance_sq <= 0.0 or self.forget_ms <= 0:
            raise ValueError("invalid target continuity limits")
        self.last_center = None
        self.last_class_id = None
        self.last_seen_ms = None

    def select(self, detections, now_ms):
        now_ms = int(now_ms)
        if (
            self.last_seen_ms is not None
            and ticks_diff(now_ms, self.last_seen_ms) >= self.forget_ms
        ):
            self.reset()

        selected = None
        if self.last_center is not None:
            selected = self._nearest_held_detection(detections)
            if selected is None:
                return None
        else:
            selected = self._highest_confidence_detection(
                detections, self.acquire_confidence
            )
        if selected is None:
            return None

        center_x, center_y = self._center(selected)
        self.last_center = (center_x, center_y)
        self.last_class_id = int(selected[5])
        self.last_seen_ms = now_ms
        return map_target_center(center_x, center_y)

    def reset(self):
        self.last_center = None
        self.last_class_id = None
        self.last_seen_ms = None

    def _nearest_held_detection(self, detections):
        best = None
        best_distance_sq = None
        for detection in detections:
            if (
                float(detection[4]) < self.hold_confidence
                or int(detection[5]) != self.last_class_id
            ):
                continue
            center_x, center_y = self._center(detection)
            dx = center_x - self.last_center[0]
            dy = center_y - self.last_center[1]
            distance_sq = dx * dx + dy * dy
            if distance_sq > self.max_match_distance_sq:
                continue
            if best is None or distance_sq < best_distance_sq:
                best = detection
                best_distance_sq = distance_sq
        return best

    @staticmethod
    def _highest_confidence_detection(detections, minimum_confidence):
        best = None
        for detection in detections:
            if float(detection[4]) < minimum_confidence:
                continue
            if best is None or detection[4] > best[4]:
                best = detection
        return best

    @staticmethod
    def _center(detection):
        return (
            (int(detection[0]) + int(detection[2])) // 2,
            (int(detection[1]) + int(detection[3])) // 2,
        )


def postprocess(results, labels, ratio, pad_top, pad_left):
    output_data = results[0][0].transpose()
    boxes = output_data[:, 0:4]
    class_scores = output_data[:, 4:]
    class_ids = np.argmax(class_scores, axis=-1)
    scores = np.max(class_scores, axis=-1)

    detections = []
    for index in range(len(boxes)):
        score = scores[index]
        if score < config.CONFIDENCE_THRESHOLD:
            continue

        x, y, width, height = (
            boxes[index][0],
            boxes[index][1],
            boxes[index][2],
            boxes[index][3],
        )
        x1 = int((x - 0.5 * width - pad_left) / ratio)
        y1 = int((y - 0.5 * height - pad_top) / ratio)
        x2 = int((x + 0.5 * width - pad_left) / ratio)
        y2 = int((y + 0.5 * height - pad_top) / ratio)

        x1 = clamp(x1, 0, config.AI_WIDTH - 1)
        y1 = clamp(y1, 0, config.AI_HEIGHT - 1)
        x2 = clamp(x2, 0, config.AI_WIDTH - 1)
        y2 = clamp(y2, 0, config.AI_HEIGHT - 1)
        if x2 <= x1 or y2 <= y1:
            continue

        class_id = int(class_ids[index])
        if class_id >= len(labels):
            continue
        add_topk_detection(
            detections,
            [x1, y1, x2, y2, float(score), class_id],
            config.PRE_NMS_TOPK,
        )

    if not detections:
        return []
    return nms_detections(detections, config.NMS_THRESHOLD)


def draw_detections(
    osd_image,
    detections,
    labels,
    hud,
):
    osd_image.clear()
    osd_image.draw_string_advanced(
        10, 6, 28, hud["state"], color=hud["state_color"]
    )
    osd_image.draw_string_advanced(
        10, 40, 21, hud["target"], color=config.HUD_INFO_COLOR
    )
    osd_image.draw_string_advanced(
        10, 66, 21, hud["error"], color=config.HUD_INFO_COLOR
    )
    osd_image.draw_string_advanced(
        580, 6, 23, hud["performance"], color=config.FPS_COLOR
    )
    osd_image.draw_string_advanced(
        650, 36, 19, hud["focus"], color=config.FOCUS_COLOR
    )
    if hud["radio"]:
        osd_image.draw_string_advanced(
            560, 62, 19, hud["radio"], color=config.HUD_INFO_COLOR
        )

    osd_image.draw_string_advanced(
        10, 418, 21, hud["yaw"], color=config.HUD_INFO_COLOR
    )
    osd_image.draw_string_advanced(
        10, 446, 21, hud["pitch"], color=config.HUD_INFO_COLOR
    )
    osd_image.draw_string_advanced(
        555, 418, 20, hud["can"], color=hud["can_color"]
    )
    osd_image.draw_string_advanced(
        555, 446, 20, hud["bus"], color=config.HUD_INFO_COLOR
    )

    center_x = config.DISPLAY_WIDTH // 2
    center_y = config.DISPLAY_HEIGHT // 2
    osd_image.draw_rectangle(
        center_x - 10,
        center_y - 10,
        20,
        20,
        color=config.FOCUS_COLOR,
        thickness=2,
    )

    for detection in detections:
        x1, y1, x2, y2 = [int(value) for value in detection[:4]]
        score = detection[4]
        class_id = int(detection[5])
        label = labels[class_id]

        draw_x = int(x1 * config.DISPLAY_WIDTH // config.AI_WIDTH)
        draw_y = int(y1 * config.DISPLAY_HEIGHT // config.AI_HEIGHT)
        draw_width = int((x2 - x1) * config.DISPLAY_WIDTH // config.AI_WIDTH)
        draw_height = int((y2 - y1) * config.DISPLAY_HEIGHT // config.AI_HEIGHT)
        osd_image.draw_rectangle(
            draw_x,
            draw_y,
            draw_width,
            draw_height,
            color=config.BOX_COLOR,
            thickness=3,
        )
        osd_image.draw_string_advanced(
            draw_x,
            max(0, draw_y - 30),
            22,
            "%s %.2f" % (label, score),
            color=config.TEXT_COLOR,
        )


def _voltage_text(millivolts):
    if not millivolts:
        return "--.-"
    return "%.1f" % (float(millivolts) / 1000.0)


def build_hud(
    fps_value,
    vision_latency_ms,
    focus_status,
    target,
    tracker=None,
    can_gate=None,
    radio_status=None,
):
    state = "VISION"
    state_color = config.HUD_GOOD_COLOR
    error_x = 0
    error_y = 0
    yaw_rpm = 0.0
    pitch_rpm = 0.0
    yaw_position = 0.0
    pitch_position = 0.0
    yaw_bus_mv = 0
    pitch_bus_mv = 0
    command_retries = 0

    if tracker is not None:
        state = tracker.state_text()
        error_x = tracker.raw_error_x
        error_y = tracker.raw_error_y
        yaw_rpm = tracker.command_yaw_rpm
        pitch_rpm = tracker.command_pitch_rpm
        supervisor = tracker.supervisor
        yaw_position = supervisor.positions["yaw"]
        pitch_position = supervisor.positions["pitch"]
        yaw_bus_mv = supervisor.bus_mv["yaw"]
        pitch_bus_mv = supervisor.bus_mv["pitch"]
        command_retries = supervisor.command_retry_count
        if "FAULT" in state:
            state_color = config.HUD_BAD_COLOR
        elif "LOST" in state or "SEARCH" in state:
            state_color = config.HUD_WARN_COLOR

    if target is None:
        target_text = "TARGET ---"
        error_text = "ERROR  X---  Y---"
    else:
        target_text = "TARGET X%03d Y%03d" % (target[0], target[1])
        error_text = "ERROR  X%+4d Y%+4d" % (error_x, error_y)

    can_text = "CAN OFF"
    can_color = config.HUD_WARN_COLOR
    if can_gate is not None:
        controller = can_gate.controller
        errors = getattr(controller, "error_count", 0) if controller else 0
        timeouts = (
            getattr(controller, "timeout_count", 0) if controller else 0
        )
        if can_gate.state == can_gate.STATE_FAILED:
            can_text = "CAN FAIL E%d T%d R%d" % (
                errors, timeouts, command_retries
            )
            can_color = config.HUD_BAD_COLOR
        elif can_gate.state == can_gate.STATE_ACTIVE:
            health = "OK" if errors == 0 and timeouts == 0 else "WARN"
            can_text = "CAN %s E%d T%d R%d" % (
                health, errors, timeouts, command_retries
            )
            can_color = (
                config.HUD_GOOD_COLOR
                if health == "OK"
                else config.HUD_WARN_COLOR
            )
        else:
            can_text = "%s E%d T%d R%d" % (
                can_gate.state_text(), errors, timeouts, command_retries
            )

    return {
        "state": state,
        "state_color": state_color,
        "target": target_text,
        "error": error_text,
        "performance": "FPS %.1f  AI %dms" %
        (fps_value, int(vision_latency_ms)),
        "focus": focus_status.replace("FOCUS ", "F "),
        "radio": radio_status or "",
        "yaw": "YAW %+5.1f RPM  %+6.1f DEG" % (yaw_rpm, yaw_position),
        "pitch": "PITCH %+5.1f RPM  %+6.1f DEG" %
        (pitch_rpm, pitch_position),
        "can": can_text,
        "can_color": can_color,
        "bus": "BUS Y%s P%sV" %
        (_voltage_text(yaw_bus_mv), _voltage_text(pitch_bus_mv)),
    }


def run():
    if not CANMV_RUNTIME:
        raise RuntimeError("vision.run requires the CanMV K230 runtime")

    os.exitpoint(os.EXITPOINT_ENABLE)
    labels = load_labels(config.LABELS_PATH)
    display_width = align_up(config.DISPLAY_WIDTH, 16)
    ai_width = align_up(config.AI_WIDTH, 16)
    sensor = None
    radio = None
    if config.CHASSIS_RADIO_ENABLED:
        from chassis_radio import ChassisRadio

        radio = ChassisRadio()
    can_gate = None
    tracker = None
    if config.GIMBAL_MOTION_ENABLED and not config.CAN_ENABLED:
        raise RuntimeError("gimbal motion requires CAN")
    if config.CAN_ENABLED:
        from mcp2515 import MCP2515RuntimeGate

        can_gate = MCP2515RuntimeGate(config)
    ai2d = None
    ai2d_builder = None
    kpu = None
    kpu_input_tensor = None
    osd_image = None
    preview_layer_bound = False
    display_initialized = False
    media_initialized = False
    sensor_running = False
    last_cx = config.TARGET_COORD_WIDTH // 2
    last_cy = config.TARGET_COORD_HEIGHT // 2
    last_target_valid = False
    target_selector = TargetSelector(
        config.TARGET_ACQUIRE_CONFIDENCE,
        config.TARGET_HOLD_CONFIDENCE,
        config.TARGET_MATCH_MAX_DISTANCE_PX,
        config.TARGET_FORGET_MS,
    )

    try:
        sensor = create_sensor()
        sensor.reset()
        sensor.set_framesize(
            width=display_width,
            height=config.DISPLAY_HEIGHT,
            chn=CAM_CHN_ID_0,
        )
        sensor.set_pixformat(Sensor.YUV420SP, chn=CAM_CHN_ID_0)
        sensor.set_framesize(
            width=ai_width,
            height=config.AI_HEIGHT,
            chn=CAM_CHN_ID_1,
        )
        sensor.set_pixformat(Sensor.RGBP888, chn=CAM_CHN_ID_1)

        focus_status = "FOCUS OFF"
        if config.ENABLE_FIXED_FOCUS:
            print("sensor id:", config.SENSOR_ID)
            print("focus", focus_caps_text(sensor))
            focus_status = (
                "FOCUS %d" % config.FIXED_FOCUS_POS
                if set_fixed_focus(sensor, "before-run")
                else "FOCUS N/A"
            )

        bind_info = sensor.bind_info(x=0, y=0, chn=CAM_CHN_ID_0)
        Display.bind_layer(**bind_info, layer=Display.LAYER_VIDEO1)
        preview_layer_bound = True
        display_type = getattr(Display, config.DISPLAY_DRIVER)
        Display.init(
            display_type,
            width=display_width,
            height=config.DISPLAY_HEIGHT,
            osd_num=1,
            to_ide=config.DISPLAY_TO_IDE,
        )
        display_initialized = True
        try:
            sensor._set_chn_fps(chn=CAM_CHN_ID_0, fps=Display.fps())
        except Exception:
            pass

        osd_image = image.Image(
            display_width, config.DISPLAY_HEIGHT, image.ARGB8888
        )
        top, bottom, left, right, ratio = letterbox_param(
            [ai_width, config.AI_HEIGHT], config.MODEL_INPUT_SIZE
        )
        ai2d = nn.ai2d()
        ai2d.set_dtype(
            nn.ai2d_format.NCHW_FMT,
            nn.ai2d_format.NCHW_FMT,
            np.uint8,
            np.uint8,
        )
        ai2d.set_pad_param(
            True,
            [0, 0, 0, 0, top, bottom, left, right],
            0,
            [114, 114, 114],
        )
        ai2d.set_resize_param(
            True, nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel
        )
        ai2d_builder = ai2d.build(
            [1, 3, config.AI_HEIGHT, ai_width],
            [1, 3, config.MODEL_INPUT_SIZE[1], config.MODEL_INPUT_SIZE[0]],
        )
        input_data = np.ones(
            (1, 3, config.MODEL_INPUT_SIZE[1], config.MODEL_INPUT_SIZE[0]),
            dtype=np.uint8,
        )
        kpu_input_tensor = nn.from_numpy(input_data)
        kpu = nn.kpu()
        kpu.load_kmodel(config.KMODEL_PATH)

        MediaManager.init()
        media_initialized = True
        sensor.run()
        sensor_running = True
        if config.ENABLE_FIXED_FOCUS:
            time.sleep_ms(300)
            focus_status = (
                "FOCUS %d" % config.FIXED_FOCUS_POS
                if set_fixed_focus(sensor, "after-run")
                else "FOCUS N/A"
            )

        fps = time.clock()
        frame_count = 0
        last_status_ms = time.ticks_ms()
        last_hud_ms = 0
        hud = None
        if radio is not None:
            radio.open(last_status_ms)
        if can_gate is not None:
            if not can_gate.start():
                raise RuntimeError("CAN gate failed: %s" % can_gate.last_error)
            if config.GIMBAL_MOTION_ENABLED:
                from gimbal_control import GimbalSupervisor, TargetTracker

                controller = can_gate.activate_normal()
                supervisor = GimbalSupervisor(
                    controller,
                    config.GIMBAL_YAW_CAN_ADDRESS,
                    config.GIMBAL_PITCH_CAN_ADDRESS,
                    config.GIMBAL_YAW_SESSION_MIN_DEG,
                    config.GIMBAL_YAW_SESSION_MAX_DEG,
                    config.GIMBAL_PITCH_SESSION_MIN_DEG,
                    config.GIMBAL_PITCH_SESSION_MAX_DEG,
                    config.GIMBAL_YAW_MAX_RPM,
                    config.GIMBAL_PITCH_MAX_RPM,
                    config.GIMBAL_COMMAND_LEASE_MS,
                    config.GIMBAL_FEEDBACK_POLL_MS,
                    config.GIMBAL_VOLTAGE_POLL_MS,
                    config.GIMBAL_POSITION_TOLERANCE_DEG,
                    yaw_origin_deg=config.GIMBAL_YAW_CENTER_DEG,
                    pitch_origin_deg=config.GIMBAL_PITCH_CENTER_DEG,
                    yaw_continuous=config.GIMBAL_YAW_CONTINUOUS,
                    pitch_continuous=config.GIMBAL_PITCH_CONTINUOUS,
                )
                tracker = TargetTracker(
                    supervisor,
                    config.TARGET_COORD_WIDTH,
                    config.TARGET_COORD_HEIGHT,
                    config.TRACKING_DEADBAND_X,
                    config.TRACKING_DEADBAND_Y,
                    config.TRACKING_DEADBAND_HYSTERESIS_X,
                    config.TRACKING_DEADBAND_HYSTERESIS_Y,
                    config.TRACKING_YAW_RPM_PER_PIXEL,
                    config.TRACKING_PITCH_RPM_PER_PIXEL,
                    config.TRACKING_FILTER_ALPHA,
                    config.TRACKING_UPDATE_MS,
                    config.TRACKING_MISSING_STOP_MS,
                    config.TRACKING_LOST_TIMEOUT_MS,
                    config.TRACKING_MIN_YAW_RPM,
                    config.TRACKING_MIN_PITCH_RPM,
                    config.TRACKING_MAX_YAW_RPM,
                    config.TRACKING_MAX_PITCH_RPM,
                    config.TRACKING_YAW_ACCELERATION_RPM_S,
                    config.TRACKING_PITCH_ACCELERATION,
                    config.TRACKING_SPEED_CHANGE_RPM,
                    config.TRACKING_COMMAND_REFRESH_MS,
                    config.TRACKING_COMMAND_LEASE_MS,
                )
                print("tracker arm:", tracker.start(time.ticks_ms()))
        print("vision build:", config.BUILD_ID)

        while True:
            fps.tick()
            os.exitpoint()
            frame_count += 1
            frame_started_ms = time.ticks_ms()

            frame = sensor.snapshot(chn=CAM_CHN_ID_1)
            ai2d_input_tensor = nn.from_numpy(frame.to_numpy_ref())
            ai2d_builder.run(ai2d_input_tensor, kpu_input_tensor)
            kpu.set_input_tensor(0, kpu_input_tensor)
            kpu.run()

            results = []
            for output_index in range(kpu.outputs_size()):
                output_tensor = kpu.get_output_tensor(output_index)
                results.append(output_tensor.to_numpy())
                del output_tensor

            detections = postprocess(results, labels, ratio, top, left)
            del results
            del ai2d_input_tensor

            now_ms = time.ticks_ms()
            vision_latency_ms = max(
                0, time.ticks_diff(now_ms, frame_started_ms)
            )
            target = target_selector.select(detections, now_ms)
            if target is None:
                last_target_valid = False
            else:
                last_cx, last_cy = target
                last_target_valid = True

            fps_value = fps.fps()
            if tracker is not None:
                tracker.task(target, now_ms)
            radio_status = None
            if radio is not None:
                radio.task(now_ms)
                radio_status = radio.state_text()
            if can_gate is not None:
                if tracker is None:
                    can_gate.task()
            if (
                hud is None
                or time.ticks_diff(now_ms, last_hud_ms) >= config.HUD_UPDATE_MS
            ):
                hud = build_hud(
                    fps_value,
                    vision_latency_ms,
                    focus_status,
                    target,
                    tracker,
                    can_gate,
                    radio_status,
                )
                last_hud_ms = now_ms
            draw_detections(
                osd_image,
                detections,
                labels,
                hud,
            )
            Display.show_image(osd_image)
            frame_done_ms = time.ticks_ms()
            frame_latency_ms = max(
                0, time.ticks_diff(frame_done_ms, frame_started_ms)
            )

            if time.ticks_diff(frame_done_ms, last_status_ms) >= config.STATUS_PRINT_MS:
                can_snapshot = (
                    can_gate.snapshot() if can_gate is not None else {}
                )
                tracker_snapshot = (
                    tracker.snapshot(frame_done_ms)
                    if tracker is not None else {}
                )
                supervisor_snapshot = tracker_snapshot.get("supervisor", {})
                tracker_timing = tracker_snapshot.get("timing", {})
                supervisor_timing = supervisor_snapshot.get("timing", {})
                motion_timing = supervisor_timing.get(
                    "motion_response_ms", {}
                )
                print(
                    "PERF fps=%.2f frame_ms=%d vision_ms=%d "
                    "control_ms=%d can_ms=%d motion_y_ms=%s "
                    "motion_p_ms=%s" %
                    (
                        fps_value,
                        frame_latency_ms,
                        vision_latency_ms,
                        tracker_timing.get("control_ms", 0),
                        supervisor_timing.get("can_command_ms", 0),
                        str(motion_timing.get("yaw")),
                        str(motion_timing.get("pitch")),
                    )
                )
                print(
                    "TRACK state=%s event=%s boxes=%d valid=%d "
                    "cx=%d cy=%d cmd=%d lease_expired=%d "
                    "lease_recovered=%d can_errors=%d timeouts=%d "
                    "tec=%d rec=%d" %
                    (
                        tracker_snapshot.get("state_text", "OFF"),
                        tracker_snapshot.get("event", "NONE"),
                        len(detections),
                        1 if last_target_valid else 0,
                        last_cx,
                        last_cy,
                        tracker_snapshot.get("command_count", 0),
                        supervisor_snapshot.get("lease_expired_count", 0),
                        tracker_snapshot.get("lease_recovery_count", 0),
                        can_snapshot.get("errors", 0),
                        can_snapshot.get("timeouts", 0),
                        can_snapshot.get("tec", 0),
                        can_snapshot.get("rec", 0),
                    )
                )
                last_status_ms = frame_done_ms

            if (
                config.GC_EVERY_N_FRAMES
                and frame_count % config.GC_EVERY_N_FRAMES == 0
            ):
                gc.collect()

    except KeyboardInterrupt as exc:
        print("user stop:", exc)
    except BaseException as exc:
        print_exception(exc)
    finally:
        if sensor_running and isinstance(sensor, Sensor):
            sensor.stop()
        if preview_layer_bound:
            try:
                Display.unbind_layer(Display.LAYER_VIDEO1)
            except Exception:
                pass
        if display_initialized:
            Display.deinit()
            time.sleep_ms(50)
        if media_initialized:
            MediaManager.deinit()
        if ai2d_builder is not None:
            del ai2d_builder
        if ai2d is not None:
            del ai2d
        if kpu_input_tensor is not None:
            del kpu_input_tensor
        if kpu is not None:
            del kpu
        nn.shrink_memory_pool()
        if radio is not None:
            radio.close()
        if tracker is not None:
            try:
                tracker.close()
            except BaseException as exc:
                print("tracker close failed:", repr(exc))
        if can_gate is not None:
            can_gate.close()
