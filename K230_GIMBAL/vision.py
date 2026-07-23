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


def postprocess(results, labels, ratio, pad_top, pad_left):
    output_data = results[0][0].transpose()
    boxes = output_data[:, 0:4]
    class_scores = output_data[:, 4:]
    class_ids = np.argmax(class_scores, axis=-1)
    scores = np.max(class_scores, axis=-1)

    detections = []
    for index in range(len(boxes)):
        score = scores[index]
        if score <= config.CONFIDENCE_THRESHOLD:
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
    fps_value,
    focus_status,
    radio_status=None,
    can_status=None,
):
    osd_image.clear()
    osd_image.draw_string_advanced(
        8, 6, 24, "FPS %.1f" % fps_value, color=config.FPS_COLOR
    )
    osd_image.draw_string_advanced(
        8, 34, 22, focus_status, color=config.FOCUS_COLOR
    )
    if radio_status is not None:
        osd_image.draw_string_advanced(
            8, 60, 20, radio_status, color=config.FOCUS_COLOR
        )
    if can_status is not None:
        can_y = 84 if radio_status is not None else 60
        osd_image.draw_string_advanced(
            8, can_y, 20, can_status, color=config.FOCUS_COLOR
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
        if radio is not None:
            radio.open(last_status_ms)
        if can_gate is not None:
            can_gate.start()
        print("vision build:", config.BUILD_ID)

        while True:
            fps.tick()
            os.exitpoint()
            frame_count += 1

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

            target = select_target(detections)
            if target is None:
                last_target_valid = False
            else:
                last_cx, last_cy = target
                last_target_valid = True

            fps_value = fps.fps()
            now_ms = time.ticks_ms()
            radio_status = None
            if radio is not None:
                radio.task(now_ms)
                radio_status = radio.state_text()
            can_status = None
            if can_gate is not None:
                can_gate.task()
                can_status = can_gate.state_text()
            draw_detections(
                osd_image,
                detections,
                labels,
                fps_value,
                focus_status,
                radio_status,
                can_status,
            )
            Display.show_image(osd_image)

            if time.ticks_diff(now_ms, last_status_ms) >= config.STATUS_PRINT_MS:
                print(
                    "fps=%.2f boxes=%d valid=%d cx=%d cy=%d %s" %
                    (
                        fps_value,
                        len(detections),
                        1 if last_target_valid else 0,
                        last_cx,
                        last_cy,
                        focus_pos_text(sensor),
                    )
                )
                if can_gate is not None:
                    print("CAN status:", can_gate.snapshot())
                last_status_ms = now_ms

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
        if can_gate is not None:
            can_gate.close()
