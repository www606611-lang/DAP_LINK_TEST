import gc
import os
import sys
import time

import image
import nncase_runtime as nn
import ulab.numpy as np
from machine import FPIOA, UART
from media.display import *
from media.media import *
from media.sensor import *


KMODEL_PATH = "/data/best.kmodel"
LABELS_PATH = "/data/labels.txt"

SENSOR_ID = 2
DISPLAY_TYPE = Display.ST7701
DISPLAY_WIDTH = 800
DISPLAY_HEIGHT = 480
DISPLAY_TO_IDE = False

AI_WIDTH = 640
AI_HEIGHT = 384
MODEL_INPUT_SIZE = [320, 320]

ENABLE_FIXED_FOCUS = True
FIXED_FOCUS_POS = 210

UART_TX_PIN = 11
UART_RX_PIN = 12
UART_COORD_WIDTH = 400
UART_COORD_HEIGHT = 240
UART_SEND_EVERY_N_FRAMES = 1

CONFIDENCE_THRESHOLD = 0.45
NMS_THRESHOLD = 0.45
PRE_NMS_TOPK = 25
MAX_BOXES = 5
GC_EVERY_N_FRAMES = 20
STATUS_PRINT_MS = 2000

BOX_COLOR = (0, 255, 0)
TEXT_COLOR = (0, 255, 0)
FPS_COLOR = (255, 255, 0)
FOCUS_COLOR = (255, 255, 0)


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
        return Sensor(id=SENSOR_ID)
    except Exception as exc:
        print("Sensor(id=%s) unavailable, use default: %s" %
              (SENSOR_ID, repr(exc)))
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
    if not ENABLE_FIXED_FOCUS:
        return False
    try:
        result = sensor.focus_pos(FIXED_FOCUS_POS)
        time.sleep_ms(120)
        print("focus %s target=%d ret=%s %s" %
              (stage, FIXED_FOCUS_POS, result, focus_pos_text(sensor)))
        return result is not False
    except Exception as exc:
        print("focus %s failed: %s" % (stage, repr(exc)))
        return False


def uart_init():
    fpioa = FPIOA()
    fpioa.set_function(UART_TX_PIN, FPIOA.UART2_TXD)
    fpioa.set_function(UART_RX_PIN, FPIOA.UART2_RXD)
    uart = UART(
        UART.UART2,
        baudrate=115200,
        bits=UART.EIGHTBITS,
        parity=UART.PARITY_NONE,
        stop=UART.STOPBITS_ONE,
    )
    print("uart2 ready tx=io%d rx=io%d baud=115200" %
          (UART_TX_PIN, UART_RX_PIN))
    return uart


def uart_send_packet(uart, valid, cx, cy):
    if uart is None:
        return
    cx = clamp(int(cx), 0, UART_COORD_WIDTH - 1)
    cy = clamp(int(cy), 0, UART_COORD_HEIGHT - 1)
    uart.write("@%d,%03d,%03d#" % (1 if valid else 0, cx, cy))


def uart_send_detection(uart, detections, last_cx, last_cy):
    if detections:
        best = detections[0]
        for detection in detections[1:]:
            if detection[4] > best[4]:
                best = detection
        center_x = (int(best[0]) + int(best[2])) // 2
        center_y = (int(best[1]) + int(best[3])) // 2
        cx = (center_x * UART_COORD_WIDTH + (AI_WIDTH // 2)) // AI_WIDTH
        cy = (center_y * UART_COORD_HEIGHT + (AI_HEIGHT // 2)) // AI_HEIGHT
        uart_send_packet(uart, True, cx, cy)
        return cx, cy, True

    uart_send_packet(uart, False, last_cx, last_cy)
    return last_cx, last_cy, False


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
            if (detection[5] == existing[5] and
                    box_iou(detection, existing) > threshold):
                keep = False
                break
        if keep:
            kept.append(detection)
            if len(kept) >= MAX_BOXES:
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


def postprocess(results, labels, ratio, pad_top, pad_left):
    output_data = results[0][0].transpose()
    boxes = output_data[:, 0:4]
    class_scores = output_data[:, 4:]
    class_ids = np.argmax(class_scores, axis=-1)
    scores = np.max(class_scores, axis=-1)

    detections = []
    for index in range(len(boxes)):
        score = scores[index]
        if score <= CONFIDENCE_THRESHOLD:
            continue

        x, y, width, height = (
            boxes[index][0], boxes[index][1],
            boxes[index][2], boxes[index][3]
        )
        x1 = int((x - 0.5 * width - pad_left) / ratio)
        y1 = int((y - 0.5 * height - pad_top) / ratio)
        x2 = int((x + 0.5 * width - pad_left) / ratio)
        y2 = int((y + 0.5 * height - pad_top) / ratio)

        x1 = clamp(x1, 0, AI_WIDTH - 1)
        y1 = clamp(y1, 0, AI_HEIGHT - 1)
        x2 = clamp(x2, 0, AI_WIDTH - 1)
        y2 = clamp(y2, 0, AI_HEIGHT - 1)
        if x2 <= x1 or y2 <= y1:
            continue

        class_id = int(class_ids[index])
        if class_id >= len(labels):
            continue
        add_topk_detection(
            detections,
            [x1, y1, x2, y2, float(score), class_id],
            PRE_NMS_TOPK,
        )

    if not detections:
        return []
    return nms_detections(detections, NMS_THRESHOLD)


def draw_detections(osd_image, detections, labels, fps_value, focus_status):
    osd_image.clear()
    osd_image.draw_string_advanced(
        8, 6, 24, "FPS %.1f" % fps_value, color=FPS_COLOR)
    osd_image.draw_string_advanced(
        8, 34, 22, focus_status, color=FOCUS_COLOR)

    for detection in detections:
        x1, y1, x2, y2 = [int(value) for value in detection[:4]]
        score = detection[4]
        class_id = int(detection[5])
        label = labels[class_id]

        draw_x = int(x1 * DISPLAY_WIDTH // AI_WIDTH)
        draw_y = int(y1 * DISPLAY_HEIGHT // AI_HEIGHT)
        draw_width = int((x2 - x1) * DISPLAY_WIDTH // AI_WIDTH)
        draw_height = int((y2 - y1) * DISPLAY_HEIGHT // AI_HEIGHT)
        osd_image.draw_rectangle(
            draw_x, draw_y, draw_width, draw_height,
            color=BOX_COLOR, thickness=3)
        osd_image.draw_string_advanced(
            draw_x, max(0, draw_y - 30), 22,
            "%s %.2f" % (label, score), color=TEXT_COLOR)


def main():
    os.exitpoint(os.EXITPOINT_ENABLE)

    labels = load_labels(LABELS_PATH)
    display_width = align_up(DISPLAY_WIDTH, 16)
    ai_width = align_up(AI_WIDTH, 16)
    sensor = None
    uart = None
    ai2d = None
    ai2d_builder = None
    kpu = None
    kpu_input_tensor = None
    osd_image = None
    preview_layer_bound = False
    display_initialized = False
    media_initialized = False
    sensor_running = False
    last_cx = UART_COORD_WIDTH // 2
    last_cy = UART_COORD_HEIGHT // 2
    last_target_valid = False

    try:
        uart = uart_init()
        sensor = create_sensor()
        sensor.reset()
        sensor.set_framesize(
            width=display_width, height=DISPLAY_HEIGHT,
            chn=CAM_CHN_ID_0)
        sensor.set_pixformat(Sensor.YUV420SP, chn=CAM_CHN_ID_0)
        sensor.set_framesize(
            width=ai_width, height=AI_HEIGHT,
            chn=CAM_CHN_ID_1)
        sensor.set_pixformat(Sensor.RGBP888, chn=CAM_CHN_ID_1)

        focus_status = "FOCUS OFF"
        if ENABLE_FIXED_FOCUS:
            print("sensor id:", SENSOR_ID)
            print("focus", focus_caps_text(sensor))
            focus_status = ("FOCUS %d" % FIXED_FOCUS_POS
                            if set_fixed_focus(sensor, "before-run")
                            else "FOCUS N/A")

        bind_info = sensor.bind_info(x=0, y=0, chn=CAM_CHN_ID_0)
        Display.bind_layer(**bind_info, layer=Display.LAYER_VIDEO1)
        preview_layer_bound = True
        Display.init(
            DISPLAY_TYPE, width=display_width, height=DISPLAY_HEIGHT,
            osd_num=1, to_ide=DISPLAY_TO_IDE)
        display_initialized = True
        try:
            sensor._set_chn_fps(chn=CAM_CHN_ID_0, fps=Display.fps())
        except Exception:
            pass

        osd_image = image.Image(
            display_width, DISPLAY_HEIGHT, image.ARGB8888)
        top, bottom, left, right, ratio = letterbox_param(
            [ai_width, AI_HEIGHT], MODEL_INPUT_SIZE)
        ai2d = nn.ai2d()
        ai2d.set_dtype(
            nn.ai2d_format.NCHW_FMT, nn.ai2d_format.NCHW_FMT,
            np.uint8, np.uint8)
        ai2d.set_pad_param(
            True, [0, 0, 0, 0, top, bottom, left, right],
            0, [114, 114, 114])
        ai2d.set_resize_param(
            True, nn.interp_method.tf_bilinear,
            nn.interp_mode.half_pixel)
        ai2d_builder = ai2d.build(
            [1, 3, AI_HEIGHT, ai_width],
            [1, 3, MODEL_INPUT_SIZE[1], MODEL_INPUT_SIZE[0]])
        input_data = np.ones(
            (1, 3, MODEL_INPUT_SIZE[1], MODEL_INPUT_SIZE[0]),
            dtype=np.uint8)
        kpu_input_tensor = nn.from_numpy(input_data)
        kpu = nn.kpu()
        kpu.load_kmodel(KMODEL_PATH)

        MediaManager.init()
        media_initialized = True
        sensor.run()
        sensor_running = True
        if ENABLE_FIXED_FOCUS:
            time.sleep_ms(300)
            focus_status = ("FOCUS %d" % FIXED_FOCUS_POS
                            if set_fixed_focus(sensor, "after-run")
                            else "FOCUS N/A")

        fps = time.clock()
        frame_count = 0
        last_status_ms = time.ticks_ms()
        print("vision build: yolo-k230-fixed-focus-uart-v1")

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

            if frame_count % UART_SEND_EVERY_N_FRAMES == 0:
                last_cx, last_cy, last_target_valid = uart_send_detection(
                    uart, detections, last_cx, last_cy)

            fps_value = fps.fps()
            draw_detections(
                osd_image, detections, labels, fps_value, focus_status)
            Display.show_image(osd_image)

            now_ms = time.ticks_ms()
            if time.ticks_diff(now_ms, last_status_ms) >= STATUS_PRINT_MS:
                print(
                    "fps=%.2f boxes=%d valid=%d cx=%d cy=%d %s" %
                    (fps_value, len(detections),
                     1 if last_target_valid else 0,
                     last_cx, last_cy, focus_pos_text(sensor)))
                last_status_ms = now_ms

            if GC_EVERY_N_FRAMES and frame_count % GC_EVERY_N_FRAMES == 0:
                gc.collect()

    except KeyboardInterrupt as exc:
        print("user stop:", exc)
    except BaseException as exc:
        uart_send_packet(uart, False, last_cx, last_cy)
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
        if uart is not None:
            uart.deinit()


if __name__ == "__main__":
    main()
