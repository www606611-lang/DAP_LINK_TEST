import os
import time
import gc

import image
from media.display import *
from media.media import *
from media.sensor import *
from machine import FPIOA
from machine import UART

sensor = None
sensor_id = 2
AUTO_FOCUS_ENABLE = True
PREVIEW_ENABLE = True
PREVIEW_W = 800
PREVIEW_H = 480

LCD_W = 800
LCD_H = 480

UART_COORD_W = 400
UART_COORD_H = 240

IMG_W = 256
IMG_H = 160

MIN_AREA = (2200 * IMG_W * IMG_H) // (UART_COORD_W * UART_COORD_H)
MIN_WIDTH = (40 * IMG_W) // UART_COORD_W
MIN_HEIGHT = (28 * IMG_H) // UART_COORD_H
MIN_FRAME_COVERAGE = 0.025

TARGET_RECT_ASPECT = 1.50
BLOB_ENABLE = True
COMBO_BLOB_ENABLE = False
BRIGHT_WINDOW_ENABLE = True
BLOB_MIN_AREA = max(900, MIN_AREA // 2)
BLOB_MIN_PIXELS = max(420, BLOB_MIN_AREA // 3)
BLOB_MIN_DENSITY = 0.08
BLOB_MAX_DENSITY = 0.52
BLOB_ASPECT_MIN = 1.10
BLOB_ASPECT_MAX = 4.20
BLOB_X_STRIDE = 2
BLOB_Y_STRIDE = 2
BLOB_THRESH_MARGIN_MIN = 12
BLOB_THRESH_MARGIN_MAX = 36
BLOB_DARK_ONLY = True
COMBO_MERGE_MARGIN = 3
COMBO_MIN_AREA = max(520, BLOB_MIN_AREA // 2)
COMBO_MIN_PIXELS = max(220, BLOB_MIN_PIXELS // 2)
COMBO_DENSITY_MIN = 0.22
COMBO_DENSITY_MAX = 0.96
COMBO_ASPECT_MIN = 0.80
COMBO_ASPECT_MAX = 5.20
TRACK_COMBO_ASPECT_MIN = 0.75
TRACK_COMBO_ASPECT_MAX = 6.20
COMBO_SCORE_BONUS = 2600.0
BRIGHT_MIN_AREA = max(300, BLOB_MIN_AREA // 3)
BRIGHT_MIN_PIXELS = max(160, BLOB_MIN_PIXELS // 3)
BRIGHT_MIN_DENSITY = 0.25
BRIGHT_MAX_DENSITY = 0.96
BRIGHT_ASPECT_MIN = 0.75
BRIGHT_ASPECT_MAX = 5.20
TRACK_BRIGHT_ASPECT_MIN = 0.70
TRACK_BRIGHT_ASPECT_MAX = 6.00
BRIGHT_EDGE_TOUCH_REJECT = 2
BRIGHT_EDGE_AREA_REJECT = 0.72
BRIGHT_OUTER_MARGIN_X = max(4, (8 * IMG_W) // UART_COORD_W)
BRIGHT_OUTER_MARGIN_Y = max(4, (8 * IMG_H) // UART_COORD_H)
BRIGHT_BOX_MARGIN_X = max(2, (4 * IMG_W) // UART_COORD_W)
BRIGHT_BOX_MARGIN_Y = max(2, (4 * IMG_H) // UART_COORD_H)
BRIGHT_SCORE_BONUS = 1800.0
BLOB_EDGE_TOUCH_REJECT = 2
BLOB_EDGE_AREA_REJECT = 0.82
BLOB_EDGE_DENSITY_REJECT = 0.55
BLOB_PIXEL_SCORE_GAIN = 1.6
BLOB_DENSITY_SCORE_GAIN = 2400.0
BLOB_ASPECT_SCORE_SCALE = 0.48
MERGE_BLOB_ENABLE = True
MERGE_BLOB_MAX_COUNT = 6
MERGE_MIN_WIDTH = max(16, MIN_WIDTH // 2)
MERGE_MIN_HEIGHT = max(12, MIN_HEIGHT // 2)
MERGE_MIN_AREA = max(320, BLOB_MIN_AREA // 4)
MERGE_MIN_PIXELS = max(120, BLOB_MIN_PIXELS // 4)
MERGE_DENSITY_MIN = 0.025
MERGE_DENSITY_MAX = 0.72
MERGE_FILL_MIN = 0.045
MERGE_FILL_MAX = 0.38
MERGE_ASPECT_MIN = 1.00
MERGE_ASPECT_MAX = 4.60
MERGE_GAP_X_MAX = max(16, (26 * IMG_W) // UART_COORD_W)
MERGE_GAP_Y_MAX = max(12, (20 * IMG_H) // UART_COORD_H)
MERGE_AREA_EXPAND_MAX = 3.80
SOURCE_SWITCH_AREA_LOW = 0.35
SOURCE_SWITCH_AREA_HIGH = 2.80
FRAME_BORDER_MIN_THICK = max(4, (6 * IMG_W) // UART_COORD_W)
FRAME_CENTER_MARGIN_X = max(8, (14 * IMG_W) // UART_COORD_W)
FRAME_CENTER_MARGIN_Y = max(6, (12 * IMG_H) // UART_COORD_H)
FRAME_SIDE_DIFF_MIN = 9
FRAME_CONTRAST_MIN = 11
FRAME_SIDE_PASS_MIN = 2
COMBO_FRAME_SIDE_DIFF_MIN = 7
COMBO_FRAME_CONTRAST_MIN = 8
COMBO_FRAME_SIDE_PASS_MIN = 2
TRACK_BLOB_ASPECT_MIN = 0.90
TRACK_BLOB_ASPECT_MAX = 4.80
TRACK_BLOB_MAX_DENSITY = 0.68
TRACK_FRAME_SIDE_DIFF_MIN = 4
TRACK_FRAME_CONTRAST_MIN = 6
TRACK_FRAME_SIDE_PASS_MIN = 2
TRACK_COMBO_FRAME_SIDE_DIFF_MIN = 3
TRACK_COMBO_FRAME_CONTRAST_MIN = 4
TRACK_COMBO_FRAME_SIDE_PASS_MIN = 2

LOST_TOL = 6
CONTROL_HOLD_MS = 120
CENTER_JUMP_LIMIT = (92 * IMG_W) // UART_COORD_W
CORNER_JUMP_LIMIT = (60 * IMG_W) // UART_COORD_W
AREA_CHANGE_LOW = 0.50
AREA_CHANGE_HIGH = 1.90
SLOW_SMOOTH_A = 0.58
MID_SMOOTH_A = 0.32
FAST_SMOOTH_A = 0.12
SLOW_MOVE_LIMIT = max(6, (10 * IMG_W) // UART_COORD_W)
FAST_MOVE_LIMIT = max(18, (30 * IMG_W) // UART_COORD_W)

USE_ROI = True
CENTER_ROI_MARGIN_X = max(32, (52 * IMG_W) // UART_COORD_W)
CENTER_ROI_MARGIN_Y = max(20, (34 * IMG_H) // UART_COORD_H)
TRACK_ROI_BASE_MARGIN_X = max(28, (44 * IMG_W) // UART_COORD_W)
TRACK_ROI_BASE_MARGIN_Y = max(18, (30 * IMG_H) // UART_COORD_H)
TRACK_ROI_LOST_GAIN_X = max(8, (12 * IMG_W) // UART_COORD_W)
TRACK_ROI_LOST_GAIN_Y = max(6, (10 * IMG_H) // UART_COORD_H)
TRACK_ROI_MAX_MARGIN_X = max(56, (84 * IMG_W) // UART_COORD_W)
TRACK_ROI_MAX_MARGIN_Y = max(36, (56 * IMG_H) // UART_COORD_H)
TRACK_PREDICT_LIMIT_X = max(16, (24 * IMG_W) // UART_COORD_W)
TRACK_PREDICT_LIMIT_Y = max(12, (18 * IMG_H) // UART_COORD_H)
TRACK_REACQUIRE_FULL_AFTER = 3

ASPECT_SCORE_PENALTY = 12000.0
CENTER_SCORE_PENALTY = 6.0
TRACK_DIST_SCORE_PENALTY = 12.0
TRACK_AREA_SCORE_PENALTY = 0.10

UART_ENABLE = True
UART_SEND_N_FRAME = 1
UART_TX_PIN = 11
UART_RX_PIN = 12
uart = None
uart_frame_cnt = 0

DISPLAY_ENABLE = True
DISPLAY_SHOW_N_FRAME = 2
DRAW_DEBUG_ROI = False
DISPLAY_TEXT_FONT_SIZE = 16
DISPLAY_TEXT_MARGIN_X = 8
DISPLAY_TEXT_MARGIN_Y = 8
DISPLAY_TEXT_CHAR_W = 8
DISPLAY_QUAD_THICKNESS = 2
DISPLAY_CROSS_SIZE = 8
DISPLAY_CROSS_THICKNESS = 1
DISPLAY_FPS_LOG_MS = 2000
GC_COLLECT_MS = 800

SHOW_X = (LCD_W - IMG_W) // 2
SHOW_Y = (LCD_H - IMG_H) // 2
PREVIEW_SCALE_X = PREVIEW_W / IMG_W
PREVIEW_SCALE_Y = PREVIEW_H / IMG_H

track_valid = False
track_lost = 0
track_corners = None
track_cx = 0
track_cy = 0
track_area = 0
track_source = None
track_vx = 0
track_vy = 0
control_seen = False
control_last_seen_ms = 0
control_cx = 0
control_cy = 0
focus_setup_ok = False
gc_last_ms = 0


def clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


def scale_coord_x(x):
    return (x * UART_COORD_W + (IMG_W // 2)) // IMG_W


def scale_coord_y(y):
    return (y * UART_COORD_H + (IMG_H // 2)) // IMG_H


def setup_sensor_focus(sensor):
    global focus_setup_ok

    if (sensor is None) or (not AUTO_FOCUS_ENABLE):
        return False

    try:
        focus_caps = sensor.focus_caps()
        is_supported, min_pos, max_pos = focus_caps
        print(
            "focus caps: support=%d range=%d..%d"
            % (1 if is_supported else 0, min_pos, max_pos)
        )

        if not is_supported:
            focus_setup_ok = False
            return False

        result = sensor.auto_focus(True)
        print("auto_focus enable:", result)
        focus_setup_ok = True if result else False
        return result
    except Exception as e:
        print("auto_focus init failed:", e)
        focus_setup_ok = False
        return False


def read_sensor_focus_status(sensor):
    if sensor is None:
        return -1, -1

    try:
        af_enabled = sensor.auto_focus()
        focus_pos = sensor.focus_pos()
        return 1 if af_enabled else 0, focus_pos
    except Exception:
        return -1, -1


def center_of(corners):
    cx = (corners[0][0] + corners[1][0] + corners[2][0] + corners[3][0]) // 4
    cy = (corners[0][1] + corners[1][1] + corners[2][1] + corners[3][1]) // 4
    return cx, cy


def edge_len2(p1, p2):
    dx = p1[0] - p2[0]
    dy = p1[1] - p2[1]
    return dx * dx + dy * dy


def bbox_from_corners(corners):
    xs = [p[0] for p in corners]
    ys = [p[1] for p in corners]
    x0 = min(xs)
    y0 = min(ys)
    x1 = max(xs)
    y1 = max(ys)
    return x0, y0, x1 - x0, y1 - y0


def bbox_corners(x, y, w, h):
    x1 = x + w
    y1 = y + h
    return [(x, y), (x1, y), (x1, y1), (x, y1)]


def bbox_union(bbox_a, bbox_b):
    ax, ay, aw, ah = bbox_a
    bx, by, bw, bh = bbox_b
    x0 = min(ax, bx)
    y0 = min(ay, by)
    x1 = max(ax + aw, bx + bw)
    y1 = max(ay + ah, by + bh)
    return x0, y0, x1 - x0, y1 - y0


def bbox_gap(bbox_a, bbox_b):
    ax, ay, aw, ah = bbox_a
    bx, by, bw, bh = bbox_b
    gap_x = max(0, max(ax, bx) - min(ax + aw, bx + bw))
    gap_y = max(0, max(ay, by) - min(ay + ah, by + bh))
    return gap_x, gap_y


def expand_bbox(x, y, w, h, margin_x, margin_y):
    return normalize_roi(
        x - margin_x,
        y - margin_y,
        w + margin_x * 2,
        h + margin_y * 2,
    )


def order_corners(corners):
    cx = (corners[0][0] + corners[1][0] + corners[2][0] + corners[3][0]) / 4
    cy = (corners[0][1] + corners[1][1] + corners[2][1] + corners[3][1]) / 4

    tl = None
    tr = None
    br = None
    bl = None

    for p in corners:
        x, y = p
        if x < cx and y < cy:
            tl = p
        elif x >= cx and y < cy:
            tr = p
        elif x >= cx and y >= cy:
            br = p
        else:
            bl = p

    if (tl is None) or (tr is None) or (br is None) or (bl is None):
        pts = [corners[0], corners[1], corners[2], corners[3]]
        sums = [p[0] + p[1] for p in pts]
        diffs = [p[0] - p[1] for p in pts]

        tl = pts[sums.index(min(sums))]
        br = pts[sums.index(max(sums))]
        tr = pts[diffs.index(max(diffs))]
        bl = pts[diffs.index(min(diffs))]

    return [tl, tr, br, bl]


def corners_jump_too_much(old_corners, new_corners, limit):
    limit2 = limit * limit
    for i in range(4):
        if edge_len2(old_corners[i], new_corners[i]) > limit2:
            return True
    return False


def smooth_corners(old_corners, new_corners, alpha):
    out = []
    for i in range(4):
        sx = int(alpha * old_corners[i][0] + (1 - alpha) * new_corners[i][0])
        sy = int(alpha * old_corners[i][1] + (1 - alpha) * new_corners[i][1])
        out.append((sx, sy))
    return out


def smooth_alpha_for_motion(dx, dy):
    dist2 = dx * dx + dy * dy
    slow_limit2 = SLOW_MOVE_LIMIT * SLOW_MOVE_LIMIT
    fast_limit2 = FAST_MOVE_LIMIT * FAST_MOVE_LIMIT

    if dist2 <= slow_limit2:
        return SLOW_SMOOTH_A
    if dist2 >= fast_limit2:
        return FAST_SMOOTH_A
    return MID_SMOOTH_A


def draw_quad(img, corners, color=(255, 0, 0), thickness=2):
    img.draw_line(corners[0][0], corners[0][1], corners[1][0], corners[1][1], color=color, thickness=thickness)
    img.draw_line(corners[1][0], corners[1][1], corners[2][0], corners[2][1], color=color, thickness=thickness)
    img.draw_line(corners[2][0], corners[2][1], corners[3][0], corners[3][1], color=color, thickness=thickness)
    img.draw_line(corners[3][0], corners[3][1], corners[0][0], corners[0][1], color=color, thickness=thickness)


def scale_preview_x(x):
    return int(x * PREVIEW_SCALE_X)


def scale_preview_y(y):
    return int(y * PREVIEW_SCALE_Y)


def scale_preview_corners(corners):
    out = []
    for x, y in corners:
        out.append((scale_preview_x(x), scale_preview_y(y)))
    return out


def draw_preview_overlay(osd_img, valid, corners, cx, cy):
    if osd_img is None:
        return

    osd_img.clear()

    if valid and corners is not None:
        preview_corners = scale_preview_corners(corners)
        draw_quad(
            osd_img,
            preview_corners,
            color=(255, 0, 0),
            thickness=max(2, DISPLAY_QUAD_THICKNESS),
        )
        osd_img.draw_cross(
            scale_preview_x(cx),
            scale_preview_y(cy),
            color=(0, 255, 0),
            size=max(10, DISPLAY_CROSS_SIZE * 2),
            thickness=max(2, DISPLAY_CROSS_THICKNESS),
        )
        osd_img.draw_string_advanced(
            8,
            8,
            32,
            "X:%03d Y:%03d" % (scale_coord_x(cx), scale_coord_y(cy)),
            color=(255, 255, 0),
        )
    else:
        osd_img.draw_string_advanced(
            8,
            8,
            32,
            "X:--- Y:---",
            color=(255, 64, 64),
        )


def get_display_target(best):
    if track_valid and track_corners is not None:
        return True, track_corners, track_cx, track_cy

    if best is not None:
        return True, best["corners"], best["cx"], best["cy"]

    return False, None, 0, 0


def draw_center_text(img, valid, cx, cy):
    if valid:
        text = "X:{:03d} Y:{:03d}".format(
            scale_coord_x(cx), scale_coord_y(cy))
        color = (255, 255, 0)
    else:
        text = "X:--- Y:---"
        color = (255, 64, 64)

    text_w = len(text) * DISPLAY_TEXT_CHAR_W
    text_x = IMG_W - DISPLAY_TEXT_MARGIN_X - text_w
    if text_x < 0:
        text_x = 0

    img.draw_string_advanced(
        text_x,
        DISPLAY_TEXT_MARGIN_Y,
        DISPLAY_TEXT_FONT_SIZE,
        text,
        color=color,
    )


def normalize_roi(x, y, w, h):
    x = clamp(x, 0, IMG_W - 1)
    y = clamp(y, 0, IMG_H - 1)
    w = clamp(w, 1, IMG_W - x)
    h = clamp(h, 1, IMG_H - y)
    return (x, y, w, h)


def center_search_roi():
    return normalize_roi(
        CENTER_ROI_MARGIN_X,
        CENTER_ROI_MARGIN_Y,
        IMG_W - CENTER_ROI_MARGIN_X * 2,
        IMG_H - CENTER_ROI_MARGIN_Y * 2,
    )


def track_search_roi(corners):
    x, y, w, h = bbox_from_corners(corners)
    bbox_cx = x + (w // 2)
    bbox_cy = y + (h // 2)
    pred_dx = clamp(track_vx, -TRACK_PREDICT_LIMIT_X, TRACK_PREDICT_LIMIT_X)
    pred_dy = clamp(track_vy, -TRACK_PREDICT_LIMIT_Y, TRACK_PREDICT_LIMIT_Y)
    margin_x = TRACK_ROI_BASE_MARGIN_X + (abs(track_vx) * 2) + (track_lost * TRACK_ROI_LOST_GAIN_X)
    margin_y = TRACK_ROI_BASE_MARGIN_Y + (abs(track_vy) * 2) + (track_lost * TRACK_ROI_LOST_GAIN_Y)
    margin_x = clamp(margin_x, TRACK_ROI_BASE_MARGIN_X, TRACK_ROI_MAX_MARGIN_X)
    margin_y = clamp(margin_y, TRACK_ROI_BASE_MARGIN_Y, TRACK_ROI_MAX_MARGIN_Y)
    half_w = (w // 2) + margin_x
    half_h = (h // 2) + margin_y
    pred_cx = clamp(bbox_cx + pred_dx, 0, IMG_W - 1)
    pred_cy = clamp(bbox_cy + pred_dy, 0, IMG_H - 1)
    return normalize_roi(
        pred_cx - half_w,
        pred_cy - half_h,
        half_w * 2,
        half_h * 2,
    )


def frame_pattern_score(
    img,
    bbox,
    side_diff_min=FRAME_SIDE_DIFF_MIN,
    contrast_min=FRAME_CONTRAST_MIN,
    side_pass_min=FRAME_SIDE_PASS_MIN,
):
    x, y, w, h = bbox
    short_side = min(w, h)
    border_t = max(FRAME_BORDER_MIN_THICK, short_side // 8)
    border_t = min(border_t, max(2, (h - 4) // 3), max(2, (w - 4) // 3))

    center_margin_x = max(border_t + 2, FRAME_CENTER_MARGIN_X, w // 5)
    center_margin_y = max(border_t + 2, FRAME_CENTER_MARGIN_Y, h // 5)
    center_w = w - center_margin_x * 2
    center_h = h - center_margin_y * 2

    if center_w < 6 or center_h < 6:
        return -1.0

    center_roi = normalize_roi(x + center_margin_x, y + center_margin_y, center_w, center_h)
    top_roi = normalize_roi(x, y, w, border_t)
    bottom_roi = normalize_roi(x, y + h - border_t, w, border_t)
    left_roi = normalize_roi(x, y, border_t, h)
    right_roi = normalize_roi(x + w - border_t, y, border_t, h)

    center_mean = img.get_statistics(roi=center_roi).mean()
    side_means = [
        img.get_statistics(roi=top_roi).mean(),
        img.get_statistics(roi=bottom_roi).mean(),
        img.get_statistics(roi=left_roi).mean(),
        img.get_statistics(roi=right_roi).mean(),
    ]

    side_pass = 0
    diff_sum = 0
    for side_mean in side_means:
        diff = center_mean - side_mean
        diff_sum += diff
        if diff >= side_diff_min:
            side_pass += 1

    avg_diff = diff_sum / 4.0
    if avg_diff < contrast_min:
        return -1.0
    if side_pass < side_pass_min:
        return -1.0

    return avg_diff + side_pass * 4.0


def blob_code(blob):
    try:
        return blob.code()
    except Exception:
        return 1


def blob_has_dark(blob):
    return (blob_code(blob) & 0x01) != 0


def blob_has_bright(blob):
    return (blob_code(blob) & 0x02) != 0


def is_blob_source(source):
    return (
        source == "blob"
        or source == "blob_merge"
        or source == "blob_combo"
        or source == "bright_window"
    )


def merge_prefilter_blob(blob, tracking_hint=False):
    if blob_has_bright(blob):
        return False
    if not blob_has_dark(blob):
        return False

    w = blob.w()
    h = blob.h()
    pixels = blob.pixels()
    bbox_area = w * h

    if w < MERGE_MIN_WIDTH or h < MERGE_MIN_HEIGHT:
        return False
    if bbox_area < MERGE_MIN_AREA or pixels < MERGE_MIN_PIXELS:
        return False

    aspect = w / h if w >= h else h / w
    aspect_max = TRACK_BLOB_ASPECT_MAX if tracking_hint else MERGE_ASPECT_MAX
    if aspect > aspect_max:
        return False

    density = blob.density()
    if density < MERGE_DENSITY_MIN or density > MERGE_DENSITY_MAX:
        return False

    return True


def candidate_from_combo_blob(img, blob, tracking_hint=False):
    if (not blob_has_dark(blob)) or (not blob_has_bright(blob)):
        return None

    x = blob.x()
    y = blob.y()
    w = blob.w()
    h = blob.h()
    pixels = blob.pixels()
    bbox_area = w * h

    if w < MIN_WIDTH or h < MIN_HEIGHT:
        return None
    if bbox_area < COMBO_MIN_AREA or pixels < COMBO_MIN_PIXELS:
        return None

    aspect = w / h if w >= h else h / w
    if tracking_hint:
        aspect_min = TRACK_COMBO_ASPECT_MIN
        aspect_max = TRACK_COMBO_ASPECT_MAX
        frame_score = frame_pattern_score(
            img,
            (x, y, w, h),
            side_diff_min=TRACK_COMBO_FRAME_SIDE_DIFF_MIN,
            contrast_min=TRACK_COMBO_FRAME_CONTRAST_MIN,
            side_pass_min=TRACK_COMBO_FRAME_SIDE_PASS_MIN,
        )
    else:
        aspect_min = COMBO_ASPECT_MIN
        aspect_max = COMBO_ASPECT_MAX
        frame_score = frame_pattern_score(
            img,
            (x, y, w, h),
            side_diff_min=COMBO_FRAME_SIDE_DIFF_MIN,
            contrast_min=COMBO_FRAME_CONTRAST_MIN,
            side_pass_min=COMBO_FRAME_SIDE_PASS_MIN,
        )

    if aspect < aspect_min or aspect > aspect_max:
        return None

    density = blob.density()
    if density < COMBO_DENSITY_MIN or density > COMBO_DENSITY_MAX:
        return None

    if frame_score < 0.0:
        return None

    return {
        "corners": bbox_corners(x, y, w, h),
        "cx": blob.cx(),
        "cy": blob.cy(),
        "area": bbox_area,
        "aspect": aspect,
        "source": "blob_combo",
        "magnitude": 0,
        "pixels": pixels,
        "density": density,
        "frame_score": frame_score,
        "bbox": (x, y, w, h),
    }


def candidate_from_bright_blob(img, blob, roi=None, tracking_hint=False):
    if not blob_has_bright(blob):
        return None
    if blob_has_dark(blob):
        return None

    x = blob.x()
    y = blob.y()
    w = blob.w()
    h = blob.h()
    pixels = blob.pixels()
    bbox_area = w * h

    if w < MERGE_MIN_WIDTH or h < MERGE_MIN_HEIGHT:
        return None
    if bbox_area < BRIGHT_MIN_AREA or pixels < BRIGHT_MIN_PIXELS:
        return None

    if roi is None:
        roi_x, roi_y, roi_w, roi_h = 0, 0, IMG_W, IMG_H
    else:
        roi_x, roi_y, roi_w, roi_h = roi

    touch_edges = 0
    if x <= (roi_x + 1):
        touch_edges += 1
    if y <= (roi_y + 1):
        touch_edges += 1
    if (x + w) >= (roi_x + roi_w - 2):
        touch_edges += 1
    if (y + h) >= (roi_y + roi_h - 2):
        touch_edges += 1
    if (
        touch_edges >= BRIGHT_EDGE_TOUCH_REJECT
        and bbox_area >= int(roi_w * roi_h * BRIGHT_EDGE_AREA_REJECT)
    ):
        return None

    density = blob.density()
    if density < BRIGHT_MIN_DENSITY or density > BRIGHT_MAX_DENSITY:
        return None

    outer_bbox = expand_bbox(
        x,
        y,
        w,
        h,
        max(BRIGHT_OUTER_MARGIN_X, w // 7),
        max(BRIGHT_OUTER_MARGIN_Y, h // 7),
    )
    ox, oy, ow, oh = outer_bbox
    outer_area = ow * oh
    if outer_area >= int(IMG_W * IMG_H * 0.92):
        return None

    outer_aspect = ow / oh if ow >= oh else oh / ow
    if tracking_hint:
        aspect_min = TRACK_BRIGHT_ASPECT_MIN
        aspect_max = TRACK_BRIGHT_ASPECT_MAX
        frame_score = frame_pattern_score(
            img,
            outer_bbox,
            side_diff_min=TRACK_COMBO_FRAME_SIDE_DIFF_MIN,
            contrast_min=TRACK_COMBO_FRAME_CONTRAST_MIN,
            side_pass_min=TRACK_COMBO_FRAME_SIDE_PASS_MIN,
        )
    else:
        aspect_min = BRIGHT_ASPECT_MIN
        aspect_max = BRIGHT_ASPECT_MAX
        frame_score = frame_pattern_score(
            img,
            outer_bbox,
            side_diff_min=COMBO_FRAME_SIDE_DIFF_MIN,
            contrast_min=COMBO_FRAME_CONTRAST_MIN,
            side_pass_min=COMBO_FRAME_SIDE_PASS_MIN,
        )

    if outer_aspect < aspect_min or outer_aspect > aspect_max:
        return None
    if frame_score < 0.0:
        return None

    box_bbox = expand_bbox(
        x,
        y,
        w,
        h,
        max(BRIGHT_BOX_MARGIN_X, w // 12),
        max(BRIGHT_BOX_MARGIN_Y, h // 12),
    )
    bx, by, bw, bh = box_bbox

    return {
        "corners": bbox_corners(bx, by, bw, bh),
        "cx": blob.cx(),
        "cy": blob.cy(),
        "area": bw * bh,
        "aspect": bw / bh if bw >= bh else bh / bw,
        "source": "bright_window",
        "magnitude": 0,
        "pixels": pixels,
        "density": density,
        "frame_score": frame_score,
        "bbox": box_bbox,
    }


def candidate_from_blob(img, blob, roi=None, tracking_hint=False):
    if not blob_has_dark(blob):
        return None
    if blob_has_bright(blob):
        return None

    x = blob.x()
    y = blob.y()
    w = blob.w()
    h = blob.h()
    pixels = blob.pixels()
    bbox_area = w * h

    if w < MIN_WIDTH or h < MIN_HEIGHT:
        return None
    if bbox_area < BLOB_MIN_AREA or pixels < BLOB_MIN_PIXELS:
        return None

    aspect = w / h if w >= h else h / w
    if tracking_hint:
        aspect_min = TRACK_BLOB_ASPECT_MIN
        aspect_max = TRACK_BLOB_ASPECT_MAX
    else:
        aspect_min = BLOB_ASPECT_MIN
        aspect_max = BLOB_ASPECT_MAX
    if aspect < aspect_min or aspect > aspect_max:
        return None

    density = blob.density()
    if density < BLOB_MIN_DENSITY:
        return None
    density_max = TRACK_BLOB_MAX_DENSITY if tracking_hint else BLOB_MAX_DENSITY
    if density > density_max:
        return None

    if roi is not None:
        roi_x, roi_y, roi_w, roi_h = roi
        touch_edges = 0
        if x <= (roi_x + 1):
            touch_edges += 1
        if y <= (roi_y + 1):
            touch_edges += 1
        if (x + w) >= (roi_x + roi_w - 2):
            touch_edges += 1
        if (y + h) >= (roi_y + roi_h - 2):
            touch_edges += 1

        if (
            touch_edges >= BLOB_EDGE_TOUCH_REJECT
            and bbox_area >= int(roi_w * roi_h * BLOB_EDGE_AREA_REJECT)
            and density >= BLOB_EDGE_DENSITY_REJECT
        ):
            return None

    if tracking_hint:
        frame_score = frame_pattern_score(
            img,
            (x, y, w, h),
            side_diff_min=TRACK_FRAME_SIDE_DIFF_MIN,
            contrast_min=TRACK_FRAME_CONTRAST_MIN,
            side_pass_min=TRACK_FRAME_SIDE_PASS_MIN,
        )
    else:
        frame_score = frame_pattern_score(img, (x, y, w, h))
    if frame_score < 0.0:
        return None

    return {
        "corners": bbox_corners(x, y, w, h),
        "cx": blob.cx(),
        "cy": blob.cy(),
        "area": bbox_area,
        "aspect": aspect,
        "source": "blob",
        "magnitude": 0,
        "pixels": pixels,
        "density": density,
        "frame_score": frame_score,
        "bbox": (x, y, w, h),
    }


def candidate_from_blob_pair(img, blob_a, blob_b, tracking_hint=False):
    bbox_a = (blob_a.x(), blob_a.y(), blob_a.w(), blob_a.h())
    bbox_b = (blob_b.x(), blob_b.y(), blob_b.w(), blob_b.h())
    gap_x, gap_y = bbox_gap(bbox_a, bbox_b)
    if gap_x > MERGE_GAP_X_MAX or gap_y > MERGE_GAP_Y_MAX:
        return None

    x, y, w, h = bbox_union(bbox_a, bbox_b)
    bbox_area = w * h
    if w < MIN_WIDTH or h < MIN_HEIGHT:
        return None
    if bbox_area < BLOB_MIN_AREA:
        return None

    aspect = w / h if w >= h else h / w
    aspect_max = TRACK_BLOB_ASPECT_MAX if tracking_hint else MERGE_ASPECT_MAX
    aspect_min = TRACK_BLOB_ASPECT_MIN if tracking_hint else MERGE_ASPECT_MIN
    if aspect < aspect_min or aspect > aspect_max:
        return None

    area_sum = bbox_a[2] * bbox_a[3] + bbox_b[2] * bbox_b[3]
    if bbox_area > int(area_sum * MERGE_AREA_EXPAND_MAX):
        return None

    pixels = blob_a.pixels() + blob_b.pixels()
    density = pixels / float(bbox_area)
    if density < MERGE_FILL_MIN or density > MERGE_FILL_MAX:
        return None

    if tracking_hint:
        frame_score = frame_pattern_score(
            img,
            (x, y, w, h),
            side_diff_min=TRACK_FRAME_SIDE_DIFF_MIN,
            contrast_min=TRACK_FRAME_CONTRAST_MIN,
            side_pass_min=TRACK_FRAME_SIDE_PASS_MIN,
        )
    else:
        frame_score = frame_pattern_score(img, (x, y, w, h))
    if frame_score < 0.0:
        return None

    return {
        "corners": bbox_corners(x, y, w, h),
        "cx": x + (w // 2),
        "cy": y + (h // 2),
        "area": bbox_area,
        "aspect": aspect,
        "source": "blob_merge",
        "magnitude": 0,
        "pixels": pixels,
        "density": density,
        "frame_score": frame_score,
        "bbox": (x, y, w, h),
    }


def score_candidate(candidate):
    score = float(candidate["area"]) + float(candidate["pixels"]) * BLOB_PIXEL_SCORE_GAIN
    score += float(candidate["density"]) * BLOB_DENSITY_SCORE_GAIN
    aspect_penalty = ASPECT_SCORE_PENALTY * BLOB_ASPECT_SCORE_SCALE

    if candidate["source"] == "blob_combo":
        score += COMBO_SCORE_BONUS
        aspect_penalty *= 0.45
    elif candidate["source"] == "bright_window":
        score += BRIGHT_SCORE_BONUS
        aspect_penalty *= 0.52

    score -= abs(candidate["aspect"] - TARGET_RECT_ASPECT) * aspect_penalty
    score += candidate["frame_score"] * 180.0

    if track_valid and track_corners is not None:
        score -= (abs(candidate["cx"] - track_cx) + abs(candidate["cy"] - track_cy)) * TRACK_DIST_SCORE_PENALTY
        score -= abs(candidate["area"] - track_area) * TRACK_AREA_SCORE_PENALTY
    else:
        score -= (abs(candidate["cx"] - (IMG_W // 2)) + abs(candidate["cy"] - (IMG_H // 2))) * CENTER_SCORE_PENALTY

    return score


def pick_best_blobs(blobs, img, roi=None, tracking_hint=False):
    best = None
    best_score = -1.0e30
    merge_blobs = []

    for blob in blobs:
        candidate = None

        if BRIGHT_WINDOW_ENABLE:
            candidate = candidate_from_bright_blob(img, blob, roi, tracking_hint)

        if candidate is None and COMBO_BLOB_ENABLE:
            candidate = candidate_from_combo_blob(img, blob, tracking_hint)

        if candidate is None:
            candidate = candidate_from_blob(img, blob, roi, tracking_hint)

        if candidate is None:
            if MERGE_BLOB_ENABLE and merge_prefilter_blob(blob, tracking_hint):
                merge_blobs.append(blob)
            continue

        score = score_candidate(candidate)
        if score > best_score:
            best_score = score
            candidate["score"] = score
            best = candidate

        if MERGE_BLOB_ENABLE and merge_prefilter_blob(blob, tracking_hint):
            merge_blobs.append(blob)

    if MERGE_BLOB_ENABLE and len(merge_blobs) >= 2:
        merge_blobs.sort(key=lambda blob: blob.pixels(), reverse=True)
        if len(merge_blobs) > MERGE_BLOB_MAX_COUNT:
            merge_blobs = merge_blobs[:MERGE_BLOB_MAX_COUNT]

        for i in range(len(merge_blobs) - 1):
            for j in range(i + 1, len(merge_blobs)):
                candidate = candidate_from_blob_pair(
                    img,
                    merge_blobs[i],
                    merge_blobs[j],
                    tracking_hint,
                )
                if candidate is None:
                    continue

                score = score_candidate(candidate)
                if score > best_score:
                    best_score = score
                    candidate["score"] = score
                    best = candidate

    return best


def blob_thresholds_for_roi(img, roi=None, include_bright=False):
    if roi is None:
        stats = img.get_statistics()
        hist = img.get_histogram()
    else:
        stats = img.get_statistics(roi=roi)
        hist = img.get_histogram(roi=roi)

    margin = clamp(int(stats.stdev() // 3), BLOB_THRESH_MARGIN_MIN, BLOB_THRESH_MARGIN_MAX)
    otsu = hist.get_threshold().value()
    dark_hi = clamp(otsu - margin, 0, 255)
    bright_lo = clamp(otsu + margin, 0, 255)

    thresholds = []
    if dark_hi >= BLOB_THRESH_MARGIN_MIN:
        thresholds.append((0, dark_hi))
    if include_bright and bright_lo <= (255 - BLOB_THRESH_MARGIN_MIN):
        thresholds.append((bright_lo, 255))
    elif (not BLOB_DARK_ONLY) and bright_lo <= (255 - BLOB_THRESH_MARGIN_MIN):
        thresholds.append((bright_lo, 255))
    return thresholds


def find_best_blob_in_roi(img, roi=None, tracking_hint=False):
    blobs = None
    thresholds = blob_thresholds_for_roi(
        img,
        roi,
        include_bright=(COMBO_BLOB_ENABLE or BRIGHT_WINDOW_ENABLE),
    )
    if len(thresholds) == 0:
        return None

    area_threshold = min(BLOB_MIN_AREA, MERGE_MIN_AREA)
    pixels_threshold = min(BLOB_MIN_PIXELS, MERGE_MIN_PIXELS)
    if BRIGHT_WINDOW_ENABLE:
        area_threshold = min(area_threshold, BRIGHT_MIN_AREA)
        pixels_threshold = min(pixels_threshold, BRIGHT_MIN_PIXELS)
    if COMBO_BLOB_ENABLE:
        area_threshold = min(area_threshold, COMBO_MIN_AREA)
        pixels_threshold = min(pixels_threshold, COMBO_MIN_PIXELS)

    try:
        if roi is None:
            blobs = img.find_blobs(
                thresholds,
                x_stride=BLOB_X_STRIDE,
                y_stride=BLOB_Y_STRIDE,
                area_threshold=area_threshold,
                pixels_threshold=pixels_threshold,
                merge=COMBO_BLOB_ENABLE,
                margin=COMBO_MERGE_MARGIN if COMBO_BLOB_ENABLE else 0,
            )
        else:
            roi_x, roi_y, roi_w, roi_h = roi
            blobs = img.find_blobs(
                thresholds,
                roi=(roi_x, roi_y, roi_w, roi_h),
                x_stride=BLOB_X_STRIDE,
                y_stride=BLOB_Y_STRIDE,
                area_threshold=area_threshold,
                pixels_threshold=pixels_threshold,
                merge=COMBO_BLOB_ENABLE,
                margin=COMBO_MERGE_MARGIN if COMBO_BLOB_ENABLE else 0,
            )
        return pick_best_blobs(blobs, img, roi, tracking_hint)
    finally:
        blobs = None


def find_best_candidate(img):
    used_roi = None

    if track_valid and track_corners is not None:
        used_roi = track_search_roi(track_corners)
        if BLOB_ENABLE:
            best = find_best_blob_in_roi(img, used_roi, tracking_hint=True)
            if best is not None:
                return best, used_roi

    center_roi = center_search_roi() if USE_ROI else None

    if BLOB_ENABLE and center_roi is not None:
        best = find_best_blob_in_roi(
            img,
            center_roi,
            tracking_hint=track_valid and (track_lost < TRACK_REACQUIRE_FULL_AFTER),
        )
        if best is not None:
            return best, center_roi

    if BLOB_ENABLE:
        best = find_best_blob_in_roi(img, None, tracking_hint=False)
        if best is not None:
            return best, None

    return None, center_roi if center_roi is not None else used_roi


def lock_track(candidate):
    global track_valid
    global track_lost
    global track_corners
    global track_cx
    global track_cy
    global track_area
    global track_source
    global track_vx
    global track_vy

    track_valid = True
    track_lost = 0
    track_corners = candidate["corners"]
    track_cx = candidate["cx"]
    track_cy = candidate["cy"]
    track_area = candidate["area"]
    track_source = candidate["source"]
    track_vx = 0
    track_vy = 0


def update_track(candidate):
    global track_lost
    global track_corners
    global track_cx
    global track_cy
    global track_area
    global track_source
    global track_vx
    global track_vy

    new_corners = candidate["corners"]
    new_cx = candidate["cx"]
    new_cy = candidate["cy"]
    new_area = candidate["area"]
    new_source = candidate["source"]

    if not track_valid or track_corners is None:
        lock_track(candidate)
        return True

    dx = new_cx - track_cx
    dy = new_cy - track_cy
    prev_track_cx = track_cx
    prev_track_cy = track_cy
    center_ok = (dx * dx + dy * dy) < (CENTER_JUMP_LIMIT * CENTER_JUMP_LIMIT)
    area_ok = (new_area > track_area * AREA_CHANGE_LOW) and (new_area < track_area * AREA_CHANGE_HIGH)
    same_source = track_source == new_source
    switch_area_ok = (
        (new_area > track_area * SOURCE_SWITCH_AREA_LOW)
        and (new_area < track_area * SOURCE_SWITCH_AREA_HIGH)
    )

    if (not same_source) and center_ok and switch_area_ok:
        lock_track(candidate)
        return True

    if is_blob_source(new_source):
        corner_limit = CORNER_JUMP_LIMIT * 2
    else:
        corner_limit = CORNER_JUMP_LIMIT
    corner_ok = not corners_jump_too_much(track_corners, new_corners, corner_limit)

    if area_ok and center_ok and corner_ok:
        track_corners = smooth_corners(
            track_corners,
            new_corners,
            smooth_alpha_for_motion(dx, dy),
        )
        track_cx, track_cy = center_of(track_corners)
        track_area = new_area
        track_source = new_source
        inst_vx = track_cx - prev_track_cx
        inst_vy = track_cy - prev_track_cy
        track_vx = (track_vx * 2 + inst_vx) // 3
        track_vy = (track_vy * 2 + inst_vy) // 3
        track_lost = 0
        return True

    if track_lost >= (LOST_TOL // 2):
        lock_track(candidate)
        return True
    else:
        track_lost += 1
        return False


def update_control_target(valid, cx, cy, now_ms):
    global control_seen
    global control_last_seen_ms
    global control_cx
    global control_cy

    if valid:
        control_cx = cx
        control_cy = cy
        control_last_seen_ms = now_ms
        control_seen = True
        return True

    if not control_seen:
        return False

    return time.ticks_diff(now_ms, control_last_seen_ms) <= CONTROL_HOLD_MS


def uart_init():
    global uart

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
    print("uart init ok")


def uart_send_packet(valid, cx, cy):
    global uart

    if uart is None:
        return

    if cx < 0:
        cx = 0
    if cy < 0:
        cy = 0
    if cx > 999:
        cx = 999
    if cy > 999:
        cy = 999

    msg = "@{:d},{:03d},{:03d}#".format(valid, cx, cy)
    uart.write(msg)


def uart_send_center(cx, cy):
    uart_send_packet(1, scale_coord_x(cx), scale_coord_y(cy))


def uart_send_lost(cx, cy):
    uart_send_packet(0, scale_coord_x(cx), scale_coord_y(cy))


def main():
    global sensor
    global uart_frame_cnt
    global track_valid
    global track_lost
    global track_corners
    global track_source
    global track_vx
    global track_vy
    global control_seen
    global control_last_seen_ms
    global focus_setup_ok
    global gc_last_ms

    osd_img = None
    preview_layer_bound = False

    try:
        if UART_ENABLE:
            uart_init()

        sensor = Sensor(id=sensor_id)
        sensor.reset()
        sensor.set_hmirror(False)
        sensor.set_vflip(False)
        if PREVIEW_ENABLE:
            sensor.set_framesize(width=PREVIEW_W, height=PREVIEW_H, chn=CAM_CHN_ID_0)
            sensor.set_pixformat(Sensor.YUV420SP, chn=CAM_CHN_ID_0)
        sensor.set_framesize(width=IMG_W, height=IMG_H, chn=CAM_CHN_ID_1)
        sensor.set_pixformat(Sensor.GRAYSCALE, chn=CAM_CHN_ID_1)
        setup_sensor_focus(sensor)

        if PREVIEW_ENABLE:
            bind_info = sensor.bind_info(chn=CAM_CHN_ID_0)
            Display.bind_layer(**bind_info, layer=Display.LAYER_VIDEO1)
            preview_layer_bound = True

        Display.init(Display.ST7701, width=LCD_W, height=LCD_H, to_ide=False)
        if PREVIEW_ENABLE and DISPLAY_ENABLE:
            osd_img = image.Image(PREVIEW_W, PREVIEW_H, image.ARGB8888)
        MediaManager.init()
        sensor.run()
        print(
            "rect tracker build: 2026-05-05 bright-window-v5-tight-box "
            "draw_string_advanced gc2093-af-safe"
        )

        fps_cnt = 0
        display_frame_cnt = 0
        t0 = time.ticks_ms()
        track_valid = False
        track_lost = 0
        track_corners = None
        track_source = None
        track_vx = 0
        track_vy = 0
        control_seen = False
        control_last_seen_ms = 0
        gc_last_ms = 0

        while True:
            os.exitpoint()

            now_ms = time.ticks_ms()
            img = sensor.snapshot(chn=CAM_CHN_ID_1)
            if img is None:
                continue

            best, used_roi = find_best_candidate(img)

            if best is not None:
                update_track(best)
            else:
                track_lost += 1

            if track_lost > LOST_TOL:
                track_valid = False
                track_corners = None
                track_source = None
                track_vx = 0
                track_vy = 0

            control_valid = update_control_target(
                track_valid and track_corners is not None,
                track_cx,
                track_cy,
                now_ms,
            )

            if UART_ENABLE:
                uart_frame_cnt += 1
                if uart_frame_cnt >= UART_SEND_N_FRAME:
                    uart_frame_cnt = 0
                    if control_valid:
                        uart_send_center(control_cx, control_cy)
                    else:
                        uart_send_lost(control_cx, control_cy)

            fps_cnt += 1
            dt = time.ticks_diff(now_ms, t0)
            if dt >= DISPLAY_FPS_LOG_MS:
                fps = fps_cnt * 1000.0 / dt
                af_enabled, focus_pos = read_sensor_focus_status(sensor)
                print("fps=%.2f valid=%d ctrl=%d src=%s afcfg=%d afq=%d fpos=%d lost=%d cx=%d cy=%d" % (
                    fps,
                    1 if track_valid else 0,
                    1 if control_valid else 0,
                    track_source if track_valid else "-",
                    1 if focus_setup_ok else 0,
                    af_enabled,
                    focus_pos,
                    track_lost,
                    track_cx if track_valid else -1,
                    track_cy if track_valid else -1,
                ))
                fps_cnt = 0
                t0 = now_ms

            if DISPLAY_ENABLE:
                display_frame_cnt += 1
                if display_frame_cnt >= DISPLAY_SHOW_N_FRAME:
                    display_frame_cnt = 0
                    display_valid, display_corners, display_cx, display_cy = get_display_target(best)

                    if PREVIEW_ENABLE:
                        draw_preview_overlay(
                            osd_img,
                            display_valid,
                            display_corners,
                            display_cx,
                            display_cy,
                        )
                        Display.show_image(osd_img, 0, 0, Display.LAYER_OSD3)
                    else:
                        if display_valid and display_corners is not None:
                            draw_quad(img, display_corners, color=(255, 0, 0), thickness=DISPLAY_QUAD_THICKNESS)
                            img.draw_cross(
                                display_cx,
                                display_cy,
                                color=(0, 255, 0),
                                size=DISPLAY_CROSS_SIZE,
                                thickness=DISPLAY_CROSS_THICKNESS,
                            )

                        if DRAW_DEBUG_ROI and used_roi is not None:
                            roi_x, roi_y, roi_w, roi_h = used_roi
                            img.draw_rectangle(roi_x, roi_y, roi_w, roi_h, color=(0, 0, 255), thickness=1)

                        draw_center_text(
                            img,
                            display_valid,
                            display_cx,
                            display_cy,
                        )

                        Display.show_image(img, x=SHOW_X, y=SHOW_Y)

                    if time.ticks_diff(now_ms, gc_last_ms) >= GC_COLLECT_MS:
                        gc.collect()
                        gc_last_ms = now_ms

    except Exception as e:
        print("error:", e)

    finally:
        try:
            if sensor:
                sensor.stop()
        except:
            pass

        try:
            if PREVIEW_ENABLE and preview_layer_bound:
                Display.unbind_layer(Display.LAYER_VIDEO1)
        except:
            pass

        try:
            Display.deinit()
        except:
            pass

        try:
            MediaManager.deinit()
        except:
            pass

        try:
            if uart is not None:
                uart.deinit()
        except:
            pass

        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        time.sleep_ms(100)


if __name__ == "__main__":
    main()

# === Agent Handoff Note (2026-05-04) ===
# 已完成的工作:
# 1. 建了安全回退点:
#    - branch: backup/2026-05-04_181103-pre-k230-stepper-tuning
#    - tag: snapshot-2026-05-04_181103-pre-k230-stepper-tuning
#    - commit: 2e1149d
# 2. K230 预览改成双通道:
#    - chn0: 800x480 预览, 绑定 Display.LAYER_VIDEO1
#    - chn1: 检测通道, 当前为 256x160 + GRAYSCALE
#    - OSD 叠加层负责显示框/十字/坐标
# 3. 已把废弃的 draw_string 全部替换为 draw_string_advanced。
# 4. 控制链和显示链已拆开:
#    - 串口协议仍是 @valid,cx,cy#
#    - 无目标时会发 valid=0, MSP 侧会停机
#    - 控制目标目前吃平滑后的 track_cx/track_cy
# 5. 跟踪侧做过这些尝试:
#    - 自适应平滑
#    - 预测式 ROI
#    - 中心 ROI / 分块重捕获
#    - overflow 冷却帧
#    - 单帧单次 find_rects + 检测后立刻 gc.collect()
# 6. 另外改过 MSP 侧 DAP_LINK_TEST/app/track_control.c:
#    - 放宽速度范围
#    - 降低零速附近 Stop/Start 抖动
#
# 当前状态 / 已知问题:
# 1. 当前 py 版本启动串是:
#    - rect tracker build: 2026-05-04 single-pass-gc ...
# 2. 这一版虽然继续压 fast frame buffer overflow, 但副作用是更容易丢目标,
#    用户反馈“效果变差了，总是丢失目标”。
# 3. fast frame buffer overflow 仍未根治:
#    - 即使做了灰度、降分辨率、分块重捕获、单次检测，运行一段时间后仍可能出现
#    - 推断根因仍在 find_rects() 本身的临时 fast frame buffer 占用
# 4. 自动对焦目前未真正打通:
#    - 日志常见 afcfg=0 afq=-1 fpos=-1
#    - 当前清晰度提升主要来自双通道高清预览, 不是已确认的 AF 生效
#
# 建议下一个 agent 优先做的事:
# 1. 不建议继续在当前 single-pass-gc 版本上硬调参数。
#    - 先评估是否回退掉“single-pass-gc 这一轮的 find_best_candidate 改法”
#    - 因为它换来了更低爆栈概率, 但明显牺牲了重捕获和跟随体验
# 2. 真正想根治 overflow, 建议改检测路线, 不再以 find_rects() 为核心:
#    - 方向 A: threshold / binary + find_blobs() + 外接矩形 / 长宽比筛选
#    - 方向 B: 边缘/线段 + 几何约束组合
#    - 目标是避免四边形检测算子持续占用 fast frame buffer
# 3. 如果暂时还要留在 find_rects() 路线:
#    - 可先回退到 gray-tile-reacquire 或 predictive-roi 思路
#    - 再逐步验证哪一层最影响丢目标
# 4. AF 需要单独排查固件/接口, 不建议和检测链继续混改。
#
# 方便定位的关键区域:
# - 检测主流程: find_best_candidate(), find_best_in_roi()
# - ROI/重捕获: center_search_roi(), track_search_roi(), next_full_search_roi()
# - 预览/OSD: draw_preview_overlay()
# - 主循环: main()
#
# === Latest Handoff (2026-05-04, blob-only-frame-v3) ===
# This block supersedes the legacy notes below.
#
# Current build string:
# - rect tracker build: 2026-05-04 blob-only-frame-v3 ...
#
# Summary of the latest pass:
# 1. Active runtime detection no longer uses find_rects().
#    - The current path is blob-only.
#    - This change was made because find_rects() still caused:
#      "warn: fast frame buffer overflow, skip frame"
#      and later "Out of fast frame buffer stack index".
# 2. Current search order per frame is:
#    - track ROI blob search
#    - center ROI blob search
#    - full-frame blob search
# 3. Blob filtering is now frame-oriented:
#    - dark blobs only
#    - moderate density only
#    - border darker than center
#    - tracking path is looser than reacquire path
# 4. Cleanup order was adjusted:
#    - unbind preview layer before Display.deinit()
#    - meant to avoid "unbind layer(1) failed."
#
# Current behavior:
# 1. track_source is effectively always "blob".
# 2. The displayed rectangle is an axis-aligned bbox from the blob candidate.
#    - Better for rough center tracking under perspective.
#    - Not a true perspective quad anymore.
# 3. UART protocol is unchanged:
#    - @valid,cx,cy#
#
# Known limitations:
# 1. If the dark frame breaks into multiple regions because of glare/exposure,
#    find_blobs() can still lose the target.
# 2. Perspective tolerance is better for center tracking, but geometry is still bbox-only.
# 3. AF still looks inactive in logs:
#    - afcfg=0 afq=-1 fpos=-1
#
# Recommended next steps:
# 1. First verify whether overflow is fully gone in this blob-only build.
# 2. If target loss remains, do not reintroduce find_rects() first.
#    Better next directions are:
#    - merge/pair multiple dark blobs into one frame candidate
#    - detect bright inner window first, then validate dark border around it
#    - refine corners from blob bbox by edge/line sampling
# 3. If center tracking is stable but box shape is poor, add a second-stage corner refiner
#    on top of the blob candidate instead of going back to find_rects().
#
# Key functions for the next agent:
# - find_best_candidate()
# - find_best_blob_in_roi()
# - candidate_from_blob()
# - frame_pattern_score()
# - update_track()
# - main()
#
# === Agent Handoff Addendum (2026-05-04, rect-blob-hybrid) ===
# 1. 已基于用户目标（深色画框 + 亮色内窗）引入 rect/blob 混合检测。
# 2. 单帧仍最多只跑 1 次 find_rects()；overflow cooldown 期间会跳过 find_rects()，
#    改用 find_blobs() 做轻量重捕获。
# 3. blob 候选已放宽 density，下限可覆盖空心框；同时增加了贴边大亮块过滤，
#    避免把整片白背景误当成目标。
# 4. 日志新增 src=rect/blob，便于观察当前由哪条检测链接管。
# 5. 上板后优先观察:
#    - fast frame buffer overflow 是否明显减少
#    - src 是否长期停留在 blob 而迟迟回不到 rect
#    - 框是否更容易跟丢到“内白窗”或“外黑框”，再决定下一轮细调 scoring
