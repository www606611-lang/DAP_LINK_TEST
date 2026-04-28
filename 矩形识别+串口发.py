import math
import os
import time

from media.display import *
from media.media import *
from media.sensor import *
from machine import FPIOA
from machine import UART

sensor = None
sensor_id = 2

LCD_W = 800
LCD_H = 480

IMG_W = 400
IMG_H = 240

RECT_THRESHOLD = 11000
MIN_AREA = 2200
MIN_WIDTH = 40
MIN_HEIGHT = 28
MIN_FRAME_COVERAGE = 0.025

RECT_ASPECT_MIN = 1.20
RECT_ASPECT_MAX = 2.20
TARGET_RECT_ASPECT = 1.50

LOST_TOL = 6
CENTER_JUMP_LIMIT = 70
CORNER_JUMP_LIMIT = 45
AREA_CHANGE_LOW = 0.55
AREA_CHANGE_HIGH = 1.65
SMOOTH_A = 0.35

USE_ROI = True
ROI_MARGIN_X = 12
ROI_MARGIN_Y = 8
TRACK_ROI_MARGIN_X = 64
TRACK_ROI_MARGIN_Y = 44
TRACK_REACQUIRE_CENTER_AFTER = 1
TRACK_REACQUIRE_FULL_AFTER = 2
FULL_SEARCH_EVERY_N_FRAME = 2

ASPECT_SCORE_PENALTY = 12000.0
CENTER_SCORE_PENALTY = 6.0
TRACK_DIST_SCORE_PENALTY = 18.0
TRACK_AREA_SCORE_PENALTY = 0.18

UART_ENABLE = True
UART_SEND_N_FRAME = 1
UART_TX_PIN = 11
UART_RX_PIN = 12
uart = None
uart_frame_cnt = 0

DISPLAY_ENABLE = True
DISPLAY_SHOW_N_FRAME = 2
DRAW_DEBUG_ROI = False

SHOW_X = (LCD_W - IMG_W) // 2
SHOW_Y = (LCD_H - IMG_H) // 2

track_valid = False
track_lost = 0
track_corners = None
track_cx = 0
track_cy = 0
track_area = 0
search_frame_cnt = 0


def clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


def center_of(corners):
    cx = (corners[0][0] + corners[1][0] + corners[2][0] + corners[3][0]) // 4
    cy = (corners[0][1] + corners[1][1] + corners[2][1] + corners[3][1]) // 4
    return cx, cy


def edge_len2(p1, p2):
    dx = p1[0] - p2[0]
    dy = p1[1] - p2[1]
    return dx * dx + dy * dy


def edge_len(p1, p2):
    return math.sqrt(edge_len2(p1, p2))


def quad_area_est(corners):
    xs = [p[0] for p in corners]
    ys = [p[1] for p in corners]
    return (max(xs) - min(xs)) * (max(ys) - min(ys))


def bbox_from_corners(corners):
    xs = [p[0] for p in corners]
    ys = [p[1] for p in corners]
    x0 = min(xs)
    y0 = min(ys)
    x1 = max(xs)
    y1 = max(ys)
    return x0, y0, x1 - x0, y1 - y0


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


def rect_side_lengths(corners):
    top = edge_len(corners[0], corners[1])
    right = edge_len(corners[1], corners[2])
    bottom = edge_len(corners[2], corners[3])
    left = edge_len(corners[3], corners[0])
    return top, right, bottom, left


def rect_aspect(corners):
    top, right, bottom, left = rect_side_lengths(corners)
    width = (top + bottom) * 0.5
    height = (left + right) * 0.5

    if height <= 0.0 or width <= 0.0:
        return 0.0

    if width >= height:
        return width / height
    return height / width


def quad_is_reasonable(corners):
    top, right, bottom, left = rect_side_lengths(corners)
    side_min = min(top, right, bottom, left)
    side_max = max(top, right, bottom, left)

    if side_min < 14.0:
        return False
    if side_max > side_min * 8.0:
        return False

    return True


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


def draw_quad(img, corners, color=(255, 0, 0), thickness=2):
    img.draw_line(corners[0][0], corners[0][1], corners[1][0], corners[1][1], color=color, thickness=thickness)
    img.draw_line(corners[1][0], corners[1][1], corners[2][0], corners[2][1], color=color, thickness=thickness)
    img.draw_line(corners[2][0], corners[2][1], corners[3][0], corners[3][1], color=color, thickness=thickness)
    img.draw_line(corners[3][0], corners[3][1], corners[0][0], corners[0][1], color=color, thickness=thickness)


def normalize_roi(x, y, w, h):
    x = clamp(x, 0, IMG_W - 1)
    y = clamp(y, 0, IMG_H - 1)
    w = clamp(w, 1, IMG_W - x)
    h = clamp(h, 1, IMG_H - y)
    return (x, y, w, h)


def center_search_roi():
    return normalize_roi(
        ROI_MARGIN_X,
        ROI_MARGIN_Y,
        IMG_W - ROI_MARGIN_X * 2,
        IMG_H - ROI_MARGIN_Y * 2,
    )


def track_search_roi(corners):
    x, y, w, h = bbox_from_corners(corners)
    return normalize_roi(
        x - TRACK_ROI_MARGIN_X,
        y - TRACK_ROI_MARGIN_Y,
        w + TRACK_ROI_MARGIN_X * 2,
        h + TRACK_ROI_MARGIN_Y * 2,
    )


def candidate_from_rect(rect, offset_x=0, offset_y=0):
    raw_corners = rect.corners()
    if len(raw_corners) != 4:
        return None

    corners = []
    for p in raw_corners:
        corners.append((p[0] + offset_x, p[1] + offset_y))

    corners = order_corners(corners)
    if not quad_is_reasonable(corners):
        return None

    x, y, w, h = bbox_from_corners(corners)
    bbox_area = w * h

    if w < MIN_WIDTH or h < MIN_HEIGHT:
        return None
    if bbox_area < MIN_AREA:
        return None
    if bbox_area < int(IMG_W * IMG_H * MIN_FRAME_COVERAGE):
        return None

    aspect = rect_aspect(corners)
    if aspect < RECT_ASPECT_MIN or aspect > RECT_ASPECT_MAX:
        return None

    cx, cy = center_of(corners)

    return {
        "corners": corners,
        "cx": cx,
        "cy": cy,
        "area": bbox_area,
        "aspect": aspect,
        "magnitude": rect.magnitude(),
        "bbox": (x, y, w, h),
    }


def score_candidate(candidate):
    score = float(candidate["area"]) + float(candidate["magnitude"]) * 8.0
    score -= abs(candidate["aspect"] - TARGET_RECT_ASPECT) * ASPECT_SCORE_PENALTY

    if track_valid and track_corners is not None:
        score -= (abs(candidate["cx"] - track_cx) + abs(candidate["cy"] - track_cy)) * TRACK_DIST_SCORE_PENALTY
        score -= abs(candidate["area"] - track_area) * TRACK_AREA_SCORE_PENALTY
    else:
        score -= (abs(candidate["cx"] - (IMG_W // 2)) + abs(candidate["cy"] - (IMG_H // 2))) * CENTER_SCORE_PENALTY

    return score


def pick_best(rects, offset_x=0, offset_y=0):
    best = None
    best_score = -1.0e30

    for rect in rects:
        candidate = candidate_from_rect(rect, offset_x, offset_y)
        if candidate is None:
            continue

        score = score_candidate(candidate)
        if score > best_score:
            best_score = score
            candidate["score"] = score
            best = candidate

    return best


def find_best_in_roi(img, roi=None):
    if roi is None:
        rects = img.find_rects(threshold=RECT_THRESHOLD)
        return pick_best(rects, 0, 0)

    roi_x, roi_y, roi_w, roi_h = roi
    rects = img.find_rects(
        roi=(roi_x, roi_y, roi_w, roi_h), threshold=RECT_THRESHOLD)
    return pick_best(rects, 0, 0)


def find_best_candidate(img):
    global search_frame_cnt

    used_roi = None
    search_frame_cnt += 1

    if track_valid and track_corners is not None:
        used_roi = track_search_roi(track_corners)
        best = find_best_in_roi(img, used_roi)
        if best is not None:
            return best, used_roi

        if track_lost < TRACK_REACQUIRE_CENTER_AFTER:
            return None, used_roi

    if USE_ROI:
        used_roi = center_search_roi()
        best = find_best_in_roi(img, used_roi)
        if best is not None:
            return best, used_roi

    if track_valid and track_corners is not None:
        if track_lost < TRACK_REACQUIRE_FULL_AFTER:
            return None, used_roi

    if (search_frame_cnt % FULL_SEARCH_EVERY_N_FRAME) != 0:
        return None, used_roi

    return find_best_in_roi(img, None), used_roi


def lock_track(candidate):
    global track_valid
    global track_lost
    global track_corners
    global track_cx
    global track_cy
    global track_area

    track_valid = True
    track_lost = 0
    track_corners = candidate["corners"]
    track_cx = candidate["cx"]
    track_cy = candidate["cy"]
    track_area = candidate["area"]


def update_track(candidate):
    global track_lost
    global track_corners
    global track_cx
    global track_cy
    global track_area

    new_corners = candidate["corners"]
    new_cx = candidate["cx"]
    new_cy = candidate["cy"]
    new_area = candidate["area"]

    if not track_valid or track_corners is None:
        lock_track(candidate)
        return

    dx = new_cx - track_cx
    dy = new_cy - track_cy
    center_ok = (dx * dx + dy * dy) < (CENTER_JUMP_LIMIT * CENTER_JUMP_LIMIT)
    area_ok = (new_area > track_area * AREA_CHANGE_LOW) and (new_area < track_area * AREA_CHANGE_HIGH)
    corner_ok = not corners_jump_too_much(track_corners, new_corners, CORNER_JUMP_LIMIT)

    if area_ok and center_ok and corner_ok:
        track_corners = smooth_corners(track_corners, new_corners, SMOOTH_A)
        track_cx = new_cx
        track_cy = new_cy
        track_area = new_area
        track_lost = 0
        return

    if track_lost >= (LOST_TOL // 2):
        lock_track(candidate)
    else:
        track_lost += 1


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
    uart_send_packet(1, cx, cy)


def uart_send_lost(cx, cy):
    uart_send_packet(0, cx, cy)


def main():
    global sensor
    global uart_frame_cnt
    global search_frame_cnt
    global track_valid
    global track_lost
    global track_corners

    try:
        if UART_ENABLE:
            uart_init()

        sensor = Sensor(id=sensor_id)
        sensor.reset()
        sensor.set_hmirror(False)
        sensor.set_vflip(False)
        sensor.set_framesize(width=IMG_W, height=IMG_H, chn=CAM_CHN_ID_0)
        sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_0)

        Display.init(Display.ST7701, width=LCD_W, height=LCD_H, to_ide=False)
        MediaManager.init()
        sensor.run()

        fps_cnt = 0
        display_frame_cnt = 0
        t0 = time.ticks_ms()
        search_frame_cnt = 0

        while True:
            os.exitpoint()

            img = sensor.snapshot(chn=CAM_CHN_ID_0)
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

            if UART_ENABLE:
                uart_frame_cnt += 1
                if uart_frame_cnt >= UART_SEND_N_FRAME:
                    uart_frame_cnt = 0
                    if track_valid and track_corners is not None:
                        uart_send_center(track_cx, track_cy)
                    else:
                        uart_send_lost(track_cx, track_cy)

            fps_cnt += 1
            dt = time.ticks_diff(time.ticks_ms(), t0)
            if dt >= 1000:
                fps = fps_cnt * 1000.0 / dt
                print("fps=%.2f valid=%d lost=%d cx=%d cy=%d" % (
                    fps,
                    1 if track_valid else 0,
                    track_lost,
                    track_cx if track_valid else -1,
                    track_cy if track_valid else -1,
                ))
                fps_cnt = 0
                t0 = time.ticks_ms()

            if DISPLAY_ENABLE:
                display_frame_cnt += 1
                if display_frame_cnt >= DISPLAY_SHOW_N_FRAME:
                    display_frame_cnt = 0

                    if track_valid and track_corners is not None:
                        draw_quad(img, track_corners, color=(255, 0, 0), thickness=3)
                        img.draw_cross(track_cx, track_cy, color=(0, 255, 0), size=10, thickness=2)
                        img.draw_circle(track_cx, track_cy, 4, color=(255, 255, 0), thickness=1)

                    if DRAW_DEBUG_ROI and used_roi is not None:
                        roi_x, roi_y, roi_w, roi_h = used_roi
                        img.draw_rectangle(roi_x, roi_y, roi_w, roi_h, color=(0, 0, 255), thickness=1)

                    Display.show_image(img, x=SHOW_X, y=SHOW_Y)

    except Exception as e:
        print("error:", e)

    finally:
        try:
            if sensor:
                sensor.stop()
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
