import sensor
import time
from pyb import UART, Pin, Timer


UART_ID = 3
UART_BAUDRATE = 115200

IMG_W = 160
IMG_H = 120
CENTER_X = IMG_W // 2

ROI_NEAR = (0, 88, IMG_W, 28)
ROI_MID = (0, 58, IMG_W, 28)
ROI_FAR = (0, 30, IMG_W, 28)
START_ROI = (15, 35, 130, 72)
FINISH_ROI = (15, 35, 130, 72)

DEBUG_DRAW = False
THRESHOLD_UPDATE_INTERVAL = 3
MID_FAR_UPDATE_INTERVAL = 2
COLOR_UPDATE_INTERVAL = 3
FRAME_COUNTER_RESET = 30000
FIXED_EXPOSURE_MODE = True
FIXED_EXPOSURE_US = 7000
FIXED_GAIN_DB = 12
LIGHT_PWM_ENABLE = True
LIGHT_PWM_PIN = "P6"
LIGHT_PWM_TIMER = 4
LIGHT_PWM_CHANNEL = 1
LIGHT_PWM_FREQ = 1000
LIGHT_PWM_DUTY = 90

# 主巡线使用 LAB 阈值，并根据当前画面亮度动态调整 L 通道。
BASE_BLACK_LINE_LAB = (0, 38, -18, 18, -18, 18)
BASE_WHITE_TRACK_LAB = (65, 100, -22, 22, -22, 28)
DYNAMIC_THRESHOLD_ROI = (0, 30, IMG_W, 85)
MIN_WHITE_L = 35
MAX_BLACK_L = 50
RED_THRESHOLD = (25, 85, 20, 80, 5, 70)
GREEN_THRESHOLD = (25, 85, -80, -20, 0, 70)

MIN_LINE_PIXELS = 12
MIN_LINE_AREA = 18
MAX_LINE_AREA_RATIO = 0.32
MAX_WIDE_LINE_HEIGHT = 20
SIDE_SAMPLE_WIDTH = 6
SIDE_SAMPLE_MARGIN = 2
MIN_WHITE_SIDE_PIXELS = 4
MIN_WHITE_SIDE_AREA_RATIO = 0.18
MAX_WHITE_SIDE_CHECKS = 3
MIN_COLOR_PIXELS = 45
MIN_FINISH_PIXELS = 300
MIN_FINISH_AREA = 450
MAX_CENTER_STEP = 28
LINE_HOLD_FRAMES = 2
FILTER_OLD_WEIGHT = 3
MIN_MARKER_CONFIDENCE = 70
MARKER_CONFIRM_FRAMES = 2

ROAD_UNKNOWN = 0
ROAD_STRAIGHT = 1
ROAD_LEFT_CURVE = 2
ROAD_RIGHT_CURVE = 3
ROAD_FORK = 4
ROAD_CROSS = 5
ROAD_T_LEFT = 6
ROAD_T_RIGHT = 7
ROAD_FINISH = 8

PREF_NEAREST = 0
PREF_LEFT = 1
PREF_RIGHT = 2

FLAG_LINE_VALID = 0x01
FLAG_START = 0x02
FLAG_FINISH = 0x04
FLAG_MARKER = 0x08

uart = UART(UART_ID, UART_BAUDRATE, timeout_char=10)
sequence = 0
segment_preference = PREF_NEAREST
line_locked = False
lost_line_frames = 0
last_center_x = CENTER_X
filtered_offset = 0
filtered_left = 56
filtered_right = 72
last_road_type = ROAD_UNKNOWN
marker_frames = 0
frame_count = 0
cached_black_threshold = BASE_BLACK_LINE_LAB
cached_white_threshold = BASE_WHITE_TRACK_LAB
cached_mid_blobs = []
cached_far_blobs = []
cached_start_candidate = False
cached_finish_color_candidate = False
cached_finish_shape_candidate = False
cached_finish_blob = None


def clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


def norm_x(x):
    return int(clamp((x * 127) // (IMG_W - 1), 0, 127))


def init_fill_light():
    if not LIGHT_PWM_ENABLE:
        return None

    try:
        tim = Timer(LIGHT_PWM_TIMER, freq=LIGHT_PWM_FREQ)
        channel = tim.channel(LIGHT_PWM_CHANNEL, Timer.PWM, pin=Pin(LIGHT_PWM_PIN))
        channel.pulse_width_percent(LIGHT_PWM_DUTY)
        return channel
    except Exception:
        # PWM 映射异常时退回常亮，避免补光板完全不亮。
        pin = Pin(LIGHT_PWM_PIN, Pin.OUT_PP)
        pin.high()
        return pin


def read_preference_command():
    global segment_preference

    while uart.any() >= 4:
        data = uart.read(4)
        if data is None or len(data) != 4:
            return

        if data[0] != 0xA5 or data[1] != 0x5A:
            continue

        if (data[0] ^ data[1] ^ data[2]) != data[3]:
            continue

        if data[2] in (PREF_NEAREST, PREF_LEFT, PREF_RIGHT):
            segment_preference = data[2]


def safe_roi(x, y, w, h):
    x = int(clamp(x, 0, IMG_W - 1))
    y = int(clamp(y, 0, IMG_H - 1))
    w = int(clamp(w, 0, IMG_W - x))
    h = int(clamp(h, 0, IMG_H - y))
    if w <= 0 or h <= 0:
        return None
    return (x, y, w, h)


def build_line_thresholds(img):
    stats = img.get_statistics(roi=DYNAMIC_THRESHOLD_ROI)
    mean_l = stats.l_mean()

    black_l_max = int(clamp(mean_l - 12, BASE_BLACK_LINE_LAB[1], MAX_BLACK_L))
    white_l_min = int(clamp(mean_l - 12, MIN_WHITE_L, BASE_WHITE_TRACK_LAB[0]))

    black_threshold = (BASE_BLACK_LINE_LAB[0], black_l_max,
                       BASE_BLACK_LINE_LAB[2], BASE_BLACK_LINE_LAB[3],
                       BASE_BLACK_LINE_LAB[4], BASE_BLACK_LINE_LAB[5])
    white_threshold = (white_l_min, BASE_WHITE_TRACK_LAB[1],
                       BASE_WHITE_TRACK_LAB[2], BASE_WHITE_TRACK_LAB[3],
                       BASE_WHITE_TRACK_LAB[4], BASE_WHITE_TRACK_LAB[5])

    return black_threshold, white_threshold


def roi_has_white_track(img, roi, white_threshold):
    if roi is None:
        return False

    blobs = img.find_blobs([white_threshold],
                           roi=roi,
                           pixels_threshold=MIN_WHITE_SIDE_PIXELS,
                           area_threshold=MIN_WHITE_SIDE_PIXELS,
                           merge=True,
                           margin=2,
                           x_stride=2,
                           y_stride=2)
    white_pixels = 0
    for blob in blobs:
        white_pixels += blob.pixels()

    return white_pixels >= int(roi[2] * roi[3] * MIN_WHITE_SIDE_AREA_RATIO)


def max_line_width_for_roi(roi):
    if roi == ROI_NEAR:
        return 60
    if roi == ROI_MID:
        return 48
    return 36


def min_line_width_for_roi(roi):
    if roi == ROI_NEAR:
        return 3
    if roi == ROI_MID:
        return 2
    return 2


def line_has_white_side(img, blob, roi, white_threshold):
    line_width = blob.w()
    side_y = blob.y() + (blob.h() // 4)
    side_h = max(4, blob.h() // 2)
    roi_x, roi_y, roi_w, roi_h = roi
    roi_right = roi_x + roi_w
    roi_bottom = roi_y + roi_h

    if line_width < min_line_width_for_roi(roi) or line_width > max_line_width_for_roi(roi):
        return False

    side_y = int(clamp(side_y, roi_y, roi_bottom - 1))
    side_h = int(clamp(side_h, 1, roi_bottom - side_y))

    left = None
    right = None
    left_x = blob.x() - SIDE_SAMPLE_MARGIN - SIDE_SAMPLE_WIDTH
    right_x = blob.x() + blob.w() + SIDE_SAMPLE_MARGIN

    left_ok = False
    right_ok = False
    if left_x >= roi_x:
        left = safe_roi(left_x, side_y, SIDE_SAMPLE_WIDTH, side_h)
        left_ok = roi_has_white_track(img, left, white_threshold)
    if (right_x + SIDE_SAMPLE_WIDTH) <= roi_right:
        right = safe_roi(right_x, side_y, SIDE_SAMPLE_WIDTH, side_h)
        right_ok = roi_has_white_track(img, right, white_threshold)

    return left_ok or right_ok


def find_line_blobs(img, roi, black_threshold, white_threshold):
    blobs = img.find_blobs([black_threshold],
                           roi=roi,
                           pixels_threshold=MIN_LINE_PIXELS,
                           area_threshold=MIN_LINE_AREA,
                           merge=True,
                           margin=8,
                           x_stride=2,
                           y_stride=2)

    candidates = []
    roi_area = roi[2] * roi[3]
    for blob in blobs:
        if blob.w() < 3 or blob.h() < 3:
            continue

        # 真实赛道线可以横向或纵向延伸，但不应像大块深色地面一样填满二维区域。
        if blob.area() > int(roi_area * MAX_LINE_AREA_RATIO):
            continue
        if blob.w() > 110 and blob.h() > MAX_WIDE_LINE_HEIGHT:
            continue

        candidates.append(blob)

    candidates = sorted(candidates, key=lambda b: b.pixels(), reverse=True)

    filtered = []
    for blob in candidates[:MAX_WHITE_SIDE_CHECKS]:
        if not line_has_white_side(img, blob, roi, white_threshold):
            continue

        filtered.append(blob)

    return filtered


def blob_center(blob):
    return blob.x() + (blob.w() // 2)


def select_blob(blobs, expected_center=None):
    if not blobs:
        return None, 0

    blobs = sorted(blobs, key=lambda b: blob_center(b))

    if segment_preference == PREF_LEFT:
        return blobs[0], 0

    if segment_preference == PREF_RIGHT:
        index = len(blobs) - 1
        return blobs[index], index

    if expected_center is None:
        expected_center = CENTER_X

    best_index = 0
    best_score = 10000
    for i, blob in enumerate(blobs):
        score = abs(blob_center(blob) - expected_center)
        if score < best_score:
            best_score = score
            best_index = i

    return blobs[best_index], best_index


def limit_jump(blobs, selected, selected_index):
    global line_locked, last_center_x

    if (selected is None) or (not line_locked):
        return selected, selected_index

    if abs(blob_center(selected) - last_center_x) <= MAX_CENTER_STEP:
        return selected, selected_index

    stable_blobs = [b for b in blobs if abs(blob_center(b) - last_center_x) <= MAX_CENTER_STEP]
    if stable_blobs:
        return select_blob(stable_blobs, last_center_x)

    return None, 0


def update_filtered_line(selected):
    global line_locked, lost_line_frames, last_center_x
    global filtered_offset, filtered_left, filtered_right

    if selected is None:
        lost_line_frames += 1
        if line_locked and lost_line_frames <= LINE_HOLD_FRAMES:
            return True, filtered_offset, filtered_left, filtered_right, 20

        line_locked = False
        return False, 0, 0, 0, 0

    center = blob_center(selected)
    raw_offset = norm_x(center) - 64
    raw_left = norm_x(selected.x())
    raw_right = norm_x(selected.x() + selected.w())

    if line_locked:
        filtered_offset = ((filtered_offset * FILTER_OLD_WEIGHT) + raw_offset) // (FILTER_OLD_WEIGHT + 1)
        filtered_left = ((filtered_left * FILTER_OLD_WEIGHT) + raw_left) // (FILTER_OLD_WEIGHT + 1)
        filtered_right = ((filtered_right * FILTER_OLD_WEIGHT) + raw_right) // (FILTER_OLD_WEIGHT + 1)
        last_center_x = ((last_center_x * FILTER_OLD_WEIGHT) + center) // (FILTER_OLD_WEIGHT + 1)
    else:
        filtered_offset = raw_offset
        filtered_left = raw_left
        filtered_right = raw_right
        last_center_x = center
        line_locked = True

    lost_line_frames = 0
    return True, filtered_offset, filtered_left, filtered_right, None


def detect_road(near_blobs, mid_blobs, far_blobs, selected):
    if selected is None:
        return ROAD_UNKNOWN

    near_count = len(near_blobs)
    mid_count = len(mid_blobs)
    far_count = len(far_blobs)

    if near_count >= 3 or (near_count >= 2 and mid_count >= 2):
        return ROAD_CROSS

    if near_count >= 2:
        left_seen = any(blob_center(b) < CENTER_X - 18 for b in near_blobs)
        right_seen = any(blob_center(b) > CENTER_X + 18 for b in near_blobs)
        if left_seen and right_seen:
            return ROAD_FORK
        if left_seen:
            return ROAD_T_LEFT
        if right_seen:
            return ROAD_T_RIGHT
        return ROAD_FORK

    mid = None
    if mid_blobs:
        mid, _ = select_blob(mid_blobs)

    far = None
    if far_blobs:
        far, _ = select_blob(far_blobs)

    dx = 0
    samples = 0
    selected_center = blob_center(selected)
    if mid is not None:
        dx += blob_center(mid) - selected_center
        samples += 1
    if far is not None:
        dx += blob_center(far) - selected_center
        samples += 1

    if samples == 0:
        return ROAD_STRAIGHT

    dx //= samples
    if dx < -9:
        return ROAD_LEFT_CURVE
    if dx > 9:
        return ROAD_RIGHT_CURVE
    return ROAD_STRAIGHT


def is_route_marker(road_type, segment_count):
    return segment_count >= 2 or road_type in (ROAD_FORK, ROAD_CROSS, ROAD_T_LEFT, ROAD_T_RIGHT)


def stabilize_marker(line_valid, road_type, segment_count, confidence):
    global marker_frames

    if line_valid and is_route_marker(road_type, segment_count) and confidence >= MIN_MARKER_CONFIDENCE:
        if marker_frames < MARKER_CONFIRM_FRAMES:
            marker_frames += 1
    else:
        marker_frames = 0

    return marker_frames >= MARKER_CONFIRM_FRAMES


def confidence_from_blobs(selected, near_blobs, mid_blobs, far_blobs):
    if selected is None:
        return 0

    conf = 45
    if selected.pixels() > 30:
        conf += 15
    if selected.h() > 6:
        conf += 10
    if mid_blobs:
        conf += 15
    if far_blobs:
        conf += 10
    if len(near_blobs) > 1:
        conf -= 5

    return int(clamp(conf, 0, 100))


def color_present(img, threshold, min_pixels, roi=None):
    if roi is None:
        blobs = img.find_blobs([threshold],
                               pixels_threshold=min_pixels,
                               area_threshold=min_pixels,
                               merge=True,
                               margin=10)
    else:
        blobs = img.find_blobs([threshold],
                               roi=roi,
                               pixels_threshold=min_pixels,
                               area_threshold=min_pixels,
                               merge=True,
                               margin=10)
    return len(blobs) > 0


def finish_block_present(img):
    blobs = img.find_blobs([RED_THRESHOLD],
                           roi=FINISH_ROI,
                           pixels_threshold=MIN_FINISH_PIXELS,
                           area_threshold=MIN_FINISH_AREA,
                           merge=True,
                           margin=12,
                           x_stride=2,
                           y_stride=2)

    for blob in blobs:
        if blob.w() >= 23 and blob.h() >= 13 and blob.pixels() >= MIN_FINISH_PIXELS:
            return True, blob

    return False, None


def send_frame(flags, offset, left, right, segment_count, selected_index, road_type, confidence):
    global sequence

    offset = int(clamp(offset, -64, 63))
    if offset < 0:
        offset_u16 = (1 << 16) + offset
    else:
        offset_u16 = offset

    payload = bytearray(14)
    payload[0] = 0xAA
    payload[1] = 0x55
    payload[2] = 10
    payload[3] = sequence & 0xFF
    payload[4] = flags & 0xFF
    payload[5] = offset_u16 & 0xFF
    payload[6] = (offset_u16 >> 8) & 0xFF
    payload[7] = left & 0x7F
    payload[8] = right & 0x7F
    payload[9] = segment_count & 0xFF
    payload[10] = selected_index & 0xFF
    payload[11] = road_type & 0xFF
    payload[12] = confidence & 0xFF

    checksum = 0
    for i in range(2, 13):
        checksum ^= payload[i]
    payload[13] = checksum & 0xFF

    uart.write(payload)
    sequence = (sequence + 1) & 0xFF


def draw_debug(img, near_blobs, mid_blobs, far_blobs, selected, finish_blob,
               road_type, confidence, black_threshold, white_threshold):
    img.draw_rectangle(ROI_FAR, color=(255, 0, 0))
    img.draw_rectangle(ROI_MID, color=(0, 255, 0))
    img.draw_rectangle(ROI_NEAR, color=(0, 0, 255))
    img.draw_rectangle(FINISH_ROI, color=(255, 255, 0))

    for blob in far_blobs:
        img.draw_rectangle(blob.rect(), color=(255, 0, 0))
    for blob in mid_blobs:
        img.draw_rectangle(blob.rect(), color=(0, 255, 0))
    for blob in near_blobs:
        img.draw_rectangle(blob.rect(), color=(0, 0, 255))

    if selected is not None:
        img.draw_rectangle(selected.rect(), color=(255, 255, 255))
        img.draw_cross(blob_center(selected), selected.cy(), color=(255, 255, 255))

    if finish_blob is not None:
        img.draw_rectangle(finish_blob.rect(), color=(255, 255, 0))

    img.draw_string(2, 2, "R:%d C:%d P:%d B:%d W:%d" %
                    (road_type, confidence, segment_preference,
                     black_threshold[1], white_threshold[0]),
                    color=(255, 255, 255), mono_space=False)


sensor.reset()
fill_light = init_fill_light()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)

sensor.set_auto_exposure(True)
sensor.set_auto_gain(True)
sensor.set_auto_whitebal(True)
sensor.skip_frames(time=2000)

# 固定曝光能避免暗光下自动曝光拉长帧周期；现场太暗时优先加补光。
if FIXED_EXPOSURE_MODE:
    sensor.set_auto_exposure(False, exposure_us=FIXED_EXPOSURE_US)
    sensor.set_auto_gain(False, gain_db=FIXED_GAIN_DB)
else:
    sensor.set_auto_exposure(True)
    sensor.set_auto_gain(True)
sensor.set_auto_whitebal(False)
sensor.skip_frames(time=500)

clock = time.clock()

while True:
    frame_count += 1
    if frame_count >= FRAME_COUNTER_RESET:
        frame_count = 1

    clock.tick()
    read_preference_command()

    img = sensor.snapshot()
    if frame_count == 1 or (frame_count % THRESHOLD_UPDATE_INTERVAL) == 0:
        cached_black_threshold, cached_white_threshold = build_line_thresholds(img)
    black_threshold = cached_black_threshold
    white_threshold = cached_white_threshold

    near_blobs = find_line_blobs(img, ROI_NEAR, black_threshold, white_threshold)
    if frame_count == 1 or (frame_count % MID_FAR_UPDATE_INTERVAL) == 0:
        cached_mid_blobs = find_line_blobs(img, ROI_MID, black_threshold, white_threshold)
        cached_far_blobs = find_line_blobs(img, ROI_FAR, black_threshold, white_threshold)
    mid_blobs = cached_mid_blobs
    far_blobs = cached_far_blobs
    selected, selected_index = select_blob(near_blobs, last_center_x if line_locked else CENTER_X)
    selected, selected_index = limit_jump(near_blobs, selected, selected_index)

    flags = 0
    offset = 0
    left = 0
    right = 0
    road_type = detect_road(near_blobs, mid_blobs, far_blobs, selected)
    line_valid, offset, left, right, hold_confidence = update_filtered_line(selected)
    confidence = confidence_from_blobs(selected, near_blobs, mid_blobs, far_blobs)

    if line_valid:
        flags |= FLAG_LINE_VALID
        if hold_confidence is not None:
            confidence = hold_confidence
            road_type = last_road_type
        else:
            last_road_type = road_type

    if frame_count == 1 or (frame_count % COLOR_UPDATE_INTERVAL) == 0:
        cached_start_candidate = color_present(img, GREEN_THRESHOLD, MIN_COLOR_PIXELS, roi=START_ROI)
        cached_finish_color_candidate = color_present(img, RED_THRESHOLD, MIN_COLOR_PIXELS, roi=FINISH_ROI)
        cached_finish_shape_candidate, cached_finish_blob = finish_block_present(img)
    start_candidate = cached_start_candidate
    finish_color_candidate = cached_finish_color_candidate
    finish_shape_candidate = cached_finish_shape_candidate
    finish_blob = cached_finish_blob
    finish_candidate = finish_color_candidate or finish_shape_candidate

    if start_candidate:
        flags |= FLAG_START
    if finish_candidate:
        flags |= FLAG_FINISH
        road_type = ROAD_FINISH

    segment_count = len(near_blobs)
    marker_confirmed = stabilize_marker(line_valid, road_type, segment_count, confidence)
    if not marker_confirmed and is_route_marker(road_type, segment_count):
        road_type = ROAD_STRAIGHT if line_valid else ROAD_UNKNOWN
        segment_count = 1 if line_valid else 0

    if marker_confirmed:
        flags |= FLAG_MARKER

    send_frame(flags, offset, left, right, segment_count, selected_index, road_type, confidence)
    if DEBUG_DRAW:
        draw_debug(img, near_blobs, mid_blobs, far_blobs, selected, finish_blob,
                   road_type, confidence, black_threshold, white_threshold)
