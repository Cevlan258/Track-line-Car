import sensor
import time
from pyb import UART

UART_ID = 3
BAUDRATE = 115200

IMG_W = 320
IMG_H = 240
CENTER_X = IMG_W // 2

ROI_FAR = (0, 45, IMG_W, 45)
ROI_MID = (0, 105, IMG_W, 45)
ROI_NEAR = (0, 165, IMG_W, 55)

BLACK_THRESHOLD = (0, 55, -25, 25, -25, 25)
RED_THRESHOLD = (25, 85, 20, 80, 5, 70)
GREEN_THRESHOLD = (25, 85, -80, -20, 0, 70)

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

uart = UART(UART_ID, BAUDRATE, timeout_char=5)
route_preference = PREF_NEAREST
last_center_x = CENTER_X
seq = 0
cmd_buf = []


def clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


def scale_x_to_128(x):
    return int(clamp((x * 128) // IMG_W, 0, 127))


def read_route_preference():
    global route_preference, cmd_buf

    while uart.any():
        b = uart.readchar()
        if b < 0:
            return
        if len(cmd_buf) == 0 and b != 0xA5:
            continue
        if len(cmd_buf) == 1 and b != 0x5A:
            cmd_buf = [0xA5] if b == 0xA5 else []
            continue

        cmd_buf.append(b)
        if len(cmd_buf) >= 4:
            checksum = cmd_buf[0] ^ cmd_buf[1] ^ cmd_buf[2]
            if checksum == cmd_buf[3] and cmd_buf[2] in (PREF_NEAREST, PREF_LEFT, PREF_RIGHT):
                route_preference = cmd_buf[2]
            cmd_buf = []


def find_line_blobs(img, roi):
    blobs = img.find_blobs([BLACK_THRESHOLD], roi=roi, pixels_threshold=30, area_threshold=40, merge=True)
    blobs = [b for b in blobs if b.w() >= 3 and b.h() >= 3]
    blobs.sort(key=lambda b: b.cx())
    return blobs


def select_blob(blobs):
    global last_center_x

    if not blobs:
        return None, 0
    if route_preference == PREF_LEFT:
        return blobs[0], 0
    if route_preference == PREF_RIGHT:
        return blobs[-1], len(blobs) - 1

    selected = 0
    best = 10000
    for i, blob in enumerate(blobs):
        err = abs(blob.cx() - last_center_x)
        if err < best:
            best = err
            selected = i
    return blobs[selected], selected


def color_present(img, threshold, min_pixels):
    blobs = img.find_blobs([threshold], pixels_threshold=min_pixels, area_threshold=min_pixels, merge=True)
    return len(blobs) > 0


def detect_road(near_blob, mid_blob, far_blobs):
    if len(far_blobs) >= 3:
        return ROAD_CROSS
    if len(far_blobs) == 2:
        left_seen = far_blobs[0].cx() < CENTER_X - 20
        right_seen = far_blobs[1].cx() > CENTER_X + 20
        if left_seen and right_seen:
            return ROAD_FORK
        if left_seen:
            return ROAD_T_LEFT
        if right_seen:
            return ROAD_T_RIGHT

    if near_blob and mid_blob:
        delta = mid_blob.cx() - near_blob.cx()
        if delta < -22:
            return ROAD_LEFT_CURVE
        if delta > 22:
            return ROAD_RIGHT_CURVE
        return ROAD_STRAIGHT

    return ROAD_UNKNOWN


def confidence_for(blob):
    if not blob:
        return 0
    value = int(blob.pixels() // 6)
    return clamp(value, 0, 100)


def send_frame(flags, offset, left, right, segment_count, selected, road_type, confidence):
    global seq

    payload_len = 10
    offset_u16 = offset & 0xFFFF
    frame = bytearray(14)
    frame[0] = 0xAA
    frame[1] = 0x55
    frame[2] = payload_len
    frame[3] = seq & 0xFF
    frame[4] = flags & 0xFF
    frame[5] = offset_u16 & 0xFF
    frame[6] = (offset_u16 >> 8) & 0xFF
    frame[7] = left & 0xFF
    frame[8] = right & 0xFF
    frame[9] = segment_count & 0xFF
    frame[10] = selected & 0xFF
    frame[11] = road_type & 0xFF
    frame[12] = confidence & 0xFF

    checksum = 0
    for b in frame[2:13]:
        checksum ^= b
    frame[13] = checksum
    uart.write(frame)
    seq = (seq + 1) & 0xFF


sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=1500)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

clock = time.clock()

while True:
    clock.tick()
    read_route_preference()

    img = sensor.snapshot()
    near_blobs = find_line_blobs(img, ROI_NEAR)
    mid_blobs = find_line_blobs(img, ROI_MID)
    far_blobs = find_line_blobs(img, ROI_FAR)
    near_blob, selected_index = select_blob(near_blobs)
    mid_blob, _ = select_blob(mid_blobs)

    flags = 0
    offset = 0
    left = 56
    right = 72
    confidence = 0

    if near_blob:
        last_center_x = near_blob.cx()
        center_128 = scale_x_to_128(near_blob.cx())
        offset = clamp(center_128 - 64, -64, 63)
        left = scale_x_to_128(near_blob.x())
        right = scale_x_to_128(near_blob.x() + near_blob.w())
        confidence = confidence_for(near_blob)
        flags |= FLAG_LINE_VALID

    if color_present(img, GREEN_THRESHOLD, 180):
        flags |= FLAG_START
    if color_present(img, RED_THRESHOLD, 180):
        flags |= FLAG_FINISH

    road_type = detect_road(near_blob, mid_blob, far_blobs)
    if flags & FLAG_FINISH:
        road_type = ROAD_FINISH
    if len(near_blobs) >= 2 or road_type in (ROAD_FORK, ROAD_CROSS, ROAD_T_LEFT, ROAD_T_RIGHT):
        flags |= FLAG_MARKER

    send_frame(flags, offset, left, right, len(near_blobs), selected_index, road_type, confidence)
