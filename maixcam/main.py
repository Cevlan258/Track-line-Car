from maix import app, camera, display, image, pinmap, time, uart


UART_DEVICE = "/dev/ttyS1"
UART_BAUD = 115200
UART_RX_PIN = "A18"
UART_TX_PIN = "A19"

IMG_W = 320
IMG_H = 240
CENTER_X = IMG_W // 2

# 调试阶段保留较宽视野，扫描线算法会从近到远抽取多个横向带。
SCAN_BAND_H = 16
SCAN_BANDS = [
    [0, 214, IMG_W, SCAN_BAND_H],
    [0, 194, IMG_W, SCAN_BAND_H],
    [0, 174, IMG_W, SCAN_BAND_H],
    [0, 154, IMG_W, SCAN_BAND_H],
    [0, 134, IMG_W, SCAN_BAND_H],
    [0, 114, IMG_W, SCAN_BAND_H],
    [0, 94, IMG_W, SCAN_BAND_H],
    [0, 74, IMG_W, SCAN_BAND_H],
    [0, 54, IMG_W, SCAN_BAND_H],
]
FINISH_ROI = [30, 70, 260, 145]

BLACK_LINE_LAB = [0, 38, -18, 18, -18, 18]
RED_FINISH_LAB = [25, 85, 20, 80, 5, 70]

HEAD_0 = 0xA6
HEAD_1 = 0x6A
TYPE_COMMAND = 0x01
TYPE_TELEMETRY = 0x81
COMMAND_BODY_LEN = 11
TELEMETRY_BODY_LEN = 39

MODE_IDLE = 0
MODE_RUN = 1
MODE_FINISH = 2
MODE_FAULT = 3

CMD_FLAG_LINE_VALID = 0x01
CMD_FLAG_FINISH = 0x02
CMD_FLAG_AVOIDING = 0x04

STATUS_FLAG_STARTED = 0x01
LORA_IDLE = 0
LORA_SENT = 2

ROAD_UNKNOWN = 0
ROAD_STRAIGHT = 1
ROAD_LEFT_CURVE = 2
ROAD_RIGHT_CURVE = 3
ROAD_FORK = 4
ROAD_CROSS = 5
ROAD_FINISH = 8

ZONE_BOOT_LOCAL = 0
ZONE_DEAD_END_FILTER = 1
ZONE_MIRROR_DISCOVERY = 2
ZONE_S_CURVE = 3
ZONE_RECT_ZONE = 4
ZONE_CIRCLE_RECT = 5
ZONE_FINISH_APPROACH = 6

ZONE_NAMES = [
    "BOOT",
    "DEAD",
    "MIRROR",
    "S",
    "RECT",
    "CIRCLE",
    "FINISH",
]

LINE_SPEED = 320
LINE_MIN_SPEED = 180
LINE_BOOT_SPEED = 170
LINE_LOOP_SPEED = 190
LINE_COMPLEX_SPEED = 155
LINE_FINISH_SPEED = 150
LINE_MAX_YAW = 1800
LINE_KP = 11
LINE_PREVIEW_KP = 9
LINE_KD = 5

SEARCH_SPEED = 120
SEARCH_YAW = 900
LOST_HOLD_FRAMES = 3
SEARCH_FRAMES = 45

GATE_MIN_CM = 15
GATE_MAX_CM = 90
GATE_ENTER_STABLE_MS = 120
GATE_EXIT_STABLE_MS = 150
GATE_LOCK_MS = 1200
GATE_LOCK_DISTANCE_MM = 300
GATE_EXIT_JUMP_CM = 30
GATE_POST_MIN_ABS_X_MM = 120
GATE_PAIR_Y_TOL_MM = 250
GATE_WIDTH_MIN_MM = 300
GATE_WIDTH_MAX_MM = 1600

AVOID_SPEED = 260
AVOID_YAW = 1100
AVOID_ENTRY_MS = 700
AVOID_PASS_MS = 900
AVOID_RECOVER_MS = 650

# 这些里程窗口来自赛道图尺寸，是现场调参入口，不作为唯一判据。
ZONE_BOOT_END_MM = 450
ZONE_DEAD_END_END_MM = 2500
ZONE_MIRROR_END_MM = 4700
ZONE_S_END_MM = 6100
ZONE_RECT_END_MM = 8000
ZONE_CIRCLE_END_MM = 10300

PATH_MAX_LINK_DX = 58
PATH_MIN_WIDTH = 3
PATH_MAX_WIDTH = 105
PATH_MIN_PIXELS = 8
DEAD_STUB_MIN_ROWS = 4
TANGENT_LOCK_MS = 420
MIRROR_LOCK_SCORE = 210


def ticks_ms():
    try:
        return time.ticks_ms()
    except Exception:
        return int(time.time() * 1000)


def clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


def sign(value):
    if value > 0:
        return 1
    if value < 0:
        return -1
    return 0


def norm_offset(x):
    return int(clamp(((x - CENTER_X) * 127) // IMG_W, -64, 63))


def crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= (byte & 0xFF) << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def put_i16(value):
    value &= 0xFFFF
    return bytes((value & 0xFF, (value >> 8) & 0xFF))


def get_u16(buf, offset):
    return buf[offset] | (buf[offset + 1] << 8)


def get_i16(buf, offset):
    value = get_u16(buf, offset)
    if value & 0x8000:
        value -= 0x10000
    return value


def get_i32(buf, offset):
    value = (buf[offset] |
             (buf[offset + 1] << 8) |
             (buf[offset + 2] << 16) |
             (buf[offset + 3] << 24))
    if value & 0x80000000:
        value -= 0x100000000
    return value


def blob_value(blob, name, default=0):
    try:
        attr = getattr(blob, name)
        return attr() if callable(attr) else attr
    except Exception:
        return default


def blob_x(blob):
    return blob_value(blob, "x", 0)


def blob_y(blob):
    return blob_value(blob, "y", 0)


def blob_w(blob):
    return blob_value(blob, "w", 0)


def blob_h(blob):
    return blob_value(blob, "h", 0)


def blob_pixels(blob):
    return blob_value(blob, "pixels", blob_w(blob) * blob_h(blob))


class CameraAcq:
    def __init__(self):
        self.cam = camera.Camera(IMG_W, IMG_H)

    def read(self):
        return self.cam.read()


class UartLink:
    def __init__(self):
        pinmap.set_pin_function(UART_RX_PIN, "UART1_RX")
        pinmap.set_pin_function(UART_TX_PIN, "UART1_TX")
        self.serial = uart.UART(UART_DEVICE, UART_BAUD)
        self.rx = bytearray()
        self.telemetry = None
        self.last_telemetry_ms = 0
        self.seq = 0

    def poll(self):
        data = self.serial.read()
        if data:
            for byte in data:
                self._feed(byte)
        return self.telemetry

    def _feed(self, byte):
        if len(self.rx) == 0:
            if byte != HEAD_0:
                return
        elif len(self.rx) == 1:
            if byte != HEAD_1:
                self.rx = bytearray([HEAD_0]) if byte == HEAD_0 else bytearray()
                return

        self.rx.append(byte)
        if len(self.rx) == 3 and self.rx[2] != TELEMETRY_BODY_LEN:
            self.rx = bytearray()
            return

        if len(self.rx) >= 3:
            total = 2 + 1 + self.rx[2] + 2
            if len(self.rx) == total:
                self._parse_frame(bytes(self.rx))
                self.rx = bytearray()

    def _parse_frame(self, frame):
        body_crc = crc16(frame[2:-2])
        frame_crc = get_u16(frame, len(frame) - 2)
        if body_crc != frame_crc or frame[3] != TYPE_TELEMETRY:
            return

        targets = []
        for index in range(3):
            offset = 15 + index * 9
            targets.append({
                "valid": frame[offset],
                "x_mm": get_i16(frame, offset + 1),
                "y_mm": get_i16(frame, offset + 3),
                "speed_cm_s": get_i16(frame, offset + 5),
                "distance_mm": get_u16(frame, offset + 7),
            })

        self.telemetry = {
            "seq": frame[4],
            "status_flags": frame[5],
            "lora_status": frame[6],
            "battery_percent": frame[7],
            "battery_x100": get_u16(frame, 8),
            "distance_mm": get_i32(frame, 10),
            "radar_count": frame[14],
            "targets": targets,
        }
        self.last_telemetry_ms = ticks_ms()

    def started(self):
        return self.telemetry is not None and (self.telemetry["status_flags"] & STATUS_FLAG_STARTED) != 0

    def send_command(self, mode, vx, yaw, flags, route_step, checkpoint, confidence):
        body = bytes((
            TYPE_COMMAND,
            self.seq & 0xFF,
            mode & 0xFF,
            flags & 0xFF,
            route_step & 0xFF,
            checkpoint & 0xFF,
            confidence & 0xFF,
        )) + put_i16(vx) + put_i16(yaw)
        frame_wo_crc = bytes((HEAD_0, HEAD_1, len(body))) + body
        checksum = crc16(frame_wo_crc[2:])
        self.serial.write(frame_wo_crc + bytes((checksum & 0xFF, (checksum >> 8) & 0xFF)))
        self.seq = (self.seq + 1) & 0xFF


class ScanlineExtractor:
    def __init__(self):
        self.last_rows = []

    def _find_blobs(self, img, roi):
        try:
            return img.find_blobs([BLACK_LINE_LAB], roi=roi, pixels_threshold=PATH_MIN_PIXELS,
                                  area_threshold=PATH_MIN_PIXELS, merge=True, margin=4)
        except Exception:
            return img.find_blobs(thresholds=[BLACK_LINE_LAB], roi=roi,
                                  pixels_threshold=PATH_MIN_PIXELS,
                                  area_threshold=PATH_MIN_PIXELS, merge=True, margin=4)

    def extract(self, img):
        rows = []
        for row_index, roi in enumerate(SCAN_BANDS):
            segments = []
            blobs = self._find_blobs(img, roi)
            for blob in blobs:
                x = int(blob_x(blob))
                y = int(blob_y(blob))
                w = int(blob_w(blob))
                h = int(blob_h(blob))
                pixels = int(blob_pixels(blob))
                if w < PATH_MIN_WIDTH or w > PATH_MAX_WIDTH or pixels < PATH_MIN_PIXELS:
                    continue
                segments.append({
                    "row": row_index,
                    "x": x,
                    "y": y,
                    "w": w,
                    "h": h,
                    "cx": x + w // 2,
                    "cy": y + h // 2,
                    "pixels": pixels,
                    "used": False,
                })
            rows.append({
                "index": row_index,
                "roi": roi,
                "y": roi[1] + roi[3] // 2,
                "segments": sorted(segments, key=lambda s: s["cx"]),
            })
        self.last_rows = rows
        return rows

    def finish_detected(self, img):
        try:
            blobs = img.find_blobs([RED_FINISH_LAB], roi=FINISH_ROI, pixels_threshold=300,
                                   area_threshold=450, merge=True, margin=6)
        except Exception:
            blobs = img.find_blobs(thresholds=[RED_FINISH_LAB], roi=FINISH_ROI,
                                   pixels_threshold=300, area_threshold=450,
                                   merge=True, margin=6)
        return bool(blobs)


class PathGraph:
    def build(self, rows):
        starts = []
        for row in rows[:3]:
            for seg in row["segments"]:
                starts.append(seg)
            if starts:
                break

        paths = []
        for start in starts:
            path = [start]
            last_dx = 0
            for row in rows[start["row"] + 1:]:
                last = path[-1]
                predicted_x = last["cx"] + last_dx
                best = None
                best_score = 10000
                for seg in row["segments"]:
                    dx = seg["cx"] - predicted_x
                    if abs(dx) > PATH_MAX_LINK_DX:
                        continue
                    score = abs(dx) + abs(seg["w"] - last["w"]) * 0.3
                    if score < best_score:
                        best = seg
                        best_score = score
                if best is None:
                    continue
                last_dx = best["cx"] - last["cx"]
                path.append(best)
            paths.append(self._summarize(path, rows))

        # 远端才出现的线段也可能是主线预瞄，保留少量候选用于调试和评分。
        for row in rows[1:4]:
            for seg in row["segments"]:
                if not any(p["points"][0] is seg for p in paths):
                    paths.append(self._summarize([seg], rows))

        paths = sorted(paths, key=lambda p: (p["rows"], -abs(p["offset"])), reverse=True)
        return paths[:10]

    def _summarize(self, points, rows):
        near = points[0]
        far = points[-1]
        lookahead = points[min(len(points) - 1, max(0, len(points) // 2))]
        dx_values = []
        for i in range(1, len(points)):
            dx_values.append(points[i]["cx"] - points[i - 1]["cx"])

        curvature = 0
        for i in range(1, len(dx_values)):
            curvature += abs(dx_values[i] - dx_values[i - 1])

        span_x = 0
        if points:
            xs = [p["cx"] for p in points]
            span_x = max(xs) - min(xs)

        branches = sum(1 for row in rows if len(row["segments"]) >= 2)
        far_reach = far["row"] >= (len(rows) - 3)
        dead_stub = (len(points) < DEAD_STUB_MIN_ROWS) or ((not far_reach) and branches >= 2)
        loop_score = 0
        if len(points) >= 4 and curvature > 55 and span_x > 55:
            loop_score = min(100, curvature + span_x // 2)

        heading = far["cx"] - near["cx"]
        road = ROAD_STRAIGHT
        if branches >= 3:
            road = ROAD_CROSS
        elif branches >= 2:
            road = ROAD_FORK
        elif heading < -24:
            road = ROAD_LEFT_CURVE
        elif heading > 24:
            road = ROAD_RIGHT_CURVE

        return {
            "points": points,
            "rows": len(points),
            "near_x": near["cx"],
            "near_y": near["cy"],
            "lookahead_x": lookahead["cx"],
            "lookahead_y": lookahead["cy"],
            "far_x": far["cx"],
            "far_y": far["cy"],
            "offset": norm_offset(near["cx"]),
            "lookahead_offset": norm_offset(lookahead["cx"]),
            "heading": heading,
            "curvature": curvature,
            "span_x": span_x,
            "branches": branches,
            "dead_stub": dead_stub,
            "loop_score": loop_score,
            "road": road,
            "score": 0,
        }


class ZonePlanner:
    def __init__(self):
        self.zone = ZONE_BOOT_LOCAL
        self.route_step = 0
        self.map_sign = 0
        self.mirror_score = 0
        self.tangent_lock_until = 0
        self.tangent_lock_x = CENTER_X
        self.last_zone = self.zone

    def update_from_telemetry(self, telemetry):
        distance = telemetry["distance_mm"] if telemetry else 0
        if distance < ZONE_BOOT_END_MM:
            self.zone = ZONE_BOOT_LOCAL
        elif distance < ZONE_DEAD_END_END_MM:
            self.zone = ZONE_DEAD_END_FILTER
        elif distance < ZONE_MIRROR_END_MM:
            self.zone = ZONE_MIRROR_DISCOVERY
        elif distance < ZONE_S_END_MM:
            self.zone = ZONE_S_CURVE
        elif distance < ZONE_RECT_END_MM:
            self.zone = ZONE_RECT_ZONE
        elif distance < ZONE_CIRCLE_END_MM:
            self.zone = ZONE_CIRCLE_RECT
        else:
            self.zone = ZONE_FINISH_APPROACH

        if self.zone != self.last_zone:
            self.last_zone = self.zone
            self.route_step = self.zone

    def update_after_path(self, path, now):
        if path is None:
            return

        if self.zone == ZONE_MIRROR_DISCOVERY and self.map_sign == 0:
            # U 型弯里累计曲率和远端偏移方向，锁定镜像手性。
            curve_vote = sign(path["heading"]) * max(1, abs(path["heading"]))
            curve_vote += sign(path["lookahead_offset"]) * max(0, abs(path["lookahead_offset"]) // 2)
            self.mirror_score += curve_vote
            if self.mirror_score >= MIRROR_LOCK_SCORE:
                self.map_sign = 1
            elif self.mirror_score <= -MIRROR_LOCK_SCORE:
                self.map_sign = -1

        if self.zone == ZONE_CIRCLE_RECT:
            if path["loop_score"] > 60 or path["curvature"] > 70:
                self.tangent_lock_until = now + TANGENT_LOCK_MS
                self.tangent_lock_x = path["near_x"]

    def tangent_locked(self, now):
        return now < self.tangent_lock_until

    def speed_limit(self, line):
        if self.zone in (ZONE_BOOT_LOCAL, ZONE_DEAD_END_FILTER, ZONE_MIRROR_DISCOVERY):
            base = LINE_BOOT_SPEED
        elif self.zone in (ZONE_RECT_ZONE, ZONE_CIRCLE_RECT):
            base = LINE_COMPLEX_SPEED
        elif self.zone == ZONE_FINISH_APPROACH:
            base = LINE_FINISH_SPEED
        else:
            base = LINE_LOOP_SPEED

        offset_abs = abs(line["offset"])
        if offset_abs > 14:
            base -= (offset_abs - 14) * 5
        return int(clamp(base, LINE_MIN_SPEED, LINE_SPEED))


class PathScorer:
    def __init__(self):
        self.last_selected_x = CENTER_X
        self.last_heading = 0

    def select(self, paths, zone_planner, now):
        best = None
        best_score = -100000
        for path in paths:
            score = self._score_path(path, zone_planner, now)
            path["score"] = score
            if score > best_score:
                best = path
                best_score = score

        if best is not None:
            self.last_selected_x = best["near_x"]
            self.last_heading = best["heading"]
        return best

    def _score_path(self, path, zone_planner, now):
        score = 0
        score += path["rows"] * 35
        score += max(0, path["points"][-1]["row"] - 2) * 12
        score -= abs(path["offset"]) * 1.5
        score -= abs(path["lookahead_offset"]) * 0.6
        score -= abs(path["near_x"] - self.last_selected_x) * 0.9
        score -= abs(path["heading"] - self.last_heading) * 0.4

        if path["dead_stub"]:
            if zone_planner.zone in (ZONE_DEAD_END_FILTER, ZONE_BOOT_LOCAL, ZONE_MIRROR_DISCOVERY):
                score -= 220
            else:
                score -= 90

        if zone_planner.zone == ZONE_RECT_ZONE:
            # 矩形区允许短时直角，但必须优先远端可延续和历史航向。
            score += path["rows"] * 12
            score -= max(0, path["curvature"] - 90) * 1.2
            if zone_planner.map_sign != 0 and sign(path["far_x"] - CENTER_X) == zone_planner.map_sign:
                score += 18

        if zone_planner.zone == ZONE_CIRCLE_RECT:
            # 连续相切圆会形成闭环，不能只按连续长度选择圆周。
            score -= path["loop_score"] * 2.0
            score -= max(0, path["curvature"] - 80) * 0.8
            if zone_planner.map_sign != 0:
                expected_exit = zone_planner.map_sign
                if sign(path["lookahead_x"] - CENTER_X) == expected_exit:
                    score += 22
                else:
                    score -= 18
            if zone_planner.tangent_locked(now):
                score -= abs(path["near_x"] - zone_planner.tangent_lock_x) * 2.2

        if zone_planner.zone == ZONE_FINISH_APPROACH:
            # 终点前保持保守，优先历史路径和近端稳定。
            score -= max(0, abs(path["heading"]) - 55) * 1.5

        return score


class CommandPlanner:
    def __init__(self):
        self.last_bias = 0
        self.last_valid_bias = 0
        self.lost_frames = 0
        self.search_frames = 0

    def make_line(self, path, paths):
        if path is None:
            return {
                "valid": False,
                "offset": self.last_valid_bias,
                "lookahead_offset": self.last_valid_bias,
                "confidence": 0,
                "road": ROAD_UNKNOWN,
                "path": None,
                "paths": paths,
            }

        confidence = 35 + path["rows"] * 8
        if not path["dead_stub"]:
            confidence += 15
        if path["loop_score"] == 0:
            confidence += 8
        confidence -= min(25, int(abs(path["offset"]) * 0.35))
        confidence = int(clamp(confidence, 0, 100))

        return {
            "valid": True,
            "offset": path["offset"],
            "lookahead_offset": path["lookahead_offset"],
            "confidence": confidence,
            "road": path["road"],
            "path": path,
            "paths": paths,
        }

    def command_from_line(self, line, zone_planner):
        if not line["valid"]:
            self.lost_frames += 1
            if self.lost_frames <= LOST_HOLD_FRAMES:
                return LINE_MIN_SPEED, self.last_valid_bias * LINE_KP, 0
            if self.search_frames < SEARCH_FRAMES:
                self.search_frames += 1
                yaw = SEARCH_YAW if self.last_valid_bias >= 0 else -SEARCH_YAW
                return SEARCH_SPEED, yaw, 0
            return 0, 0, 0

        self.lost_frames = 0
        self.search_frames = 0
        bias = line["offset"]
        preview = line["lookahead_offset"]
        yaw = bias * LINE_KP + preview * LINE_PREVIEW_KP + (bias - self.last_bias) * LINE_KD

        if line["road"] == ROAD_LEFT_CURVE:
            yaw -= 120
        elif line["road"] == ROAD_RIGHT_CURVE:
            yaw += 120

        self.last_bias = bias
        self.last_valid_bias = bias
        return zone_planner.speed_limit(line), int(clamp(yaw, -LINE_MAX_YAW, LINE_MAX_YAW)), CMD_FLAG_LINE_VALID


class GateDetector:
    def __init__(self):
        self.state = "clear"
        self.state_since = 0
        self.lock_since = 0
        self.lock_distance = 0
        self.last_distance_cm = 0
        self.count = 0

    def _pair_distance_cm(self, telemetry):
        if telemetry is None:
            return None
        targets = telemetry["targets"]
        for left in targets:
            if not left["valid"] or left["x_mm"] >= 0:
                continue
            if abs(left["x_mm"]) < GATE_POST_MIN_ABS_X_MM:
                continue
            if left["y_mm"] < GATE_MIN_CM * 10 or left["y_mm"] > GATE_MAX_CM * 10:
                continue
            for right in targets:
                width = right["x_mm"] - left["x_mm"]
                if not right["valid"] or right["x_mm"] <= 0:
                    continue
                if abs(right["x_mm"]) < GATE_POST_MIN_ABS_X_MM:
                    continue
                if right["y_mm"] < GATE_MIN_CM * 10 or right["y_mm"] > GATE_MAX_CM * 10:
                    continue
                if abs(left["y_mm"] - right["y_mm"]) > GATE_PAIR_Y_TOL_MM:
                    continue
                if width < GATE_WIDTH_MIN_MM or width > GATE_WIDTH_MAX_MM:
                    continue
                return (left["y_mm"] + right["y_mm"]) // 20
        return None

    def update(self, telemetry, now):
        distance_cm = self._pair_distance_cm(telemetry)
        active = distance_cm is not None
        event = 0
        distance_mm = telemetry["distance_mm"] if telemetry else 0

        if self.state == "clear":
            if active:
                self.state = "candidate"
                self.state_since = now
        elif self.state == "candidate":
            if not active:
                self.state = "clear"
            elif now - self.state_since >= GATE_ENTER_STABLE_MS:
                self.state = "under"
        elif self.state == "under":
            exiting = not active
            if active and distance_cm > self.last_distance_cm:
                exiting = (distance_cm - self.last_distance_cm) >= GATE_EXIT_JUMP_CM
            if exiting:
                self.state = "exiting"
                self.state_since = now
        elif self.state == "exiting":
            if active:
                self.state = "under"
            elif now - self.state_since >= GATE_EXIT_STABLE_MS:
                self.count += 1
                event = self.count
                self.state = "locked"
                self.lock_since = now
                self.lock_distance = distance_mm
        elif self.state == "locked":
            if now - self.lock_since >= GATE_LOCK_MS and distance_mm - self.lock_distance >= GATE_LOCK_DISTANCE_MM:
                self.state = "clear"

        if active:
            self.last_distance_cm = distance_cm
        return event


class ObstaclePlanner:
    def __init__(self):
        self.active = False
        self.side = 1
        self.phase = 0
        self.phase_start = 0

    def start(self, telemetry, now):
        left_score = 0
        right_score = 0
        if telemetry:
            for target in telemetry["targets"]:
                if not target["valid"]:
                    continue
                if target["y_mm"] < 300 or target["y_mm"] > 1800:
                    continue
                if target["x_mm"] < -120:
                    left_score += 1
                elif target["x_mm"] > 120:
                    right_score += 1
        # 雷达只做辅助，默认向目标较少的一侧轻微绕行。
        self.side = 1 if left_score > right_score else -1
        self.active = True
        self.phase = 0
        self.phase_start = now

    def command(self, now):
        if not self.active:
            return None
        elapsed = now - self.phase_start
        turn = AVOID_YAW * self.side
        if self.phase == 0:
            if elapsed >= AVOID_ENTRY_MS:
                self.phase = 1
                self.phase_start = now
            return AVOID_SPEED, turn
        if self.phase == 1:
            if elapsed >= AVOID_PASS_MS:
                self.phase = 2
                self.phase_start = now
            return AVOID_SPEED, 0
        if elapsed >= AVOID_RECOVER_MS:
            self.active = False
            return None
        return AVOID_SPEED, -turn // 2


class SimpleDebugView:
    def __init__(self):
        self.disp = display.Display()

    def show(self, img, rows, line, zone_planner, mode, telemetry):
        try:
            zone = ZONE_NAMES[zone_planner.zone]
            path_count = len(line.get("paths", []))
            state = "M{} {} R{} S{} P{} C{}".format(mode, zone, zone_planner.route_step,
                                                    zone_planner.map_sign, path_count,
                                                    line["confidence"])
            img.draw_string(2, 2, state, image.COLOR_GREEN)
            if telemetry:
                img.draw_string(2, 18, "D{} B{}%".format(telemetry["distance_mm"],
                                                          telemetry["battery_percent"]),
                                image.COLOR_BLUE)
            for row in rows:
                for seg in row["segments"]:
                    try:
                        img.draw_rectangle([seg["x"], seg["y"], seg["w"], seg["h"]], image.COLOR_RED)
                    except Exception:
                        pass
            path = line.get("path")
            if path:
                points = path["points"]
                for i in range(1, len(points)):
                    p0 = points[i - 1]
                    p1 = points[i]
                    try:
                        img.draw_line(p0["cx"], p0["cy"], p1["cx"], p1["cy"], image.COLOR_GREEN)
                    except Exception:
                        pass
            if zone_planner.tangent_locked(ticks_ms()):
                img.draw_string(2, 34, "TANGENT", image.COLOR_RED)
            self.disp.show(img)
        except Exception:
            pass


def main():
    cam = CameraAcq()
    link = UartLink()
    extractor = ScanlineExtractor()
    graph = PathGraph()
    zone = ZonePlanner()
    scorer = PathScorer()
    command_planner = CommandPlanner()
    gate = GateDetector()
    obstacle = ObstaclePlanner()
    debug = SimpleDebugView()
    pending_checkpoint = 0
    checkpoint_status_armed = False

    while not app.need_exit():
        telemetry = link.poll()
        img = cam.read()
        now = ticks_ms()

        if telemetry is None or not link.started():
            link.send_command(MODE_IDLE, 0, 0, 0, zone.route_step, 0, 0)
            empty_line = {"confidence": 0, "path": None}
            debug.show(img, [], empty_line, zone, MODE_IDLE, telemetry)
            time.sleep_ms(5)
            continue

        zone.update_from_telemetry(telemetry)
        rows = extractor.extract(img)
        paths = graph.build(rows)
        selected = scorer.select(paths, zone, now)
        line = command_planner.make_line(selected, paths)
        zone.update_after_path(selected, now)

        gate_event = gate.update(telemetry, now)
        if gate_event in (1, 2):
            pending_checkpoint = gate_event
            checkpoint_status_armed = False
        elif gate_event == 3:
            obstacle.start(telemetry, now)

        checkpoint = pending_checkpoint
        if pending_checkpoint != 0:
            if telemetry["lora_status"] != LORA_SENT:
                checkpoint_status_armed = True
            elif checkpoint_status_armed:
                pending_checkpoint = 0
                checkpoint_status_armed = False
                checkpoint = 0

        if zone.zone == ZONE_FINISH_APPROACH and extractor.finish_detected(img):
            flags = CMD_FLAG_FINISH
            link.send_command(MODE_FINISH, 0, 0, flags, zone.route_step, 0, 100)
            debug.show(img, rows, line, zone, MODE_FINISH, telemetry)
            time.sleep_ms(5)
            continue

        avoid_cmd = obstacle.command(now)
        if avoid_cmd and zone.zone == ZONE_CIRCLE_RECT:
            vx, yaw = avoid_cmd
            flags = CMD_FLAG_AVOIDING
            confidence = max(40, line["confidence"])
        else:
            vx, yaw, flags = command_planner.command_from_line(line, zone)
            confidence = line["confidence"]

        link.send_command(MODE_RUN, vx, yaw, flags, zone.route_step, checkpoint, confidence)
        debug.show(img, rows, line, zone, MODE_RUN, telemetry)
        time.sleep_ms(5)


if __name__ == "__main__":
    main()
