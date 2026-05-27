from maix import app, camera, display, image, pinmap, time, uart


UART_DEVICE = "/dev/ttyS1"
UART_BAUD = 115200
UART_RX_PIN = "A18"
UART_TX_PIN = "A19"

IMG_W = 320
IMG_H = 240
CENTER_X = IMG_W // 2

ROI_NEAR = [0, 176, IMG_W, 54]
ROI_MID = [0, 116, IMG_W, 54]
ROI_FAR = [0, 58, IMG_W, 54]
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
LORA_SENDING = 1
LORA_SENT = 2
LORA_ERROR = 3

PREF_NEAREST = 0
PREF_LEFT = 1
PREF_RIGHT = 2

ROAD_UNKNOWN = 0
ROAD_STRAIGHT = 1
ROAD_LEFT_CURVE = 2
ROAD_RIGHT_CURVE = 3
ROAD_FORK = 4
ROAD_CROSS = 5
ROAD_T_LEFT = 6
ROAD_T_RIGHT = 7
ROAD_FINISH = 8

LINE_SPEED = 320
LINE_MIN_SPEED = 180
LINE_LOOP_SPEED = 190
LINE_EXIT_SPEED = 180
LINE_MAX_YAW = 1800
LINE_KP = 14
LINE_KD = 5
ROUTE_LOCK_MS = 700
ROUTE_CONFIRM_FRAMES = 2
ROUTE_MIN_CONFIDENCE = 70

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

AVOID_SPEED = 350
AVOID_YAW = 1600
AVOID_ENTRY_MS = 700
AVOID_PASS_MS = 900
AVOID_RECOVER_MS = 650


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


def blob_cx(blob):
    try:
        return blob.cx()
    except Exception:
        return blob.x() + blob.w() // 2


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
        self.pending_checkpoint = 0

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


class LineDetector:
    def __init__(self):
        self.last_offset = 0

    def _find_blobs(self, img, roi):
        try:
            return img.find_blobs([BLACK_LINE_LAB], roi=roi, pixels_threshold=25,
                                  area_threshold=35, merge=True, margin=8)
        except Exception:
            return img.find_blobs(thresholds=[BLACK_LINE_LAB], roi=roi,
                                  pixels_threshold=25, area_threshold=35,
                                  merge=True, margin=8)

    def _select_blob(self, blobs, preference):
        if not blobs:
            return None, 0
        ordered = sorted(blobs, key=lambda b: blob_cx(b))
        if preference == PREF_LEFT:
            return ordered[0], 0
        if preference == PREF_RIGHT:
            return ordered[-1], len(ordered) - 1
        selected = min(ordered, key=lambda b: abs(blob_cx(b) - CENTER_X))
        return selected, ordered.index(selected)

    def detect(self, img, preference):
        near = self._find_blobs(img, ROI_NEAR)
        mid = self._find_blobs(img, ROI_MID)
        far = self._find_blobs(img, ROI_FAR)
        selected, selected_index = self._select_blob(near, preference)

        if selected is None:
            return {
                "valid": False,
                "offset": self.last_offset,
                "confidence": 0,
                "segments": len(near),
                "selected": 0,
                "road": ROAD_UNKNOWN,
            }

        center = blob_cx(selected)
        offset = int(clamp(((center - CENTER_X) * 127) // IMG_W, -64, 63))
        self.last_offset = offset

        confidence = 45
        if selected.pixels() > 60:
            confidence += 15
        if selected.h() > 10:
            confidence += 10
        if mid:
            confidence += 15
        if far:
            confidence += 15
        confidence = int(clamp(confidence, 0, 100))

        road = ROAD_STRAIGHT
        if len(near) >= 2:
            left_seen = any(blob_cx(b) < CENTER_X - 35 for b in near)
            right_seen = any(blob_cx(b) > CENTER_X + 35 for b in near)
            if left_seen and right_seen:
                road = ROAD_FORK
            elif left_seen:
                road = ROAD_T_LEFT
            elif right_seen:
                road = ROAD_T_RIGHT
        elif mid or far:
            samples = []
            if mid:
                samples.append(blob_cx(max(mid, key=lambda b: b.pixels())))
            if far:
                samples.append(blob_cx(max(far, key=lambda b: b.pixels())))
            if samples:
                dx = (sum(samples) // len(samples)) - center
                if dx < -18:
                    road = ROAD_LEFT_CURVE
                elif dx > 18:
                    road = ROAD_RIGHT_CURVE

        return {
            "valid": True,
            "offset": offset,
            "confidence": confidence,
            "segments": len(near),
            "selected": selected_index,
            "road": road,
        }

    def finish_detected(self, img):
        try:
            blobs = img.find_blobs([RED_FINISH_LAB], roi=FINISH_ROI, pixels_threshold=300,
                                   area_threshold=450, merge=True, margin=6)
        except Exception:
            blobs = img.find_blobs(thresholds=[RED_FINISH_LAB], roi=FINISH_ROI,
                                   pixels_threshold=300, area_threshold=450,
                                   merge=True, margin=6)
        return bool(blobs)


class RoutePlanner:
    def __init__(self):
        self.route_step = 0
        self.route_event_frames = 0
        self.lock_until_ms = 0
        self.last_bias = 0
        self.search_frames = 0
        self.lost_frames = 0
        self.route_steps = [
            (PREF_LEFT, LINE_EXIT_SPEED),
            (PREF_RIGHT, LINE_EXIT_SPEED),
            (PREF_NEAREST, LINE_LOOP_SPEED),
            (PREF_NEAREST, LINE_LOOP_SPEED),
            (PREF_NEAREST, LINE_MIN_SPEED),
        ]

    def preference(self):
        if self.route_step < len(self.route_steps):
            return self.route_steps[self.route_step][0]
        return PREF_NEAREST

    def _speed_limit(self, line):
        if self.route_step < len(self.route_steps):
            limit = self.route_steps[self.route_step][1]
        else:
            limit = LINE_MIN_SPEED
        if line["road"] in (ROAD_LEFT_CURVE, ROAD_RIGHT_CURVE, ROAD_FORK, ROAD_CROSS):
            limit = min(limit, LINE_LOOP_SPEED)
        offset_abs = abs(line["offset"])
        if offset_abs > 14:
            limit -= (offset_abs - 14) * 6
        return int(clamp(limit, LINE_MIN_SPEED, LINE_SPEED))

    def update_route(self, line, now):
        route_event = (line["valid"] and
                       line["confidence"] >= ROUTE_MIN_CONFIDENCE and
                       (line["segments"] >= 2 or line["road"] in
                        (ROAD_FORK, ROAD_CROSS, ROAD_T_LEFT, ROAD_T_RIGHT)))
        if route_event:
            self.route_event_frames += 1
        else:
            self.route_event_frames = 0

        if now < self.lock_until_ms:
            return

        if self.route_event_frames >= ROUTE_CONFIRM_FRAMES and self.route_step < len(self.route_steps):
            self.route_step += 1
            self.route_event_frames = 0
            self.lock_until_ms = now + ROUTE_LOCK_MS

    def command_from_line(self, line):
        if not line["valid"]:
            self.lost_frames += 1
            if self.lost_frames <= LOST_HOLD_FRAMES:
                return LINE_MIN_SPEED, self.last_bias * LINE_KP, 0
            if self.search_frames < SEARCH_FRAMES:
                self.search_frames += 1
                yaw = SEARCH_YAW if self.last_bias >= 0 else -SEARCH_YAW
                return SEARCH_SPEED, yaw, 0
            return 0, 0, 0

        self.lost_frames = 0
        self.search_frames = 0
        bias = line["offset"]
        yaw = bias * LINE_KP + (bias - self.last_bias) * LINE_KD
        if line["road"] == ROAD_LEFT_CURVE:
            yaw -= 150
        elif line["road"] == ROAD_RIGHT_CURVE:
            yaw += 150
        self.last_bias = bias
        return self._speed_limit(line), int(clamp(yaw, -LINE_MAX_YAW, LINE_MAX_YAW)), CMD_FLAG_LINE_VALID


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
        self.side = PREF_LEFT
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
        self.side = PREF_RIGHT if left_score > right_score else PREF_LEFT
        self.active = True
        self.phase = 0
        self.phase_start = now

    def command(self, now):
        if not self.active:
            return None
        elapsed = now - self.phase_start
        turn = AVOID_YAW if self.side == PREF_LEFT else -AVOID_YAW
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


class DebugView:
    def __init__(self):
        self.disp = display.Display()

    def show(self, img, line, route_step, mode, telemetry):
        try:
            state = "M{} R{} C{}".format(mode, route_step, line["confidence"])
            img.draw_string(2, 2, state, image.COLOR_GREEN)
            if telemetry:
                img.draw_string(2, 18, "D{} B{}%".format(telemetry["distance_mm"],
                                                          telemetry["battery_percent"]),
                                image.COLOR_BLUE)
            self.disp.show(img)
        except Exception:
            pass


def main():
    cam = CameraAcq()
    link = UartLink()
    detector = LineDetector()
    route = RoutePlanner()
    gate = GateDetector()
    obstacle = ObstaclePlanner()
    debug = DebugView()
    pending_checkpoint = 0
    checkpoint_status_armed = False

    while not app.need_exit():
        telemetry = link.poll()
        img = cam.read()
        now = ticks_ms()

        if telemetry is None or not link.started():
            link.send_command(MODE_IDLE, 0, 0, 0, route.route_step, 0, 0)
            debug.show(img, {"confidence": 0}, route.route_step, MODE_IDLE, telemetry)
            time.sleep_ms(5)
            continue

        line = detector.detect(img, route.preference())
        route.update_route(line, now)
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

        if route.route_step >= len(route.route_steps) and detector.finish_detected(img):
            flags = CMD_FLAG_FINISH
            link.send_command(MODE_FINISH, 0, 0, flags, route.route_step, 0, 100)
            debug.show(img, line, route.route_step, MODE_FINISH, telemetry)
            time.sleep_ms(5)
            continue

        avoid_cmd = obstacle.command(now)
        if avoid_cmd:
            vx, yaw = avoid_cmd
            flags = CMD_FLAG_AVOIDING
        else:
            vx, yaw, flags = route.command_from_line(line)

        link.send_command(MODE_RUN, vx, yaw, flags, route.route_step, checkpoint, line["confidence"])
        debug.show(img, line, route.route_step, MODE_RUN, telemetry)
        time.sleep_ms(5)


if __name__ == "__main__":
    main()
