import gc
import math
import os
import time

import cv_lite
from machine import FPIOA, UART
from media.display import *
from media.media import *
from media.sensor import *


# ============================================================
# A4 rectangle tracking + bounded automatic PID tuning
#
# Wireless commands:
#   PING
#   STATUS
#   PARAMS,x_kp,x_ki,x_kd,x_polarity,y_kp,y_ki,y_kd,y_polarity
#   PARAMS,x_kp,x_ki,x_kd,x_polarity,y_kp,y_ki,y_kd,y_polarity,ff_gain,ff_max,integral_limit,large_i_scale
#   AUTOTUNE
#   STOP
#   TRACK ON
#   TRACK OFF
#   RELEASE
#   LOCK
#
# Safety:
#   - tuning starts when the target is stably visible, then self-centers
#   - one axis is tuned at a time
#   - motor speed is capped
#   - each motor has a 500 ms hardware heartbeat timeout
#   - target loss, STOP, communication loss, or any exception stops motors
# ============================================================

IMAGE_WIDTH = 480
IMAGE_HEIGHT = 320
IMAGE_CENTER_X = IMAGE_WIDTH // 2
IMAGE_CENTER_Y = IMAGE_HEIGHT // 2
IMAGE_SHAPE = [IMAGE_HEIGHT, IMAGE_WIDTH]
SENSOR_ID = 2

CANNY_THRESHOLD_1 = 32
CANNY_THRESHOLD_2 = 105
APPROX_EPSILON = 0.04
MIN_AREA_RATIO = 0.001
MAX_ANGLE_COSINE = 0.90
GAUSSIAN_BLUR = 3
MIN_RECTANGLE_AREA = 1200
MIN_EDGE_LENGTH = 18
MAX_EDGE_LENGTH_RATIO = 5.5
MIN_QUAD_FILL_RATIO = 0.32
DETECTION_HOLD_FRAMES = 3
DETECTION_HOLD_MAX_RPM = 4.0
INITIAL_LOCK_CONFIRM_FRAMES = 2
TRACK_ASSOCIATION_LIMIT = 1.35
TRACK_MAX_CENTER_JUMP_PIXELS = 58.0
TRACK_MAX_AXIS_JUMP_PIXELS = 50.0
TRACK_REACQUIRE_ASSOCIATION_LIMIT = 3.20
TRACK_REACQUIRE_MAX_CENTER_JUMP_PIXELS = 220.0
TRACK_REACQUIRE_MAX_AXIS_JUMP_PIXELS = 200.0
TRACK_PREDICT_ALPHA = 0.45
TRACK_MAX_PREDICT_PIXELS_PER_FRAME = 26.0
TRACK_SUSPICIOUS_JUMP_PIXELS = 34.0
TRACK_AREA_RATIO_MIN = 0.72
TRACK_AREA_RATIO_MAX = 1.38
BLACK_PIXEL_MAX = 160
BLACK_CHANNEL_SPREAD_MAX = 65
MIN_BLACK_EDGE_SAMPLES = 7
MIN_BLACK_EDGES_WITH_HIT = 3
INITIAL_LOCK_MIN_BLACK_EDGE_SAMPLES = 8
INITIAL_LOCK_MIN_BLACK_EDGES_WITH_HIT = 4
MAX_RAW_QUADRILATERALS = 10
MAX_VALID_QUADRILATERALS = 4

ESP_TX_PIN = 32  # 40Pin pin 37, UART3_TX
ESP_RX_PIN = 33  # 40Pin pin 40, UART3_RX

X_TX_PIN = 3     # 40Pin pin 8, UART1_TX
X_RX_PIN = 4     # 40Pin pin 10, UART1_RX
Y_TX_PIN = 5     # 40Pin pin 11, UART2_TX
Y_RX_PIN = 6     # 40Pin pin 13, UART2_RX

BAUD_RATE = 115200
MOTOR_ADDRESS = 1
MOTOR_FIRMWARE_MODE = "EMM"
MOTOR_LINK_MODE = "TTL"
MOTOR_ACCELERATION = 220
MOTOR_X_ACCELERATION_RPM_S = 1800
MOTOR_CLOSED_LOOP_MAX_CURRENT_MA = 4200
MOTOR_HEARTBEAT_TIMEOUT_MS = 500
MOTOR_KEEPALIVE_PERIOD_MS = 150

# This gimbal uses a geared, higher-load mechanism.  The previous limits were
# tuned for a light direct-drive frame and could not reliably overcome static
# friction after a direction probe or while returning to centre.
TUNE_MAX_RPM = 45.0
RECENTER_MAX_RPM = 45.0
X_TRACK_MAX_RPM = 60.0
Y_TUNE_MAX_RPM = 40.0
Y_RECENTER_MAX_RPM = 40.0
Y_TRACK_MAX_RPM = 50.0
DEADBAND_PIXELS = 3
MIN_ACTIVE_ERROR_PIXELS = 5
RECENTER_ACCEPT_PIXELS = 4
DISTANCE_AREA_REFERENCE = 12000.0
MAX_DISTANCE_GAIN = 1.8
LARGE_ERROR_START_PIXELS = 18
LARGE_ERROR_FULL_PIXELS = 70
MAX_LARGE_ERROR_GAIN = 2.2
ERROR_FILTER_ALPHA = 0.65
DERIVATIVE_LIMIT_PX_PER_SECOND = 240.0
MAX_D_TO_P_RATIO = 1.00
MIN_ACTIVE_RPM = 2.4
DEFAULT_TRACK_KI = 0.25
MIN_SUSTAINED_TRACK_KI = 0.08
TRACK_INTEGRAL_UNWIND_MULTIPLIER = 3.0
TRACK_INTEGRAL_ZONE_PIXELS = 36
DEFAULT_TRACK_LARGE_ERROR_INTEGRAL_SCALE = 0.15
DEFAULT_MAX_TRACK_INTEGRAL_RPM = 16.0
TRACK_DRIVE_ACTIVE_RPM = 0.45
DEFAULT_TRACK_VELOCITY_FF_GAIN = 0.045
TRACK_VELOCITY_FF_MIN_PX_PER_SECOND = 8.0
DEFAULT_MAX_TRACK_VELOCITY_FF_RPM = 8.0
TRACK_FAST_FF_DERIVATIVE_PX_PER_SECOND = 45.0
TRACK_FAST_FF_ERROR_PIXELS = 8.0
TRACK_FAST_FF_GAIN_MULTIPLIER = 1.55
TRACK_FAST_FF_MIN_MAX_RPM = 9.0
CENTER_SETTLE_FRAMES = 4
CENTER_SETTLE_DERIVATIVE_PX_PER_SECOND = 6.0
CENTER_RESIDUAL_DRIVE_RPM = 1.2
GEAR_REVERSAL_BOOST_FRAMES = 3
GEAR_REVERSAL_MAX_RPM = 14.0

# Keep the probe deliberately slow (better torque margin) but long enough for
# the geared mechanism to take up backlash and visibly move the target.
X_PROBE_RPM_LEVELS = (10, 20, 35)
Y_PROBE_RPM_LEVELS = (10, 20, 30, 40)
PROBE_DURATION_MS = 500
Y_PROBE_DURATION_MS = 500
PROBE_MIN_PIXEL_CHANGE = 3

KICK_TARGET_PIXELS = 90
KICK_MIN_DEGREES = 10.0
KICK_MAX_DEGREES = 14.0
PID_CANDIDATE_DURATION_MS = 6600
PID_SCORE_PROFILE = "PID_GEARED_SYSID_V16"

PID_COARSE_KP = (0.14, 0.36, 0.68)
PID_COARSE_KI = (0.12, 0.40)
PID_COARSE_KD = (0.000, 0.025, 0.060)
PID_PARAMETER_EPSILON = 0.0005
PID_ROBUST_WORST_WEIGHT = 0.35
PID_REFERENCE_EDGE_MARGIN_PIXELS = 12
PID_ABORT_EDGE_MARGIN_PIXELS = 4
PID_MAX_STEP_PIXELS = {"X": 60.0, "Y": 36.0}
PID_EARLY_ABORT_GRACE_MS = 850
PID_SATURATION_ABORT_MS = 650
PID_RUNAWAY_ERROR_ABORT_MS = 450
PID_SATURATION_ERROR_RATIO = 0.55
PID_RUNAWAY_ERROR_RATIO = 1.65
MOTOR_NO_MOTION_MIN_RPM = 8.0
MOTOR_NO_MOTION_ABORT_MS = 800
MOTOR_NO_MOTION_PIXEL_DELTA = 2.0

FOLLOW_PROFILE_CANDIDATES = (
    # ff_gain, ff_max_rpm, integral_limit_rpm, large_error_i_scale
    (0.035, 6.0, 14.0, 0.14),
    (0.045, 8.0, 16.0, 0.18),
    (0.055, 10.0, 18.0, 0.22),
    (0.065, 12.0, 20.0, 0.26),
)
FOLLOW_PARAMETER_EPSILON = 0.0005
PID_POLISH_REPEAT_COUNT = 2
FOLLOW_POLISH_REPEAT_COUNT = 2

AUTOTUNE_TARGET_FRAMES = 4
TELEMETRY_PERIOD_MS = 500
TUNE_TELEMETRY_PERIOD_MS = 100
RECENTER_TIMEOUT_MS = 6000
DETECTOR_VERSION = "PERSPECTIVE_QUAD_V2"
CONTROL_PROFILE = "GIMBAL_PID_V16"


class TuneAbort(Exception):
    pass


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def create_uart(uart_id, tx_pin, rx_pin, tx_function, rx_function, fpioa=None):
    if fpioa is None:
        fpioa = FPIOA()
    fpioa.set_function(tx_pin, tx_function)
    fpioa.set_function(rx_pin, rx_function)
    return UART(
        uart_id,
        baudrate=BAUD_RATE,
        bits=UART.EIGHTBITS,
        parity=UART.PARITY_NONE,
        stop=UART.STOPBITS_ONE,
    )


class ZDTMotor:
    def __init__(self, label, uart):
        self.label = label
        self.uart = uart
        self.last_direction = None
        self.last_speed = None
        self.last_command_ms = time.ticks_add(time.ticks_ms(), -1000)
        self.speed_scale = 1
        self.speed_scale_verified = False
        self.firmware = "UNKNOWN"
        self.firmware_verified = False
        self.protocol_motion_verified = False
        self.option_flags = None
        self.turbo_enabled = False
        self.runtime_configured = False

    def write(self, data):
        # Replies are not used by this real-time controller. Drain old ACKs
        # so they cannot accumulate in the small UART receive buffer.
        self.uart.read()
        return self.uart.write(bytes(data))

    def configure_setting(self, command, function_code, label):
        expected = bytes(
            (MOTOR_ADDRESS, function_code, 0x02, 0x6B)
        )
        all_replies = b""
        for _ in range(3):
            self.write(command)
            deadline = time.ticks_add(time.ticks_ms(), 100)
            reply = b""
            while time.ticks_diff(deadline, time.ticks_ms()) > 0:
                chunk = self.uart.read()
                if chunk:
                    reply += chunk
                    all_replies += chunk
                    if expected in reply:
                        print("%s motor %s configured" % (self.label, label))
                        return True
                time.sleep_ms(3)
        print(
            "WARNING,%s %s command sent, ACK not received,reply=%r"
            % (self.label, label, all_replies)
        )
        return False

    def configure_heavy_load_mode(self):
        if self.runtime_configured:
            return True
        # The installed motors are confirmed Emm firmware over TTL. Force one
        # deterministic packet format and explicitly disable 0.1RPM scaling,
        # so a command value of 20 always means exactly 20RPM even without RX.
        self.firmware = MOTOR_FIRMWARE_MODE
        # The firmware type is user-confirmed.  Keep "verified" reserved for
        # an actual UART reply; otherwise the host log can falsely imply that
        # the motor link has been proven healthy.
        self.option_flags = self.read_option_flags()
        self.firmware_verified = self.option_flags is not None
        scale_ok = self.configure_setting(
            (
                MOTOR_ADDRESS,
                0x4F,
                0x71,
                0x00,
                0x00,
                0x6B,
            ),
            0x4F,
            "integer RPM mode",
        )
        self.speed_scale = 1
        self.speed_scale_verified = scale_ok
        current_ma = int(MOTOR_CLOSED_LOOP_MAX_CURRENT_MA)
        current_ok = self.configure_setting(
            (
                MOTOR_ADDRESS,
                0x45,
                0x66,
                0x00,
                (current_ma >> 8) & 0xFF,
                current_ma & 0xFF,
                0x6B,
            ),
            0x45,
            "closed-loop current %dmA" % current_ma,
        )
        self.runtime_configured = True
        return scale_ok and current_ok

    def read_option_flags(self):
        # Read commands return even when control-command acknowledgements are
        # disabled. bit1 selects X/Emm firmware; bit7 is Emm 0.1RPM input.
        for _ in range(3):
            self.write((MOTOR_ADDRESS, 0x1A, 0x6B))
            deadline = time.ticks_add(time.ticks_ms(), 120)
            reply = b""
            while time.ticks_diff(deadline, time.ticks_ms()) > 0:
                chunk = self.uart.read()
                if chunk:
                    reply += chunk
                    for index in range(max(0, len(reply) - 8), len(reply) - 3):
                        if (
                            reply[index] == MOTOR_ADDRESS
                            and reply[index + 1] == 0x1A
                            and reply[index + 3] == 0x6B
                        ):
                            return reply[index + 2]
                time.sleep_ms(3)
        return None

    def read_reply(self, function_code, reply_length, timeout_ms=120):
        for _ in range(3):
            self.write((MOTOR_ADDRESS, function_code, 0x6B))
            deadline = time.ticks_add(time.ticks_ms(), timeout_ms)
            reply = b""
            while time.ticks_diff(deadline, time.ticks_ms()) > 0:
                chunk = self.uart.read()
                if chunk:
                    reply += chunk
                    start_min = max(0, len(reply) - reply_length - 8)
                    start_max = len(reply) - reply_length + 1
                    for index in range(start_min, start_max):
                        packet = reply[index:index + reply_length]
                        if (
                            len(packet) == reply_length
                            and packet[0] == MOTOR_ADDRESS
                            and packet[1] == function_code
                            and packet[-1] == 0x6B
                        ):
                            return packet
                time.sleep_ms(3)
        return None

    def read_status_flags(self):
        reply = self.read_reply(0x3A, 4)
        return None if reply is None else reply[2]

    def read_phase_current_ma(self):
        reply = self.read_reply(0x27, 5)
        if reply is None:
            return None
        return (reply[2] << 8) | reply[3]

    def read_realtime_speed_rpm(self):
        reply = self.read_reply(0x35, 6)
        if reply is None:
            return None
        magnitude = (reply[3] << 8) | reply[4]
        if self.firmware == "X":
            magnitude /= 10.0
        return -magnitude if reply[2] else magnitude

    def read_position_degrees(self):
        reply = self.read_reply(0x36, 8)
        if reply is None:
            return None
        raw = (
            (reply[3] << 24)
            | (reply[4] << 16)
            | (reply[5] << 8)
            | reply[6]
        )
        if self.firmware == "X":
            degrees = raw / 10.0
        else:
            degrees = raw * 360.0 / 65536.0
        return -degrees if reply[2] else degrees

    def clear_protection(self):
        ok = self.configure_setting(
            (MOTOR_ADDRESS, 0x0E, 0x52, 0x6B),
            0x0E,
            "clear protection",
        )
        self.enable()
        return ok

    def enable_emm_turbo(self):
        if self.firmware != "EMM" or self.turbo_enabled:
            return self.turbo_enabled
        ok = self.configure_setting(
            (
                MOTOR_ADDRESS,
                0xD5,
                0x69,
                0x00,  # runtime only
                0x02,  # Emm turbo firmware mode
                0x6B,
            ),
            0xD5,
            "Emm turbo",
        )
        if ok:
            self.turbo_enabled = True
            self.enable()
        return ok

    def configure_heartbeat(self, timeout_ms=MOTOR_HEARTBEAT_TIMEOUT_MS):
        timeout_ms = int(timeout_ms)
        command = (
            MOTOR_ADDRESS,
            0x68,
            0x38,
            0x00,  # runtime only; do not store permanently
            (timeout_ms >> 24) & 0xFF,
            (timeout_ms >> 16) & 0xFF,
            (timeout_ms >> 8) & 0xFF,
            timeout_ms & 0xFF,
            0x6B,
        )
        expected = bytes((MOTOR_ADDRESS, 0x68, 0x02, 0x6B))
        all_replies = b""
        for _ in range(3):
            self.write(command)
            deadline = time.ticks_add(time.ticks_ms(), 100)
            reply = b""
            while time.ticks_diff(deadline, time.ticks_ms()) > 0:
                chunk = self.uart.read()
                if chunk:
                    reply += chunk
                    all_replies += chunk
                    if expected in reply:
                        print(
                            "%s motor heartbeat: %d ms"
                            % (self.label, timeout_ms)
                        )
                        return True
                time.sleep_ms(3)
        # Motor setting "control command response" may be disabled. In that
        # case the configuration can take effect without an ACK, so lack of
        # reply must not prevent the vision program from starting.
        print(
            "WARNING,%s heartbeat command sent, ACK not received,reply=%r"
            % (self.label, all_replies)
        )
        return False

    def enable(self):
        self.write((MOTOR_ADDRESS, 0xF3, 0xAB, 0x01, 0x00, 0x6B))
        time.sleep_ms(30)
        self.uart.read()

    def disable(self):
        self.stop(True)
        time.sleep_ms(20)
        self.write((MOTOR_ADDRESS, 0xF3, 0xAB, 0x00, 0x00, 0x6B))
        self.last_direction = None
        self.last_speed = 0

    def stop(self, force=False):
        if force or self.last_speed != 0:
            self.write((MOTOR_ADDRESS, 0xFE, 0x98, 0x00, 0x6B))
        self.last_direction = None
        self.last_speed = 0
        self.last_command_ms = time.ticks_ms()

    def smooth_stop(self):
        if self.last_speed == 0:
            return
        direction = self.last_direction if self.last_direction is not None else 0
        if self.firmware == "X":
            acceleration = MOTOR_X_ACCELERATION_RPM_S
            command = (
                MOTOR_ADDRESS,
                0xF6,
                direction,
                (acceleration >> 8) & 0xFF,
                acceleration & 0xFF,
                0x00,
                0x00,
                0x00,
                0x6B,
            )
        else:
            command = (
                MOTOR_ADDRESS,
                0xF6,
                direction,
                0x00,
                0x00,
                MOTOR_ACCELERATION,
                0x00,
                0x6B,
            )
        self.write(command)
        self.last_direction = None
        self.last_speed = 0
        self.last_command_ms = time.ticks_ms()

    def set_physical_speed(self, signed_rpm, maximum_rpm):
        signed_rpm = clamp(signed_rpm, -maximum_rpm, maximum_rpm)
        magnitude = int(abs(signed_rpm) * self.speed_scale + 0.5)
        if magnitude <= 0:
            self.smooth_stop()
            return

        direction = 0 if signed_rpm > 0 else 1
        now_ms = time.ticks_ms()

        if self.firmware == "X":
            acceleration = MOTOR_X_ACCELERATION_RPM_S
            command = (
                MOTOR_ADDRESS,
                0xF6,
                direction,
                (acceleration >> 8) & 0xFF,
                acceleration & 0xFF,
                (magnitude >> 8) & 0xFF,
                magnitude & 0xFF,
                0x00,
                0x6B,
            )
        else:
            command = (
                MOTOR_ADDRESS,
                0xF6,
                direction,
                (magnitude >> 8) & 0xFF,
                magnitude & 0xFF,
                MOTOR_ACCELERATION,
                0x00,
                0x6B,
            )
        self.write(command)
        self.last_direction = direction
        self.last_speed = magnitude / float(self.speed_scale)
        self.last_command_ms = now_ms

    def keepalive(self):
        # Reading position is a valid, non-motion command. It feeds the
        # motor's own heartbeat without changing the current speed.
        self.write((MOTOR_ADDRESS, 0x36, 0x6B))


def line_intersection(x1, y1, x2, y2, x3, y3, x4, y4):
    determinant = (
        (x2 - x1) * (y4 - y3)
        - (y2 - y1) * (x4 - x3)
    )
    if determinant == 0:
        return None
    factor = (
        (x4 - x3) * (y1 - y3)
        - (y4 - y3) * (x1 - x3)
    ) / determinant
    return (
        int(x1 + factor * (x2 - x1)),
        int(y1 + factor * (y2 - y1)),
    )


def point_distance(point_a, point_b):
    return math.sqrt(
        (point_a[0] - point_b[0]) ** 2
        + (point_a[1] - point_b[1]) ** 2
    )


def polygon_area(corners):
    twice_area = 0
    for index in range(4):
        next_index = (index + 1) % 4
        twice_area += (
            corners[index][0] * corners[next_index][1]
            - corners[next_index][0] * corners[index][1]
        )
    return abs(twice_area) * 0.5


def is_convex_quadrilateral(corners):
    sign = 0
    for index in range(4):
        point_a = corners[index]
        point_b = corners[(index + 1) % 4]
        point_c = corners[(index + 2) % 4]
        cross = (
            (point_b[0] - point_a[0])
            * (point_c[1] - point_b[1])
            - (point_b[1] - point_a[1])
            * (point_c[0] - point_b[0])
        )
        if abs(cross) < 10:
            return False
        current_sign = 1 if cross > 0 else -1
        if sign and current_sign != sign:
            return False
        sign = current_sign
    return True


def pixel_is_black(pixel):
    try:
        red = int(pixel[0])
        green = int(pixel[1])
        blue = int(pixel[2])
        maximum = max(red, green, blue)
        minimum = min(red, green, blue)
        return (
            maximum <= BLACK_PIXEL_MAX
            and maximum - minimum <= BLACK_CHANNEL_SPREAD_MAX
        )
    except Exception:
        return False


def black_border_metrics(image_array, corners):
    black_samples = 0
    edge_hits = 0
    fractions = (0.18, 0.38, 0.62, 0.82)
    for edge_index in range(4):
        point_a = corners[edge_index]
        point_b = corners[(edge_index + 1) % 4]
        edge_black = 0
        for fraction in fractions:
            center_x = int(
                point_a[0] + (point_b[0] - point_a[0]) * fraction
            )
            center_y = int(
                point_a[1] + (point_b[1] - point_a[1]) * fraction
            )
            found_black = False
            for offset_y in range(-2, 3):
                y = center_y + offset_y
                if y < 0 or y >= IMAGE_HEIGHT:
                    continue
                for offset_x in range(-2, 3):
                    x = center_x + offset_x
                    if x < 0 or x >= IMAGE_WIDTH:
                        continue
                    try:
                        pixel = image_array[y, x]
                    except Exception:
                        return {
                            "black_samples": MIN_BLACK_EDGE_SAMPLES,
                            "edge_hits": MIN_BLACK_EDGES_WITH_HIT,
                        }
                    if pixel_is_black(pixel):
                        found_black = True
                        break
                if found_black:
                    break
            if found_black:
                black_samples += 1
                edge_black += 1
        if edge_black:
            edge_hits += 1
    return {
        "black_samples": black_samples,
        "edge_hits": edge_hits,
    }


def rectangle_to_target(rectangle, image_array):
    width = rectangle[2]
    height = rectangle[3]
    if width <= 0 or height <= 0:
        return None

    corners = [
        [rectangle[4], rectangle[5]],
        [rectangle[6], rectangle[7]],
        [rectangle[8], rectangle[9]],
        [rectangle[10], rectangle[11]],
    ]
    if not is_convex_quadrilateral(corners):
        return None

    edge_lengths = [
        point_distance(corners[index], corners[(index + 1) % 4])
        for index in range(4)
    ]
    shortest_edge = min(edge_lengths)
    if shortest_edge < MIN_EDGE_LENGTH:
        return None
    if max(edge_lengths) / shortest_edge > MAX_EDGE_LENGTH_RATIO:
        return None

    area = polygon_area(corners)
    bounding_area = width * height
    if area < MIN_RECTANGLE_AREA or bounding_area <= 0:
        return None
    fill_ratio = area / bounding_area
    if fill_ratio < MIN_QUAD_FILL_RATIO:
        return None

    center = line_intersection(
        corners[0][0], corners[0][1],
        corners[2][0], corners[2][1],
        corners[1][0], corners[1][1],
        corners[3][0], corners[3][1],
    )
    if center is None:
        return None
    if not (
        0 <= center[0] < IMAGE_WIDTH
        and 0 <= center[1] < IMAGE_HEIGHT
    ):
        return None

    metrics = black_border_metrics(image_array, corners)
    black_samples = metrics["black_samples"]
    if black_samples < MIN_BLACK_EDGE_SAMPLES:
        return None
    if metrics["edge_hits"] < MIN_BLACK_EDGES_WITH_HIT:
        return None

    target_quality = (
        area
        * fill_ratio
        * (1.0 + 0.18 * black_samples)
        * (1.0 + 0.25 * metrics["edge_hits"])
    )
    return {
        "corners": corners,
        "cx": center[0],
        "cy": center[1],
        "ex": center[0] - IMAGE_CENTER_X,
        "ey": center[1] - IMAGE_CENTER_Y,
        "area": int(area),
        "black_samples": black_samples,
        "edge_hits": metrics["edge_hits"],
        "fill_ratio": fill_ratio,
        "quality": target_quality,
        "fresh": True,
    }


def detect_targets(image):
    global detector_error_count
    global last_detector_error_report_ms
    try:
        image_array = image.to_numpy_ref()
        rectangles = cv_lite.rgb888_find_rectangles_with_corners(
            IMAGE_SHAPE,
            image_array,
            CANNY_THRESHOLD_1,
            CANNY_THRESHOLD_2,
            APPROX_EPSILON,
            MIN_AREA_RATIO,
            MAX_ANGLE_COSINE,
            GAUSSIAN_BLUR,
        )
    except Exception as error:
        detector_error_count += 1
        now_ms = time.ticks_ms()
        if (
            time.ticks_diff(
                now_ms, last_detector_error_report_ms
            )
            >= 2000
        ):
            message = str(error).replace(",", ";").replace("\n", " ")
            wireless_send(
                "VISION_ERROR,count=%d,%s"
                % (detector_error_count, message)
            )
            last_detector_error_report_ms = now_ms
        return []

    candidates = []
    if rectangles:
        ordered = sorted(
            rectangles,
            key=lambda item: item[2] * item[3],
            reverse=True,
        )
        for rectangle in ordered[:MAX_RAW_QUADRILATERALS]:
            candidate = rectangle_to_target(rectangle, image_array)
            if candidate is not None:
                candidates.append(candidate)
                if len(candidates) >= MAX_VALID_QUADRILATERALS:
                    break
    return candidates


def association_score(candidate, previous):
    dx = candidate["cx"] - previous["cx"]
    dy = candidate["cy"] - previous["cy"]
    previous_size = max(30.0, math.sqrt(previous["area"]))
    distance_score = math.sqrt(dx * dx + dy * dy) / previous_size
    area_ratio = max(
        0.01,
        candidate["area"] / max(1.0, previous["area"]),
    )
    area_score = abs(math.log(area_ratio))
    return distance_score + 1.25 * area_score


def target_center_delta(candidate, previous):
    dx = candidate["cx"] - previous["cx"]
    dy = candidate["cy"] - previous["cy"]
    return dx, dy, math.sqrt(dx * dx + dy * dy)


def target_area_ratio(candidate, previous):
    return candidate["area"] / max(1.0, previous["area"])


def target_area_is_compatible(candidate, previous):
    area_ratio = target_area_ratio(candidate, previous)
    return 0.55 <= area_ratio <= 1.80


def target_is_strong_lock(candidate):
    return (
        candidate["black_samples"] >= INITIAL_LOCK_MIN_BLACK_EDGE_SAMPLES
        and candidate["edge_hits"] >= INITIAL_LOCK_MIN_BLACK_EDGES_WITH_HIT
    )


def plausible_continuation(candidate, previous):
    dx, dy, distance = target_center_delta(candidate, previous)
    if distance > TRACK_MAX_CENTER_JUMP_PIXELS:
        return False
    if (
        abs(dx) > TRACK_MAX_AXIS_JUMP_PIXELS
        or abs(dy) > TRACK_MAX_AXIS_JUMP_PIXELS
    ):
        return False
    if distance > TRACK_SUSPICIOUS_JUMP_PIXELS:
        area_ratio = target_area_ratio(candidate, previous)
        if (
            area_ratio < TRACK_AREA_RATIO_MIN
            or area_ratio > TRACK_AREA_RATIO_MAX
        ):
            return False
    return True


def plausible_reacquisition(candidate, reference):
    dx, dy, distance = target_center_delta(candidate, reference)
    if distance > TRACK_REACQUIRE_MAX_CENTER_JUMP_PIXELS:
        return False
    if (
        abs(dx) > TRACK_REACQUIRE_MAX_AXIS_JUMP_PIXELS
        or abs(dy) > TRACK_REACQUIRE_MAX_AXIS_JUMP_PIXELS
    ):
        return False
    if not target_area_is_compatible(candidate, reference):
        return False
    return (
        association_score(candidate, reference)
        <= TRACK_REACQUIRE_ASSOCIATION_LIMIT
    )


def highest_quality_candidate(candidates):
    best = None
    best_quality = -1.0
    for candidate in candidates:
        if candidate["quality"] > best_quality:
            best = candidate
            best_quality = candidate["quality"]
    return best


def area_compatible_candidate(candidates, previous):
    best = None
    best_score = 1000000.0
    for candidate in candidates:
        if not target_area_is_compatible(candidate, previous):
            continue
        score = association_score(candidate, previous)
        if score < best_score:
            best = candidate
            best_score = score
    return best


def initial_target_candidate(candidates):
    strong = [
        candidate
        for candidate in candidates
        if target_is_strong_lock(candidate)
    ]
    if strong:
        return highest_quality_candidate(strong)
    return highest_quality_candidate(candidates)


class RectangleTracker:
    def __init__(self):
        self.reset()

    def reset(self):
        self.last = None
        self.missed_frames = 0
        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.pending = None
        self.pending_frames = 0

    def predicted_reference(self):
        if self.last is None:
            return None
        frame_count = min(3, self.missed_frames + 1)
        reference = dict(self.last)
        reference["cx"] = int(
            self.last["cx"] + self.velocity_x * frame_count
        )
        reference["cy"] = int(
            self.last["cy"] + self.velocity_y * frame_count
        )
        return reference

    def remember_target(self, target):
        if self.last is not None and target.get("fresh", True):
            dx = clamp(
                target["cx"] - self.last["cx"],
                -TRACK_MAX_PREDICT_PIXELS_PER_FRAME,
                TRACK_MAX_PREDICT_PIXELS_PER_FRAME,
            )
            dy = clamp(
                target["cy"] - self.last["cy"],
                -TRACK_MAX_PREDICT_PIXELS_PER_FRAME,
                TRACK_MAX_PREDICT_PIXELS_PER_FRAME,
            )
            self.velocity_x = (
                self.velocity_x * (1.0 - TRACK_PREDICT_ALPHA)
                + dx * TRACK_PREDICT_ALPHA
            )
            self.velocity_y = (
                self.velocity_y * (1.0 - TRACK_PREDICT_ALPHA)
                + dy * TRACK_PREDICT_ALPHA
            )
        self.last = target
        self.missed_frames = 0
        self.pending = None
        self.pending_frames = 0

    def confirmed_initial_target(self, candidates):
        candidate = initial_target_candidate(candidates)
        if candidate is None:
            self.pending = None
            self.pending_frames = 0
            return None
        if self.pending is not None and plausible_reacquisition(
            candidate, self.pending
        ):
            self.pending_frames += 1
        else:
            self.pending = candidate
            self.pending_frames = 1
        if (
            self.pending_frames >= INITIAL_LOCK_CONFIRM_FRAMES
            and target_is_strong_lock(candidate)
        ):
            self.remember_target(candidate)
            return self.last
        if self.pending_frames >= INITIAL_LOCK_CONFIRM_FRAMES + 2:
            self.remember_target(candidate)
            return self.last
        return None

    def update(self, image):
        candidates = detect_targets(image)
        target = None

        if candidates and self.last is not None:
            reference = self.predicted_reference()
            target_score = 1000000.0
            for candidate in candidates:
                if not plausible_continuation(candidate, reference):
                    continue
                score = association_score(candidate, reference)
                if score < target_score:
                    target = candidate
                    target_score = score
            if target_score > TRACK_ASSOCIATION_LIMIT:
                target = None
        elif candidates:
            return self.confirmed_initial_target(candidates)

        if target is not None:
            self.remember_target(target)
            return self.last

        self.missed_frames += 1
        if (
            self.last is not None
            and self.missed_frames <= DETECTION_HOLD_FRAMES
        ):
            self.last["fresh"] = False
            return self.last

        if candidates:
            # Reacquire near the predicted target location. A pure area-only
            # reacquire can jump to another rectangle with a similar size.
            reference = self.predicted_reference()
            best_score = 1000000.0
            for candidate in candidates:
                if not plausible_reacquisition(candidate, reference):
                    continue
                score = association_score(candidate, reference)
                if score < best_score:
                    target = candidate
                    best_score = score
            if target is not None:
                self.remember_target(target)
                return self.last

        self.last = None
        return None


sensor = None
wireless_uart = None
x_motor = None
y_motor = None
wireless_buffer = b""
detector_error_count = 0
last_detector_error_report_ms = time.ticks_add(
    time.ticks_ms(), -2000
)

autotune_requested = False
abort_requested = False
tracking_enabled = False
autotune_running = False

axis_polarity = {"X": None, "Y": None}
gains = {
    "X": {"kp": 0.0, "ki": DEFAULT_TRACK_KI, "kd": 0.0},
    "Y": {"kp": 0.0, "ki": DEFAULT_TRACK_KI, "kd": 0.0},
}
follow_settings = {
    "ff_gain": DEFAULT_TRACK_VELOCITY_FF_GAIN,
    "ff_max": DEFAULT_MAX_TRACK_VELOCITY_FF_RPM,
    "integral_limit": DEFAULT_MAX_TRACK_INTEGRAL_RPM,
    "large_i_scale": DEFAULT_TRACK_LARGE_ERROR_INTEGRAL_SCALE,
}
pid_test_step_amplitude = {
    "X": PID_MAX_STEP_PIXELS["X"],
    "Y": PID_MAX_STEP_PIXELS["Y"],
}
axis_plant = {
    "X": {
        "pixels_per_motor_degree": 0.0,
        "pixels_per_second_per_rpm": 0.0,
        "backlash_pixels": 0.0,
        "tune_max_rpm": TUNE_MAX_RPM,
        "recenter_max_rpm": RECENTER_MAX_RPM,
        "track_max_rpm": X_TRACK_MAX_RPM,
    },
    "Y": {
        "pixels_per_motor_degree": 0.0,
        "pixels_per_second_per_rpm": 0.0,
        "backlash_pixels": 0.0,
        "tune_max_rpm": Y_TUNE_MAX_RPM,
        "recenter_max_rpm": Y_RECENTER_MAX_RPM,
        "track_max_rpm": Y_TRACK_MAX_RPM,
    },
}

tracker = RectangleTracker()
last_observation = None
last_telemetry_ms = time.ticks_ms()
last_tune_telemetry_ms = time.ticks_ms()
tracking_centered_frames = 0
last_stop_heartbeat_ms = time.ticks_add(time.ticks_ms(), -1000)
last_effect_rpm = {"X": 0.0, "Y": 0.0}
last_capture_ms = time.ticks_ms()
frame_interval_ms = 0
last_motor_keepalive_ms = time.ticks_add(time.ticks_ms(), -1000)

control_state = {
    "X": {
        "e": None,
        "last_n": None,
        "last_ms": None,
        "d": 0.0,
        "i": 0.0,
        "last_error_sign": 0,
        "centered_frames": 0,
        "reversal_boost_frames": 0,
    },
    "Y": {
        "e": None,
        "last_n": None,
        "last_ms": None,
        "d": 0.0,
        "i": 0.0,
        "last_error_sign": 0,
        "centered_frames": 0,
        "reversal_boost_frames": 0,
    },
}


def wireless_send(line):
    print(line)
    if wireless_uart is not None:
        wireless_uart.write(line + "\r\n")


def send_tune_telemetry(observation):
    global last_tune_telemetry_ms

    if not autotune_running or wireless_uart is None:
        return
    now_ms = time.ticks_ms()
    if (
        time.ticks_diff(now_ms, last_tune_telemetry_ms)
        < TUNE_TELEMETRY_PERIOD_MS
    ):
        return

    if observation is None:
        line = (
            "RECT,%d,0,0,0,0,0,0,%.2f,%.2f,%d"
            % (
                now_ms,
                last_effect_rpm["X"],
                last_effect_rpm["Y"],
                frame_interval_ms,
            )
        )
    else:
        valid_code = (
            1 if observation.get("fresh", True) else 2
        )
        line = (
            "RECT,%d,%d,%d,%d,%d,%d,%d,%.2f,%.2f,%d"
            % (
                now_ms,
                valid_code,
                observation["cx"],
                observation["cy"],
                observation["ex"],
                observation["ey"],
                observation["area"],
                last_effect_rpm["X"],
                last_effect_rpm["Y"],
                frame_interval_ms,
            )
        )
    # High-rate graph data goes only to the wireless UART; avoid printing
    # every sample to the IDE terminal and slowing the vision loop.
    wireless_uart.write(line + "\r\n")
    last_tune_telemetry_ms = now_ms


def all_motors_stop(force=False):
    last_effect_rpm["X"] = 0.0
    last_effect_rpm["Y"] = 0.0
    if x_motor is not None:
        x_motor.stop(force)
    if y_motor is not None:
        y_motor.stop(force)


def ensure_motors_stopped():
    global last_stop_heartbeat_ms
    now_ms = time.ticks_ms()
    force = (
        time.ticks_diff(now_ms, last_stop_heartbeat_ms) >= 200
    )
    all_motors_stop(force)
    if force:
        last_stop_heartbeat_ms = now_ms


def service_motor_keepalive():
    global last_motor_keepalive_ms
    now_ms = time.ticks_ms()
    if (
        time.ticks_diff(now_ms, last_motor_keepalive_ms)
        < MOTOR_KEEPALIVE_PERIOD_MS
    ):
        return
    if x_motor is not None:
        x_motor.keepalive()
    if y_motor is not None:
        y_motor.keepalive()
    last_motor_keepalive_ms = now_ms


def all_motors_enable():
    if x_motor is not None:
        x_motor.configure_heavy_load_mode()
        wireless_send(
            "MOTOR_CONFIG,X,FIRMWARE,%s,LINK,%s,FIRMWARE_VERIFIED,%d,SPEED_SCALE,%d,SCALE_VERIFIED,%d,CURRENT_REQUEST_MA,%d"
            % (
                x_motor.firmware,
                MOTOR_LINK_MODE,
                1 if x_motor.firmware_verified else 0,
                x_motor.speed_scale,
                1 if x_motor.speed_scale_verified else 0,
                MOTOR_CLOSED_LOOP_MAX_CURRENT_MA,
            )
        )
        x_motor.configure_heartbeat()
    if y_motor is not None:
        y_motor.configure_heavy_load_mode()
        wireless_send(
            "MOTOR_CONFIG,Y,FIRMWARE,%s,LINK,%s,FIRMWARE_VERIFIED,%d,SPEED_SCALE,%d,SCALE_VERIFIED,%d,CURRENT_REQUEST_MA,%d"
            % (
                y_motor.firmware,
                MOTOR_LINK_MODE,
                1 if y_motor.firmware_verified else 0,
                y_motor.speed_scale,
                1 if y_motor.speed_scale_verified else 0,
                MOTOR_CLOSED_LOOP_MAX_CURRENT_MA,
            )
        )
        y_motor.configure_heartbeat()
    # Clear a previous run's stall/over-current latch even when status reads
    # are unavailable, then enable after both heartbeat settings complete.
    if x_motor is not None:
        x_motor.clear_protection()
    if y_motor is not None:
        y_motor.clear_protection()
    all_motors_stop(True)


def all_motors_disable():
    if x_motor is not None:
        x_motor.disable()
    if y_motor is not None:
        y_motor.disable()


def reset_controller(axis):
    control_state[axis]["e"] = None
    control_state[axis]["last_n"] = None
    control_state[axis]["last_ms"] = None
    control_state[axis]["d"] = 0.0
    control_state[axis]["i"] = 0.0
    control_state[axis]["last_error_sign"] = 0
    control_state[axis]["centered_frames"] = 0
    control_state[axis]["reversal_boost_frames"] = 0


def apply_follow_settings(
    ff_gain,
    ff_max,
    integral_limit,
    large_i_scale,
):
    follow_settings["ff_gain"] = clamp(ff_gain, 0.020, 0.090)
    follow_settings["ff_max"] = clamp(ff_max, 3.0, 12.0)
    follow_settings["integral_limit"] = clamp(
        integral_limit, 8.0, 22.0
    )
    follow_settings["large_i_scale"] = clamp(
        large_i_scale, 0.08, 0.40
    )
    reset_controller("X")
    reset_controller("Y")


def is_bridge_noise_command(command):
    noisy_prefixes = (
        "LOAD ",
        "TAIL ",
        "CHKSUM ",
        "CSUM ",
        "V000",
        "PID_BRIDGE_SIDE",
        "ESP8266_MAC",
        "BRIDGE_READY",
    )
    if command in ("~LD", "READY"):
        return True
    for prefix in noisy_prefixes:
        if command.startswith(prefix):
            return True
    return False


def handle_command(command):
    global autotune_requested
    global abort_requested
    global tracking_enabled
    global target_ready_frames

    command = command.strip().upper()
    if not command:
        return
    if is_bridge_noise_command(command):
        return
    if command == "PING":
        wireless_send("PONG")
    elif command == "STATUS":
        wireless_send(
            "STATUS,%s,%s,tune=%d,track=%d,Xkp=%.3f,Xki=%.3f,Xkd=%.3f,Ykp=%.3f,Yki=%.3f,Ykd=%.3f,FFG=%.3f,FFMAX=%.1f,ILIM=%.1f,LISCALE=%.2f,Xpol=%s,Ypol=%s"
            % (
                DETECTOR_VERSION,
                CONTROL_PROFILE,
                1 if autotune_running else 0,
                1 if tracking_enabled else 0,
                gains["X"]["kp"],
                gains["X"]["ki"],
                gains["X"]["kd"],
                gains["Y"]["kp"],
                gains["Y"]["ki"],
                gains["Y"]["kd"],
                follow_settings["ff_gain"],
                follow_settings["ff_max"],
                follow_settings["integral_limit"],
                follow_settings["large_i_scale"],
                str(axis_polarity["X"]),
                str(axis_polarity["Y"]),
            )
        )
    elif command.startswith("PARAMS,"):
        if autotune_running or autotune_requested:
            wireless_send("ERR,PARAMS_BUSY")
            return
        try:
            parts = command.split(",")
            if len(parts) == 9:
                x_kp = float(parts[1])
                x_ki = float(parts[2])
                x_kd = float(parts[3])
                x_polarity = int(parts[4])
                y_kp = float(parts[5])
                y_ki = float(parts[6])
                y_kd = float(parts[7])
                y_polarity = int(parts[8])
                ff_gain = DEFAULT_TRACK_VELOCITY_FF_GAIN
                ff_max = DEFAULT_MAX_TRACK_VELOCITY_FF_RPM
                integral_limit = DEFAULT_MAX_TRACK_INTEGRAL_RPM
                large_i_scale = (
                    DEFAULT_TRACK_LARGE_ERROR_INTEGRAL_SCALE
                )
            elif len(parts) == 13:
                x_kp = float(parts[1])
                x_ki = float(parts[2])
                x_kd = float(parts[3])
                x_polarity = int(parts[4])
                y_kp = float(parts[5])
                y_ki = float(parts[6])
                y_kd = float(parts[7])
                y_polarity = int(parts[8])
                ff_gain = float(parts[9])
                ff_max = float(parts[10])
                integral_limit = float(parts[11])
                large_i_scale = float(parts[12])
            elif len(parts) == 7:
                # Backward-compatible V4-V6 champion command.
                x_kp = float(parts[1])
                x_ki = DEFAULT_TRACK_KI
                x_kd = float(parts[2])
                x_polarity = int(parts[3])
                y_kp = float(parts[4])
                y_ki = DEFAULT_TRACK_KI
                y_kd = float(parts[5])
                y_polarity = int(parts[6])
                ff_gain = DEFAULT_TRACK_VELOCITY_FF_GAIN
                ff_max = DEFAULT_MAX_TRACK_VELOCITY_FF_RPM
                integral_limit = DEFAULT_MAX_TRACK_INTEGRAL_RPM
                large_i_scale = (
                    DEFAULT_TRACK_LARGE_ERROR_INTEGRAL_SCALE
                )
            else:
                raise ValueError("field count")
            if not (
                0.01 <= x_kp <= 1.0
                and 0.0 <= x_ki <= 1.0
                and 0.0 <= x_kd <= 0.15
                and 0.01 <= y_kp <= 1.0
                and 0.0 <= y_ki <= 1.0
                and 0.0 <= y_kd <= 0.15
                and x_polarity in (-1, 1)
                and y_polarity in (-1, 1)
                and 0.020 <= ff_gain <= 0.090
                and 3.0 <= ff_max <= 12.0
                and 8.0 <= integral_limit <= 22.0
                and 0.08 <= large_i_scale <= 0.40
            ):
                raise ValueError("range")
        except Exception:
            wireless_send("ERR,INVALID_PARAMS")
            return

        gains["X"]["kp"] = x_kp
        gains["X"]["ki"] = x_ki
        gains["X"]["kd"] = x_kd
        gains["Y"]["kp"] = y_kp
        gains["Y"]["ki"] = y_ki
        gains["Y"]["kd"] = y_kd
        axis_polarity["X"] = x_polarity
        axis_polarity["Y"] = y_polarity
        apply_follow_settings(
            ff_gain,
            ff_max,
            integral_limit,
            large_i_scale,
        )
        reset_controller("X")
        reset_controller("Y")
        wireless_send(
            "OK,PARAMS_LOADED,XKP,%.3f,XKI,%.3f,XKD,%.3f,XPOL,%d,YKP,%.3f,YKI,%.3f,YKD,%.3f,YPOL,%d,FFG,%.3f,FFMAX,%.1f,ILIM,%.1f,LISCALE,%.2f"
            % (
                x_kp,
                x_ki,
                x_kd,
                x_polarity,
                y_kp,
                y_ki,
                y_kd,
                y_polarity,
                follow_settings["ff_gain"],
                follow_settings["ff_max"],
                follow_settings["integral_limit"],
                follow_settings["large_i_scale"],
            )
        )
    elif command == "AUTOTUNE":
        if autotune_running:
            wireless_send("ERR,AUTOTUNE_ALREADY_RUNNING")
        else:
            abort_requested = False
            autotune_requested = True
            tracking_enabled = False
            tracker.reset()
            target_ready_frames = 0
            all_motors_enable()
            all_motors_stop(True)
            wireless_send("OK,AUTOTUNE_WAIT_TARGET")
    elif command == "STOP":
        abort_requested = True
        autotune_requested = False
        tracking_enabled = False
        all_motors_stop(True)
        wireless_send("OK,STOP")
    elif command == "TRACK ON":
        if axis_polarity["X"] is None or axis_polarity["Y"] is None:
            wireless_send("ERR,TRACK_NEEDS_AUTOTUNE")
        else:
            all_motors_enable()
            tracking_enabled = True
            reset_controller("X")
            reset_controller("Y")
            wireless_send("OK,TRACK_ON")
    elif command == "TRACK OFF":
        tracking_enabled = False
        all_motors_stop(True)
        wireless_send("OK,TRACK_OFF")
    elif command == "RELEASE":
        if autotune_running:
            wireless_send("ERR,RELEASE_DURING_AUTOTUNE")
        else:
            tracking_enabled = False
            autotune_requested = False
            all_motors_disable()
            wireless_send("OK,MOTORS_RELEASED")
    elif command == "LOCK":
        all_motors_enable()
        wireless_send("OK,MOTORS_LOCKED")
    else:
        wireless_send("ERR,UNKNOWN_COMMAND," + command)


def service_wireless():
    global wireless_buffer
    if wireless_uart is None:
        return

    while wireless_uart.any():
        chunk = wireless_uart.read()
        if not chunk:
            break
        wireless_buffer += chunk
        if len(wireless_buffer) > 512:
            wireless_buffer = wireless_buffer[-512:]

    while b"\n" in wireless_buffer:
        raw, wireless_buffer = wireless_buffer.split(b"\n", 1)
        try:
            handle_command(raw.strip().decode())
        except Exception:
            wireless_send("ERR,INVALID_COMMAND")

    pending = wireless_buffer.strip().upper()
    known = (
        b"PING",
        b"STATUS",
        b"AUTOTUNE",
        b"STOP",
        b"TRACK ON",
        b"TRACK OFF",
        b"RELEASE",
        b"LOCK",
    )
    if pending in known:
        wireless_buffer = b""
        handle_command(pending.decode())


def capture_observation():
    global last_observation
    global last_capture_ms
    global frame_interval_ms

    service_wireless()
    service_motor_keepalive()
    if abort_requested and autotune_running:
        raise TuneAbort("user STOP")

    capture_ms = time.ticks_ms()
    frame_interval_ms = time.ticks_diff(capture_ms, last_capture_ms)
    last_capture_ms = capture_ms
    image = sensor.snapshot(chn=CAM_CHN_ID_0)
    observation = tracker.update(image)
    # Vision is the longest operation in a frame. Service commands again
    # immediately afterwards so an AUTOTUNE/STOP request never waits for the
    # next frame.
    service_wireless()
    service_motor_keepalive()
    if abort_requested and autotune_running:
        raise TuneAbort("user STOP")

    image.draw_cross(
        IMAGE_CENTER_X,
        IMAGE_CENTER_Y,
        color=(0, 100, 255),
        size=7,
        thickness=1,
    )

    if observation is not None:
        corners = observation["corners"]
        outline_color = (
            (255, 0, 0)
            if observation.get("fresh", True)
            else (255, 165, 0)
        )
        for index in range(4):
            next_index = (index + 1) % 4
            image.draw_line(
                corners[index][0],
                corners[index][1],
                corners[next_index][0],
                corners[next_index][1],
                color=outline_color,
                thickness=3,
            )
            image.draw_circle(
                corners[index][0],
                corners[index][1],
                2,
                color=(0, 0, 255),
                fill=True,
                thickness=3,
            )
        image.draw_circle(
            observation["cx"],
            observation["cy"],
            3,
            color=(255, 255, 0),
            fill=True,
            thickness=2,
        )
        if (
            abs(observation["ex"]) > RECENTER_ACCEPT_PIXELS
            or abs(observation["ey"]) > RECENTER_ACCEPT_PIXELS
        ):
            image.draw_line(
                IMAGE_CENTER_X,
                IMAGE_CENTER_Y,
                observation["cx"],
                observation["cy"],
                color=(255, 255, 0),
                thickness=1,
            )

    Display.show_image(image)
    last_observation = observation
    send_tune_telemetry(observation)
    return observation


def wait_and_observe(duration_ms):
    deadline = time.ticks_add(time.ticks_ms(), duration_ms)
    latest = None
    while time.ticks_diff(deadline, time.ticks_ms()) > 0:
        observation = capture_observation()
        if (
            observation is not None
            and observation.get("fresh", True)
        ):
            latest = observation
    return latest


def wait_for_target(reason):
    all_motors_stop(True)
    last_notice_ms = time.ticks_add(time.ticks_ms(), -2000)
    while True:
        ensure_motors_stopped()
        observation = capture_observation()
        if (
            observation is not None
            and observation.get("fresh", True)
        ):
            wireless_send("TUNE,TARGET_REACQUIRED,%s" % reason)
            return observation
        now_ms = time.ticks_ms()
        if time.ticks_diff(now_ms, last_notice_ms) >= 2000:
            wireless_send("TUNE,WAIT_TARGET,%s" % reason)
            last_notice_ms = now_ms


def axis_objects(axis):
    if axis == "X":
        return x_motor, "ex", IMAGE_WIDTH / 2.0
    return y_motor, "ey", IMAGE_HEIGHT / 2.0


def axis_tune_max_rpm(axis):
    return axis_plant[axis]["tune_max_rpm"]


def axis_recenter_max_rpm(axis):
    return axis_plant[axis]["recenter_max_rpm"]


def axis_track_max_rpm(axis):
    return axis_plant[axis]["track_max_rpm"]


def update_axis_plant(axis, pixel_delta, motor_degree_delta):
    motor_degree_delta = abs(float(motor_degree_delta))
    if motor_degree_delta < 0.5:
        return False
    pixels_per_degree = abs(float(pixel_delta)) / motor_degree_delta
    pixels_per_second_per_rpm = 6.0 * pixels_per_degree
    if pixels_per_second_per_rpm < 0.20:
        return False

    # Convert desired image speeds to motor RPM using the measured effective
    # transmission. This automatically includes gear ratio and camera FOV.
    if axis == "X":
        tune_ceiling = TUNE_MAX_RPM
        recenter_ceiling = RECENTER_MAX_RPM
        track_ceiling = X_TRACK_MAX_RPM
        desired_tune_px_s = 125.0
        desired_recenter_px_s = 95.0
        desired_track_px_s = 190.0
    else:
        tune_ceiling = Y_TUNE_MAX_RPM
        recenter_ceiling = Y_RECENTER_MAX_RPM
        track_ceiling = Y_TRACK_MAX_RPM
        desired_tune_px_s = 90.0
        desired_recenter_px_s = 75.0
        desired_track_px_s = 145.0

    plant = axis_plant[axis]
    plant["pixels_per_motor_degree"] = pixels_per_degree
    plant["pixels_per_second_per_rpm"] = pixels_per_second_per_rpm
    plant["tune_max_rpm"] = clamp(
        desired_tune_px_s / pixels_per_second_per_rpm,
        10.0,
        tune_ceiling,
    )
    plant["recenter_max_rpm"] = clamp(
        desired_recenter_px_s / pixels_per_second_per_rpm,
        8.0,
        recenter_ceiling,
    )
    plant["track_max_rpm"] = clamp(
        desired_track_px_s / pixels_per_second_per_rpm,
        15.0,
        track_ceiling,
    )
    wireless_send(
        "TUNE,%s,PLANT,PX_PER_MOTOR_DEG,%.4f,PX_PER_S_PER_RPM,%.3f,TUNE_MAXRPM,%.1f,RECENTER_MAXRPM,%.1f,TRACK_MAXRPM,%.1f"
        % (
            axis,
            pixels_per_degree,
            pixels_per_second_per_rpm,
            plant["tune_max_rpm"],
            plant["recenter_max_rpm"],
            plant["track_max_rpm"],
        )
    )
    return True


def reset_axis_plant_limits():
    for axis in ("X", "Y"):
        axis_plant[axis]["pixels_per_motor_degree"] = 0.0
        axis_plant[axis]["pixels_per_second_per_rpm"] = 0.0
        axis_plant[axis]["backlash_pixels"] = 0.0
    axis_plant["X"]["tune_max_rpm"] = TUNE_MAX_RPM
    axis_plant["X"]["recenter_max_rpm"] = RECENTER_MAX_RPM
    axis_plant["X"]["track_max_rpm"] = X_TRACK_MAX_RPM
    axis_plant["Y"]["tune_max_rpm"] = Y_TUNE_MAX_RPM
    axis_plant["Y"]["recenter_max_rpm"] = Y_RECENTER_MAX_RPM
    axis_plant["Y"]["track_max_rpm"] = Y_TRACK_MAX_RPM


def calibrate_axis_polarity(axis):
    motor, error_key, _ = axis_objects(axis)
    probe_duration_ms = (
        PROBE_DURATION_MS
        if axis == "X"
        else Y_PROBE_DURATION_MS
    )
    probe_levels = (
        X_PROBE_RPM_LEVELS
        if axis == "X"
        else Y_PROBE_RPM_LEVELS
    )
    motor.clear_protection()
    for probe_index, probe_rpm in enumerate(probe_levels):
        all_motors_stop(True)
        baseline = wait_and_observe(350)
        if baseline is None:
            baseline = wait_for_target("%s_CALIBRATION" % axis)
        baseline_error = baseline[error_key]
        motor_position_before = motor.read_position_degrees()

        wireless_send(
            "TUNE,%s,CALIBRATE,PROBE,RPM,%d,FIRMWARE,%s"
            % (axis, probe_rpm, motor.firmware)
        )
        last_effect_rpm[axis] = probe_rpm
        motor.set_physical_speed(probe_rpm, axis_tune_max_rpm(axis))
        wait_and_observe(probe_duration_ms)
        motor.stop(True)
        last_effect_rpm[axis] = 0.0
        motor_position_after = motor.read_position_degrees()
        status_flags = motor.read_status_flags()
        phase_current = motor.read_phase_current_ma()

        after_probe = wait_and_observe(350)
        if after_probe is None:
            after_probe = wait_for_target("%s_PROBE" % axis)
        delta = after_probe[error_key] - baseline_error
        if (
            motor_position_before is not None
            and motor_position_after is not None
        ):
            motor_degree_source = "ENCODER"
            motor_degree_delta = abs(
                motor_position_after - motor_position_before
            )
        else:
            motor_degree_source = "COMMAND_ESTIMATE"
            motor_degree_delta = (
                probe_rpm * 6.0 * probe_duration_ms / 1000.0
            )
        wireless_send(
            "TUNE,%s,PROBE_RESULT,RPM,%d,PIX_DELTA,%d,MOTOR_DEG,%.2f,MOTOR_DEG_SOURCE,%s,STATUS,%s,CURRENT_MA,%s"
            % (
                axis,
                probe_rpm,
                delta,
                motor_degree_delta,
                motor_degree_source,
                "NA" if status_flags is None else "%02X" % status_flags,
                "NA" if phase_current is None else str(phase_current),
            )
        )

        if status_flags is not None and (status_flags & 0x0C):
            wireless_send(
                "TUNE,%s,MOTOR_PROTECTION,STATUS,%02X,CLEARING"
                % (axis, status_flags)
            )
            motor.clear_protection()

        # Always undo the probe before accepting or retrying it.
        last_effect_rpm[axis] = -probe_rpm
        motor.set_physical_speed(-probe_rpm, axis_tune_max_rpm(axis))
        wait_and_observe(probe_duration_ms)
        motor.stop(True)
        last_effect_rpm[axis] = 0.0
        undo_position = motor.read_position_degrees()
        undo_observation = wait_and_observe(450)

        reverse_pixel_delta = None
        reverse_motor_degree_delta = None
        if undo_observation is not None:
            reverse_pixel_delta = (
                undo_observation[error_key] - after_probe[error_key]
            )
            axis_plant[axis]["backlash_pixels"] = abs(
                undo_observation[error_key] - baseline_error
            )
        if (
            undo_position is not None
            and motor_position_after is not None
        ):
            reverse_motor_degree_delta = abs(
                undo_position - motor_position_after
            )

        if abs(delta) < PROBE_MIN_PIXEL_CHANGE:
            if probe_index == 0:
                motor.clear_protection()
                wireless_send(
                    "TUNE,%s,CALIBRATE_RECOVERY,CLEAR_PROTECTION"
                    % axis
                )
            elif probe_index == 1:
                if motor.firmware == "EMM":
                    turbo_ok = motor.enable_emm_turbo()
                    wireless_send(
                        "TUNE,%s,CALIBRATE_RECOVERY,EMM_TURBO,%d"
                        % (axis, 1 if turbo_ok else 0)
                    )
            wireless_send(
                "TUNE,%s,CALIBRATE_RETRY,DELTA,%d,NEXT_RPM"
                % (axis, delta)
            )
            continue

        # +physical means protocol direction 0. Record its image effect.
        motor.protocol_motion_verified = True
        axis_polarity[axis] = 1 if delta > 0 else -1
        combined_pixel_delta = abs(delta)
        combined_motor_degree_delta = motor_degree_delta
        if (
            reverse_pixel_delta is not None
            and reverse_motor_degree_delta is not None
            and reverse_motor_degree_delta >= 0.5
        ):
            combined_pixel_delta += abs(reverse_pixel_delta)
            combined_motor_degree_delta += reverse_motor_degree_delta
        update_axis_plant(
            axis,
            combined_pixel_delta,
            combined_motor_degree_delta,
        )
        wireless_send(
            "TUNE,%s,GEAR,BACKLASH_PX,%.1f,FORWARD_PX,%d,REVERSE_PX,%s"
            % (
                axis,
                axis_plant[axis]["backlash_pixels"],
                delta,
                "NA" if reverse_pixel_delta is None else str(reverse_pixel_delta),
            )
        )
        probe_degrees = max(0.1, motor_degree_delta)
        pixels_per_degree = abs(delta) / probe_degrees
        kick_degrees = clamp(
            KICK_TARGET_PIXELS / max(0.1, pixels_per_degree),
            KICK_MIN_DEGREES,
            KICK_MAX_DEGREES,
        )
        wireless_send(
            "TUNE,%s,POLARITY,%d,DELTA,%d,KICK_DEG,%.1f"
            % (axis, axis_polarity[axis], delta, kick_degrees)
        )
        return

    motor.stop(True)
    wireless_send(
        "TUNE,%s,CALIBRATE_FAILED,NO_MOTION,FIRMWARE,%s"
        % (axis, motor.firmware)
    )
    raise TuneAbort("%s_MOTOR_NO_MOTION" % axis)


def command_axis_effect_speed(axis, effect_rpm, maximum_rpm):
    motor, _, _ = axis_objects(axis)
    polarity = axis_polarity[axis]
    if polarity is None:
        last_effect_rpm[axis] = 0.0
        motor.stop()
        return
    # polarity tells which physical sign increases image error.
    last_effect_rpm[axis] = clamp(
        effect_rpm, -maximum_rpm, maximum_rpm
    )
    physical_rpm = effect_rpm * polarity
    motor.set_physical_speed(physical_rpm, maximum_rpm)


def controller_output(
    axis,
    observation,
    kp,
    kd,
    disturbance_rpm=0.0,
    follow_target_motion=False,
    ki=0.0,
):
    _, error_key, _ = axis_objects(axis)
    raw_error_pixels = observation[error_key]
    now_ms = time.ticks_ms()
    state = control_state[axis]

    if state["e"] is None:
        filtered_error = float(raw_error_pixels)
    else:
        filtered_error = (
            state["e"]
            + ERROR_FILTER_ALPHA
            * (raw_error_pixels - state["e"])
        )
    state["e"] = filtered_error

    dt = 0.0
    if state["last_ms"] is None:
        derivative = 0.0
    else:
        dt = time.ticks_diff(now_ms, state["last_ms"]) / 1000.0
        if dt <= 0.0:
            derivative = 0.0
        else:
            derivative = (
                filtered_error - state["last_n"]
            ) / dt

    derivative = clamp(
        derivative,
        -DERIVATIVE_LIMIT_PX_PER_SECOND,
        DERIVATIVE_LIMIT_PX_PER_SECOND,
    )
    state["d"] = 0.75 * state["d"] + 0.25 * derivative
    state["last_n"] = filtered_error
    state["last_ms"] = now_ms

    error_sign = (
        1 if filtered_error > DEADBAND_PIXELS
        else (-1 if filtered_error < -DEADBAND_PIXELS else 0)
    )
    if (
        error_sign
        and state["last_error_sign"]
        and error_sign != state["last_error_sign"]
    ):
        # A genuine centre crossing or target reversal makes the retained
        # following drive stale. Release it quickly to avoid geared backlash
        # producing a second overshoot in the opposite direction.
        state["i"] *= 0.35
        state["reversal_boost_frames"] = GEAR_REVERSAL_BOOST_FRAMES
    if error_sign:
        state["last_error_sign"] = error_sign

    area = max(1.0, float(observation["area"]))
    distance_gain = clamp(
        math.sqrt(DISTANCE_AREA_REFERENCE / area),
        1.0,
        MAX_DISTANCE_GAIN,
    )
    large_error_ratio = clamp(
        (
            abs(filtered_error) - LARGE_ERROR_START_PIXELS
        ) / (
            LARGE_ERROR_FULL_PIXELS - LARGE_ERROR_START_PIXELS
        ),
        0.0,
        1.0,
    )
    large_error_gain = (
        1.0
        + large_error_ratio * (MAX_LARGE_ERROR_GAIN - 1.0)
    )

    # A pure position controller needs a permanent pixel error to maintain
    # speed against a continuously moving target. During normal tracking only,
    # use a bounded integral drive to remember the required following speed.
    # Preserve that drive during a large same-direction error and keep
    # learning at a reduced rate instead of erasing it every frame. P/D then
    # recover the position while the target speed is retained. Reversal is
    # deliberately much faster.
    integral_drive = 0.0
    velocity_feedforward = 0.0
    if follow_target_motion:
        if dt > 0.0:
            unwinding = state["i"] * filtered_error < 0.0
            integral_gain = ki
            if unwinding:
                integral_gain *= TRACK_INTEGRAL_UNWIND_MULTIPLIER
            elif abs(filtered_error) > TRACK_INTEGRAL_ZONE_PIXELS:
                integral_gain *= follow_settings["large_i_scale"]
            state["i"] = clamp(
                state["i"]
                + integral_gain
                * filtered_error
                * min(dt, 0.10),
                -follow_settings["integral_limit"],
                follow_settings["integral_limit"],
            )
        integral_drive = state["i"]

        # The filtered image velocity gives an immediate, bounded response
        # when a target starts moving. The integral drive above then learns
        # and retains the speed after the image error has settled.
        if abs(state["d"]) >= TRACK_VELOCITY_FF_MIN_PX_PER_SECOND:
            ff_gain = follow_settings["ff_gain"]
            ff_max = follow_settings["ff_max"]
            if (
                abs(state["d"])
                >= TRACK_FAST_FF_DERIVATIVE_PX_PER_SECOND
                and abs(filtered_error) >= TRACK_FAST_FF_ERROR_PIXELS
            ):
                ff_gain *= TRACK_FAST_FF_GAIN_MULTIPLIER
                ff_max = max(ff_max, TRACK_FAST_FF_MIN_MAX_RPM)
            velocity_feedforward = clamp(
                ff_gain * state["d"],
                -ff_max,
                ff_max,
            )

        if (
            abs(filtered_error) <= DEADBAND_PIXELS
            and abs(state["d"])
            <= CENTER_SETTLE_DERIVATIVE_PX_PER_SECOND
        ):
            state["centered_frames"] += 1
            if (
                state["centered_frames"] >= CENTER_SETTLE_FRAMES
                and abs(state["i"]) <= CENTER_RESIDUAL_DRIVE_RPM
            ):
                # Clear only a small residual drive. A larger learned drive
                # is preserved so a slowly moving target remains centred.
                state["i"] *= 0.55
                if abs(state["i"]) < 0.15:
                    state["i"] = 0.0
                integral_drive = state["i"]
                if abs(velocity_feedforward) < 0.5:
                    velocity_feedforward = 0.0
        else:
            state["centered_frames"] = 0
    else:
        state["i"] = 0.0
        state["centered_frames"] = 0

    p_term = kp * filtered_error
    d_limit = max(0.5, abs(p_term) * MAX_D_TO_P_RATIO)
    d_term = clamp(kd * state["d"], -d_limit, d_limit)
    effect_rpm = (
        -(p_term + d_term)
        * distance_gain
        * large_error_gain
        - integral_drive
        - velocity_feedforward
        + disturbance_rpm
    )
    tracking_drive_active = (
        follow_target_motion
        and (
            abs(integral_drive) >= TRACK_DRIVE_ACTIVE_RPM
            or abs(velocity_feedforward) >= TRACK_DRIVE_ACTIVE_RPM
        )
    )
    if (
        abs(filtered_error) <= DEADBAND_PIXELS
        and abs(disturbance_rpm) < 0.1
        and not tracking_drive_active
    ):
        effect_rpm = 0.0
    elif (
        abs(filtered_error) >= MIN_ACTIVE_ERROR_PIXELS
        and abs(disturbance_rpm) < 0.1
        and abs(effect_rpm) < MIN_ACTIVE_RPM
    ):
        effect_rpm = (
            -MIN_ACTIVE_RPM if filtered_error > 0
            else MIN_ACTIVE_RPM
        )

    if state["reversal_boost_frames"] > 0:
        if (
            abs(filtered_error) >= MIN_ACTIVE_ERROR_PIXELS
            and abs(disturbance_rpm) < 0.1
        ):
            image_gain = max(
                0.2,
                axis_plant[axis]["pixels_per_second_per_rpm"],
            )
            backlash_pixels = axis_plant[axis]["backlash_pixels"]
            reversal_rpm = clamp(
                backlash_pixels / (image_gain * 0.12),
                MIN_ACTIVE_RPM,
                GEAR_REVERSAL_MAX_RPM,
            )
            correction_sign = -1.0 if filtered_error > 0 else 1.0
            if abs(effect_rpm) < reversal_rpm:
                effect_rpm = correction_sign * reversal_rpm
            state["reversal_boost_frames"] -= 1
        else:
            state["reversal_boost_frames"] = 0
    return effect_rpm, raw_error_pixels


def recover_axis(axis, timeout_ms=RECENTER_TIMEOUT_MS):
    wireless_send("TUNE,%s,RECENTER" % axis)
    reset_controller(axis)
    stable = 0
    retries = 0
    deadline = time.ticks_add(time.ticks_ms(), timeout_ms)
    while True:
        if time.ticks_diff(deadline, time.ticks_ms()) <= 0:
            motor = axis_objects(axis)[0]
            status_flags = motor.read_status_flags()
            phase_current = motor.read_phase_current_ma()
            motor.stop(True)
            retries += 1
            wireless_send(
                "TUNE,%s,RECENTER_RETRY,COUNT,%d,STATUS,%s,CURRENT_MA,%s"
                % (
                    axis,
                    retries,
                    "NA" if status_flags is None else "%02X" % status_flags,
                    "NA" if phase_current is None else str(phase_current),
                )
            )
            if status_flags is not None and (status_flags & 0x0C):
                motor.clear_protection()
            turbo_ok = motor.enable_emm_turbo()
            if motor.firmware == "EMM":
                wireless_send(
                    "TUNE,%s,MOTOR_TORQUE_ESCALATION,EMM_TURBO,%d"
                    % (axis, 1 if turbo_ok else 0)
                )
            if retries >= 2:
                raise TuneAbort("%s_RECENTER_FAILED" % axis)
            reset_controller(axis)
            stable = 0
            deadline = time.ticks_add(time.ticks_ms(), timeout_ms)

        observation = capture_observation()
        if observation is None:
            all_motors_stop(True)
            wait_for_target("%s_RECENTER" % axis)
            reset_controller(axis)
            stable = 0
            deadline = time.ticks_add(time.ticks_ms(), timeout_ms)
            continue

        effect, error = controller_output(
            axis, observation, 0.34, 0.014, 0.0
        )
        command_axis_effect_speed(
            axis,
            effect,
            axis_recenter_max_rpm(axis),
        )
        if abs(error) <= RECENTER_ACCEPT_PIXELS:
            stable += 1
            if stable >= 5:
                motor = axis_objects(axis)[0]
                motor.stop(True)
                wireless_send("TUNE,%s,RECENTERED,ERROR,%d" % (axis, error))
                return observation
        else:
            stable = 0


def recover_both_axes(timeout_ms=RECENTER_TIMEOUT_MS):
    wireless_send("TUNE,XY,FINAL_RECENTER")
    reset_controller("X")
    reset_controller("Y")
    stable = 0
    deadline = time.ticks_add(time.ticks_ms(), timeout_ms)

    while True:
        if time.ticks_diff(deadline, time.ticks_ms()) <= 0:
            all_motors_stop(True)
            wireless_send("TUNE,XY,RECENTER_RETRY")
            reset_controller("X")
            reset_controller("Y")
            stable = 0
            deadline = time.ticks_add(time.ticks_ms(), timeout_ms)

        observation = capture_observation()
        if observation is None:
            all_motors_stop(True)
            wait_for_target("XY_FINAL_RECENTER")
            reset_controller("X")
            reset_controller("Y")
            stable = 0
            deadline = time.ticks_add(time.ticks_ms(), timeout_ms)
            continue

        x_effect, x_error = controller_output(
            "X", observation, 0.34, 0.014, 0.0
        )
        y_effect, y_error = controller_output(
            "Y", observation, 0.34, 0.014, 0.0
        )
        command_axis_effect_speed(
            "X", x_effect, axis_recenter_max_rpm("X")
        )
        command_axis_effect_speed(
            "Y", y_effect, axis_recenter_max_rpm("Y")
        )

        if (
            abs(x_error) <= RECENTER_ACCEPT_PIXELS
            and abs(y_error) <= RECENTER_ACCEPT_PIXELS
        ):
            stable += 1
            if stable >= 6:
                all_motors_stop(True)
                wireless_send(
                    "TUNE,XY,RECENTERED,XERR,%d,YERR,%d"
                    % (x_error, y_error)
                )
                return
        else:
            stable = 0

def safe_pid_step_amplitude(axis, observation):
    coordinate_index = 0 if axis == "X" else 1
    image_extent = IMAGE_WIDTH if axis == "X" else IMAGE_HEIGHT
    coordinates = [
        corner[coordinate_index]
        for corner in observation["corners"]
    ]
    negative_room = (
        min(coordinates) - PID_REFERENCE_EDGE_MARGIN_PIXELS
    )
    positive_room = (
        image_extent
        - 1
        - PID_REFERENCE_EDGE_MARGIN_PIXELS
        - max(coordinates)
    )
    return max(
        0.0,
        min(
            PID_MAX_STEP_PIXELS[axis],
            negative_room,
            positive_room,
        ),
    )


def target_axis_edge_clearance(axis, observation):
    coordinate_index = 0 if axis == "X" else 1
    image_extent = IMAGE_WIDTH if axis == "X" else IMAGE_HEIGHT
    coordinates = [
        corner[coordinate_index]
        for corner in observation["corners"]
    ]
    return min(
        min(coordinates),
        image_extent - 1 - max(coordinates),
    )


def pid_reference_offset(elapsed_ms, step_amplitude=60.0):
    # Geared V13 profile: a sudden target stays in its new position long
    # enough for acceleration, braking and settling. Slow ramps run in both
    # directions to expose backlash; the final fast ramp tests catch-up.
    slow_ramp_amplitude = 0.70 * step_amplitude
    fast_ramp_amplitude = 0.80 * step_amplitude
    if elapsed_ms < 200:
        return 0.0
    if elapsed_ms < 800:
        return step_amplitude
    if elapsed_ms < 1100:
        return (
            step_amplitude
            * (1.0 - (elapsed_ms - 800) / 300.0)
        )
    if elapsed_ms < 1300:
        return 0.0
    if elapsed_ms < 1900:
        return -step_amplitude
    if elapsed_ms < 2200:
        return (
            -step_amplitude
            * (1.0 - (elapsed_ms - 1900) / 300.0)
        )
    if elapsed_ms < 2350:
        return 0.0
    if elapsed_ms < 3350:
        return (
            slow_ramp_amplitude
            * (elapsed_ms - 2350)
            / 1000.0
        )
    if elapsed_ms < 3650:
        return (
            slow_ramp_amplitude
            * (1.0 - (elapsed_ms - 3350) / 300.0)
        )
    if elapsed_ms < 3750:
        return 0.0
    if elapsed_ms < 4750:
        return (
            -slow_ramp_amplitude
            * (elapsed_ms - 3750)
            / 1000.0
        )
    if elapsed_ms < 5050:
        return (
            -slow_ramp_amplitude
            * (1.0 - (elapsed_ms - 4750) / 300.0)
        )
    if elapsed_ms < 5150:
        return 0.0
    if elapsed_ms < 5750:
        return (
            fast_ramp_amplitude
            * (elapsed_ms - 5150)
            / 600.0
        )
    if elapsed_ms < 6050:
        return (
            fast_ramp_amplitude
            * (1.0 - (elapsed_ms - 5750) / 300.0)
        )
    return 0.0


def score_pid_samples(samples, step_amplitude=60.0):
    if not samples:
        return 1000000.0

    absolute = [abs(sample[1]) for sample in samples]
    mean_abs = sum(absolute) / len(absolute)
    max_abs = max(absolute)

    large_step = [
        abs(sample[1])
        for sample in samples
        if (
            200 <= sample[0] < 800
            or 1300 <= sample[0] < 1900
        )
    ]
    large_step_mean = (
        sum(large_step) / len(large_step)
        if large_step else 100.0
    )
    large_step_tail = [
        abs(sample[1])
        for sample in samples
        if (
            600 <= sample[0] < 800
            or 1700 <= sample[0] < 1900
        )
    ]
    large_step_tail_mean = (
        sum(large_step_tail) / len(large_step_tail)
        if large_step_tail else 100.0
    )
    moving = [
        abs(sample[1])
        for sample in samples
        if (
            2350 <= sample[0] < 3350
            or 3750 <= sample[0] < 4750
        )
    ]
    moving_mean = (
        sum(moving) / len(moving) if moving else 100.0
    )
    sustained_tail = [
        abs(sample[1])
        for sample in samples
        if (
            3050 <= sample[0] < 3350
            or 4450 <= sample[0] < 4750
        )
    ]
    sustained_tail_mean = (
        sum(sustained_tail) / len(sustained_tail)
        if sustained_tail else 100.0
    )
    fast_moving = [
        abs(sample[1])
        for sample in samples
        if (
            5150 <= sample[0] < 5750
        )
    ]
    fast_moving_mean = (
        sum(fast_moving) / len(fast_moving)
        if fast_moving else 100.0
    )
    fast_tail = [
        abs(sample[1])
        for sample in samples
        if (
            5500 <= sample[0] < 5750
        )
    ]
    fast_tail_mean = (
        sum(fast_tail) / len(fast_tail)
        if fast_tail else 100.0
    )
    # During the positive reference ramp, a negative tracking error means the
    # axis is falling behind; during the negative ramp the sign is reversed.
    # Penalize this directional lag separately from ordinary absolute error so
    # a zero-integral controller cannot win merely by avoiding overshoot.
    directional_lag = []
    directional_tail_lag = []
    fast_directional_lag = []
    fast_directional_tail_lag = []
    for sample in samples:
        elapsed = sample[0]
        error = sample[1]
        lag = None
        fast_lag = None
        if 2350 <= elapsed < 3350:
            lag = max(0.0, -error)
        elif 3750 <= elapsed < 4750:
            lag = max(0.0, error)
        elif 5150 <= elapsed < 5750:
            fast_lag = max(0.0, -error)
        if lag is not None:
            directional_lag.append(lag)
            if (
                3050 <= elapsed < 3350
                or 4450 <= elapsed < 4750
            ):
                directional_tail_lag.append(lag)
        if fast_lag is not None:
            fast_directional_lag.append(fast_lag)
            if (
                5500 <= elapsed < 5750
            ):
                fast_directional_tail_lag.append(fast_lag)
    directional_lag_mean = (
        sum(directional_lag) / len(directional_lag)
        if directional_lag else 100.0
    )
    directional_tail_lag_mean = (
        sum(directional_tail_lag) / len(directional_tail_lag)
        if directional_tail_lag else 100.0
    )
    fast_directional_lag_mean = (
        sum(fast_directional_lag) / len(fast_directional_lag)
        if fast_directional_lag else 100.0
    )
    fast_directional_tail_lag_mean = (
        sum(fast_directional_tail_lag)
        / len(fast_directional_tail_lag)
        if fast_directional_tail_lag else 100.0
    )
    tail = [
        abs(sample[1])
        for sample in samples
        if sample[0] >= 6050
    ]
    tail_mean = sum(tail) / len(tail) if tail else 100.0

    crossings = 0
    last_sign = 0
    output_changes = []
    output_jerks = []
    output_reversals = 0
    last_output_delta = 0.0
    has_output_delta = False
    last_output_sign = 0
    for index, sample in enumerate(samples):
        error = sample[1]
        command = sample[2]
        sign = 1 if error > DEADBAND_PIXELS else (
            -1 if error < -DEADBAND_PIXELS else 0
        )
        if sign:
            if last_sign and sign != last_sign:
                crossings += 1
            last_sign = sign
        output_sign = 1 if command > TRACK_DRIVE_ACTIVE_RPM else (
            -1 if command < -TRACK_DRIVE_ACTIVE_RPM else 0
        )
        if output_sign:
            if last_output_sign and output_sign != last_output_sign:
                output_reversals += 1
            last_output_sign = output_sign
        if index:
            output_delta = command - samples[index - 1][2]
            output_changes.append(abs(output_delta))
            if has_output_delta:
                output_jerks.append(
                    abs(output_delta - last_output_delta)
                )
            last_output_delta = output_delta
            has_output_delta = True
    change_mean = (
        sum(output_changes) / len(output_changes)
        if output_changes else 0.0
    )
    jerk_mean = (
        sum(output_jerks) / len(output_jerks)
        if output_jerks else 0.0
    )
    quiet_commands = [
        abs(sample[2])
        for sample in samples
        if (
            1100 <= sample[0] < 1300
            or 2200 <= sample[0] < 2350
            or 3650 <= sample[0] < 3750
            or 5050 <= sample[0] < 5150
            or sample[0] >= 6050
        )
    ]
    quiet_command_mean = (
        sum(quiet_commands) / len(quiet_commands)
        if quiet_commands else 0.0
    )
    tail_commands = [
        abs(sample[2])
        for sample in samples
        if sample[0] >= 6050
    ]
    tail_command_mean = (
        sum(tail_commands) / len(tail_commands)
        if tail_commands else 0.0
    )

    max_actual_error = max(abs(sample[4]) for sample in samples)
    safety_penalty = max(0.0, max_actual_error - 75.0) * 3.0
    # These are soft walls around the required behavior. A candidate cannot
    # compensate for failing sustained follow or final stability merely by
    # being excellent in one other section of the test.
    large_step_limit = max(10.0, 0.58 * step_amplitude)
    sustained_limit = max(7.0, 0.33 * step_amplitude)
    fast_follow_limit = max(9.0, 0.42 * step_amplitude)
    requirement_penalty = (
        max(0.0, large_step_tail_mean - large_step_limit) * 4.0
        + max(0.0, sustained_tail_mean - sustained_limit) * 6.0
        + max(0.0, fast_tail_mean - fast_follow_limit) * 7.0
        + max(0.0, tail_mean - 12.0) * 8.0
        + max(0.0, quiet_command_mean - 5.0) * 3.0
        + max(0.0, tail_command_mean - 4.0) * 4.0
    )
    return (
        mean_abs
        + 0.25 * max_abs
        + 1.5 * large_step_mean
        + 2.0 * large_step_tail_mean
        + 1.8 * moving_mean
        + 2.5 * sustained_tail_mean
        + 2.4 * fast_moving_mean
        + 3.4 * fast_tail_mean
        + 2.0 * directional_lag_mean
        + 3.0 * directional_tail_lag_mean
        + 3.2 * fast_directional_lag_mean
        + 4.5 * fast_directional_tail_lag_mean
        + 2.5 * tail_mean
        + 2.0 * crossings
        + 1.4 * output_reversals
        + 0.9 * change_mean
        + 0.45 * jerk_mean
        + 1.1 * quiet_command_mean
        + 1.5 * tail_command_mean
        + safety_penalty
        + requirement_penalty
    )


def evaluate_pid_candidate(axis, kp, ki, kd, mode="PID"):
    motor, error_key, _ = axis_objects(axis)
    recover_axis(axis)
    step_amplitude = pid_test_step_amplitude[axis]
    reset_controller(axis)
    samples = []
    start_ms = time.ticks_ms()
    tune_max_rpm = axis_tune_max_rpm(axis)
    last_elapsed = None
    saturated_ms = 0
    runaway_ms = 0
    no_motion_ms = 0
    motion_anchor_error = None
    previous_effect = 0.0
    saturation_error_limit = max(
        18.0,
        PID_SATURATION_ERROR_RATIO * step_amplitude,
    )
    runaway_error_limit = max(
        55.0,
        PID_RUNAWAY_ERROR_RATIO * step_amplitude,
    )

    wireless_send(
        "TUNE,%s,%s_TEST,KP,%.3f,KI,%.3f,KD,%.3f,STEP,%.1f,MAXRPM,%.1f,FFG,%.3f,FFMAX,%.1f,ILIM,%.1f,LISCALE,%.2f"
        % (
            axis,
            mode,
            kp,
            ki,
            kd,
            step_amplitude,
            tune_max_rpm,
            follow_settings["ff_gain"],
            follow_settings["ff_max"],
            follow_settings["integral_limit"],
            follow_settings["large_i_scale"],
        )
    )

    while (
        time.ticks_diff(time.ticks_ms(), start_ms)
        < PID_CANDIDATE_DURATION_MS
    ):
        elapsed = time.ticks_diff(time.ticks_ms(), start_ms)
        if last_elapsed is None:
            sample_dt_ms = 0
        else:
            sample_dt_ms = int(
                clamp(elapsed - last_elapsed, 0, 160)
            )
        last_elapsed = elapsed
        observation = capture_observation()
        if observation is None:
            motor.stop(True)
            score = 1000000.0
            wireless_send(
                "TUNE,%s,RESULT,MODE,%s,KP,%.3f,KI,%.3f,KD,%.3f,SCORE,%.3f,FAIL,TARGET_LOST"
                % (axis, mode, kp, ki, kd, score)
            )
            samples = None
            gc.collect()
            wait_for_target("%s_PID_FAILED" % axis)
            return score

        if (
            target_axis_edge_clearance(axis, observation)
            <= PID_ABORT_EDGE_MARGIN_PIXELS
        ):
            motor.stop(True)
            score = 750000.0
            wireless_send(
                "TUNE,%s,RESULT,MODE,%s,KP,%.3f,KI,%.3f,KD,%.3f,SCORE,%.3f,FAIL,EDGE_LIMIT"
                % (axis, mode, kp, ki, kd, score)
            )
            samples = None
            gc.collect()
            return score

        reference = pid_reference_offset(
            elapsed, step_amplitude
        )
        actual_error = observation[error_key]
        observation[error_key] = actual_error - reference
        try:
            effect, tracking_error = controller_output(
                axis,
                observation,
                kp,
                kd,
                follow_target_motion=True,
                ki=ki,
            )
        finally:
            observation[error_key] = actual_error

        if abs(previous_effect) >= MOTOR_NO_MOTION_MIN_RPM:
            if motion_anchor_error is None:
                motion_anchor_error = actual_error
                no_motion_ms = 0
            elif (
                abs(actual_error - motion_anchor_error)
                <= MOTOR_NO_MOTION_PIXEL_DELTA
            ):
                no_motion_ms += sample_dt_ms
            else:
                motion_anchor_error = actual_error
                no_motion_ms = 0
        else:
            motion_anchor_error = actual_error
            no_motion_ms = 0

        if no_motion_ms >= MOTOR_NO_MOTION_ABORT_MS:
            status_flags = motor.read_status_flags()
            actual_speed = motor.read_realtime_speed_rpm()
            phase_current = motor.read_phase_current_ma()
            motor.stop(True)
            wireless_send(
                "TUNE,%s,MOTOR_STALL,CMD_RPM,%.1f,ACTUAL_RPM,%s,STATUS,%s,CURRENT_MA,%s"
                % (
                    axis,
                    previous_effect,
                    "NA" if actual_speed is None else "%.1f" % actual_speed,
                    "NA" if status_flags is None else "%02X" % status_flags,
                    "NA" if phase_current is None else str(phase_current),
                )
            )
            if status_flags is not None and (status_flags & 0x0C):
                motor.clear_protection()
            turbo_ok = motor.enable_emm_turbo()
            if motor.firmware == "EMM":
                wireless_send(
                    "TUNE,%s,MOTOR_TORQUE_ESCALATION,EMM_TURBO,%d"
                    % (axis, 1 if turbo_ok else 0)
                )
            score = 625000.0
            wireless_send(
                "TUNE,%s,RESULT,MODE,%s,KP,%.3f,KI,%.3f,KD,%.3f,SCORE,%.3f,FAIL,MOTOR_STALL"
                % (axis, mode, kp, ki, kd, score)
            )
            samples = None
            gc.collect()
            return score

        if elapsed > PID_EARLY_ABORT_GRACE_MS:
            if (
                abs(effect) >= 0.94 * tune_max_rpm
                and abs(tracking_error) >= saturation_error_limit
            ):
                saturated_ms += sample_dt_ms
            else:
                saturated_ms = max(0, saturated_ms - sample_dt_ms)

            if abs(tracking_error) >= runaway_error_limit:
                runaway_ms += sample_dt_ms
            else:
                runaway_ms = max(0, runaway_ms - sample_dt_ms)

            if (
                saturated_ms >= PID_SATURATION_ABORT_MS
                or runaway_ms >= PID_RUNAWAY_ERROR_ABORT_MS
            ):
                motor.stop(True)
                score = 650000.0
                wireless_send(
                    "TUNE,%s,RESULT,MODE,%s,KP,%.3f,KI,%.3f,KD,%.3f,SCORE,%.3f,FAIL,DIVERGED"
                    % (axis, mode, kp, ki, kd, score)
                )
                samples = None
                gc.collect()
                return score

        command_axis_effect_speed(axis, effect, tune_max_rpm)
        previous_effect = clamp(effect, -tune_max_rpm, tune_max_rpm)
        samples.append(
            (
                elapsed,
                tracking_error,
                effect,
                reference,
                actual_error,
            )
        )

    motor.stop(True)
    score = score_pid_samples(samples, step_amplitude)
    wireless_send(
        "TUNE,%s,RESULT,MODE,%s,KP,%.3f,KI,%.3f,KD,%.3f,SCORE,%.3f"
        % (axis, mode, kp, ki, kd, score)
    )
    samples = None
    gc.collect()
    return score


def pid_candidate_rank(candidate):
    scores = sorted(candidate["scores"])
    count = len(scores)
    if count == 0:
        return 1000000.0
    middle = count // 2
    if count % 2:
        median = scores[middle]
    else:
        median = 0.5 * (scores[middle - 1] + scores[middle])
    return median + PID_ROBUST_WORST_WEIGHT * (
        scores[-1] - median
    )


def add_pid_candidate(candidates, kp, ki, kd):
    kp = round(clamp(kp, 0.08, 0.90), 4)
    ki = round(
        clamp(ki, MIN_SUSTAINED_TRACK_KI, 0.85),
        4,
    )
    kd = round(clamp(kd, 0.0, 0.120), 4)
    for candidate in candidates:
        if (
            abs(candidate["kp"] - kp) < PID_PARAMETER_EPSILON
            and abs(candidate["ki"] - ki) < PID_PARAMETER_EPSILON
            and abs(candidate["kd"] - kd) < PID_PARAMETER_EPSILON
        ):
            return candidate
    candidate = {
        "kp": kp,
        "ki": ki,
        "kd": kd,
        "scores": [],
        "rank": 1000000.0,
    }
    candidates.append(candidate)
    return candidate


def evaluate_pid_entry(axis, candidate):
    score = evaluate_pid_candidate(
        axis,
        candidate["kp"],
        candidate["ki"],
        candidate["kd"],
    )
    candidate["scores"].append(score)
    candidate["rank"] = pid_candidate_rank(candidate)
    return score


def select_diverse_pid_candidates(
    candidates,
    count,
    kp_separation=0.02,
    ki_separation=0.05,
    kd_separation=0.002,
):
    ordered = sorted(
        [item for item in candidates if item["scores"]],
        key=lambda item: item["rank"],
    )
    selected = []
    for candidate in ordered:
        too_close = False
        for existing in selected:
            if (
                abs(candidate["kp"] - existing["kp"])
                < kp_separation
                and abs(candidate["ki"] - existing["ki"])
                < ki_separation
                and abs(candidate["kd"] - existing["kd"])
                < kd_separation
            ):
                too_close = True
                break
        if not too_close:
            selected.append(candidate)
            if len(selected) >= count:
                return selected
    for candidate in ordered:
        already_selected = False
        for existing in selected:
            if candidate is existing:
                already_selected = True
                break
        if not already_selected:
            selected.append(candidate)
            if len(selected) >= count:
                break
    return selected


def add_pid_neighborhood(
    candidates,
    seed,
    kp_step,
    ki_step,
    kd_step,
):
    start_count = len(candidates)
    for delta in (-kp_step, kp_step):
        add_pid_candidate(
            candidates,
            seed["kp"] + delta,
            seed["ki"],
            seed["kd"],
        )
    for delta in (-ki_step, ki_step):
        add_pid_candidate(
            candidates,
            seed["kp"],
            seed["ki"] + delta,
            seed["kd"],
        )
    for delta in (-kd_step, kd_step):
        add_pid_candidate(
            candidates,
            seed["kp"],
            seed["ki"],
            seed["kd"] + delta,
        )
    return candidates[start_count:]


def add_pid_polish_candidates(candidates, seed):
    start_count = len(candidates)
    offsets = (
        (-0.010, 0.000, 0.000),
        (0.010, 0.000, 0.000),
        (-0.005, 0.000, 0.000),
        (0.005, 0.000, 0.000),
        (0.000, -0.030, 0.000),
        (0.000, 0.030, 0.000),
        (0.000, -0.015, 0.000),
        (0.000, 0.015, 0.000),
        (0.000, 0.000, -0.0005),
        (0.000, 0.000, 0.0005),
    )
    for offset in offsets:
        add_pid_candidate(
            candidates,
            seed["kp"] + offset[0],
            seed["ki"] + offset[1],
            seed["kd"] + offset[2],
        )
    return candidates[start_count:]


def ensure_pid_repeats(axis, candidates, repeat_count):
    for candidate in candidates:
        while len(candidate["scores"]) < repeat_count:
            evaluate_pid_entry(axis, candidate)


def follow_candidate_rank(candidate):
    scores = sorted(candidate["scores"])
    count = len(scores)
    if count == 0:
        return 1000000.0
    middle = count // 2
    if count % 2:
        median = scores[middle]
    else:
        median = 0.5 * (scores[middle - 1] + scores[middle])
    return median + PID_ROBUST_WORST_WEIGHT * (
        scores[-1] - median
    )


def add_follow_candidate(
    candidates,
    ff_gain,
    ff_max,
    integral_limit,
    large_i_scale,
):
    ff_gain = round(clamp(ff_gain, 0.020, 0.090), 4)
    ff_max = round(clamp(ff_max, 3.0, 12.0), 3)
    integral_limit = round(clamp(integral_limit, 8.0, 22.0), 3)
    large_i_scale = round(clamp(large_i_scale, 0.08, 0.40), 4)
    for candidate in candidates:
        if (
            abs(candidate["ff_gain"] - ff_gain)
            < FOLLOW_PARAMETER_EPSILON
            and abs(candidate["ff_max"] - ff_max)
            < FOLLOW_PARAMETER_EPSILON
            and abs(candidate["integral_limit"] - integral_limit)
            < FOLLOW_PARAMETER_EPSILON
            and abs(candidate["large_i_scale"] - large_i_scale)
            < FOLLOW_PARAMETER_EPSILON
        ):
            return candidate
    candidate = {
        "ff_gain": ff_gain,
        "ff_max": ff_max,
        "integral_limit": integral_limit,
        "large_i_scale": large_i_scale,
        "scores": [],
        "rank": 1000000.0,
    }
    candidates.append(candidate)
    return candidate


def add_follow_polish_candidates(candidates, seed):
    start_count = len(candidates)
    offsets = (
        (-0.005, 0.0, 0.0, 0.00),
        (0.005, 0.0, 0.0, 0.00),
        (0.000, -1.0, 0.0, 0.00),
        (0.000, 1.0, 0.0, 0.00),
    )
    for offset in offsets:
        add_follow_candidate(
            candidates,
            seed["ff_gain"] + offset[0],
            seed["ff_max"] + offset[1],
            seed["integral_limit"] + offset[2],
            seed["large_i_scale"] + offset[3],
        )
    return candidates[start_count:]


def evaluate_follow_entry(candidate):
    apply_follow_settings(
        candidate["ff_gain"],
        candidate["ff_max"],
        candidate["integral_limit"],
        candidate["large_i_scale"],
    )
    total_score = 0.0
    for axis in ("X", "Y"):
        total_score += evaluate_pid_candidate(
            axis,
            gains[axis]["kp"],
            gains[axis]["ki"],
            gains[axis]["kd"],
            mode="FOLLOW",
        )
        all_motors_stop(True)
        gc.collect()
        wait_and_observe(250)
    candidate["scores"].append(total_score)
    candidate["rank"] = follow_candidate_rank(candidate)
    wireless_send(
        "TUNE,FOLLOW,RESULT,MODE,FOLLOW,KP,0.000,KI,0.000,KD,0.000,FFG,%.3f,FFMAX,%.1f,ILIM,%.1f,LISCALE,%.2f,SCORE,%.3f"
        % (
            candidate["ff_gain"],
            candidate["ff_max"],
            candidate["integral_limit"],
            candidate["large_i_scale"],
            candidate["rank"],
        )
    )
    return total_score


def ensure_follow_repeats(candidates, repeat_count):
    for candidate in candidates:
        while len(candidate["scores"]) < repeat_count:
            evaluate_follow_entry(candidate)


def tune_follow_profile():
    candidates = []
    add_follow_candidate(
        candidates,
        follow_settings["ff_gain"],
        follow_settings["ff_max"],
        follow_settings["integral_limit"],
        follow_settings["large_i_scale"],
    )
    for profile in FOLLOW_PROFILE_CANDIDATES:
        add_follow_candidate(
            candidates,
            profile[0],
            profile[1],
            profile[2],
            profile[3],
        )

    wireless_send(
        "TUNE,FOLLOW,BEGIN,MODE,FOLLOW_CHAMPIONSHIP,COUNT,%d"
        % len(candidates)
    )
    for candidate in candidates:
        evaluate_follow_entry(candidate)

    finalists = sorted(
        candidates,
        key=lambda item: item["rank"],
    )[:2]
    wireless_send(
        "TUNE,FOLLOW,FINAL,COUNT,%d,REPEATS,2"
        % len(finalists)
    )
    ensure_follow_repeats(finalists, 2)
    finalists.sort(key=lambda item: item["rank"])
    champion = finalists[0]

    polish_entries = add_follow_polish_candidates(candidates, champion)
    wireless_send(
        "TUNE,FOLLOW,POLISH,COUNT,%d"
        % len(polish_entries)
    )
    for candidate in polish_entries:
        evaluate_follow_entry(candidate)

    polish_pool = sorted(
        [champion] + polish_entries,
        key=lambda item: item["rank"],
    )[:2]
    wireless_send(
        "TUNE,FOLLOW,POLISH_FINAL,COUNT,%d,REPEATS,%d"
        % (len(polish_pool), FOLLOW_POLISH_REPEAT_COUNT)
    )
    ensure_follow_repeats(polish_pool, FOLLOW_POLISH_REPEAT_COUNT)
    polish_pool.sort(key=lambda item: item["rank"])
    champion = polish_pool[0]

    apply_follow_settings(
        champion["ff_gain"],
        champion["ff_max"],
        champion["integral_limit"],
        champion["large_i_scale"],
    )
    wireless_send(
        "TUNE,FOLLOW,COMPLETE,FFG,%.3f,FFMAX,%.1f,ILIM,%.1f,LISCALE,%.2f,SCORE,%.3f,TESTS,%d"
        % (
            champion["ff_gain"],
            champion["ff_max"],
            champion["integral_limit"],
            champion["large_i_scale"],
            champion["rank"],
            len(champion["scores"]),
        )
    )
    return champion["rank"]


def tune_axis(axis):
    incumbent_kp = gains[axis]["kp"]
    incumbent_ki = gains[axis]["ki"]
    incumbent_kd = gains[axis]["kd"]

    wireless_send("TUNE,%s,BEGIN,MODE,PID_CHAMPIONSHIP" % axis)
    calibrate_axis_polarity(axis)
    centered_observation = recover_axis(axis)
    pid_test_step_amplitude[axis] = safe_pid_step_amplitude(
        axis, centered_observation
    )
    wireless_send(
        "TUNE,%s,SAFE_LIMIT,STEP,%.1f,MAXRPM,%.1f"
        % (
            axis,
            pid_test_step_amplitude[axis],
            axis_tune_max_rpm(axis),
        )
    )

    candidates = []
    for kp in PID_COARSE_KP:
        for ki in PID_COARSE_KI:
            for kd in PID_COARSE_KD:
                add_pid_candidate(candidates, kp, ki, kd)

    incumbent_candidate = None
    if (
        0.08 <= incumbent_kp <= 0.90
        and MIN_SUSTAINED_TRACK_KI <= incumbent_ki <= 0.85
        and 0.0 <= incumbent_kd <= 0.120
    ):
        incumbent_candidate = add_pid_candidate(
            candidates,
            incumbent_kp,
            incumbent_ki,
            incumbent_kd,
        )

    wireless_send(
        "TUNE,%s,CHAMPIONSHIP_COARSE,COUNT,%d"
        % (axis, len(candidates))
    )
    for candidate in candidates:
        evaluate_pid_entry(axis, candidate)

    # Successive-halving search. Only the best diverse candidates advance;
    # weak candidates no longer consume several repeated 5.9 s tests.
    quarterfinalists = select_diverse_pid_candidates(
        candidates, 4, 0.08, 0.12, 0.004
    )
    wireless_send(
        "TUNE,%s,CHAMPIONSHIP_QUARTERFINAL,COUNT,%d"
        % (axis, len(quarterfinalists))
    )
    ensure_pid_repeats(axis, quarterfinalists, 2)

    local_seeds = select_diverse_pid_candidates(
        quarterfinalists, 2, 0.06, 0.10, 0.003
    )
    local_entries = []
    for seed in local_seeds:
        local_entries.extend(
            add_pid_neighborhood(
                candidates,
                seed,
                0.11,
                0.14,
                0.020,
            )
        )
    wireless_send(
        "TUNE,%s,CHAMPIONSHIP_LOCAL,COUNT,%d"
        % (axis, len(local_entries))
    )
    for candidate in local_entries:
        evaluate_pid_entry(axis, candidate)

    semifinal_pool = quarterfinalists + local_entries
    semifinalists = select_diverse_pid_candidates(
        semifinal_pool, 4, 0.04, 0.08, 0.002
    )
    wireless_send(
        "TUNE,%s,CHAMPIONSHIP_SEMIFINAL,COUNT,%d"
        % (axis, len(semifinalists))
    )
    ensure_pid_repeats(axis, semifinalists, 2)

    fine_seeds = select_diverse_pid_candidates(
        semifinalists, 1, 0.02, 0.04, 0.001
    )
    fine_entries = []
    for seed in fine_seeds:
        fine_entries.extend(
            add_pid_neighborhood(
                candidates,
                seed,
                0.04,
                0.06,
                0.008,
            )
        )
    wireless_send(
        "TUNE,%s,CHAMPIONSHIP_FINE,COUNT,%d"
        % (axis, len(fine_entries))
    )
    for candidate in fine_entries:
        evaluate_pid_entry(axis, candidate)

    final_pool = semifinalists + fine_entries
    finalists = select_diverse_pid_candidates(
        final_pool, 3, 0.02, 0.04, 0.001
    )
    wireless_send(
        "TUNE,%s,CHAMPIONSHIP_FINAL,COUNT,%d,REPEATS,2"
        % (axis, len(finalists))
    )
    ensure_pid_repeats(axis, finalists, 2)
    finalists.sort(key=lambda item: item["rank"])
    champion = finalists[0]

    best_kp = champion["kp"]
    best_ki = champion["ki"]
    best_kd = champion["kd"]
    best_score = champion["rank"]
    gains[axis]["kp"] = best_kp
    gains[axis]["ki"] = best_ki
    gains[axis]["kd"] = best_kd
    recover_axis(axis)
    axis_objects(axis)[0].stop(True)

    wireless_send(
        "TUNE,%s,COMPLETE,KP,%.3f,KI,%.3f,KD,%.3f,SCORE,%.3f,TESTS,%d"
        % (
            axis,
            best_kp,
            best_ki,
            best_kd,
            best_score,
            len(champion["scores"]),
        )
    )
    return best_score


def run_autotune():
    global autotune_running
    global autotune_requested
    global tracking_enabled
    global abort_requested

    autotune_running = True
    autotune_requested = False
    tracking_enabled = False
    abort_requested = False
    all_motors_enable()
    all_motors_stop(True)
    reset_axis_plant_limits()
    # Do not let an older gimbal champion's conservative follow limits bias
    # every V16 PID candidate. Start from the geared baseline; FOLLOW tuning
    # will optimize these values after both axis PID gains are selected.
    apply_follow_settings(
        DEFAULT_TRACK_VELOCITY_FF_GAIN,
        DEFAULT_MAX_TRACK_VELOCITY_FF_RPM,
        DEFAULT_MAX_TRACK_INTEGRAL_RPM,
        DEFAULT_TRACK_LARGE_ERROR_INTEGRAL_SCALE,
    )

    try:
        wireless_send(
            "AUTOTUNE_SCORE_PROFILE,%s" % PID_SCORE_PROFILE
        )
        tune_axis("X")
        all_motors_stop(True)
        gc.collect()
        wait_and_observe(500)
        tune_axis("Y")
        all_motors_stop(True)
        gc.collect()
        follow_score = tune_follow_profile()
        all_motors_stop(True)
        gc.collect()
        wait_and_observe(500)
        x_score = evaluate_pid_candidate(
            "X",
            gains["X"]["kp"],
            gains["X"]["ki"],
            gains["X"]["kd"],
            mode="VERIFY",
        )
        all_motors_stop(True)
        gc.collect()
        wait_and_observe(500)
        y_score = evaluate_pid_candidate(
            "Y",
            gains["Y"]["kp"],
            gains["Y"]["ki"],
            gains["Y"]["kd"],
            mode="VERIFY",
        )
        all_motors_stop(True)
        gc.collect()
        recover_both_axes()

        tracking_enabled = True
        reset_controller("X")
        reset_controller("Y")
        wireless_send("OK,TRACK_ON")
        wireless_send(
            "AUTOTUNE_COMPLETE,XKP,%.3f,XKI,%.3f,XKD,%.3f,XPOL,%d,XSCORE,%.3f,YKP,%.3f,YKI,%.3f,YKD,%.3f,YPOL,%d,YSCORE,%.3f,FFG,%.3f,FFMAX,%.1f,ILIM,%.1f,LISCALE,%.2f,FSCORE,%.3f"
            % (
                gains["X"]["kp"],
                gains["X"]["ki"],
                gains["X"]["kd"],
                axis_polarity["X"],
                x_score,
                gains["Y"]["kp"],
                gains["Y"]["ki"],
                gains["Y"]["kd"],
                axis_polarity["Y"],
                y_score,
                follow_settings["ff_gain"],
                follow_settings["ff_max"],
                follow_settings["integral_limit"],
                follow_settings["large_i_scale"],
                follow_score,
            )
        )
    except TuneAbort as error:
        all_motors_stop(True)
        tracking_enabled = False
        wireless_send("AUTOTUNE_ABORT,%s" % error)
    except BaseException as error:
        all_motors_stop(True)
        tracking_enabled = False
        wireless_send("AUTOTUNE_ERROR,%s" % error)
    finally:
        autotune_running = False
        abort_requested = False


def tracking_step(observation):
    global tracking_centered_frames

    if observation is None:
        ensure_motors_stopped()
        reset_controller("X")
        reset_controller("Y")
        tracking_centered_frames = 0
        return

    if not observation.get("fresh", True):
        # A short detector dropout should not reset PID or integrate a stale
        # pixel error. Keep a capped version of the last motor speed until a
        # fresh quad returns; the normal target-loss path stops everything
        # after the short hold window.
        command_axis_effect_speed(
            "X",
            clamp(
                last_effect_rpm["X"],
                -DETECTION_HOLD_MAX_RPM,
                DETECTION_HOLD_MAX_RPM,
            ),
            axis_track_max_rpm("X"),
        )
        command_axis_effect_speed(
            "Y",
            clamp(
                last_effect_rpm["Y"],
                -DETECTION_HOLD_MAX_RPM,
                DETECTION_HOLD_MAX_RPM,
            ),
            axis_track_max_rpm("Y"),
        )
        return

    x_effect, _ = controller_output(
        "X",
        observation,
        gains["X"]["kp"],
        gains["X"]["kd"],
        follow_target_motion=True,
        ki=gains["X"]["ki"],
    )
    y_effect, _ = controller_output(
        "Y",
        observation,
        gains["Y"]["kp"],
        gains["Y"]["kd"],
        follow_target_motion=True,
        ki=gains["Y"]["ki"],
    )
    command_axis_effect_speed(
        "X", x_effect, axis_track_max_rpm("X")
    )
    command_axis_effect_speed(
        "Y", y_effect, axis_track_max_rpm("Y")
    )

    if (
        abs(observation["ex"]) <= DEADBAND_PIXELS
        and abs(observation["ey"]) <= DEADBAND_PIXELS
        and abs(x_effect) < TRACK_DRIVE_ACTIVE_RPM
        and abs(y_effect) < TRACK_DRIVE_ACTIVE_RPM
        and abs(control_state["X"]["i"]) < TRACK_DRIVE_ACTIVE_RPM
        and abs(control_state["Y"]["i"]) < TRACK_DRIVE_ACTIVE_RPM
    ):
        tracking_centered_frames += 1
        if tracking_centered_frames >= 30:
            ensure_motors_stopped()
            reset_controller("X")
            reset_controller("Y")
            tracking_centered_frames = 0
    else:
        tracking_centered_frames = 0

print("INIT,UART")
_fpioa = FPIOA()
wireless_uart = create_uart(
    UART.UART3,
    ESP_TX_PIN,
    ESP_RX_PIN,
    FPIOA.UART3_TXD,
    FPIOA.UART3_RXD,
    _fpioa,
)
x_motor = ZDTMotor(
    "X",
    create_uart(
        UART.UART1,
        X_TX_PIN,
        X_RX_PIN,
        FPIOA.UART1_TXD,
        FPIOA.UART1_RXD,
        _fpioa,
    ),
)
y_motor = ZDTMotor(
    "Y",
    create_uart(
        UART.UART2,
        Y_TX_PIN,
        Y_RX_PIN,
        FPIOA.UART2_TXD,
        FPIOA.UART2_RXD,
        _fpioa,
    ),
)

target_ready_frames = 0
frame_count = 0

try:
    all_motors_enable()

    print("INIT,CAMERA")
    sensor = Sensor(id=SENSOR_ID)
    sensor.reset()
    sensor.set_framesize(
        width=IMAGE_WIDTH,
        height=IMAGE_HEIGHT,
        chn=CAM_CHN_ID_0,
    )
    sensor.set_pixformat(Sensor.RGB888, chn=CAM_CHN_ID_0)
    Display.init(
        Display.VIRT,
        width=IMAGE_WIDTH,
        height=IMAGE_HEIGHT,
        fps=30,
        to_ide=True,
    )
    MediaManager.init()
    sensor.run()
    time.sleep_ms(1000)

    wireless_send("K230_READY,AUTOTUNE_TRACKER")

    while True:
        os.exitpoint()
        observation = capture_observation()
        now_ms = time.ticks_ms()

        if (
            observation is not None
            and observation.get("fresh", True)
        ):
            target_ready_frames += 1
        else:
            target_ready_frames = 0

        if autotune_requested:
            if target_ready_frames >= AUTOTUNE_TARGET_FRAMES:
                wireless_send("AUTOTUNE_START")
                run_autotune()
                target_ready_frames = 0
            elif (
                time.ticks_diff(now_ms, last_telemetry_ms)
                >= TELEMETRY_PERIOD_MS
            ):
                wireless_send(
                    "AUTOTUNE_WAIT_TARGET,frames=%d"
                    % target_ready_frames
                )

        if tracking_enabled and not autotune_running:
            try:
                tracking_step(observation)
            except TuneAbort as error:
                tracking_enabled = False
                all_motors_stop(True)
                wireless_send("TRACK_ABORT,%s" % error)
        elif not autotune_running:
            ensure_motors_stopped()

        if (
            time.ticks_diff(now_ms, last_telemetry_ms)
            >= TELEMETRY_PERIOD_MS
        ):
            if observation is None:
                wireless_send(
                    "RECT,%d,0,0,0,0,0,0,%.2f,%.2f,%d"
                    % (
                        now_ms,
                        last_effect_rpm["X"],
                        last_effect_rpm["Y"],
                        frame_interval_ms,
                    )
                )
            else:
                valid_code = (
                    1 if observation.get("fresh", True) else 2
                )
                wireless_send(
                    "RECT,%d,%d,%d,%d,%d,%d,%d,%.2f,%.2f,%d"
                    % (
                        now_ms,
                        valid_code,
                        observation["cx"],
                        observation["cy"],
                        observation["ex"],
                        observation["ey"],
                        observation["area"],
                        last_effect_rpm["X"],
                        last_effect_rpm["Y"],
                        frame_interval_ms,
                    )
                )
            last_telemetry_ms = now_ms

        frame_count += 1
        if frame_count >= 120:
            gc.collect()
            frame_count = 0

except KeyboardInterrupt:
    print("Stopped by user")
except BaseException as error:
    print("Fatal error:", error)
    if wireless_uart is not None:
        wireless_send("FATAL,%s" % error)
finally:
    all_motors_stop(True)
    if x_motor is not None:
        x_motor.uart.deinit()
    if y_motor is not None:
        y_motor.uart.deinit()
    if wireless_uart is not None:
        wireless_uart.deinit()
    if isinstance(sensor, Sensor):
        sensor.stop()
    Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    MediaManager.deinit()
