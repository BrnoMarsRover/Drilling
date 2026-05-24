"""
protocol.py

Handles building and parsing of UART messages according to the drill protocol spec.

Message format:
  0x02 | length | payload... | checksum | 0x03

Checksum: sum complement of payload bytes (0x100 - (sum(payload) & 0xFF)) & 0xFF

Two-step commands (trigger then poll):
  WEIGH_DEEP       -> GET_WEIGHT_DEEP
  WEIGH_SURFACE    -> GET_WEIGHT_SURFACE
  START_DEV_CHECK  -> GET_DEVICE_STATUS
"""

import struct

# --- Message framing bytes ---
STX = 0x02
ETX = 0x03

# --- Command codes ---
CMD_RESTART            = 0x01
CMD_STATE              = 0x02
CMD_DRILL_AUTO         = 0x03
CMD_STOP_AUTO          = 0x04
CMD_CALIBRATE_HEIGHT   = 0x05
CMD_START_DEV_CHECK    = 0x06
CMD_GET_DEVICE_STATUS  = 0x07
CMD_DRILL_SPEED        = 0x20
CMD_VERTICAL_SPEED     = 0x21
CMD_STORAGE_POSITION   = 0x22
CMD_WEIGH_DEEP         = 0x40
CMD_WEIGH_SURFACE      = 0x41
CMD_GET_WEIGHT_DEEP    = 0x42
CMD_GET_WEIGHT_SURFACE = 0x43
CMD_CALIBRATE_0_DEEP   = 0x44
CMD_CALIBRATE_X_DEEP   = 0x45
CMD_CALIBRATE_0_SURFACE= 0x46
CMD_CALIBRATE_X_SURFACE= 0x47
CMD_ROCK_OPEN          = 0x50
CMD_ROCK_CLOSE         = 0x51
CMD_SAND_OPEN          = 0x52
CMD_SAND_CLOSE         = 0x53

# --- Software state codes ---
STATE_CODES = {
    0x00: "Initializing",
    0x01: "Error — try restarting",
    0x02: "Ready",
    0xF0: "Auto: drilling down",
    0xF1: "Auto: could not reach depth",
    0xF2: "Auto: depth reached, moving up",
    0xF3: "Auto: storing sample",
}

# --- Device status bit order (LSB first) ---
DEVICE_NAMES = [
    "Vertical drive stepper driver",   # bit 0
    "Vertical drive encoder",          # bit 1
    "Vertical drive current sensor",   # bit 2
    "Spiral motor",                    # bit 3
    "Height sensor",                   # bit 4
    "Deep storage stepper driver",     # bit 5
    "Deep storage encoder",            # bit 6
    "Deep sample ADC",                 # bit 7
    "Surface sample ADC",              # bit 8
]


def checksum(payload: bytes) -> int:
    """Sum complement checksum over payload bytes."""
    return (0x100 - (sum(payload) & 0xFF)) & 0xFF


def build_message(payload: bytes) -> bytes:
    """Wrap a payload in the full message frame."""
    cs = checksum(payload)
    return bytes([STX, len(payload)]) + payload + bytes([cs, ETX])


def build_command(code: int, argument: bytes = b"") -> bytes:
    """Build a complete framed message for a given command code and optional argument."""
    payload = bytes([code]) + argument
    return build_message(payload)


# --- Convenience command builders ---

def cmd_restart():
    return build_command(CMD_RESTART)

def cmd_state():
    return build_command(CMD_STATE)

def cmd_drill_auto(depth_cm: int):
    # depth is uint8
    return build_command(CMD_DRILL_AUTO, struct.pack("B", depth_cm))

def cmd_stop_auto():
    return build_command(CMD_STOP_AUTO)

def cmd_calibrate_height():
    return build_command(CMD_CALIBRATE_HEIGHT)

def cmd_start_dev_check():
    return build_command(CMD_START_DEV_CHECK)

def cmd_get_device_status():
    return build_command(CMD_GET_DEVICE_STATUS)

def cmd_drill_speed(rpm: int):
    # rpm is int16, big-endian
    return build_command(CMD_DRILL_SPEED, struct.pack(">h", rpm))

def cmd_vertical_speed(mm_per_s: float):
    # convert float mm/s to int8 (units of 0.1 mm/s)
    raw = int(round(mm_per_s * 10))
    return build_command(CMD_VERTICAL_SPEED, struct.pack("b", raw))

def cmd_storage_position(position: int):
    return build_command(CMD_STORAGE_POSITION, struct.pack("B", position))

def cmd_weigh_deep():
    return build_command(CMD_WEIGH_DEEP)

def cmd_weigh_surface():
    return build_command(CMD_WEIGH_SURFACE)

def cmd_get_weight_deep():
    return build_command(CMD_GET_WEIGHT_DEEP)

def cmd_get_weight_surface():
    return build_command(CMD_GET_WEIGHT_SURFACE)

def cmd_calibrate_0_deep():
    return build_command(CMD_CALIBRATE_0_DEEP)

def cmd_calibrate_x_deep(weight_g: float):
    return build_command(CMD_CALIBRATE_X_DEEP, struct.pack(">f", weight_g))

def cmd_calibrate_0_surface():
    return build_command(CMD_CALIBRATE_0_SURFACE)

def cmd_calibrate_x_surface(weight_g: float):
    return build_command(CMD_CALIBRATE_X_SURFACE, struct.pack(">f", weight_g))

def cmd_rock_open():
    return build_command(CMD_ROCK_OPEN)

def cmd_rock_close():
    return build_command(CMD_ROCK_CLOSE)

def cmd_sand_open():
    return build_command(CMD_SAND_OPEN)

def cmd_sand_close():
    return build_command(CMD_SAND_CLOSE)


# --- Response parsing ---

def parse_response(data: bytes):
    """
    Parse a complete raw message (including framing bytes).
    Returns a dict with parsed fields, or None if the message is invalid.

    Possible keys in the returned dict:
      'code'          : int  — the echoed command code (or 0 for NACK)
      'nack'          : bool — True if the drill refused the command
      'state'         : dict — present if code == CMD_STATE
      'weight'        : float — present if code == CMD_GET_WEIGHT_DEEP/SURFACE
      'adc_raw'       : int   — raw ADC value, present alongside 'weight'
      'device_status' : list of dicts — present if code == CMD_GET_DEVICE_STATUS
    """
    # Minimum valid message: STX + len + code + checksum + ETX = 5 bytes
    if len(data) < 5:
        return None
    if data[0] != STX or data[-1] != ETX:
        return None

    payload_length = data[1]
    payload = data[2 : 2 + payload_length]
    received_checksum = data[2 + payload_length]

    if len(payload) != payload_length:
        return None
    if checksum(payload) != received_checksum:
        return None

    code = payload[0]

    if code == 0x00:
        return {"code": 0, "nack": True}

    result = {"code": code, "nack": False}

    if code == CMD_STATE and len(payload) >= 9:
        # int16 height_mm, uint8 stepper_current, int16 rpm, uint8 temp, uint16 tray_angle, uint8 sw_state
        height, current, rpm, temp, tray_angle, sw_state = struct.unpack_from(">hBhBHB", payload, 1)
        result["state"] = {
            "height_mm":    height,
            "current_a":    current / 100.0,
            "rpm":          rpm,
            "temp_c":       temp,
            "tray_angle":   tray_angle,
            "sw_state":     sw_state,
            "sw_state_str": STATE_CODES.get(sw_state, f"Unknown (0x{sw_state:02X})"),
        }

    elif code in (CMD_GET_WEIGHT_DEEP, CMD_GET_WEIGHT_SURFACE) and len(payload) >= 9:
        weight, adc_raw = struct.unpack_from(">fI", payload, 1)
        result["weight"]  = weight
        result["adc_raw"] = adc_raw

    elif code == CMD_GET_DEVICE_STATUS and len(payload) >= 3:
        status_bits = struct.unpack_from(">H", payload, 1)[0]
        devices = []
        for i, name in enumerate(DEVICE_NAMES):
            devices.append({
                "name": name,
                "ok":   bool(status_bits & (1 << i)),
            })
        result["device_status"] = devices

    return result
