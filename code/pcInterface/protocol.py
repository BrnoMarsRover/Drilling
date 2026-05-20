"""
protocol.py

Handles building and parsing of UART messages according to the drill protocol spec.

Message format:
  0x02 | length | payload... | checksum | 0x03

Checksum: sum complement of payload bytes (0x100 - (sum(payload) & 0xFF)) & 0xFF
"""

import struct

# --- Message framing bytes ---
STX = 0x02
ETX = 0x03

# --- Command codes ---
CMD_RESTART          = 0x01
CMD_STATE            = 0x02
CMD_DRILL_AUTO       = 0x03
CMD_STOP_AUTO        = 0x04
CMD_CALIBRATE_HEIGHT = 0x05
CMD_DRILL_SPEED      = 0x20
CMD_VERTICAL_SPEED   = 0x21
CMD_STORAGE_POSITION = 0x22
CMD_WEIGH_DEEP       = 0x40
CMD_WEIGH_SURFACE    = 0x41
CMD_GET_WEIGHT_DEEP  = 0x42
CMD_GET_WEIGHT_SURFACE = 0x43
CMD_ROCK_OPEN        = 0x50
CMD_ROCK_CLOSE       = 0x51
CMD_SAND_OPEN        = 0x52
CMD_SAND_CLOSE       = 0x53

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

# Commands that expect a data payload in the response (beyond just the echo code)
RESPONSE_HAS_DATA = {
    CMD_STATE,
    CMD_GET_WEIGHT_DEEP,
    CMD_GET_WEIGHT_SURFACE,
}


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
    return build_command(CMD_CALIBRATE_HEIGHT, struct.pack("B", 0))

def cmd_drill_speed(rpm: int):
    # rpm is int16, big-endian
    return build_command(CMD_DRILL_SPEED, struct.pack(">h", rpm))

def cmd_vertical_speed(mm_per_s: int):
    # mm/s is int8; positive = down, negative = up
    return build_command(CMD_VERTICAL_SPEED, struct.pack("b", mm_per_s))

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
      'code'      : int  — the echoed command code (or 0 for NACK)
      'nack'      : bool — True if the drill refused the command
      'state'     : dict — present if code == CMD_STATE
      'weight'    : float — present if code == CMD_GET_WEIGHT_DEEP/SURFACE
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

    if code == CMD_STATE and len(payload) >= 7:
        # uint8 height, int16 rpm, uint8 temp, uint16 tray_angle, uint8 sw_state
        height, rpm, temp, tray_angle, sw_state = struct.unpack_from(">bhBHB", payload, 1)
        result["state"] = {
            "height_cm":   height,
            "rpm":         rpm,
            "temp_c":      temp,
            "tray_angle":  tray_angle,
            "sw_state":    sw_state,
            "sw_state_str": STATE_CODES.get(sw_state, f"Unknown (0x{sw_state:02X})"),
        }

    elif code in (CMD_GET_WEIGHT_DEEP, CMD_GET_WEIGHT_SURFACE) and len(payload) >= 5:
        weight = struct.unpack_from(">f", payload, 1)[0]
        result["weight"] = weight

    return result
