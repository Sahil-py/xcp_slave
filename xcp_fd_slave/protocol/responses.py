"""XCP response packet builders — ASAM XCP Part 2, Section 2.3."""
import struct

# Packet type identifiers (first byte of every XCP packet from slave)
PID_RES  = 0xFF  # Positive response
PID_ERR  = 0xFE  # Error response
PID_EV   = 0xFD  # Event packet
PID_SERV = 0xFC  # Service request


def build_res(payload: bytes = b"") -> bytes:
    return bytes([PID_RES]) + bytes(payload)


def build_err(error_code: int) -> bytes:
    return bytes([PID_ERR, error_code])


def build_ev(event_code: int, result: int = 0x00) -> bytes:
    return bytes([PID_EV, event_code, result, 0x00])
