"""
XCP Master CAN test script — works with any python-can interface.

Examples:
    # vcan (Linux virtual CAN, FD):
    python3 xcp_master_cantest.py

    # PCAN USB, classic CAN:
    python3 xcp_master_cantest.py --interface pcan --channel PCAN_USBBUS1 --no-fd

    # PCAN USB, CAN FD:
    python3 xcp_master_cantest.py --interface pcan --channel PCAN_USBBUS1 --bitrate 500000 --data-bitrate 2000000

    # TI board mcan, classic CAN:
    python3 xcp_master_cantest.py --interface socketcan --channel main_mcan0 --no-fd
"""
import argparse
import struct
import sys
import time
import can

# XCP command PIDs
CMD_CONNECT             = 0xFF
CMD_DISCONNECT          = 0xFE
CMD_GET_STATUS          = 0xFD
CMD_GET_ID              = 0xFA
CMD_SET_MTA             = 0xF6
CMD_UPLOAD              = 0xF5
CMD_DOWNLOAD            = 0xF0
CMD_FREE_DAQ            = 0xD6
CMD_ALLOC_DAQ           = 0xD5
CMD_ALLOC_ODT           = 0xD4
CMD_ALLOC_ODT_ENTRY     = 0xD3
CMD_SET_DAQ_PTR         = 0xE2
CMD_WRITE_DAQ           = 0xE1
CMD_SET_DAQ_LIST_MODE   = 0xE0
CMD_START_STOP_DAQ_LIST = 0xDE
CMD_START_STOP_SYNCH    = 0xDD

PID_RES = 0xFF
PID_ERR = 0xFE

ADDR_ENGINE_SPEED     = 0x00001000
ADDR_MAX_ENGINE_SPEED = 0x00008000
ADDR_COOLANT_TEMP     = 0x00002000

_t0 = time.monotonic()


def _parse_args():
    p = argparse.ArgumentParser(
        description="XCP 1.3 master test — runs all 12 steps against the slave",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--interface", default="socketcan",
                   help="python-can interface (socketcan, pcan, kvaser, ...)")
    p.add_argument("--channel", default="vcan0",
                   help="CAN channel (vcan0, PCAN_USBBUS1, PCAN_USBBUS2, ...)")
    p.add_argument("--tx-id", default="0x650",
                   help="Master→Slave CAN ID (hex)")
    p.add_argument("--rx-id", default="0x651",
                   help="Slave→Master CAN ID (hex)")
    p.add_argument("--bitrate", type=int, default=500_000,
                   help="Nominal CAN bitrate (bps)")
    p.add_argument("--data-bitrate", type=int, default=2_000_000,
                   help="CAN FD data phase bitrate (bps), ignored with --no-fd")
    p.add_argument("--no-fd", action="store_true",
                   help="Use classic CAN frames (8-byte max) instead of CAN FD")
    p.add_argument("--timeout", type=float, default=2.0,
                   help="Response timeout per command (seconds)")
    return p.parse_args()


def _open_bus(args) -> can.Bus:
    kwargs = dict(interface=args.interface, channel=args.channel, bitrate=args.bitrate)
    if not args.no_fd:
        kwargs["fd"] = True
        kwargs["data_bitrate"] = args.data_bitrate
    return can.Bus(**kwargs)


def _ts():
    return f"[{time.monotonic() - _t0:7.3f}s]"


def send_recv(bus, data, label, tx_id, rx_id, fd, timeout):
    raw = bytes(data)
    hex_tx = " ".join(f"{b:02X}" for b in raw)
    print(f"{_ts()} TX 0x{tx_id:03X} [{len(raw):2d}]  {hex_tx:<48}  ; {label}")

    msg = can.Message(
        arbitration_id=tx_id,
        data=raw,
        is_extended_id=False,
        is_fd=fd,
        bitrate_switch=fd,
    )
    bus.send(msg)

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rx = bus.recv(timeout=0.1)
        if rx is None:
            continue
        if rx.arbitration_id != rx_id:
            continue
        hex_rx = " ".join(f"{b:02X}" for b in rx.data)
        print(f"{_ts()} RX 0x{rx_id:03X} [{len(rx.data):2d}]  {hex_rx:<48}  ; {label} response")
        return bytes(rx.data)

    print(f"{_ts()} !! TIMEOUT waiting for response to: {label}")
    sys.exit(1)


def assert_pos(resp, label):
    if resp[0] != PID_RES:
        err_code = resp[1] if len(resp) > 1 else 0xFF
        print(f"  !! ERROR: expected 0xFF (POS), got 0x{resp[0]:02X}  ERR_CODE=0x{err_code:02X}  label={label}")
        sys.exit(1)
    print(f"            >> {label}: OK")


def main():
    args = _parse_args()
    tx_id   = int(args.tx_id, 16)
    rx_id   = int(args.rx_id, 16)
    fd      = not args.no_fd
    timeout = args.timeout

    def sr(data, label):
        return send_recv(bus, data, label, tx_id, rx_id, fd, timeout)

    print()
    print("=" * 72)
    print(f"  XCP 1.3 Master Test")
    print(f"  Interface : {args.interface}  Channel: {args.channel}")
    print(f"  TX ID     : 0x{tx_id:03X}  RX ID: 0x{rx_id:03X}")
    print(f"  Mode      : {'CAN FD  bitrate=' + str(args.bitrate) + '  data-bitrate=' + str(args.data_bitrate) if fd else 'Classic CAN  bitrate=' + str(args.bitrate)}")
    print("=" * 72)

    bus = _open_bus(args)

    try:
        # -------------------------------------------------------- Step 1 CONNECT
        print("\n--- Step 1: CONNECT ---")
        resp = sr([CMD_CONNECT, 0x00], "CONNECT")
        assert_pos(resp, "CONNECT")
        resource   = resp[1]
        comm_mode  = resp[2]
        max_cto    = resp[3]
        byte_order = "Motorola/MSB_FIRST" if comm_mode & 0x80 else "Intel/LSB_FIRST"
        print(f"            >> RESOURCE=0x{resource:02X}  COMM_MODE=0x{comm_mode:02X} ({byte_order})  MAX_CTO={max_cto}")

        # -------------------------------------------------------- Step 2 GET_STATUS
        print("\n--- Step 2: GET_STATUS ---")
        resp = sr([CMD_GET_STATUS], "GET_STATUS")
        assert_pos(resp, "GET_STATUS")
        print(f"            >> SESSION_STATUS=0x{resp[1]:02X}  PROTECTION=0x{resp[2]:02X}")

        # -------------------------------------------------------- Step 3 GET_ID
        print("\n--- Step 3: GET_ID (ASCII ECU name) ---")
        resp = sr([CMD_GET_ID, 0x00], "GET_ID type=0x00")
        assert_pos(resp, "GET_ID")
        length = struct.unpack_from("<I", resp, 4)[0]
        print(f"            >> LENGTH={length}")
        resp2 = sr([CMD_UPLOAD, length], f"UPLOAD {length} bytes")
        assert_pos(resp2, "UPLOAD ECU name")
        ecu_name = resp2[1:1 + length].decode("ascii")
        print(f"            >> ECU name = \"{ecu_name}\"")

        # -------------------------------------------------------- Step 4 SET_MTA EngineSpeed
        print("\n--- Step 4: SET_MTA → EngineSpeed (0x00001000) ---")
        pkt = bytes([CMD_SET_MTA, 0x00, 0x00, 0x00]) + struct.pack("<I", ADDR_ENGINE_SPEED)
        assert_pos(sr(pkt, "SET_MTA EngineSpeed"), "SET_MTA")

        # -------------------------------------------------------- Step 5 UPLOAD EngineSpeed
        print("\n--- Step 5: UPLOAD EngineSpeed ---")
        resp = sr([CMD_UPLOAD, 0x04], "UPLOAD 4 bytes")
        assert_pos(resp, "UPLOAD EngineSpeed")
        speed = struct.unpack_from(">I", resp, 1)[0]
        print(f"            >> EngineSpeed = {speed} rpm  (raw: {resp[1:5].hex().upper()})")

        # -------------------------------------------------------- Step 6 SET_MTA MaxEngineSpeed
        print("\n--- Step 6: SET_MTA → MaxEngineSpeed (0x00008000) ---")
        pkt = bytes([CMD_SET_MTA, 0x00, 0x00, 0x00]) + struct.pack("<I", ADDR_MAX_ENGINE_SPEED)
        assert_pos(sr(pkt, "SET_MTA MaxEngineSpeed"), "SET_MTA")

        # -------------------------------------------------------- Step 7 UPLOAD MaxEngineSpeed
        print("\n--- Step 7: UPLOAD MaxEngineSpeed ---")
        resp = sr([CMD_UPLOAD, 0x04], "UPLOAD 4 bytes")
        assert_pos(resp, "UPLOAD MaxEngineSpeed")
        max_spd_before = struct.unpack_from(">f", resp, 1)[0]
        print(f"            >> MaxEngineSpeed = {max_spd_before:.1f} rpm  (raw: {resp[1:5].hex().upper()})")

        # -------------------------------------------------------- Step 8 DOWNLOAD new value
        print("\n--- Step 8: DOWNLOAD MaxEngineSpeed = 6500.0 rpm ---")
        pkt = bytes([CMD_SET_MTA, 0x00, 0x00, 0x00]) + struct.pack("<I", ADDR_MAX_ENGINE_SPEED)
        sr(pkt, "SET_MTA MaxEngineSpeed (reset)")
        payload = struct.pack(">f", 6500.0)
        resp = sr(bytes([CMD_DOWNLOAD, 0x04]) + payload, "DOWNLOAD 6500.0 rpm")
        assert_pos(resp, "DOWNLOAD")
        print(f"            >> Written {payload.hex().upper()} = 6500.0 rpm")

        # -------------------------------------------------------- Step 9 verify
        print("\n--- Step 9: UPLOAD MaxEngineSpeed (verify) ---")
        pkt = bytes([CMD_SET_MTA, 0x00, 0x00, 0x00]) + struct.pack("<I", ADDR_MAX_ENGINE_SPEED)
        sr(pkt, "SET_MTA MaxEngineSpeed (reset)")
        resp = sr([CMD_UPLOAD, 0x04], "UPLOAD 4 bytes")
        assert_pos(resp, "UPLOAD MaxEngineSpeed verify")
        max_spd_after = struct.unpack_from(">f", resp, 1)[0]
        print(f"            >> MaxEngineSpeed = {max_spd_after:.1f} rpm  (was {max_spd_before:.1f}, delta={max_spd_after - max_spd_before:+.1f})")

        # -------------------------------------------------------- Step 10 configure DAQ
        print("\n--- Step 10: Configure DAQ (EngineSpeed + CoolantTemp) ---")
        for pkt, label in [
            ([CMD_FREE_DAQ],                                       "FREE_DAQ"),
            ([CMD_ALLOC_DAQ, 0x00, 0x01, 0x00],                   "ALLOC_DAQ count=1"),
            ([CMD_ALLOC_ODT, 0x00, 0x00, 0x00, 0x01],             "ALLOC_ODT list=0 count=1"),
            ([CMD_ALLOC_ODT_ENTRY, 0x00, 0x00, 0x00, 0x00, 0x02], "ALLOC_ODT_ENTRY list=0 odt=0 entries=2"),
            ([CMD_SET_DAQ_PTR, 0x00, 0x00, 0x00, 0x00, 0x00],     "SET_DAQ_PTR list=0 odt=0 entry=0"),
        ]:
            assert_pos(sr(pkt, label), label)

        pkt = bytes([CMD_WRITE_DAQ, 0xFF, 0x04, 0x00]) + struct.pack("<I", ADDR_ENGINE_SPEED)
        assert_pos(sr(pkt, "WRITE_DAQ entry0=EngineSpeed"), "WRITE_DAQ EngineSpeed")

        pkt = bytes([CMD_WRITE_DAQ, 0xFF, 0x04, 0x00]) + struct.pack("<I", ADDR_COOLANT_TEMP)
        assert_pos(sr(pkt, "WRITE_DAQ entry1=CoolantTemp"), "WRITE_DAQ CoolantTemp")

        pkt = bytes([CMD_SET_DAQ_LIST_MODE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00])
        assert_pos(sr(pkt, "SET_DAQ_LIST_MODE list=0 prescaler=1"), "SET_DAQ_LIST_MODE")

        # -------------------------------------------------------- Step 11 start DAQ
        print("\n--- Step 11: Start DAQ ---")
        assert_pos(sr([CMD_START_STOP_DAQ_LIST, 0x02, 0x00, 0x00], "START_STOP_DAQ_LIST SELECT list=0"), "SELECT")
        assert_pos(sr([CMD_START_STOP_SYNCH, 0x01], "START_STOP_SYNCH start-selected"), "START_STOP_SYNCH")
        print("            >> DAQ RUNNING — slave sends DTOs on 0x651")

        # -------------------------------------------------------- Step 12 capture DTOs
        print("\n--- Step 12: Capture 10 DTO frames ---")
        print(f"  {'#':<4}  {'Raw DTO bytes':<50}  EngineSpeed   CoolantTemp")
        print(f"  {'-'*4}  {'-'*50}  {'-'*12}  {'-'*11}")

        collected = 0
        deadline  = time.monotonic() + 5.0

        while collected < 10 and time.monotonic() < deadline:
            rx = bus.recv(timeout=0.5)
            if rx is None:
                continue
            if rx.arbitration_id != rx_id:
                continue
            d = bytes(rx.data)
            if len(d) < 10 or d[0] in (PID_RES, PID_ERR):
                continue
            hex_str = " ".join(f"{b:02X}" for b in d)
            speed   = struct.unpack_from(">I", d, 2)[0]
            coolant = struct.unpack_from(">i", d, 6)[0]
            print(f"  [{collected:2d}]  {hex_str:<50}  {speed:>7} rpm  {coolant / 10.0:>7.1f} °C")
            collected += 1

        if collected < 10:
            print(f"  !! Only received {collected}/10 DTOs within timeout")

        # -------------------------------------------------------- DISCONNECT
        print("\n--- DISCONNECT ---")
        assert_pos(sr([CMD_DISCONNECT], "DISCONNECT"), "DISCONNECT")

        print()
        print("=" * 72)
        print(f"  Done. {collected}/10 DTO frames captured.")
        print("=" * 72)

    finally:
        bus.shutdown()


if __name__ == "__main__":
    main()
