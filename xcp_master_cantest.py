"""
XCP Master CAN test script.

The --no-fd flag only controls how the CAN bus is opened (classic vs FD frames).
ALL protocol decisions (UPLOAD chunk size, DAQ layout, DTO decode) are driven by
MAX_CTO and MAX_DTO reported by the slave in the CONNECT response — so the master
adapts automatically regardless of slave configuration.

Examples:
    # vcan, CAN FD (default):
    python3 xcp_master_cantest.py

    # PCAN USB, classic CAN:
    python3 xcp_master_cantest.py --interface pcan --channel PCAN_USBBUS1 --no-fd

    # PCAN USB, CAN FD:
    python3 xcp_master_cantest.py --interface pcan --channel PCAN_USBBUS1

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
        description="XCP 1.3 master test — all 12 steps against the slave",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--interface",    default="socketcan",
                   help="python-can interface (socketcan, pcan, ...)")
    p.add_argument("--channel",      default="vcan0",
                   help="CAN channel (vcan0, PCAN_USBBUS1, ...)")
    p.add_argument("--tx-id",        default="0x650",
                   help="Master→Slave CAN ID (hex)")
    p.add_argument("--rx-id",        default="0x651",
                   help="Slave→Master CAN ID (hex)")
    p.add_argument("--bitrate",      type=int, default=500_000,
                   help="Nominal CAN bitrate bps")
    p.add_argument("--data-bitrate", type=int, default=2_000_000,
                   help="CAN FD data bitrate bps (ignored with --no-fd)")
    p.add_argument("--no-fd",        action="store_true",
                   help="Open bus as classic CAN (8-byte frames). "
                        "Protocol decisions are still driven by slave CONNECT response.")
    p.add_argument("--timeout",      type=float, default=2.0,
                   help="Per-command response timeout (seconds)")
    return p.parse_args()


def _open_bus(args) -> can.Bus:
    kwargs = dict(interface=args.interface, channel=args.channel, bitrate=args.bitrate)
    if not args.no_fd:
        kwargs["fd"] = True
        kwargs["data_bitrate"] = args.data_bitrate
    return can.Bus(**kwargs)


def _ts():
    return f"[{time.monotonic() - _t0:7.3f}s]"


def send_recv(bus, data, label, tx_id, rx_id, use_fd, timeout):
    raw = bytes(data)
    hex_tx = " ".join(f"{b:02X}" for b in raw)
    print(f"{_ts()} TX 0x{tx_id:03X} [{len(raw):2d}]  {hex_tx:<48}  ; {label}")

    msg = can.Message(
        arbitration_id=tx_id,
        data=raw,
        is_extended_id=False,
        is_fd=use_fd,
        bitrate_switch=use_fd,
    )
    bus.send(msg)

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rx = bus.recv(timeout=0.1)
        if rx is None or rx.arbitration_id != rx_id:
            continue
        hex_rx = " ".join(f"{b:02X}" for b in rx.data)
        print(f"{_ts()} RX 0x{rx_id:03X} [{len(rx.data):2d}]  {hex_rx:<48}  ; {label} response")
        return bytes(rx.data)

    print(f"{_ts()} !! TIMEOUT waiting for: {label}")
    sys.exit(1)


def assert_pos(resp, label):
    if resp[0] != PID_RES:
        err = resp[1] if len(resp) > 1 else 0xFF
        print(f"  !! NEGATIVE RESPONSE 0x{resp[0]:02X}  ERR=0x{err:02X}  ({label})")
        sys.exit(1)
    print(f"            >> {label}: OK")


def main():
    args    = _parse_args()
    tx_id   = int(args.tx_id, 16)
    rx_id   = int(args.rx_id, 16)
    use_fd  = not args.no_fd
    timeout = args.timeout

    # Convenience wrapper — use_fd is fixed for the whole session (physical layer only)
    def sr(data, label):
        return send_recv(bus, data, label, tx_id, rx_id, use_fd, timeout)

    print()
    print("=" * 72)
    print(f"  XCP 1.3 Master — {args.interface}/{args.channel}")
    print(f"  Bus: {'CAN FD  data-bitrate=' + str(args.data_bitrate) if use_fd else 'Classic CAN'}"
          f"  nominal={args.bitrate}  TX=0x{tx_id:03X}  RX=0x{rx_id:03X}")
    print("=" * 72)

    bus = _open_bus(args)

    try:
        # ---------------------------------------------------------------- Step 1
        print("\n--- Step 1: CONNECT ---")
        resp = sr([CMD_CONNECT, 0x00], "CONNECT")
        assert_pos(resp, "CONNECT")
        resource   = resp[1]
        comm_mode  = resp[2]
        # Read slave-reported limits — all protocol decisions below use these
        max_cto    = resp[3]
        max_dto    = struct.unpack_from("<H", resp, 4)[0]
        upload_chunk  = max_cto - 1   # max data bytes per UPLOAD
        dto_data_max  = max_dto - 2   # max data bytes per DTO (after 2-byte header)
        # DAQ layout: put both 4-byte signals in one ODT only if they fit
        single_odt = (dto_data_max >= 8)
        byte_order = "Motorola/MSB_FIRST" if comm_mode & 0x80 else "Intel"
        print(f"            >> RESOURCE=0x{resource:02X}  COMM_MODE=0x{comm_mode:02X} ({byte_order})")
        print(f"            >> MAX_CTO={max_cto}  MAX_DTO={max_dto}  "
              f"upload_chunk={upload_chunk}  dto_data_max={dto_data_max}")
        print(f"            >> DAQ layout: {'1 ODT × 2 signals (FD)' if single_odt else '2 ODTs × 1 signal (classic CAN)'}")

        # ---------------------------------------------------------------- Step 2
        print("\n--- Step 2: GET_STATUS ---")
        resp = sr([CMD_GET_STATUS], "GET_STATUS")
        assert_pos(resp, "GET_STATUS")
        print(f"            >> SESSION_STATUS=0x{resp[1]:02X}  PROTECTION=0x{resp[2]:02X}")

        # ---------------------------------------------------------------- Step 3
        print("\n--- Step 3: GET_ID (ASCII ECU name) ---")
        resp = sr([CMD_GET_ID, 0x00], "GET_ID type=0x00")
        assert_pos(resp, "GET_ID")
        length = struct.unpack_from("<I", resp, 4)[0]
        print(f"            >> LENGTH={length}  reading in chunks of {upload_chunk}")

        name_bytes = b""
        remaining  = length
        while remaining > 0:
            n = min(remaining, upload_chunk)
            r = sr([CMD_UPLOAD, n], f"UPLOAD {n}/{remaining} bytes")
            assert_pos(r, "UPLOAD chunk")
            name_bytes += r[1:1 + n]
            remaining  -= n
        ecu_name = name_bytes.decode("ascii")
        print(f"            >> ECU name = \"{ecu_name}\"")

        # ---------------------------------------------------------------- Step 4
        print("\n--- Step 4: SET_MTA → EngineSpeed ---")
        pkt = bytes([CMD_SET_MTA, 0x00, 0x00, 0x00]) + struct.pack("<I", ADDR_ENGINE_SPEED)
        assert_pos(sr(pkt, f"SET_MTA 0x{ADDR_ENGINE_SPEED:08X}"), "SET_MTA")

        # ---------------------------------------------------------------- Step 5
        print("\n--- Step 5: UPLOAD EngineSpeed (ULONG big-endian) ---")
        resp = sr([CMD_UPLOAD, 0x04], "UPLOAD 4 bytes")
        assert_pos(resp, "UPLOAD EngineSpeed")
        speed = struct.unpack_from(">I", resp, 1)[0]
        print(f"            >> EngineSpeed = {speed} rpm  ({resp[1:5].hex().upper()})")

        # ---------------------------------------------------------------- Step 6
        print("\n--- Step 6: SET_MTA → MaxEngineSpeed ---")
        pkt = bytes([CMD_SET_MTA, 0x00, 0x00, 0x00]) + struct.pack("<I", ADDR_MAX_ENGINE_SPEED)
        assert_pos(sr(pkt, f"SET_MTA 0x{ADDR_MAX_ENGINE_SPEED:08X}"), "SET_MTA")

        # ---------------------------------------------------------------- Step 7
        print("\n--- Step 7: UPLOAD MaxEngineSpeed (FLOAT32 big-endian) ---")
        resp = sr([CMD_UPLOAD, 0x04], "UPLOAD 4 bytes")
        assert_pos(resp, "UPLOAD MaxEngineSpeed")
        max_spd_before = struct.unpack_from(">f", resp, 1)[0]
        print(f"            >> MaxEngineSpeed = {max_spd_before:.1f} rpm  ({resp[1:5].hex().upper()})")

        # ---------------------------------------------------------------- Step 8
        print("\n--- Step 8: DOWNLOAD MaxEngineSpeed = 6500.0 rpm ---")
        sr(bytes([CMD_SET_MTA, 0x00, 0x00, 0x00]) + struct.pack("<I", ADDR_MAX_ENGINE_SPEED),
           "SET_MTA (reset)")
        payload = struct.pack(">f", 6500.0)
        assert_pos(sr(bytes([CMD_DOWNLOAD, 0x04]) + payload, "DOWNLOAD 6500.0"), "DOWNLOAD")
        print(f"            >> Written {payload.hex().upper()} = 6500.0 rpm")

        # ---------------------------------------------------------------- Step 9
        print("\n--- Step 9: UPLOAD MaxEngineSpeed (verify) ---")
        sr(bytes([CMD_SET_MTA, 0x00, 0x00, 0x00]) + struct.pack("<I", ADDR_MAX_ENGINE_SPEED),
           "SET_MTA (reset)")
        resp = sr([CMD_UPLOAD, 0x04], "UPLOAD 4 bytes")
        assert_pos(resp, "UPLOAD verify")
        max_spd_after = struct.unpack_from(">f", resp, 1)[0]
        print(f"            >> MaxEngineSpeed = {max_spd_after:.1f}  (was {max_spd_before:.1f}, "
              f"delta={max_spd_after - max_spd_before:+.1f})")

        # ---------------------------------------------------------------- Step 10
        print("\n--- Step 10: Configure DAQ ---")
        assert_pos(sr([CMD_FREE_DAQ], "FREE_DAQ"), "FREE_DAQ")
        assert_pos(sr([CMD_ALLOC_DAQ, 0x00, 0x01, 0x00], "ALLOC_DAQ count=1"), "ALLOC_DAQ")

        if single_odt:
            # CAN FD: both signals in ODT 0 → one DTO = 2 header + 4 + 4 = 10 bytes
            print(f"  [single ODT: 2 entries, DTO={2+8} bytes ≤ MAX_DTO={max_dto}]")
            assert_pos(sr([CMD_ALLOC_ODT, 0x00, 0x00, 0x00, 0x01],
                          "ALLOC_ODT count=1"), "ALLOC_ODT")
            assert_pos(sr([CMD_ALLOC_ODT_ENTRY, 0x00, 0x00, 0x00, 0x00, 0x02],
                          "ALLOC_ODT_ENTRY odt=0 entries=2"), "ALLOC_ODT_ENTRY")
            assert_pos(sr([CMD_SET_DAQ_PTR, 0x00, 0x00, 0x00, 0x00, 0x00],
                          "SET_DAQ_PTR odt=0 entry=0"), "SET_DAQ_PTR")
            pkt = bytes([CMD_WRITE_DAQ, 0xFF, 0x04, 0x00]) + struct.pack("<I", ADDR_ENGINE_SPEED)
            assert_pos(sr(pkt, "WRITE_DAQ entry0=EngineSpeed"), "WRITE_DAQ")
            pkt = bytes([CMD_WRITE_DAQ, 0xFF, 0x04, 0x00]) + struct.pack("<I", ADDR_COOLANT_TEMP)
            assert_pos(sr(pkt, "WRITE_DAQ entry1=CoolantTemp"), "WRITE_DAQ")
        else:
            # Classic CAN: one signal per ODT → two DTOs = 2+4 = 6 bytes each ≤ MAX_DTO=8
            print(f"  [two ODTs: 1 entry each, DTO={2+4} bytes ≤ MAX_DTO={max_dto}]")
            assert_pos(sr([CMD_ALLOC_ODT, 0x00, 0x00, 0x00, 0x02],
                          "ALLOC_ODT count=2"), "ALLOC_ODT")
            assert_pos(sr([CMD_ALLOC_ODT_ENTRY, 0x00, 0x00, 0x00, 0x00, 0x01],
                          "ALLOC_ODT_ENTRY odt=0 entries=1"), "ALLOC_ODT_ENTRY")
            assert_pos(sr([CMD_ALLOC_ODT_ENTRY, 0x00, 0x00, 0x00, 0x01, 0x01],
                          "ALLOC_ODT_ENTRY odt=1 entries=1"), "ALLOC_ODT_ENTRY")
            for odt_idx, addr, sig in [(0, ADDR_ENGINE_SPEED, "EngineSpeed"),
                                        (1, ADDR_COOLANT_TEMP, "CoolantTemp")]:
                assert_pos(sr([CMD_SET_DAQ_PTR, 0x00, 0x00, 0x00, odt_idx, 0x00],
                               f"SET_DAQ_PTR odt={odt_idx}"), "SET_DAQ_PTR")
                pkt = bytes([CMD_WRITE_DAQ, 0xFF, 0x04, 0x00]) + struct.pack("<I", addr)
                assert_pos(sr(pkt, f"WRITE_DAQ odt{odt_idx}={sig}"), "WRITE_DAQ")

        pkt = bytes([CMD_SET_DAQ_LIST_MODE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00])
        assert_pos(sr(pkt, "SET_DAQ_LIST_MODE list=0 prescaler=1"), "SET_DAQ_LIST_MODE")

        # ---------------------------------------------------------------- Step 11
        print("\n--- Step 11: Start DAQ ---")
        assert_pos(sr([CMD_START_STOP_DAQ_LIST, 0x02, 0x00, 0x00],
                      "START_STOP_DAQ_LIST SELECT list=0"), "SELECT")
        assert_pos(sr([CMD_START_STOP_SYNCH, 0x01],
                      "START_STOP_SYNCH start-selected"), "START_STOP_SYNCH")
        print(f"            >> DAQ RUNNING — slave sends DTOs on 0x{rx_id:03X}")

        # ---------------------------------------------------------------- Step 12
        print("\n--- Step 12: Capture 10 DTO samples ---")

        def is_dto(d):
            return len(d) >= 6 and d[0] not in (PID_RES, PID_ERR)

        collected = 0
        deadline  = time.monotonic() + 5.0

        if single_odt:
            print(f"  {'#':<4}  {'Raw DTO':<52}  EngineSpeed   CoolantTemp")
            print(f"  {'-'*4}  {'-'*52}  {'-'*12}  {'-'*11}")
            while collected < 10 and time.monotonic() < deadline:
                rx = bus.recv(timeout=0.5)
                if rx is None or rx.arbitration_id != rx_id:
                    continue
                d = bytes(rx.data)
                if not is_dto(d) or len(d) < 10:
                    continue
                speed   = struct.unpack_from(">I", d, 2)[0]
                coolant = struct.unpack_from(">i", d, 6)[0]
                hex_str = " ".join(f"{b:02X}" for b in d)
                print(f"  [{collected:2d}]  {hex_str:<52}  {speed:>7} rpm  {coolant/10.0:>7.1f} °C")
                collected += 1
        else:
            print(f"  {'#':<4}  {'DTO0 [EngineSpeed]':<26}  {'DTO1 [CoolantTemp]':<26}  Speed    Coolant")
            print(f"  {'-'*4}  {'-'*26}  {'-'*26}  {'-'*7}  {'-'*7}")
            pending = {}
            while collected < 10 and time.monotonic() < deadline:
                rx = bus.recv(timeout=0.5)
                if rx is None or rx.arbitration_id != rx_id:
                    continue
                d = bytes(rx.data)
                if not is_dto(d):
                    continue
                pending[d[0]] = d          # key = ODT number
                if 0 in pending and 1 in pending:
                    d0, d1  = pending.pop(0), pending.pop(1)
                    speed   = struct.unpack_from(">I", d0, 2)[0]
                    coolant = struct.unpack_from(">i", d1, 2)[0]
                    h0 = " ".join(f"{b:02X}" for b in d0)
                    h1 = " ".join(f"{b:02X}" for b in d1)
                    print(f"  [{collected:2d}]  {h0:<26}  {h1:<26}  {speed:>5} rpm  {coolant/10.0:>5.1f} °C")
                    collected += 1

        if collected < 10:
            print(f"  !! Only captured {collected}/10 samples within timeout")

        # ---------------------------------------------------------------- Disconnect
        print("\n--- DISCONNECT ---")
        assert_pos(sr([CMD_DISCONNECT], "DISCONNECT"), "DISCONNECT")

        print()
        print("=" * 72)
        print(f"  Done. {collected}/10 DTO samples captured.")
        print("=" * 72)

    finally:
        bus.shutdown()


if __name__ == "__main__":
    main()
