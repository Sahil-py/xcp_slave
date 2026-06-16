"""
XCP Master CAN test script.

Sends real CAN FD frames to the XCP slave on vcan0.
Run the slave first:
    python3 -m xcp_fd_slave.main --channel vcan0

Then in another terminal:
    candump -fd vcan0

Then run this script:
    python3 xcp_master_cantest.py
"""
import struct
import sys
import time
import can

CHANNEL    = "vcan0"
TX_ID      = 0x650   # master → slave
RX_ID      = 0x651   # slave → master
TIMEOUT    = 2.0      # seconds to wait for a response

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

ADDR_ENGINE_SPEED    = 0x00001000
ADDR_MAX_ENGINE_SPEED = 0x00008000
ADDR_COOLANT_TEMP    = 0x00002000

_t0 = time.monotonic()

def _ts():
    return f"[{time.monotonic()-_t0:7.3f}s]"

def send_recv(bus, data, label):
    raw = bytes(data)
    hex_tx = " ".join(f"{b:02X}" for b in raw)
    print(f"{_ts()} TX 0x{TX_ID:03X} [{len(raw):2d}]  {hex_tx:<48}  ; {label}")

    msg = can.Message(
        arbitration_id=TX_ID,
        data=raw,
        is_fd=True,
        bitrate_switch=True,
        is_extended_id=False,
    )
    bus.send(msg)

    deadline = time.monotonic() + TIMEOUT
    while time.monotonic() < deadline:
        rx = bus.recv(timeout=0.1)
        if rx is None:
            continue
        if rx.arbitration_id != RX_ID:
            continue
        hex_rx = " ".join(f"{b:02X}" for b in rx.data)
        print(f"{_ts()} RX 0x{RX_ID:03X} [{len(rx.data):2d}]  {hex_rx:<48}  ; {label} response")
        return bytes(rx.data)

    print(f"{_ts()} !! TIMEOUT waiting for response to: {label}")
    sys.exit(1)

def assert_pos(resp, label):
    if resp[0] != PID_RES:
        print(f"  !! ERROR: expected PID_RES 0xFF, got 0x{resp[0]:02X} (ERR code 0x{resp[1]:02X})")
        sys.exit(1)
    print(f"            >> {label}: OK")

def main():
    print()
    print("=" * 72)
    print("  XCP 1.3 Master — CAN FD test  (vcan0  TX=0x650 RX=0x651)")
    print("=" * 72)

    bus = can.Bus(channel=CHANNEL, interface="socketcan", fd=True)

    try:
        # -------------------------------------------------------- Step 1 CONNECT
        print("\n--- Step 1: CONNECT ---")
        resp = send_recv(bus, [CMD_CONNECT, 0x00], "CONNECT")
        assert_pos(resp, "CONNECT")
        resource  = resp[1]
        comm_mode = resp[2]
        max_cto   = resp[3]
        byte_order = "Motorola/MSB_FIRST" if comm_mode & 0x80 else "Intel"
        print(f"            >> RESOURCE=0x{resource:02X}  COMM_MODE_BASIC=0x{comm_mode:02X} ({byte_order})"
              f"  MAX_CTO={max_cto}")

        # ------------------------------------------------------ Step 2 GET_STATUS
        print("\n--- Step 2: GET_STATUS ---")
        resp = send_recv(bus, [CMD_GET_STATUS], "GET_STATUS")
        assert_pos(resp, "GET_STATUS")
        print(f"            >> SESSION_STATUS=0x{resp[1]:02X}  PROTECTION=0x{resp[2]:02X}")

        # -------------------------------------------------------- Step 3 GET_ID
        print("\n--- Step 3: GET_ID (ASCII ECU name) ---")
        resp = send_recv(bus, [CMD_GET_ID, 0x00], "GET_ID type=0x00")
        assert_pos(resp, "GET_ID")
        length = struct.unpack_from("<I", resp, 4)[0]
        print(f"            >> MODE=0x{resp[1]:02X}  LENGTH={length}")

        resp2 = send_recv(bus, [CMD_UPLOAD, length], f"UPLOAD {length} bytes (ECU name)")
        assert_pos(resp2, "UPLOAD ECU name")
        ecu_name = resp2[1:1+length].decode("ascii")
        print(f"            >> ECU name = \"{ecu_name}\"")

        # ---------------------------------------------------- Step 4 SET_MTA EngineSpeed
        print("\n--- Step 4: SET_MTA → EngineSpeed (0x00001000) ---")
        pkt = bytes([CMD_SET_MTA, 0x00, 0x00, 0x00]) + struct.pack("<I", ADDR_ENGINE_SPEED)
        resp = send_recv(bus, pkt, "SET_MTA EngineSpeed")
        assert_pos(resp, "SET_MTA")

        # ---------------------------------------------------- Step 5 UPLOAD EngineSpeed
        print("\n--- Step 5: UPLOAD EngineSpeed (4 bytes ULONG big-endian) ---")
        resp = send_recv(bus, [CMD_UPLOAD, 0x04], "UPLOAD 4")
        assert_pos(resp, "UPLOAD EngineSpeed")
        speed = struct.unpack_from(">I", resp, 1)[0]
        print(f"            >> EngineSpeed = {speed} rpm  (raw: {resp[1:5].hex().upper()})")

        # -------------------------------------------------- Step 6 SET_MTA MaxEngineSpeed
        print("\n--- Step 6: SET_MTA → MaxEngineSpeed (0x00008000) ---")
        pkt = bytes([CMD_SET_MTA, 0x00, 0x00, 0x00]) + struct.pack("<I", ADDR_MAX_ENGINE_SPEED)
        resp = send_recv(bus, pkt, "SET_MTA MaxEngineSpeed")
        assert_pos(resp, "SET_MTA")

        # -------------------------------------------------- Step 7 UPLOAD MaxEngineSpeed
        print("\n--- Step 7: UPLOAD MaxEngineSpeed (4 bytes FLOAT32 big-endian) ---")
        resp = send_recv(bus, [CMD_UPLOAD, 0x04], "UPLOAD 4")
        assert_pos(resp, "UPLOAD MaxEngineSpeed")
        max_spd_before = struct.unpack_from(">f", resp, 1)[0]
        print(f"            >> MaxEngineSpeed = {max_spd_before:.1f} rpm  (raw: {resp[1:5].hex().upper()})")

        # -------------------------------------------------- Step 8 DOWNLOAD new value
        print("\n--- Step 8: DOWNLOAD MaxEngineSpeed = 6500.0 rpm ---")
        # reset MTA first
        pkt = bytes([CMD_SET_MTA, 0x00, 0x00, 0x00]) + struct.pack("<I", ADDR_MAX_ENGINE_SPEED)
        send_recv(bus, pkt, "SET_MTA MaxEngineSpeed (reset)")
        payload = struct.pack(">f", 6500.0)
        pkt = bytes([CMD_DOWNLOAD, 0x04]) + payload
        resp = send_recv(bus, pkt, "DOWNLOAD 6500.0 rpm")
        assert_pos(resp, "DOWNLOAD")
        print(f"            >> Written: {payload.hex().upper()} = 6500.0 rpm")

        # -------------------------------------------------- Step 9 verify DOWNLOAD
        print("\n--- Step 9: UPLOAD MaxEngineSpeed (verify change) ---")
        pkt = bytes([CMD_SET_MTA, 0x00, 0x00, 0x00]) + struct.pack("<I", ADDR_MAX_ENGINE_SPEED)
        send_recv(bus, pkt, "SET_MTA MaxEngineSpeed (reset)")
        resp = send_recv(bus, [CMD_UPLOAD, 0x04], "UPLOAD 4")
        assert_pos(resp, "UPLOAD MaxEngineSpeed verify")
        max_spd_after = struct.unpack_from(">f", resp, 1)[0]
        print(f"            >> MaxEngineSpeed = {max_spd_after:.1f} rpm  "
              f"(was {max_spd_before:.1f}, delta={max_spd_after-max_spd_before:+.1f})")

        # -------------------------------------------------- Step 10 configure DAQ
        print("\n--- Step 10: Configure DAQ list 0 (EngineSpeed + CoolantTemp) ---")

        cmds = [
            ([CMD_FREE_DAQ],                                   "FREE_DAQ"),
            ([CMD_ALLOC_DAQ, 0x00, 0x01, 0x00],               "ALLOC_DAQ count=1"),
            ([CMD_ALLOC_ODT, 0x00, 0x00, 0x00, 0x01],         "ALLOC_ODT list=0 count=1"),
            ([CMD_ALLOC_ODT_ENTRY, 0x00, 0x00, 0x00, 0x00, 0x02], "ALLOC_ODT_ENTRY list=0 odt=0 entries=2"),
            ([CMD_SET_DAQ_PTR, 0x00, 0x00, 0x00, 0x00, 0x00], "SET_DAQ_PTR list=0 odt=0 entry=0"),
        ]
        for pkt, label in cmds:
            r = send_recv(bus, pkt, label)
            assert_pos(r, label)

        pkt = bytes([CMD_WRITE_DAQ, 0xFF, 0x04, 0x00]) + struct.pack("<I", ADDR_ENGINE_SPEED)
        r = send_recv(bus, pkt, "WRITE_DAQ entry0=EngineSpeed")
        assert_pos(r, "WRITE_DAQ EngineSpeed")

        pkt = bytes([CMD_WRITE_DAQ, 0xFF, 0x04, 0x00]) + struct.pack("<I", ADDR_COOLANT_TEMP)
        r = send_recv(bus, pkt, "WRITE_DAQ entry1=CoolantTemp")
        assert_pos(r, "WRITE_DAQ CoolantTemp")

        pkt = bytes([CMD_SET_DAQ_LIST_MODE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00])
        r = send_recv(bus, pkt, "SET_DAQ_LIST_MODE list=0 event=0 prescaler=1")
        assert_pos(r, "SET_DAQ_LIST_MODE")

        # -------------------------------------------------- Step 11 start DAQ
        print("\n--- Step 11: Start DAQ ---")
        r = send_recv(bus, [CMD_START_STOP_DAQ_LIST, 0x02, 0x00, 0x00], "START_STOP_DAQ_LIST SELECT list=0")
        assert_pos(r, "START_STOP_DAQ_LIST SELECT")

        r = send_recv(bus, [CMD_START_STOP_SYNCH, 0x01], "START_STOP_SYNCH start-selected")
        assert_pos(r, "START_STOP_SYNCH")
        print("            >> DAQ list 0 is now RUNNING — slave will send DTOs on 0x651")

        # -------------------------------------------------- Step 12 capture DTOs
        print("\n--- Step 12: Capture 10 DTO frames (10 ms interval from slave) ---")
        print(f"  {'#':<4}  {'Raw DTO bytes':<50}  EngineSpeed   CoolantTemp")
        print(f"  {'-'*4}  {'-'*50}  {'-'*12}  {'-'*11}")

        collected = 0
        deadline  = time.monotonic() + 5.0   # 5 s window

        while collected < 10 and time.monotonic() < deadline:
            rx = bus.recv(timeout=0.5)
            if rx is None:
                continue
            if rx.arbitration_id != RX_ID:
                continue
            d = bytes(rx.data)
            if d[0] in (PID_RES, PID_ERR):
                continue   # command response, not a DTO
            if len(d) < 10:
                continue

            hex_str = " ".join(f"{b:02X}" for b in d)
            speed   = struct.unpack_from(">I", d, 2)[0]
            coolant = struct.unpack_from(">i", d, 6)[0]
            print(f"  [{collected:2d}]  {hex_str:<50}  {speed:>7} rpm  {coolant/10.0:>7.1f} °C")
            collected += 1

        if collected < 10:
            print(f"  !! Only received {collected}/10 DTOs within timeout")

        # -------------------------------------------------- DISCONNECT
        print("\n--- Session complete: DISCONNECT ---")
        resp = send_recv(bus, [CMD_DISCONNECT], "DISCONNECT")
        assert_pos(resp, "DISCONNECT")

        print()
        print("=" * 72)
        print(f"  Done. {collected}/10 DTO frames captured.")
        print("=" * 72)

    finally:
        bus.shutdown()

if __name__ == "__main__":
    main()
