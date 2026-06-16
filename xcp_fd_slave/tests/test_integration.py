"""
Integration test: simulates a complete XCP master session against the slave.

Each step prints raw bytes (as a CAN trace tool would show) plus human-readable
decode.  No CAN hardware required — dispatcher called directly.
"""
import struct
import time
import pytest

from xcp_fd_slave.protocol.session import XcpSession
from xcp_fd_slave.memory.memory_map import (
    MemoryMap,
    ADDR_ENGINE_SPEED, ADDR_MAX_ENGINE_SPEED, ADDR_COOLANT_TEMP,
)
from xcp_fd_slave.protocol.dispatcher import XcpDispatcher
from xcp_fd_slave.protocol import commands as cmd, responses as rsp

_t0 = time.monotonic()

def _trace(direction, label, data, decode=""):
    elapsed = time.monotonic() - _t0
    hex_str = " ".join(f"{b:02X}" for b in data)
    line = f"  [{elapsed:7.3f}s] {direction}  [{len(data):2d}]  {hex_str:<48}  ; {label}"
    if decode:
        line += f"\n            >> {decode}"
    print(line)

def _tx(label, data, decode=""):
    _trace("TX→SLAVE", label, data, decode)

def _rx(label, data, decode=""):
    _trace("RX←SLAVE", label, data, decode)

def _set_mta(slave, addr):
    pkt = bytes([cmd.CMD_SET_MTA, 0x00, 0x00, 0x00]) + struct.pack("<I", addr)
    r = slave.process(pkt)
    assert r[0] == rsp.PID_RES
    return r


def test_master_session_sequence():
    session = XcpSession()
    memory  = MemoryMap()
    slave   = XcpDispatcher(session, memory)

    print()
    print("=" * 72)
    print("  XCP 1.3 Master Session — CANdoit_Test ECU  (CAN FD 0x650/0x651)")
    print("=" * 72)

    # ------------------------------------------------------------------ Step 1
    print("\n--- Step 1: CONNECT ---")
    req = bytes([cmd.CMD_CONNECT, 0x00])
    _tx("CONNECT", req, "mode=0x00 (normal)")
    resp = slave.process(req)
    assert resp[0] == rsp.PID_RES
    resource  = resp[1]
    comm_mode = resp[2]
    max_cto   = resp[3]
    max_dto   = struct.unpack_from("<H", resp, 4)[0]
    byte_order = "Motorola/MSB_FIRST" if comm_mode & 0x80 else "Intel/MSB_LAST"
    _rx("CONNECT pos.response", resp,
        f"RESOURCE=0x{resource:02X} (CAL/PAG={bool(resource&1)}, DAQ={bool(resource&4)})  "
        f"COMM_MODE_BASIC=0x{comm_mode:02X} ({byte_order}, AG=BYTE)  "
        f"MAX_CTO={max_cto}  MAX_DTO={max_dto}  "
        f"PROTO_VER=0x{resp[6]:02X}  XPORT_VER=0x{resp[7]:02X}")
    assert session.connected
    assert comm_mode & 0x80, "Expected Motorola byte order per A2L MSB_FIRST"

    # ------------------------------------------------------------------ Step 2
    print("\n--- Step 2: GET_STATUS ---")
    req = bytes([cmd.CMD_GET_STATUS])
    _tx("GET_STATUS", req)
    resp = slave.process(req)
    assert resp[0] == rsp.PID_RES
    status = resp[1]
    _rx("GET_STATUS pos.response", resp,
        f"SESSION_STATUS=0x{status:02X} "
        f"(DAQ_RUNNING={bool(status&0x08)}, RESUME={bool(status&0x02)})  "
        f"PROTECTION=0x{resp[2]:02X} (all unlocked)")
    assert resp[2] == 0x00

    # ------------------------------------------------------------------ Step 3
    print("\n--- Step 3: GET_ID (ASCII ECU name) ---")
    req = bytes([cmd.CMD_GET_ID, 0x00])
    _tx("GET_ID", req, "type=0x00 (ASCII)")
    resp = slave.process(req)
    assert resp[0] == rsp.PID_RES
    length = struct.unpack_from("<I", resp, 4)[0]
    _rx("GET_ID pos.response", resp,
        f"MODE=0x{resp[1]:02X} (not compressed, data via UPLOAD)  LENGTH={length}")
    req2 = bytes([cmd.CMD_UPLOAD, length])
    _tx("UPLOAD", req2, f"NR={length} (read ECU name from MTA)")
    resp2 = slave.process(req2)
    assert resp2[0] == rsp.PID_RES
    ecu_name = resp2[1: 1 + length].decode("ascii")
    _rx("UPLOAD pos.response", resp2, f'ECU name = "{ecu_name}"')
    assert ecu_name == "CANdoit_Test"

    # ------------------------------------------------------------------ Step 4
    print("\n--- Step 4: SET_MTA → EngineSpeed (0x00001000) ---")
    req = bytes([cmd.CMD_SET_MTA, 0x00, 0x00, 0x00]) + struct.pack("<I", ADDR_ENGINE_SPEED)
    _tx("SET_MTA", req, f"addr=0x{ADDR_ENGINE_SPEED:08X}")
    resp = slave.process(req)
    assert resp[0] == rsp.PID_RES
    _rx("SET_MTA pos.response", resp, "MTA set to EngineSpeed")

    # ------------------------------------------------------------------ Step 5
    print("\n--- Step 5: UPLOAD EngineSpeed ---")
    req = bytes([cmd.CMD_UPLOAD, 0x04])
    _tx("UPLOAD", req, "NR=4 (ULONG, big-endian)")
    resp = slave.process(req)
    assert resp[0] == rsp.PID_RES
    speed = struct.unpack_from(">I", resp, 1)[0]
    _rx("UPLOAD pos.response", resp,
        f"EngineSpeed = {speed} rpm  (raw big-endian: {resp[1:5].hex().upper()})")
    assert speed == 800

    # ------------------------------------------------------------------ Step 6
    print("\n--- Step 6: SET_MTA → MaxEngineSpeed (0x00008000) ---")
    req = bytes([cmd.CMD_SET_MTA, 0x00, 0x00, 0x00]) + struct.pack("<I", ADDR_MAX_ENGINE_SPEED)
    _tx("SET_MTA", req, f"addr=0x{ADDR_MAX_ENGINE_SPEED:08X}")
    resp = slave.process(req)
    assert resp[0] == rsp.PID_RES
    _rx("SET_MTA pos.response", resp, "MTA set to MaxEngineSpeed")

    # ------------------------------------------------------------------ Step 7
    print("\n--- Step 7: UPLOAD MaxEngineSpeed ---")
    req = bytes([cmd.CMD_UPLOAD, 0x04])
    _tx("UPLOAD", req, "NR=4 (FLOAT32_IEEE, big-endian)")
    resp = slave.process(req)
    assert resp[0] == rsp.PID_RES
    max_spd_before = struct.unpack_from(">f", resp, 1)[0]
    _rx("UPLOAD pos.response", resp,
        f"MaxEngineSpeed = {max_spd_before:.1f} rpm  (raw: {resp[1:5].hex().upper()})")
    assert abs(max_spd_before - 7500.0) < 0.1

    # ------------------------------------------------------------------ Step 8
    print("\n--- Step 8: DOWNLOAD MaxEngineSpeed = 6500.0 rpm ---")
    new_val = 6500.0
    payload = struct.pack(">f", new_val)
    _set_mta(slave, ADDR_MAX_ENGINE_SPEED)   # reset MTA (was advanced by UPLOAD)
    req = bytes([cmd.CMD_DOWNLOAD, 0x04]) + payload
    _tx("DOWNLOAD", req,
        f"NR=4  value={new_val:.1f} rpm  (big-endian bytes: {payload.hex().upper()})")
    resp = slave.process(req)
    assert resp[0] == rsp.PID_RES
    _rx("DOWNLOAD pos.response", resp, "calibration written to ECU memory")

    # ------------------------------------------------------------------ Step 9
    print("\n--- Step 9: UPLOAD MaxEngineSpeed (verify DOWNLOAD) ---")
    _set_mta(slave, ADDR_MAX_ENGINE_SPEED)
    req = bytes([cmd.CMD_UPLOAD, 0x04])
    _tx("UPLOAD", req, "NR=4 (verify calibration change)")
    resp = slave.process(req)
    assert resp[0] == rsp.PID_RES
    max_spd_after = struct.unpack_from(">f", resp, 1)[0]
    _rx("UPLOAD pos.response", resp,
        f"MaxEngineSpeed = {max_spd_after:.1f} rpm  "
        f"(was {max_spd_before:.1f}, delta = {max_spd_after - max_spd_before:+.1f})")
    assert abs(max_spd_after - new_val) < 0.001

    # ----------------------------------------------------------------- Step 10
    print("\n--- Step 10: Configure DAQ (EngineSpeed + CoolantTemp) ---")
    for label, pkt, decode in [
        ("FREE_DAQ",
         bytes([cmd.CMD_FREE_DAQ]),
         "release all dynamic DAQ lists"),
        ("ALLOC_DAQ",
         bytes([cmd.CMD_ALLOC_DAQ, 0x00, 0x01, 0x00]),
         "COUNT=1 DAQ list"),
        ("ALLOC_ODT",
         bytes([cmd.CMD_ALLOC_ODT, 0x00, 0x00, 0x00, 0x01]),
         "DAQ_LIST=0  COUNT=1 ODT"),
        ("ALLOC_ODT_ENTRY",
         bytes([cmd.CMD_ALLOC_ODT_ENTRY, 0x00, 0x00, 0x00, 0x00, 0x02]),
         "DAQ_LIST=0  ODT=0  ENTRIES=2"),
        ("SET_DAQ_PTR",
         bytes([cmd.CMD_SET_DAQ_PTR, 0x00, 0x00, 0x00, 0x00, 0x00]),
         "DAQ_LIST=0  ODT=0  ENTRY=0"),
    ]:
        _tx(label, pkt, decode)
        r = slave.process(pkt)
        assert r[0] == rsp.PID_RES
        _rx(f"{label} pos.response", r, "OK")

    req = bytes([cmd.CMD_WRITE_DAQ, 0xFF, 0x04, 0x00]) + struct.pack("<I", ADDR_ENGINE_SPEED)
    _tx("WRITE_DAQ", req,
        f"BIT_OFFSET=0xFF  SIZE=4  ADDR=0x{ADDR_ENGINE_SPEED:08X} (EngineSpeed ULONG)")
    r = slave.process(req)
    assert r[0] == rsp.PID_RES
    _rx("WRITE_DAQ pos.response", r, "entry 0 → EngineSpeed; ptr→entry 1")

    req = bytes([cmd.CMD_WRITE_DAQ, 0xFF, 0x04, 0x00]) + struct.pack("<I", ADDR_COOLANT_TEMP)
    _tx("WRITE_DAQ", req,
        f"BIT_OFFSET=0xFF  SIZE=4  ADDR=0x{ADDR_COOLANT_TEMP:08X} (CoolantTemp SLONG)")
    r = slave.process(req)
    assert r[0] == rsp.PID_RES
    _rx("WRITE_DAQ pos.response", r, "entry 1 → CoolantTemp; ptr→entry 2")

    req = bytes([cmd.CMD_SET_DAQ_LIST_MODE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00])
    _tx("SET_DAQ_LIST_MODE", req,
        "DAQ_LIST=0  MODE=0x00  EVENT_CH=0  PRESCALER=1  PRIORITY=0")
    r = slave.process(req)
    assert r[0] == rsp.PID_RES
    _rx("SET_DAQ_LIST_MODE pos.response", r, "DAQ list 0 configured")

    # ----------------------------------------------------------------- Step 11
    print("\n--- Step 11: Start DAQ ---")
    req = bytes([cmd.CMD_START_STOP_DAQ_LIST, 0x02, 0x00, 0x00])
    _tx("START_STOP_DAQ_LIST", req, "MODE=0x02 (SELECT)  DAQ_LIST=0")
    resp = slave.process(req)
    assert resp[0] == rsp.PID_RES
    _rx("START_STOP_DAQ_LIST pos.response", resp,
        f"DAQ list 0 selected  FIRST_PID=0x{resp[1]:02X}")

    req = bytes([cmd.CMD_START_STOP_SYNCH, 0x01])
    _tx("START_STOP_SYNCH", req, "MODE=0x01 (start selected)")
    resp = slave.process(req)
    assert resp[0] == rsp.PID_RES
    _rx("START_STOP_SYNCH pos.response", resp, "DAQ list 0 RUNNING")
    assert session.daq_lists[0].running

    sr = slave.process(bytes([cmd.CMD_GET_STATUS]))
    assert sr[1] & 0x08
    print(f"            >> GET_STATUS: SESSION_STATUS=0x{sr[1]:02X}  "
          f"DAQ_RUNNING={bool(sr[1]&0x08)}")

    # ----------------------------------------------------------------- Step 12
    print("\n--- Step 12: Capture 10 DTO frames ---")
    print(f"  {'#':<4}  {'Raw DTO bytes':<50}  EngineSpeed  CoolantTemp")
    print(f"  {'-'*4}  {'-'*50}  {'-'*11}  {'-'*11}")

    for i in range(10):
        dtos = slave.collect_daq_dtos()
        assert len(dtos) == 1
        dto = dtos[0]
        odt_num  = dto[0]   # noqa: F841
        list_num = dto[1]   # noqa: F841
        speed    = struct.unpack_from(">I", dto, 2)[0]   # ULONG big-endian
        coolant  = struct.unpack_from(">i", dto, 6)[0]   # SLONG big-endian
        hex_str  = " ".join(f"{b:02X}" for b in dto)
        print(f"  [{i:2d}]  {hex_str:<50}  "
              f"{speed:>7} rpm  {coolant/10.0:>7.1f} °C")
        assert 0 <= speed <= 8000,      f"EngineSpeed {speed} out of range"
        assert -400 <= coolant <= 1300, f"CoolantTemp {coolant} out of range"

    # ------------------------------------------------------------ Disconnect
    print("\n--- Session complete: DISCONNECT ---")
    req = bytes([cmd.CMD_DISCONNECT])
    _tx("DISCONNECT", req)
    resp = slave.process(req)
    assert resp[0] == rsp.PID_RES
    _rx("DISCONNECT pos.response", resp, "session terminated")
    assert not session.connected

    print()
    print("=" * 72)
    print("  All 12 steps PASSED.")
    print("=" * 72)
