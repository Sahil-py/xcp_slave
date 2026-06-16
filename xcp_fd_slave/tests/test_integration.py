"""
Integration test: simulates a complete XCP master session against the slave.

Runs in two modes:
  - CAN FD      (max_cto=64): single UPLOAD, both signals in one ODT (10-byte DTO)
  - Classic CAN (max_cto=8 ): chunked UPLOAD, one signal per ODT (6-byte DTOs)

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

def _upload_chunked(slave, total, chunk, label):
    """UPLOAD total bytes in chunks of chunk — handles classic CAN MAX_CTO limit."""
    result = b""
    remaining = total
    while remaining > 0:
        n = min(remaining, chunk)
        req = bytes([cmd.CMD_UPLOAD, n])
        _tx(f"UPLOAD chunk", req, f"NR={n} of {remaining} remaining")
        resp = slave.process(req)
        assert resp[0] == rsp.PID_RES, f"UPLOAD chunk failed: 0x{resp[0]:02X}"
        _rx(f"UPLOAD chunk response", resp, f"{n} bytes")
        result += resp[1:1 + n]
        remaining -= n
    return result


@pytest.mark.parametrize("max_cto,label", [
    (64, "CAN FD"),
    (8,  "Classic CAN"),
])
def test_master_session_sequence(max_cto, label):
    session = XcpSession(max_cto=max_cto, max_dto=max_cto)
    memory  = MemoryMap()
    slave   = XcpDispatcher(session, memory)

    upload_chunk = max_cto - 1   # max data bytes per UPLOAD
    download_max = max_cto - 2   # max data bytes per DOWNLOAD

    print()
    print("=" * 72)
    print(f"  XCP 1.3 Master Session — CANdoit_Test ECU  [{label}  MAX_CTO={max_cto}]")
    print("=" * 72)

    # ------------------------------------------------------------------ Step 1
    print("\n--- Step 1: CONNECT ---")
    req = bytes([cmd.CMD_CONNECT, 0x00])
    _tx("CONNECT", req, "mode=0x00 (normal)")
    resp = slave.process(req)
    assert resp[0] == rsp.PID_RES
    resource  = resp[1]
    comm_mode = resp[2]
    reported_max_cto = resp[3]
    max_dto   = struct.unpack_from("<H", resp, 4)[0]
    byte_order = "Motorola/MSB_FIRST" if comm_mode & 0x80 else "Intel/MSB_LAST"
    _rx("CONNECT pos.response", resp,
        f"RESOURCE=0x{resource:02X}  COMM_MODE=0x{comm_mode:02X} ({byte_order})  "
        f"MAX_CTO={reported_max_cto}  MAX_DTO={max_dto}")
    assert session.connected
    assert comm_mode & 0x80, "Expected Motorola byte order per A2L MSB_FIRST"
    assert reported_max_cto == max_cto

    # ------------------------------------------------------------------ Step 2
    print("\n--- Step 2: GET_STATUS ---")
    req = bytes([cmd.CMD_GET_STATUS])
    _tx("GET_STATUS", req)
    resp = slave.process(req)
    assert resp[0] == rsp.PID_RES
    _rx("GET_STATUS pos.response", resp,
        f"SESSION_STATUS=0x{resp[1]:02X}  PROTECTION=0x{resp[2]:02X}")
    assert resp[2] == 0x00

    # ------------------------------------------------------------------ Step 3
    print("\n--- Step 3: GET_ID (ASCII ECU name) ---")
    req = bytes([cmd.CMD_GET_ID, 0x00])
    _tx("GET_ID", req, "type=0x00 (ASCII)")
    resp = slave.process(req)
    assert resp[0] == rsp.PID_RES
    length = struct.unpack_from("<I", resp, 4)[0]
    _rx("GET_ID pos.response", resp,
        f"MODE=0x{resp[1]:02X}  LENGTH={length}  upload_chunk={upload_chunk}")

    name_bytes = _upload_chunked(slave, length, upload_chunk, "ECU name")
    ecu_name = name_bytes.decode("ascii")
    print(f"            >> ECU name = \"{ecu_name}\"")
    assert ecu_name == "CANdoit_Test"

    # ------------------------------------------------------------------ Step 4
    print("\n--- Step 4: SET_MTA → EngineSpeed (0x00001000) ---")
    req = bytes([cmd.CMD_SET_MTA, 0x00, 0x00, 0x00]) + struct.pack("<I", ADDR_ENGINE_SPEED)
    _tx("SET_MTA", req, f"addr=0x{ADDR_ENGINE_SPEED:08X}")
    resp = slave.process(req)
    assert resp[0] == rsp.PID_RES
    _rx("SET_MTA pos.response", resp, "MTA set to EngineSpeed")

    # ------------------------------------------------------------------ Step 5
    print("\n--- Step 5: UPLOAD EngineSpeed (4 bytes ULONG big-endian) ---")
    req = bytes([cmd.CMD_UPLOAD, 0x04])
    _tx("UPLOAD", req, "NR=4")
    resp = slave.process(req)
    assert resp[0] == rsp.PID_RES
    speed = struct.unpack_from(">I", resp, 1)[0]
    _rx("UPLOAD pos.response", resp,
        f"EngineSpeed = {speed} rpm  (raw: {resp[1:5].hex().upper()})")
    assert speed == 800

    # ------------------------------------------------------------------ Step 6
    print("\n--- Step 6: SET_MTA → MaxEngineSpeed (0x00008000) ---")
    req = bytes([cmd.CMD_SET_MTA, 0x00, 0x00, 0x00]) + struct.pack("<I", ADDR_MAX_ENGINE_SPEED)
    _tx("SET_MTA", req, f"addr=0x{ADDR_MAX_ENGINE_SPEED:08X}")
    resp = slave.process(req)
    assert resp[0] == rsp.PID_RES
    _rx("SET_MTA pos.response", resp, "MTA set to MaxEngineSpeed")

    # ------------------------------------------------------------------ Step 7
    print("\n--- Step 7: UPLOAD MaxEngineSpeed (4 bytes FLOAT32 big-endian) ---")
    req = bytes([cmd.CMD_UPLOAD, 0x04])
    _tx("UPLOAD", req, "NR=4")
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
    _set_mta(slave, ADDR_MAX_ENGINE_SPEED)
    req = bytes([cmd.CMD_DOWNLOAD, 0x04]) + payload
    _tx("DOWNLOAD", req,
        f"NR=4  value={new_val:.1f} rpm  (bytes: {payload.hex().upper()})")
    resp = slave.process(req)
    assert resp[0] == rsp.PID_RES
    _rx("DOWNLOAD pos.response", resp, "calibration written")

    # ------------------------------------------------------------------ Step 9
    print("\n--- Step 9: UPLOAD MaxEngineSpeed (verify DOWNLOAD) ---")
    _set_mta(slave, ADDR_MAX_ENGINE_SPEED)
    req = bytes([cmd.CMD_UPLOAD, 0x04])
    _tx("UPLOAD", req, "NR=4 (verify)")
    resp = slave.process(req)
    assert resp[0] == rsp.PID_RES
    max_spd_after = struct.unpack_from(">f", resp, 1)[0]
    _rx("UPLOAD pos.response", resp,
        f"MaxEngineSpeed = {max_spd_after:.1f} rpm  "
        f"(was {max_spd_before:.1f}, delta={max_spd_after - max_spd_before:+.1f})")
    assert abs(max_spd_after - new_val) < 0.001

    # ----------------------------------------------------------------- Step 10
    print("\n--- Step 10: Configure DAQ ---")

    if max_cto == 64:
        # CAN FD: both signals in one ODT → one 10-byte DTO per tick
        print("  [CAN FD mode: 2 entries in ODT 0]")
        for lbl, pkt, decode in [
            ("FREE_DAQ",        bytes([cmd.CMD_FREE_DAQ]),                              "release all"),
            ("ALLOC_DAQ",       bytes([cmd.CMD_ALLOC_DAQ, 0x00, 0x01, 0x00]),          "COUNT=1"),
            ("ALLOC_ODT",       bytes([cmd.CMD_ALLOC_ODT, 0x00, 0x00, 0x00, 0x01]),    "list=0 COUNT=1"),
            ("ALLOC_ODT_ENTRY", bytes([cmd.CMD_ALLOC_ODT_ENTRY, 0x00, 0x00, 0x00, 0x00, 0x02]), "list=0 odt=0 entries=2"),
            ("SET_DAQ_PTR",     bytes([cmd.CMD_SET_DAQ_PTR, 0x00, 0x00, 0x00, 0x00, 0x00]),     "list=0 odt=0 entry=0"),
        ]:
            _tx(lbl, pkt, decode)
            r = slave.process(pkt)
            assert r[0] == rsp.PID_RES
            _rx(f"{lbl} pos.response", r, "OK")

        for addr, sig in [(ADDR_ENGINE_SPEED, "EngineSpeed"), (ADDR_COOLANT_TEMP, "CoolantTemp")]:
            pkt = bytes([cmd.CMD_WRITE_DAQ, 0xFF, 0x04, 0x00]) + struct.pack("<I", addr)
            _tx("WRITE_DAQ", pkt, f"SIZE=4  ADDR=0x{addr:08X} ({sig})")
            r = slave.process(pkt)
            assert r[0] == rsp.PID_RES
            _rx("WRITE_DAQ pos.response", r, f"entry → {sig}")

    else:
        # Classic CAN: max DTO = 8 bytes = 2 header + 6 data
        # Each 4-byte signal needs its own ODT (2+4=6 bytes fits, 2+8=10 does not)
        print("  [Classic CAN mode: 1 entry per ODT, 2 ODTs]")
        for lbl, pkt, decode in [
            ("FREE_DAQ",        bytes([cmd.CMD_FREE_DAQ]),                              "release all"),
            ("ALLOC_DAQ",       bytes([cmd.CMD_ALLOC_DAQ, 0x00, 0x01, 0x00]),          "COUNT=1"),
            ("ALLOC_ODT",       bytes([cmd.CMD_ALLOC_ODT, 0x00, 0x00, 0x00, 0x02]),    "list=0 COUNT=2"),
            ("ALLOC_ODT_ENTRY ODT0", bytes([cmd.CMD_ALLOC_ODT_ENTRY, 0x00, 0x00, 0x00, 0x00, 0x01]), "list=0 odt=0 entries=1"),
            ("ALLOC_ODT_ENTRY ODT1", bytes([cmd.CMD_ALLOC_ODT_ENTRY, 0x00, 0x00, 0x00, 0x01, 0x01]), "list=0 odt=1 entries=1"),
        ]:
            _tx(lbl, pkt, decode)
            r = slave.process(pkt)
            assert r[0] == rsp.PID_RES
            _rx(f"{lbl} pos.response", r, "OK")

        for odt_idx, addr, sig in [
            (0, ADDR_ENGINE_SPEED, "EngineSpeed"),
            (1, ADDR_COOLANT_TEMP, "CoolantTemp"),
        ]:
            ptr = bytes([cmd.CMD_SET_DAQ_PTR, 0x00, 0x00, 0x00, odt_idx, 0x00])
            _tx("SET_DAQ_PTR", ptr, f"list=0 odt={odt_idx} entry=0")
            r = slave.process(ptr)
            assert r[0] == rsp.PID_RES

            pkt = bytes([cmd.CMD_WRITE_DAQ, 0xFF, 0x04, 0x00]) + struct.pack("<I", addr)
            _tx("WRITE_DAQ", pkt, f"SIZE=4  ADDR=0x{addr:08X} ({sig})")
            r = slave.process(pkt)
            assert r[0] == rsp.PID_RES
            _rx("WRITE_DAQ pos.response", r, f"ODT{odt_idx} → {sig}")

    pkt = bytes([cmd.CMD_SET_DAQ_LIST_MODE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00])
    _tx("SET_DAQ_LIST_MODE", pkt, "list=0 prescaler=1")
    r = slave.process(pkt)
    assert r[0] == rsp.PID_RES
    _rx("SET_DAQ_LIST_MODE pos.response", r, "OK")

    # ----------------------------------------------------------------- Step 11
    print("\n--- Step 11: Start DAQ ---")
    req = bytes([cmd.CMD_START_STOP_DAQ_LIST, 0x02, 0x00, 0x00])
    _tx("START_STOP_DAQ_LIST", req, "MODE=SELECT list=0")
    resp = slave.process(req)
    assert resp[0] == rsp.PID_RES
    _rx("START_STOP_DAQ_LIST pos.response", resp, f"FIRST_PID=0x{resp[1]:02X}")

    req = bytes([cmd.CMD_START_STOP_SYNCH, 0x01])
    _tx("START_STOP_SYNCH", req, "start selected")
    resp = slave.process(req)
    assert resp[0] == rsp.PID_RES
    _rx("START_STOP_SYNCH pos.response", resp, "DAQ RUNNING")
    assert session.daq_lists[0].running

    sr = slave.process(bytes([cmd.CMD_GET_STATUS]))
    assert sr[1] & 0x08
    print(f"            >> SESSION_STATUS=0x{sr[1]:02X}  DAQ_RUNNING=True")

    # ----------------------------------------------------------------- Step 12
    print("\n--- Step 12: Capture 10 DTO frames ---")

    if max_cto == 64:
        # One DTO per tick: [ODT, LIST, EngineSpeed(4), CoolantTemp(4)] = 10 bytes
        print(f"  {'#':<4}  {'Raw DTO bytes':<50}  EngineSpeed  CoolantTemp")
        print(f"  {'-'*4}  {'-'*50}  {'-'*11}  {'-'*11}")
        for i in range(10):
            dtos = slave.collect_daq_dtos()
            assert len(dtos) == 1
            dto = dtos[0]
            speed   = struct.unpack_from(">I", dto, 2)[0]
            coolant = struct.unpack_from(">i", dto, 6)[0]
            hex_str = " ".join(f"{b:02X}" for b in dto)
            print(f"  [{i:2d}]  {hex_str:<50}  {speed:>7} rpm  {coolant/10.0:>7.1f} °C")
            assert 0 <= speed <= 8000
            assert -400 <= coolant <= 1300
    else:
        # Two DTOs per tick (one per ODT):
        #   DTO0: [0x00, 0x00, EngineSpeed(4)] = 6 bytes ≤ 8 ✓
        #   DTO1: [0x01, 0x00, CoolantTemp(4)] = 6 bytes ≤ 8 ✓
        print(f"  {'#':<4}  {'DTO0 (EngineSpeed)':<30}  {'DTO1 (CoolantTemp)':<30}  Speed    Coolant")
        print(f"  {'-'*4}  {'-'*30}  {'-'*30}  {'-'*7}  {'-'*7}")
        for i in range(10):
            dtos = slave.collect_daq_dtos()
            assert len(dtos) == 2, f"Expected 2 DTOs, got {len(dtos)}"
            dto0, dto1 = dtos[0], dtos[1]
            assert len(dto0) == 6, f"DTO0 should be 6 bytes, got {len(dto0)}"
            assert len(dto1) == 6, f"DTO1 should be 6 bytes, got {len(dto1)}"
            speed   = struct.unpack_from(">I", dto0, 2)[0]
            coolant = struct.unpack_from(">i", dto1, 2)[0]
            h0 = " ".join(f"{b:02X}" for b in dto0)
            h1 = " ".join(f"{b:02X}" for b in dto1)
            print(f"  [{i:2d}]  {h0:<30}  {h1:<30}  {speed:>5} rpm  {coolant/10.0:>5.1f} °C")
            assert 0 <= speed <= 8000
            assert -400 <= coolant <= 1300

    # ---------------------------------------------------------------- Disconnect
    print("\n--- Session complete: DISCONNECT ---")
    req = bytes([cmd.CMD_DISCONNECT])
    _tx("DISCONNECT", req)
    resp = slave.process(req)
    assert resp[0] == rsp.PID_RES
    _rx("DISCONNECT pos.response", resp, "session terminated")
    assert not session.connected

    print()
    print("=" * 72)
    print(f"  All 12 steps PASSED  [{label}  MAX_CTO={max_cto}]")
    print("=" * 72)
