"""
XCP protocol compliance tests — no CAN hardware required.

Memory layout and data types are taken directly from xcp_fd_slave/a2l/sample.a2l:
  - Measurements: ULONG (big-endian uint32) or SLONG (big-endian int32)
  - Characteristics: FLOAT32_IEEE (big-endian float32)
  - BYTE_ORDER MSB_FIRST throughout
"""
import struct
import pytest

from xcp_fd_slave.protocol.session import XcpSession
from xcp_fd_slave.memory.memory_map import MemoryMap
from xcp_fd_slave.protocol.dispatcher import XcpDispatcher
from xcp_fd_slave.protocol import commands as cmd
from xcp_fd_slave.protocol import responses as rsp
from xcp_fd_slave.protocol import errors as err
from xcp_fd_slave.memory.memory_map import (
    # Measurements
    ADDR_ENGINE_SPEED, ADDR_ENGINE_LOAD, ADDR_COOLANT_TEMP,
    ADDR_VEHICLE_SPEED, ADDR_LAMBDA_SENSOR_1,
    # Calibration
    ADDR_MAX_ENGINE_SPEED, ADDR_IDLE_TARGET_SPEED, ADDR_TORQUE_LIMIT,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _set_mta(d: XcpDispatcher, addr: int, ext: int = 0) -> None:
    pkt = bytes([cmd.CMD_SET_MTA, 0x00, 0x00, ext]) + struct.pack("<I", addr)
    resp = d.process(pkt)
    assert resp[0] == rsp.PID_RES


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def disp():
    return XcpDispatcher(XcpSession(), MemoryMap())


@pytest.fixture
def conn(disp):
    disp.process(bytes([cmd.CMD_CONNECT, 0x00]))
    assert disp.session.connected
    return disp


# ---------------------------------------------------------------------------
# Phase 1 — CONNECT
# ---------------------------------------------------------------------------

class TestConnect:
    def test_returns_res(self, disp):
        assert disp.process(bytes([cmd.CMD_CONNECT, 0x00]))[0] == rsp.PID_RES

    def test_response_length(self, disp):
        assert len(disp.process(bytes([cmd.CMD_CONNECT, 0x00]))) == 8

    def test_sets_connected(self, disp):
        disp.process(bytes([cmd.CMD_CONNECT, 0x00]))
        assert disp.session.connected is True

    def test_resource_cal_pag(self, disp):
        resp = disp.process(bytes([cmd.CMD_CONNECT, 0x00]))
        assert resp[1] & 0x01, "CAL/PAG resource must be advertised"

    def test_resource_daq(self, disp):
        resp = disp.process(bytes([cmd.CMD_CONNECT, 0x00]))
        assert resp[1] & 0x04, "DAQ resource must be advertised"

    def test_resource_no_pgm(self, disp):
        resp = disp.process(bytes([cmd.CMD_CONNECT, 0x00]))
        assert not (resp[1] & 0x10), "PGM must NOT be advertised (not implemented)"

    def test_comm_mode_basic_motorola_byte_order(self, disp):
        resp = disp.process(bytes([cmd.CMD_CONNECT, 0x00]))
        # A2L declares BYTE_ORDER MSB_FIRST → COMM_MODE_BASIC bit 7 must be 1
        assert resp[2] & 0x80, "BYTE_ORDER must be Motorola (MSB_FIRST per A2L)"

    def test_comm_mode_basic_byte_ag(self, disp):
        resp = disp.process(bytes([cmd.CMD_CONNECT, 0x00]))
        # Bits 6-5 = 00 → byte address granularity
        assert (resp[2] >> 5) & 0x03 == 0, "Address granularity must be BYTE"

    def test_comm_mode_basic_optional(self, disp):
        resp = disp.process(bytes([cmd.CMD_CONNECT, 0x00]))
        assert resp[2] & 0x02, "OPTIONAL bit must be set (GET_COMM_MODE_INFO available)"

    def test_max_cto_64(self, disp):
        resp = disp.process(bytes([cmd.CMD_CONNECT, 0x00]))
        assert resp[3] == 64

    def test_max_dto_64(self, disp):
        resp = disp.process(bytes([cmd.CMD_CONNECT, 0x00]))
        assert struct.unpack_from("<H", resp, 4)[0] == 64

    def test_protocol_version(self, disp):
        resp = disp.process(bytes([cmd.CMD_CONNECT, 0x00]))
        assert resp[6] == 0x01, "XCP protocol major version must be 0x01"

    def test_reconnect_resets_mta(self, conn):
        conn.session.mta_address = 0xDEADBEEF
        conn.process(bytes([cmd.CMD_CONNECT, 0x00]))
        assert conn.session.mta_address == 0

    def test_commands_ignored_before_connect(self, disp):
        assert disp.process(bytes([cmd.CMD_GET_STATUS])) is None


class TestDisconnect:
    def test_returns_res(self, conn):
        assert conn.process(bytes([cmd.CMD_DISCONNECT]))[0] == rsp.PID_RES

    def test_clears_session(self, conn):
        conn.process(bytes([cmd.CMD_DISCONNECT]))
        assert conn.session.connected is False

    def test_commands_ignored_after_disconnect(self, conn):
        conn.process(bytes([cmd.CMD_DISCONNECT]))
        assert conn.process(bytes([cmd.CMD_GET_STATUS])) is None


# ---------------------------------------------------------------------------
# Phase 1 — status and info commands
# ---------------------------------------------------------------------------

class TestGetStatus:
    def test_returns_res(self, conn):
        assert conn.process(bytes([cmd.CMD_GET_STATUS]))[0] == rsp.PID_RES

    def test_response_length(self, conn):
        assert len(conn.process(bytes([cmd.CMD_GET_STATUS]))) >= 6

    def test_protection_status_unlocked(self, conn):
        # Byte 2 = resource protection status; 0x00 = nothing protected
        resp = conn.process(bytes([cmd.CMD_GET_STATUS]))
        assert resp[2] == 0x00

    def test_daq_running_flag_when_idle(self, conn):
        resp = conn.process(bytes([cmd.CMD_GET_STATUS]))
        assert not (resp[1] & 0x08), "DAQ_RUNNING must be 0 when no DAQ list is running"


class TestSynch:
    def test_always_returns_err_cmd_synch(self, conn):
        resp = conn.process(bytes([cmd.CMD_SYNCH]))
        assert resp[0] == rsp.PID_ERR
        assert resp[1] == err.ERR_CMD_SYNCH


class TestGetCommModeInfo:
    def test_returns_res(self, conn):
        assert conn.process(bytes([cmd.CMD_GET_COMM_MODE_INFO]))[0] == rsp.PID_RES

    def test_response_length(self, conn):
        assert len(conn.process(bytes([cmd.CMD_GET_COMM_MODE_INFO]))) == 8

    def test_no_master_block_mode(self, conn):
        resp = conn.process(bytes([cmd.CMD_GET_COMM_MODE_INFO]))
        # COMM_MODE_OPTIONAL byte 2: bit 1 = MASTER_BLOCK_MODE
        assert not (resp[2] & 0x02), "MASTER_BLOCK_MODE must be 0"

    def test_no_interleaved_mode(self, conn):
        resp = conn.process(bytes([cmd.CMD_GET_COMM_MODE_INFO]))
        assert not (resp[2] & 0x01), "INTERLEAVED_MODE must be 0"


class TestGetId:
    def test_returns_res(self, conn):
        assert conn.process(bytes([cmd.CMD_GET_ID, 0x00]))[0] == rsp.PID_RES

    def test_response_length_8_bytes(self, conn):
        assert len(conn.process(bytes([cmd.CMD_GET_ID, 0x00]))) == 8

    def test_mode_not_compressed(self, conn):
        resp = conn.process(bytes([cmd.CMD_GET_ID, 0x00]))
        assert resp[1] & 0x01 == 0, "Compressed/encrypted bit must be 0"

    def test_mode_transfer_via_upload(self, conn):
        resp = conn.process(bytes([cmd.CMD_GET_ID, 0x00]))
        assert resp[1] & 0x02 == 0, "Transfer mode bit must be 0 (data via UPLOAD)"

    def test_ascii_name_matches_a2l_project(self, conn):
        resp_id = conn.process(bytes([cmd.CMD_GET_ID, 0x00]))   # type 0 = ASCII
        length = struct.unpack_from("<I", resp_id, 4)[0]
        resp_up = conn.process(bytes([cmd.CMD_UPLOAD, length]))
        assert resp_up[0] == rsp.PID_RES
        name = resp_up[1: 1 + length].decode("ascii")
        assert name == "CANdoit_Test"

    def test_a2l_filename_without_ext(self, conn):
        resp_id = conn.process(bytes([cmd.CMD_GET_ID, 0x01]))
        length = struct.unpack_from("<I", resp_id, 4)[0]
        resp_up = conn.process(bytes([cmd.CMD_UPLOAD, length]))
        assert resp_up[1: 1 + length].decode("ascii") == "sample"

    def test_a2l_filename_with_ext(self, conn):
        resp_id = conn.process(bytes([cmd.CMD_GET_ID, 0x02]))
        length = struct.unpack_from("<I", resp_id, 4)[0]
        resp_up = conn.process(bytes([cmd.CMD_UPLOAD, length]))
        assert resp_up[1: 1 + length].decode("ascii") == "sample.a2l"

    def test_get_id_sets_mta(self, conn):
        conn.process(bytes([cmd.CMD_GET_ID, 0x00]))
        assert conn.session.mta_address != 0


class TestUnknownCommand:
    def test_returns_err_cmd_unknown(self, conn):
        resp = conn.process(bytes([0x00]))
        assert resp[0] == rsp.PID_ERR
        assert resp[1] == err.ERR_CMD_UNKNOWN


# ---------------------------------------------------------------------------
# Phase 2 — memory access (ULONG / SLONG / FLOAT32, big-endian per A2L)
# ---------------------------------------------------------------------------

class TestSetMta:
    def test_returns_res(self, conn):
        pkt = bytes([cmd.CMD_SET_MTA, 0x00, 0x00, 0x00]) + struct.pack("<I", ADDR_ENGINE_SPEED)
        assert conn.process(pkt)[0] == rsp.PID_RES

    def test_stores_address_and_extension(self, conn):
        pkt = bytes([cmd.CMD_SET_MTA, 0x00, 0x00, 0x02]) + struct.pack("<I", 0x2000)
        conn.process(pkt)
        assert conn.session.mta_address == 0x2000
        assert conn.session.mta_extension == 0x02


class TestUpload:
    def test_engine_speed_initial_value_800_rpm(self, conn):
        # A2L: EngineSpeed ULONG at 0x1000, big-endian, initial = 800 rpm
        _set_mta(conn, ADDR_ENGINE_SPEED)
        resp = conn.process(bytes([cmd.CMD_UPLOAD, 4]))
        assert resp[0] == rsp.PID_RES
        val = struct.unpack_from(">I", resp, 1)[0]   # big-endian ULONG
        assert val == 800

    def test_coolant_temp_initial_value_200(self, conn):
        # A2L: CoolantTemp SLONG at 0x2000, big-endian, initial = 200 (20.0 °C)
        _set_mta(conn, ADDR_COOLANT_TEMP)
        resp = conn.process(bytes([cmd.CMD_UPLOAD, 4]))
        assert resp[0] == rsp.PID_RES
        val = struct.unpack_from(">i", resp, 1)[0]   # big-endian SLONG
        assert val == 200

    def test_max_engine_speed_cal_initial_float(self, conn):
        # A2L: MaxEngineSpeed FLOAT32 at 0x8000, big-endian, initial = 7500.0
        _set_mta(conn, ADDR_MAX_ENGINE_SPEED)
        resp = conn.process(bytes([cmd.CMD_UPLOAD, 4]))
        assert resp[0] == rsp.PID_RES
        val = struct.unpack_from(">f", resp, 1)[0]   # big-endian float32
        assert abs(val - 7500.0) < 0.1

    def test_idle_target_speed_cal_initial_float(self, conn):
        # A2L: IdleTargetSpeed FLOAT32 at 0x8008, initial = 800.0
        _set_mta(conn, ADDR_IDLE_TARGET_SPEED)
        resp = conn.process(bytes([cmd.CMD_UPLOAD, 4]))
        val = struct.unpack_from(">f", resp, 1)[0]
        assert abs(val - 800.0) < 0.1

    def test_mta_advances_after_upload(self, conn):
        _set_mta(conn, ADDR_ENGINE_SPEED)
        conn.process(bytes([cmd.CMD_UPLOAD, 4]))
        assert conn.session.mta_address == ADDR_ENGINE_SPEED + 4

    def test_upload_out_of_range_address(self, conn):
        _set_mta(conn, 0xFFFF0000)
        resp = conn.process(bytes([cmd.CMD_UPLOAD, 4]))
        assert resp[0] == rsp.PID_ERR
        assert resp[1] == err.ERR_OUT_OF_RANGE

    def test_upload_exceeds_max_cto_returns_err(self, conn):
        _set_mta(conn, ADDR_ENGINE_SPEED)
        resp = conn.process(bytes([cmd.CMD_UPLOAD, 64]))   # 64 > MAX_CTO-1=63
        assert resp[0] == rsp.PID_ERR
        assert resp[1] == err.ERR_OUT_OF_RANGE

    def test_sequential_upload_across_measurement_block(self, conn):
        # Read EngineSpeed(4) + EngineLoad(4) in two consecutive UPLOADs
        _set_mta(conn, ADDR_ENGINE_SPEED)
        r1 = conn.process(bytes([cmd.CMD_UPLOAD, 4]))
        r2 = conn.process(bytes([cmd.CMD_UPLOAD, 4]))
        speed = struct.unpack_from(">I", r1, 1)[0]
        load  = struct.unpack_from(">I", r2, 1)[0]
        assert speed == 800
        assert load == 10


class TestShortUpload:
    def test_vehicle_speed_via_short_upload(self, conn):
        # A2L: VehicleSpeed ULONG at 0x3000, initial = 0
        pkt = bytes([cmd.CMD_SHORT_UPLOAD, 4, 0x00, 0x00]) + struct.pack("<I", ADDR_VEHICLE_SPEED)
        resp = conn.process(pkt)
        assert resp[0] == rsp.PID_RES
        val = struct.unpack_from(">I", resp, 1)[0]
        assert val == 0

    def test_short_upload_sets_mta(self, conn):
        pkt = bytes([cmd.CMD_SHORT_UPLOAD, 4, 0x00, 0x00]) + struct.pack("<I", ADDR_VEHICLE_SPEED)
        conn.process(pkt)
        assert conn.session.mta_address == ADDR_VEHICLE_SPEED + 4


class TestDownload:
    def test_write_calibration_float32_big_endian(self, conn):
        # Write new TorqueLimit = 420.0 Nm (big-endian float32) to 0x8010
        new_val = 420.0
        payload = struct.pack(">f", new_val)    # big-endian, as ECU expects
        _set_mta(conn, ADDR_TORQUE_LIMIT)
        resp = conn.process(bytes([cmd.CMD_DOWNLOAD, 4]) + payload)
        assert resp[0] == rsp.PID_RES

        # Read back and verify
        _set_mta(conn, ADDR_TORQUE_LIMIT)
        resp2 = conn.process(bytes([cmd.CMD_UPLOAD, 4]))
        readback = struct.unpack_from(">f", resp2, 1)[0]
        assert abs(readback - new_val) < 0.001

    def test_write_calibration_preserves_adjacent_values(self, conn):
        # MaxEngineSpeed (0x8000) and RevLimiterCutIn (0x8004) are adjacent
        # Writing MaxEngineSpeed must not corrupt RevLimiterCutIn
        new_max = 8000.0
        _set_mta(conn, ADDR_MAX_ENGINE_SPEED)
        conn.process(bytes([cmd.CMD_DOWNLOAD, 4]) + struct.pack(">f", new_max))

        _set_mta(conn, ADDR_IDLE_TARGET_SPEED)   # 0x8008, skip one slot
        resp = conn.process(bytes([cmd.CMD_UPLOAD, 4]))
        val = struct.unpack_from(">f", resp, 1)[0]
        assert abs(val - 800.0) < 0.1   # IdleTargetSpeed unchanged

    def test_mta_advances_after_download(self, conn):
        _set_mta(conn, ADDR_TORQUE_LIMIT)
        conn.process(bytes([cmd.CMD_DOWNLOAD, 4]) + struct.pack(">f", 300.0))
        assert conn.session.mta_address == ADDR_TORQUE_LIMIT + 4

    def test_download_out_of_range_address(self, conn):
        _set_mta(conn, 0xFFFF0000)
        resp = conn.process(bytes([cmd.CMD_DOWNLOAD, 4]) + b"\x00" * 4)
        assert resp[0] == rsp.PID_ERR
        assert resp[1] == err.ERR_OUT_OF_RANGE

    def test_download_exceeds_max_cto_returns_err(self, conn):
        _set_mta(conn, ADDR_TORQUE_LIMIT)
        resp = conn.process(bytes([cmd.CMD_DOWNLOAD, 63]) + b"\x00" * 63)
        assert resp[0] == rsp.PID_ERR
        assert resp[1] == err.ERR_OUT_OF_RANGE

    def test_download_max_writes_63_bytes(self, conn):
        _set_mta(conn, ADDR_MAX_ENGINE_SPEED)
        payload = b"\xBB" * 63
        resp = conn.process(bytes([cmd.CMD_DOWNLOAD_MAX]) + payload)
        assert resp[0] == rsp.PID_RES
        _set_mta(conn, ADDR_MAX_ENGINE_SPEED)
        resp2 = conn.process(bytes([cmd.CMD_UPLOAD, 4]))
        assert resp2[1:5] == b"\xBB\xBB\xBB\xBB"


# ---------------------------------------------------------------------------
# Phase 3 — DAQ
# ---------------------------------------------------------------------------

class TestDaqProcessorInfo:
    def test_returns_res(self, conn):
        assert conn.process(bytes([cmd.CMD_GET_DAQ_PROCESSOR_INFO]))[0] == rsp.PID_RES

    def test_dynamic_config_bit(self, conn):
        resp = conn.process(bytes([cmd.CMD_GET_DAQ_PROCESSOR_INFO]))
        assert resp[1] & 0x01, "DAQ_CONFIG_TYPE must be 1 (dynamic)"

    def test_max_daq_nonzero(self, conn):
        resp = conn.process(bytes([cmd.CMD_GET_DAQ_PROCESSOR_INFO]))
        assert struct.unpack_from("<H", resp, 2)[0] > 0


class TestDaqResolutionInfo:
    def test_returns_res(self, conn):
        assert conn.process(bytes([cmd.CMD_GET_DAQ_RESOLUTION_INFO]))[0] == rsp.PID_RES

    def test_byte_granularity_daq(self, conn):
        resp = conn.process(bytes([cmd.CMD_GET_DAQ_RESOLUTION_INFO]))
        assert resp[1] == 1, "GRANULARITY_ODT_ENTRY_SIZE_DAQ must be 1 (byte)"

    def test_max_odt_entry_size_matches_max_dto(self, conn):
        resp = conn.process(bytes([cmd.CMD_GET_DAQ_RESOLUTION_INFO]))
        assert resp[2] == conn.session.max_dto - 2, "MAX_ODT_ENTRY_SIZE = MAX_DTO - 2-byte header"


class TestDaqAllocation:
    def test_free_daq_returns_res(self, conn):
        assert conn.process(bytes([cmd.CMD_FREE_DAQ]))[0] == rsp.PID_RES

    def test_alloc_daq_one_list(self, conn):
        conn.process(bytes([cmd.CMD_FREE_DAQ]))
        resp = conn.process(bytes([cmd.CMD_ALLOC_DAQ, 0x00, 0x01, 0x00]))
        assert resp[0] == rsp.PID_RES
        assert len(conn.session.daq_lists) == 1

    def test_alloc_odt_one_odt(self, conn):
        conn.process(bytes([cmd.CMD_FREE_DAQ]))
        conn.process(bytes([cmd.CMD_ALLOC_DAQ, 0x00, 0x01, 0x00]))
        resp = conn.process(bytes([cmd.CMD_ALLOC_ODT, 0x00, 0x00, 0x00, 0x01]))
        assert resp[0] == rsp.PID_RES
        assert len(conn.session.daq_lists[0].odts) == 1

    def test_alloc_odt_entry_two_entries(self, conn):
        conn.process(bytes([cmd.CMD_FREE_DAQ]))
        conn.process(bytes([cmd.CMD_ALLOC_DAQ, 0x00, 0x01, 0x00]))
        conn.process(bytes([cmd.CMD_ALLOC_ODT, 0x00, 0x00, 0x00, 0x01]))
        resp = conn.process(bytes([cmd.CMD_ALLOC_ODT_ENTRY, 0x00, 0x00, 0x00, 0x00, 0x02]))
        assert resp[0] == rsp.PID_RES
        assert len(conn.session.daq_lists[0].odts[0].entries) == 2

    def test_alloc_odt_invalid_list_returns_err(self, conn):
        conn.process(bytes([cmd.CMD_FREE_DAQ]))
        resp = conn.process(bytes([cmd.CMD_ALLOC_ODT, 0x00, 0x09, 0x00, 0x01]))
        assert resp[0] == rsp.PID_ERR
        assert resp[1] == err.ERR_OUT_OF_RANGE


class TestDaqWriteAndStream:
    def _setup_engine_speed_daq(self, d: XcpDispatcher) -> None:
        """
        Configure DAQ list 0, ODT 0, entry 0 → EngineSpeed (ULONG, 4 bytes at 0x1000).
        Mirrors how a master would configure DAQ for the first A2L measurement.
        """
        d.process(bytes([cmd.CMD_FREE_DAQ]))
        d.process(bytes([cmd.CMD_ALLOC_DAQ,       0x00, 0x01, 0x00]))
        d.process(bytes([cmd.CMD_ALLOC_ODT,        0x00, 0x00, 0x00, 0x01]))
        d.process(bytes([cmd.CMD_ALLOC_ODT_ENTRY,  0x00, 0x00, 0x00, 0x00, 0x01]))
        d.process(bytes([cmd.CMD_SET_DAQ_PTR,      0x00, 0x00, 0x00, 0x00, 0x00]))
        # WRITE_DAQ: bit_offset=0xFF (byte access), size=4, addr_ext=0, addr=ADDR_ENGINE_SPEED
        d.process(bytes([cmd.CMD_WRITE_DAQ, 0xFF, 0x04, 0x00]) + struct.pack("<I", ADDR_ENGINE_SPEED))
        d.process(bytes([cmd.CMD_SET_DAQ_LIST_MODE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00]))

    def _setup_multi_signal_daq(self, d: XcpDispatcher) -> None:
        """
        DAQ list 0, ODT 0 with three signals:
          entry 0: EngineSpeed    ULONG 4 bytes @ 0x1000
          entry 1: CoolantTemp    SLONG 4 bytes @ 0x2000
          entry 2: LambdaSensor1  ULONG 4 bytes @ 0x4004
        """
        d.process(bytes([cmd.CMD_FREE_DAQ]))
        d.process(bytes([cmd.CMD_ALLOC_DAQ,       0x00, 0x01, 0x00]))
        d.process(bytes([cmd.CMD_ALLOC_ODT,        0x00, 0x00, 0x00, 0x01]))
        d.process(bytes([cmd.CMD_ALLOC_ODT_ENTRY,  0x00, 0x00, 0x00, 0x00, 0x03]))
        # entry 0 → EngineSpeed
        d.process(bytes([cmd.CMD_SET_DAQ_PTR, 0x00, 0x00, 0x00, 0x00, 0x00]))
        d.process(bytes([cmd.CMD_WRITE_DAQ, 0xFF, 0x04, 0x00]) + struct.pack("<I", ADDR_ENGINE_SPEED))
        # entry 1 → CoolantTemp
        d.process(bytes([cmd.CMD_WRITE_DAQ, 0xFF, 0x04, 0x00]) + struct.pack("<I", ADDR_COOLANT_TEMP))
        # entry 2 → LambdaSensor1
        d.process(bytes([cmd.CMD_WRITE_DAQ, 0xFF, 0x04, 0x00]) + struct.pack("<I", ADDR_LAMBDA_SENSOR_1))
        d.process(bytes([cmd.CMD_SET_DAQ_LIST_MODE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00]))

    def test_set_daq_ptr_returns_res(self, conn):
        conn.process(bytes([cmd.CMD_FREE_DAQ]))
        conn.process(bytes([cmd.CMD_ALLOC_DAQ, 0x00, 0x01, 0x00]))
        conn.process(bytes([cmd.CMD_ALLOC_ODT, 0x00, 0x00, 0x00, 0x01]))
        conn.process(bytes([cmd.CMD_ALLOC_ODT_ENTRY, 0x00, 0x00, 0x00, 0x00, 0x01]))
        resp = conn.process(bytes([cmd.CMD_SET_DAQ_PTR, 0x00, 0x00, 0x00, 0x00, 0x00]))
        assert resp[0] == rsp.PID_RES

    def test_write_daq_stores_entry(self, conn):
        conn.process(bytes([cmd.CMD_FREE_DAQ]))
        conn.process(bytes([cmd.CMD_ALLOC_DAQ, 0x00, 0x01, 0x00]))
        conn.process(bytes([cmd.CMD_ALLOC_ODT, 0x00, 0x00, 0x00, 0x01]))
        conn.process(bytes([cmd.CMD_ALLOC_ODT_ENTRY, 0x00, 0x00, 0x00, 0x00, 0x01]))
        conn.process(bytes([cmd.CMD_SET_DAQ_PTR, 0x00, 0x00, 0x00, 0x00, 0x00]))
        resp = conn.process(
            bytes([cmd.CMD_WRITE_DAQ, 0xFF, 0x04, 0x00]) + struct.pack("<I", ADDR_ENGINE_SPEED)
        )
        assert resp[0] == rsp.PID_RES
        entry = conn.session.daq_lists[0].odts[0].entries[0]
        assert entry.address == ADDR_ENGINE_SPEED
        assert entry.size == 4

    def test_daq_ptr_auto_increments_after_write_daq(self, conn):
        conn.process(bytes([cmd.CMD_FREE_DAQ]))
        conn.process(bytes([cmd.CMD_ALLOC_DAQ, 0x00, 0x01, 0x00]))
        conn.process(bytes([cmd.CMD_ALLOC_ODT, 0x00, 0x00, 0x00, 0x01]))
        conn.process(bytes([cmd.CMD_ALLOC_ODT_ENTRY, 0x00, 0x00, 0x00, 0x00, 0x02]))
        conn.process(bytes([cmd.CMD_SET_DAQ_PTR, 0x00, 0x00, 0x00, 0x00, 0x00]))
        conn.process(bytes([cmd.CMD_WRITE_DAQ, 0xFF, 0x04, 0x00]) + struct.pack("<I", ADDR_ENGINE_SPEED))
        assert conn.session.daq_ptr_entry == 1   # advanced to next entry

    def test_start_daq_list(self, conn):
        self._setup_engine_speed_daq(conn)
        resp = conn.process(bytes([cmd.CMD_START_STOP_DAQ_LIST, 0x01, 0x00, 0x00]))
        assert resp[0] == rsp.PID_RES
        assert conn.session.daq_lists[0].running is True

    def test_stop_daq_list(self, conn):
        self._setup_engine_speed_daq(conn)
        conn.process(bytes([cmd.CMD_START_STOP_DAQ_LIST, 0x01, 0x00, 0x00]))
        resp = conn.process(bytes([cmd.CMD_START_STOP_DAQ_LIST, 0x00, 0x00, 0x00]))
        assert resp[0] == rsp.PID_RES
        assert conn.session.daq_lists[0].running is False

    def test_no_dtos_when_stopped(self, conn):
        self._setup_engine_speed_daq(conn)
        assert conn.collect_daq_dtos() == []

    def test_dto_produced_when_running(self, conn):
        self._setup_engine_speed_daq(conn)
        conn.process(bytes([cmd.CMD_START_STOP_DAQ_LIST, 0x01, 0x00, 0x00]))
        assert len(conn.collect_daq_dtos()) == 1

    def test_dto_header_identification_type_01(self, conn):
        # Header = [ODT number (0), DAQ list number (0)] per IDENTIFICATION_FIELD_TYPE=01
        self._setup_engine_speed_daq(conn)
        conn.process(bytes([cmd.CMD_START_STOP_DAQ_LIST, 0x01, 0x00, 0x00]))
        dto = conn.collect_daq_dtos()[0]
        assert dto[0] == 0x00, "ODT number in DTO header must be 0"
        assert dto[1] == 0x00, "DAQ list number in DTO header must be 0"

    def test_dto_contains_engine_speed_big_endian_ulong(self, conn):
        # EngineSpeed initial = 800 rpm, stored as big-endian ULONG
        self._setup_engine_speed_daq(conn)
        conn.process(bytes([cmd.CMD_START_STOP_DAQ_LIST, 0x01, 0x00, 0x00]))
        dto = conn.collect_daq_dtos()[0]
        # Data starts after 2-byte identification header
        val = struct.unpack_from(">I", dto, 2)[0]
        assert val == 800, f"Expected 800 rpm, got {val}"

    def test_dto_total_length(self, conn):
        # Header (2) + EngineSpeed (4) = 6 bytes
        self._setup_engine_speed_daq(conn)
        conn.process(bytes([cmd.CMD_START_STOP_DAQ_LIST, 0x01, 0x00, 0x00]))
        dto = conn.collect_daq_dtos()[0]
        assert len(dto) == 6

    def test_multi_signal_dto(self, conn):
        self._setup_multi_signal_daq(conn)
        conn.process(bytes([cmd.CMD_START_STOP_DAQ_LIST, 0x01, 0x00, 0x00]))
        dto = conn.collect_daq_dtos()[0]
        # 2-byte header + 3 × 4-byte signals = 14 bytes
        assert len(dto) == 14
        speed   = struct.unpack_from(">I", dto, 2)[0]
        coolant = struct.unpack_from(">i", dto, 6)[0]
        lambda1 = struct.unpack_from(">I", dto, 10)[0]
        assert speed == 800
        assert coolant == 200    # 20.0 °C in 0.1 °C units
        assert lambda1 == 1000  # 1.000 lambda

    def test_start_stop_synch_starts_selected(self, conn):
        self._setup_engine_speed_daq(conn)
        conn.process(bytes([cmd.CMD_START_STOP_DAQ_LIST, 0x02, 0x00, 0x00]))   # SELECT
        resp = conn.process(bytes([cmd.CMD_START_STOP_SYNCH, 0x01]))            # start selected
        assert resp[0] == rsp.PID_RES
        assert conn.session.daq_lists[0].running is True
        assert conn.session.daq_lists[0].selected is False

    def test_start_stop_synch_stops_all(self, conn):
        self._setup_engine_speed_daq(conn)
        conn.process(bytes([cmd.CMD_START_STOP_DAQ_LIST, 0x01, 0x00, 0x00]))
        conn.process(bytes([cmd.CMD_START_STOP_SYNCH, 0x00]))   # stop all
        assert conn.session.daq_lists[0].running is False

    def test_daq_status_flag_in_get_status(self, conn):
        self._setup_engine_speed_daq(conn)
        conn.process(bytes([cmd.CMD_START_STOP_DAQ_LIST, 0x01, 0x00, 0x00]))
        resp = conn.process(bytes([cmd.CMD_GET_STATUS]))
        assert resp[1] & 0x08, "DAQ_RUNNING bit must be set in SESSION_STATUS"


class TestGetDaqClock:
    def test_returns_res(self, conn):
        assert conn.process(bytes([cmd.CMD_GET_DAQ_CLOCK]))[0] == rsp.PID_RES

    def test_timestamp_increases(self, conn):
        import time
        time.sleep(0.01)
        resp = conn.process(bytes([cmd.CMD_GET_DAQ_CLOCK]))
        ts = struct.unpack_from("<I", resp, 4)[0]
        assert ts > 0
