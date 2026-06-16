"""
XCP command dispatcher — the protocol brain of the slave.

Each public handle_* method is called by the transport layer with the raw
CTO payload (bytes).  It returns a CTO response (bytes) or None when no
response should be sent (e.g. unconnected slave ignoring commands).

Design rule: this module follows the ASAM XCP specification, never the
master's behaviour.  Deviations from the spec in the master are intentionally
exposed through correct error responses.
"""
from __future__ import annotations

import logging
import struct
import time
from typing import Callable, List, Optional

from .commands import CMD_CONNECT, CMD_DISCONNECT, CMD_GET_STATUS, CMD_SYNCH
from .commands import CMD_GET_COMM_MODE_INFO, CMD_GET_ID
from .commands import CMD_SET_MTA, CMD_UPLOAD, CMD_SHORT_UPLOAD
from .commands import CMD_DOWNLOAD, CMD_DOWNLOAD_NEXT, CMD_DOWNLOAD_MAX
from .commands import CMD_SET_DAQ_PTR, CMD_WRITE_DAQ
from .commands import CMD_SET_DAQ_LIST_MODE, CMD_GET_DAQ_LIST_MODE
from .commands import CMD_START_STOP_DAQ_LIST, CMD_START_STOP_SYNCH
from .commands import CMD_GET_DAQ_CLOCK
from .commands import CMD_GET_DAQ_PROCESSOR_INFO, CMD_GET_DAQ_RESOLUTION_INFO
from .commands import CMD_FREE_DAQ, CMD_ALLOC_DAQ, CMD_ALLOC_ODT, CMD_ALLOC_ODT_ENTRY
from .commands import CMD_CLEAR_DAQ_LIST
from .commands import name as cmd_name
from . import errors
from .responses import PID_RES, PID_ERR, build_res, build_err
from .session import XcpSession, DaqList, Odt, OdtEntry
from ..memory.memory_map import MemoryMap, ADDR_ID_STRING_AREA

log = logging.getLogger(__name__)

# Identification type codes (GET_ID request byte 1)
ID_TYPE_ASCII           = 0x00   # plain ASCII ECU name
ID_TYPE_ASAM_MC2_WO_EXT = 0x01  # A2L filename without extension
ID_TYPE_ASAM_MC2_W_EXT  = 0x02  # A2L filename with extension
ID_TYPE_ASAM_MC2_URL    = 0x03  # A2L URL
ID_TYPE_ASAM_MC2_UPLOAD = 0x04  # A2L file upload via MTA


class XcpDispatcher:
    """Stateful XCP command processor."""

    def __init__(self, session: XcpSession, memory: MemoryMap) -> None:
        self.session = session
        self.memory = memory
        self._start_time = time.monotonic()

        self._handlers = {
            CMD_CONNECT:                self._connect,
            CMD_DISCONNECT:             self._disconnect,
            CMD_GET_STATUS:             self._get_status,
            CMD_SYNCH:                  self._synch,
            CMD_GET_COMM_MODE_INFO:     self._get_comm_mode_info,
            CMD_GET_ID:                 self._get_id,
            CMD_SET_MTA:                self._set_mta,
            CMD_UPLOAD:                 self._upload,
            CMD_SHORT_UPLOAD:           self._short_upload,
            CMD_DOWNLOAD:               self._download,
            CMD_DOWNLOAD_NEXT:          self._download_next,
            CMD_DOWNLOAD_MAX:           self._download_max,
            CMD_GET_DAQ_PROCESSOR_INFO: self._get_daq_processor_info,
            CMD_GET_DAQ_RESOLUTION_INFO: self._get_daq_resolution_info,
            CMD_FREE_DAQ:               self._free_daq,
            CMD_ALLOC_DAQ:              self._alloc_daq,
            CMD_ALLOC_ODT:              self._alloc_odt,
            CMD_ALLOC_ODT_ENTRY:        self._alloc_odt_entry,
            CMD_CLEAR_DAQ_LIST:         self._clear_daq_list,
            CMD_SET_DAQ_PTR:            self._set_daq_ptr,
            CMD_WRITE_DAQ:              self._write_daq,
            CMD_SET_DAQ_LIST_MODE:      self._set_daq_list_mode,
            CMD_GET_DAQ_LIST_MODE:      self._get_daq_list_mode,
            CMD_START_STOP_DAQ_LIST:    self._start_stop_daq_list,
            CMD_START_STOP_SYNCH:       self._start_stop_synch,
            CMD_GET_DAQ_CLOCK:          self._get_daq_clock,
        }

    # ------------------------------------------------------------------
    # Entry point
    # ------------------------------------------------------------------

    def process(self, data: bytes) -> Optional[bytes]:
        """
        Parse one CTO frame and return the response CTO (or None).

        The caller must not send anything when None is returned.
        """
        if not data:
            return None

        cmd = data[0]
        log.debug("RX: %s  raw=%s", cmd_name(cmd), data.hex())

        # CONNECT is the only command accepted while disconnected
        if not self.session.connected and cmd != CMD_CONNECT:
            log.debug("Ignoring command %s — not connected", cmd_name(cmd))
            return None

        handler = self._handlers.get(cmd, self._unknown_command)
        try:
            response = handler(data)
        except Exception as exc:  # noqa: BLE001
            log.exception("Unhandled exception in handler for %s: %s", cmd_name(cmd), exc)
            response = build_err(errors.ERR_GENERIC)

        if response is not None:
            log.debug("TX: %s", response.hex())
        return response

    # ------------------------------------------------------------------
    # DAQ tick — called periodically by the transport layer
    # ------------------------------------------------------------------

    def collect_daq_dtos(self) -> List[bytes]:
        """Sample memory for all running DAQ lists and return DTO packets."""
        dtos: List[bytes] = []
        self.session.update_status_byte()

        for list_idx, daq_list in enumerate(self.session.daq_lists):
            if not daq_list.running:
                continue
            for odt_idx, odt in enumerate(daq_list.odts):
                dto = self._build_dto(list_idx, odt_idx, odt)
                if dto:
                    dtos.append(dto)
        return dtos

    # ------------------------------------------------------------------
    # Phase 1 — connection management
    # ------------------------------------------------------------------

    def _connect(self, data: bytes) -> bytes:
        if len(data) < 2:
            return build_err(errors.ERR_CMD_SYNTAX)

        # A new CONNECT always resets an existing session (spec section 3.3)
        if self.session.connected:
            log.info("Re-connecting: resetting existing session")
            self.session.reset()

        self.session.connected = True
        log.info("CONNECT accepted (mode=0x%02X)", data[1])

        # Response layout (8 bytes):
        #  [0] PID_RES
        #  [1] RESOURCE    — CAL/PAG | DAQ
        #  [2] COMM_MODE_BASIC — little-endian, byte AG, OPTIONAL=1
        #  [3] MAX_CTO
        #  [4-5] MAX_DTO   (little-endian)
        #  [6] XCP Protocol Layer version qualifier (0x01 = 1.x)
        #  [7] XCP Transport Layer version qualifier (0x01)
        resource = self.session.resource_mask
        comm_mode_basic = (
            0b10_000_000   # BYTE_ORDER=1 (Motorola/big-endian per A2L MSB_FIRST), AG=00 (byte)
            | (1 << 1)     # OPTIONAL=1 → GET_COMM_MODE_INFO is available
        )
        return build_res(bytes([
            resource,
            comm_mode_basic,
            self.session.max_cto,
            self.session.max_dto & 0xFF,
            (self.session.max_dto >> 8) & 0xFF,
            0x01,   # XCP protocol layer version qualifier
            0x01,   # XCP transport layer version qualifier
        ]))

    def _disconnect(self, data: bytes) -> bytes:
        log.info("DISCONNECT")
        self.session.reset()
        return build_res()

    def _get_status(self, data: bytes) -> bytes:
        self.session.update_status_byte()
        # Response:
        #  [0] PID_RES
        #  [1] SESSION_STATUS
        #  [2] RESOURCE_PROTECTION_STATUS (0 = nothing locked)
        #  [3] STATE_NUMBER (reserved, 0)
        #  [4-5] SESSION_CONFIGURATION_ID
        return build_res(bytes([
            self.session.session_status,
            self.session.protection_status,
            0x00,
            0x00, 0x00,  # SESSION_CONFIGURATION_ID
        ]))

    def _synch(self, data: bytes) -> bytes:
        # SYNCH always returns ERR_CMD_SYNCH — this is mandated by the spec
        return build_err(errors.ERR_CMD_SYNCH)

    def _get_comm_mode_info(self, data: bytes) -> bytes:
        # Response:
        #  [0] PID_RES
        #  [1] reserved
        #  [2] COMM_MODE_OPTIONAL (bits: MASTER_BLOCK_MODE=0, INTERLEAVED_MODE=0)
        #  [3] reserved
        #  [4] MAX_BS   (0 = master block mode not supported)
        #  [5] MIN_ST   (0 = no minimum separation time)
        #  [6] QUEUE_SIZE (0 = no interleaved)
        #  [7] XCP_DRIVER_VERSION_NUMBER
        return build_res(bytes([
            0x00,  # reserved
            0x00,  # COMM_MODE_OPTIONAL
            0x00,  # reserved
            0x00,  # MAX_BS
            0x00,  # MIN_ST
            0x00,  # QUEUE_SIZE
            0x01,  # driver version
        ]))

    def _get_id(self, data: bytes) -> bytes:
        if len(data) < 2:
            return build_err(errors.ERR_CMD_SYNTAX)

        id_type = data[1]
        # Names and paths taken directly from sample.a2l PROJECT / HEADER
        id_map = {
            ID_TYPE_ASCII:            "CANdoit_Test",
            ID_TYPE_ASAM_MC2_WO_EXT: "sample",
            ID_TYPE_ASAM_MC2_W_EXT:  "sample.a2l",
            ID_TYPE_ASAM_MC2_URL:    "file:///a2l/sample.a2l",
            ID_TYPE_ASAM_MC2_UPLOAD: "sample.a2l",
        }
        id_text = id_map.get(id_type, "XCP_FD_TEST_SLAVE")
        addr = self.memory.store_id_string(id_text)

        # Set MTA to the stored string so the master can UPLOAD it
        self.session.mta_address = addr
        self.session.mta_extension = 0x00
        self.session.pending_id_length = len(id_text)

        length = len(id_text)
        # Response:
        #  [0] PID_RES
        #  [1] MODE (bit0=compressed/encrypted=0; bit1=transfer-mode: 0=via UPLOAD)
        #  [2-3] reserved
        #  [4-7] LENGTH (little-endian 32-bit)
        return build_res(bytes([
            0x00,   # MODE — not compressed, use UPLOAD
            0x00,   # reserved
            0x00,   # reserved
        ]) + struct.pack("<I", length))

    # ------------------------------------------------------------------
    # Phase 2 — memory access
    # ------------------------------------------------------------------

    def _set_mta(self, data: bytes) -> bytes:
        # Byte 1-2: reserved; Byte 3: addr extension; Bytes 4-7: address
        if len(data) < 8:
            return build_err(errors.ERR_CMD_SYNTAX)
        self.session.mta_extension = data[3]
        self.session.mta_address = struct.unpack_from("<I", data, 4)[0]
        log.debug("SET_MTA addr=0x%08X ext=0x%02X",
                  self.session.mta_address, self.session.mta_extension)
        return build_res()

    def _upload(self, data: bytes) -> bytes:
        if len(data) < 2:
            return build_err(errors.ERR_CMD_SYNTAX)
        n = data[1]
        # Response is PID_RES + n data bytes; total must not exceed MAX_CTO
        if n > self.session.max_cto - 1:
            return build_err(errors.ERR_OUT_OF_RANGE)
        try:
            payload = self.memory.read(self.session.mta_address, n)
        except ValueError:
            log.warning("UPLOAD out of range: addr=0x%08X n=%d",
                        self.session.mta_address, n)
            return build_err(errors.ERR_OUT_OF_RANGE)

        self.session.mta_address += n
        # Response: PID_RES + DATA (byte AG, no alignment padding needed)
        return build_res(payload)

    def _short_upload(self, data: bytes) -> bytes:
        # Byte 1: NR; Byte 2: reserved; Byte 3: addr ext; Bytes 4-7: addr
        if len(data) < 8:
            return build_err(errors.ERR_CMD_SYNTAX)
        n = data[1]
        ext = data[3]
        addr = struct.unpack_from("<I", data, 4)[0]
        try:
            payload = self.memory.read(addr, n)
        except ValueError:
            log.warning("SHORT_UPLOAD out of range: addr=0x%08X n=%d", addr, n)
            return build_err(errors.ERR_OUT_OF_RANGE)

        # SHORT_UPLOAD also updates MTA
        self.session.mta_address = addr + n
        self.session.mta_extension = ext
        return build_res(payload)

    def _download(self, data: bytes) -> bytes:
        # Byte 0: CMD; Byte 1: NR_OF_DATA_ELEMENTS; Bytes 2+: DATA (byte AG)
        if len(data) < 2:
            return build_err(errors.ERR_CMD_SYNTAX)
        n = data[1]
        # Request is CMD + NR + n data bytes; total must not exceed MAX_CTO
        if n > self.session.max_cto - 2:
            return build_err(errors.ERR_OUT_OF_RANGE)
        if len(data) < 2 + n:
            return build_err(errors.ERR_CMD_SYNTAX)
        payload = bytes(data[2: 2 + n])
        try:
            self.memory.write(self.session.mta_address, payload)
        except ValueError:
            return build_err(errors.ERR_OUT_OF_RANGE)
        self.session.mta_address += n
        return build_res()

    def _download_next(self, data: bytes) -> bytes:
        # Block mode continuation — Byte 1: NR_OF_DATA_ELEMENTS; Bytes 2+: DATA
        # We do not advertise MASTER_BLOCK_MODE so this should not arrive in
        # normal use.  Accept it anyway and apply the same size limit as DOWNLOAD.
        if len(data) < 2:
            return build_err(errors.ERR_CMD_SYNTAX)
        n = data[1]
        if n > self.session.max_cto - 2:
            return build_err(errors.ERR_OUT_OF_RANGE)
        if len(data) < 2 + n:
            return build_err(errors.ERR_CMD_SYNTAX)
        payload = bytes(data[2: 2 + n])
        try:
            self.memory.write(self.session.mta_address, payload)
        except ValueError:
            return build_err(errors.ERR_OUT_OF_RANGE)
        self.session.mta_address += n
        return build_res()

    def _download_max(self, data: bytes) -> bytes:
        # Transfers exactly MAX_CTO-1 bytes starting from MTA
        n = self.session.max_cto - 1
        if len(data) < 1 + n:
            return build_err(errors.ERR_CMD_SYNTAX)
        payload = bytes(data[1: 1 + n])
        try:
            self.memory.write(self.session.mta_address, payload)
        except ValueError:
            return build_err(errors.ERR_OUT_OF_RANGE)
        self.session.mta_address += n
        return build_res()

    # ------------------------------------------------------------------
    # Phase 3 — DAQ processor info
    # ------------------------------------------------------------------

    def _get_daq_processor_info(self, data: bytes) -> bytes:
        # Response:
        #  [0] PID_RES
        #  [1] DAQ_PROPERTIES
        #       Bit 0: DAQ_CONFIG_TYPE = 1 (dynamic)
        #  [2-3] MAX_DAQ (little-endian, 0 = no limit, but we cap to 16)
        #  [4-5] MAX_EVENT_CHANNEL
        #  [6] MIN_DAQ (0 = all dynamic)
        #  [7] DAQ_KEY_BYTE
        #       Bits 7-6: OPTIMISATION_TYPE = 00
        #       Bits 5-4: ADDRESS_EXTENSION = 00 (same per DAQ list)
        #       Bits 3-2: IDENTIFICATION_FIELD_TYPE = 01
        #                 (relative ODT num + absolute DAQ list num, 2-byte header)
        #       Bits 1-0: GRANULARITY_DAQ_ODT_ENTRY_SIZE = 00 (byte)
        daq_properties = 0x01          # dynamic config
        max_daq = 16
        max_event_ch = 2
        min_daq = 0
        daq_key_byte = (0b00 << 6) | (0b00 << 4) | (0b01 << 2) | 0b00  # = 0x04
        return build_res(bytes([
            daq_properties,
            max_daq & 0xFF,
            (max_daq >> 8) & 0xFF,
            max_event_ch & 0xFF,
            (max_event_ch >> 8) & 0xFF,
            min_daq,
            daq_key_byte,
        ]))

    def _get_daq_resolution_info(self, data: bytes) -> bytes:
        # Response:
        #  [0] PID_RES
        #  [1] GRANULARITY_ODT_ENTRY_SIZE_DAQ  (1 = byte)
        #  [2] MAX_ODT_ENTRY_SIZE_DAQ          (MAX_DTO - 2-byte header)
        #  [3] GRANULARITY_ODT_ENTRY_SIZE_STIM (1, STIM not enabled but filled)
        #  [4] MAX_ODT_ENTRY_SIZE_STIM         (0, not supported)
        #  [5] TIMESTAMP_MODE                  (0 = no timestamp)
        #  [6-7] TIMESTAMP_TICKS               (0, not used)
        max_odt_entry = self.session.max_dto - 2   # subtract 2-byte identification header
        return build_res(bytes([
            0x01,           # GRANULARITY_DAQ
            max_odt_entry,  # MAX_ODT_ENTRY_SIZE_DAQ
            0x01,           # GRANULARITY_STIM
            0x00,           # MAX_ODT_ENTRY_SIZE_STIM
            0x00,           # TIMESTAMP_MODE
            0x00, 0x00,     # TIMESTAMP_TICKS
        ]))

    # ------------------------------------------------------------------
    # Phase 3 — DAQ configuration
    # ------------------------------------------------------------------

    def _free_daq(self, data: bytes) -> bytes:
        if self.session.daq_running:
            return build_err(errors.ERR_DAQ_ACTIVE)
        self.session.daq_lists.clear()
        log.info("FREE_DAQ: all DAQ lists released")
        return build_res()

    def _alloc_daq(self, data: bytes) -> bytes:
        # Bytes 2-3: COUNT
        if len(data) < 4:
            return build_err(errors.ERR_CMD_SYNTAX)
        if self.session.daq_running:
            return build_err(errors.ERR_DAQ_ACTIVE)
        count = struct.unpack_from("<H", data, 2)[0]
        if len(self.session.daq_lists) + count > 16:
            return build_err(errors.ERR_MEMORY_OVERFLOW)
        for _ in range(count):
            self.session.daq_lists.append(DaqList())
        log.info("ALLOC_DAQ: allocated %d list(s), total=%d", count, len(self.session.daq_lists))
        return build_res()

    def _alloc_odt(self, data: bytes) -> bytes:
        # Bytes 2-3: DAQ_LIST_NUMBER; Byte 4: COUNT
        if len(data) < 5:
            return build_err(errors.ERR_CMD_SYNTAX)
        if self.session.daq_running:
            return build_err(errors.ERR_DAQ_ACTIVE)
        list_num = struct.unpack_from("<H", data, 2)[0]
        count = data[4]
        if list_num >= len(self.session.daq_lists):
            return build_err(errors.ERR_OUT_OF_RANGE)
        daq_list = self.session.daq_lists[list_num]
        if len(daq_list.odts) + count > 252:  # ODT numbers 0x00..0xFB are valid
            return build_err(errors.ERR_MEMORY_OVERFLOW)
        for _ in range(count):
            daq_list.odts.append(Odt())
        log.debug("ALLOC_ODT: list=%d ODTs=%d", list_num, len(daq_list.odts))
        return build_res()

    def _alloc_odt_entry(self, data: bytes) -> bytes:
        # Bytes 2-3: DAQ_LIST_NUMBER; Byte 4: ODT_NUMBER; Byte 5: ENTRIES_COUNT
        if len(data) < 6:
            return build_err(errors.ERR_CMD_SYNTAX)
        if self.session.daq_running:
            return build_err(errors.ERR_DAQ_ACTIVE)
        list_num = struct.unpack_from("<H", data, 2)[0]
        odt_num = data[4]
        count = data[5]
        if list_num >= len(self.session.daq_lists):
            return build_err(errors.ERR_OUT_OF_RANGE)
        daq_list = self.session.daq_lists[list_num]
        if odt_num >= len(daq_list.odts):
            return build_err(errors.ERR_OUT_OF_RANGE)
        odt = daq_list.odts[odt_num]
        for _ in range(count):
            odt.entries.append(OdtEntry())
        log.debug("ALLOC_ODT_ENTRY: list=%d odt=%d entries=%d",
                  list_num, odt_num, len(odt.entries))
        return build_res()

    def _clear_daq_list(self, data: bytes) -> bytes:
        # Bytes 2-3: DAQ_LIST_NUMBER
        if len(data) < 4:
            return build_err(errors.ERR_CMD_SYNTAX)
        list_num = struct.unpack_from("<H", data, 2)[0]
        if list_num >= len(self.session.daq_lists):
            return build_err(errors.ERR_OUT_OF_RANGE)
        daq_list = self.session.daq_lists[list_num]
        if daq_list.running:
            return build_err(errors.ERR_DAQ_ACTIVE)
        for odt in daq_list.odts:
            for entry in odt.entries:
                entry.address = 0
                entry.address_extension = 0
                entry.bit_offset = 0xFF
                entry.size = 0
        return build_res()

    def _set_daq_ptr(self, data: bytes) -> bytes:
        # Byte 1: reserved; Bytes 2-3: list num; Byte 4: ODT num; Byte 5: entry num
        if len(data) < 6:
            return build_err(errors.ERR_CMD_SYNTAX)
        list_num = struct.unpack_from("<H", data, 2)[0]
        odt_num = data[4]
        entry_num = data[5]
        if list_num >= len(self.session.daq_lists):
            return build_err(errors.ERR_OUT_OF_RANGE)
        daq_list = self.session.daq_lists[list_num]
        if odt_num >= len(daq_list.odts):
            return build_err(errors.ERR_OUT_OF_RANGE)
        odt = daq_list.odts[odt_num]
        if entry_num >= len(odt.entries):
            return build_err(errors.ERR_OUT_OF_RANGE)
        self.session.daq_ptr_list = list_num
        self.session.daq_ptr_odt = odt_num
        self.session.daq_ptr_entry = entry_num
        log.debug("SET_DAQ_PTR list=%d odt=%d entry=%d", list_num, odt_num, entry_num)
        return build_res()

    def _write_daq(self, data: bytes) -> bytes:
        # Byte 1: BIT_OFFSET; Byte 2: SIZE; Byte 3: addr ext; Bytes 4-7: address
        if len(data) < 8:
            return build_err(errors.ERR_CMD_SYNTAX)
        bit_offset = data[1]
        size = data[2]
        addr_ext = data[3]
        address = struct.unpack_from("<I", data, 4)[0]

        li = self.session.daq_ptr_list
        oi = self.session.daq_ptr_odt
        ei = self.session.daq_ptr_entry

        if li >= len(self.session.daq_lists):
            return build_err(errors.ERR_SEQUENCE)
        daq_list = self.session.daq_lists[li]
        if oi >= len(daq_list.odts):
            return build_err(errors.ERR_SEQUENCE)
        odt = daq_list.odts[oi]
        if ei >= len(odt.entries):
            return build_err(errors.ERR_SEQUENCE)

        entry = odt.entries[ei]
        entry.address = address
        entry.address_extension = addr_ext
        entry.bit_offset = bit_offset
        entry.size = size

        log.debug("WRITE_DAQ list=%d odt=%d entry=%d → addr=0x%08X size=%d",
                  li, oi, ei, address, size)

        # Advance pointer to next entry
        self.session.daq_ptr_entry += 1
        return build_res()

    def _set_daq_list_mode(self, data: bytes) -> bytes:
        # Byte 1: mode; Bytes 2-3: list num; Bytes 4-5: event ch; Byte 6: prescaler; Byte 7: priority
        if len(data) < 8:
            return build_err(errors.ERR_CMD_SYNTAX)
        mode = data[1]
        list_num = struct.unpack_from("<H", data, 2)[0]
        event_ch = struct.unpack_from("<H", data, 4)[0]
        prescaler = data[6]
        priority = data[7]

        if list_num >= len(self.session.daq_lists):
            return build_err(errors.ERR_OUT_OF_RANGE)

        daq_list = self.session.daq_lists[list_num]
        if daq_list.running:
            return build_err(errors.ERR_DAQ_ACTIVE)

        daq_list.mode = mode & 0b11110110   # mask out RUNNING and SELECTED bits
        daq_list.event_channel = event_ch
        daq_list.prescaler = max(1, prescaler)
        daq_list.priority = priority
        log.info("SET_DAQ_LIST_MODE list=%d mode=0x%02X event_ch=%d prescaler=%d",
                 list_num, mode, event_ch, prescaler)
        return build_res()

    def _get_daq_list_mode(self, data: bytes) -> bytes:
        # Bytes 2-3: list num
        if len(data) < 4:
            return build_err(errors.ERR_CMD_SYNTAX)
        list_num = struct.unpack_from("<H", data, 2)[0]
        if list_num >= len(self.session.daq_lists):
            return build_err(errors.ERR_OUT_OF_RANGE)
        dl = self.session.daq_lists[list_num]

        mode = dl.mode
        if dl.running:
            mode |= 0x10   # RUNNING bit
        if dl.selected:
            mode |= 0x01   # SELECTED bit

        return build_res(bytes([
            mode,
            0x00, 0x00,    # reserved
            dl.event_channel & 0xFF,
            (dl.event_channel >> 8) & 0xFF,
            dl.prescaler,
            dl.priority,
        ]))

    def _start_stop_daq_list(self, data: bytes) -> bytes:
        # Byte 1: mode (0=stop, 1=start, 2=select); Bytes 2-3: list num
        if len(data) < 4:
            return build_err(errors.ERR_CMD_SYNTAX)
        mode = data[1]
        list_num = struct.unpack_from("<H", data, 2)[0]
        if list_num >= len(self.session.daq_lists):
            return build_err(errors.ERR_OUT_OF_RANGE)
        dl = self.session.daq_lists[list_num]

        if mode == 0x00:        # STOP
            dl.running = False
            dl.selected = False
        elif mode == 0x01:      # START
            dl.running = True
            dl.selected = False
        elif mode == 0x02:      # SELECT (for START_STOP_SYNCH)
            dl.selected = True
        else:
            return build_err(errors.ERR_MODE_NOT_VALID)

        log.info("START_STOP_DAQ_LIST list=%d mode=%d running=%s",
                 list_num, mode, dl.running)

        self.session.update_status_byte()

        # Response: PID_RES + FIRST_PID of this DAQ list
        return build_res(bytes([dl.first_pid]))

    def _start_stop_synch(self, data: bytes) -> bytes:
        # Byte 1: mode (0=stop all, 1=start selected, 2=stop selected)
        if len(data) < 2:
            return build_err(errors.ERR_CMD_SYNTAX)
        mode = data[1]

        if mode == 0x00:   # stop all
            for dl in self.session.daq_lists:
                dl.running = False
                dl.selected = False
        elif mode == 0x01:  # start selected
            for dl in self.session.daq_lists:
                if dl.selected:
                    dl.running = True
                    dl.selected = False
        elif mode == 0x02:  # stop selected
            for dl in self.session.daq_lists:
                if dl.selected:
                    dl.running = False
                    dl.selected = False
        else:
            return build_err(errors.ERR_MODE_NOT_VALID)

        self.session.update_status_byte()
        log.info("START_STOP_SYNCH mode=%d", mode)
        return build_res()

    def _get_daq_clock(self, data: bytes) -> bytes:
        # Returns a 32-bit µs-resolution timestamp
        elapsed_us = int((time.monotonic() - self._start_time) * 1_000_000) & 0xFFFFFFFF
        return build_res(bytes([0x00, 0x00, 0x00]) + struct.pack("<I", elapsed_us))

    # ------------------------------------------------------------------
    # Fallback
    # ------------------------------------------------------------------

    def _unknown_command(self, data: bytes) -> bytes:
        log.warning("Unknown command 0x%02X", data[0])
        return build_err(errors.ERR_CMD_UNKNOWN)

    # ------------------------------------------------------------------
    # DAQ DTO assembly (identification field type 01: ODT + DAQ list num)
    # ------------------------------------------------------------------

    def _build_dto(self, list_idx: int, odt_idx: int, odt: Odt) -> Optional[bytes]:
        """Build one DTO packet for the given ODT; return None on error."""
        payload = bytearray()
        for entry in odt.entries:
            if entry.size == 0:
                continue
            try:
                raw = self.memory.read(entry.address, entry.size)
            except ValueError:
                log.warning("DAQ entry read failed: addr=0x%08X size=%d", entry.address, entry.size)
                return None
            payload.extend(raw)

        if not payload:
            return None

        # 2-byte identification header: [ODT number, DAQ list number]
        header = bytes([odt_idx & 0xFF, list_idx & 0xFF])
        return header + bytes(payload)
