"""XCP session state and DAQ data structures."""
from __future__ import annotations
from dataclasses import dataclass, field
from typing import List, Optional


@dataclass
class OdtEntry:
    """One entry within an ODT — maps a memory region to a DTO byte offset."""
    address: int = 0
    address_extension: int = 0
    bit_offset: int = 0xFF   # 0xFF = byte access (no bit offset)
    size: int = 0            # element size in bytes


@dataclass
class Odt:
    """Object Descriptor Table — a collection of ODT entries sent in one DTO."""
    entries: List[OdtEntry] = field(default_factory=list)


@dataclass
class DaqList:
    """One DAQ list with its ODTs and run-time mode flags."""
    mode: int = 0x00          # SET_DAQ_LIST_MODE mode byte (bits: TIMESTAMP, PID_OFF…)
    event_channel: int = 0
    prescaler: int = 1
    priority: int = 0
    odts: List[Odt] = field(default_factory=list)
    running: bool = False
    selected: bool = False    # set by START_STOP_DAQ_LIST mode=SELECT
    first_pid: int = 0        # absolute PID assigned to first ODT of this list


class XcpSession:
    # Resource capability bits (CONNECT response byte 1)
    RESOURCE_CAL_PAG = 0x01
    RESOURCE_DAQ     = 0x04
    RESOURCE_STIM    = 0x08
    RESOURCE_PGM     = 0x10

    # Session status bits (GET_STATUS response byte 1)
    STATUS_DAQ_RUNNING  = 0x08
    STATUS_RESUME       = 0x02

    def __init__(self, max_cto: int = 64, max_dto: int = 64) -> None:
        self.connected: bool = False

        # Transport capabilities (reported in CONNECT response)
        self.max_cto: int = max_cto
        self.max_dto: int = max_dto

        # Available resources: CAL/PAG + DAQ
        self.resource_mask: int = self.RESOURCE_CAL_PAG | self.RESOURCE_DAQ

        # No resources are seed/key protected
        self.protection_status: int = 0x00

        # Internal session status
        self.session_status: int = 0x00

        # Memory Transfer Address (set by SET_MTA, updated by UPLOAD/DOWNLOAD)
        self.mta_address: int = 0x00000000
        self.mta_extension: int = 0x00

        # Pending GET_ID upload length (0 means no pending ID upload)
        self.pending_id_length: int = 0

        # DAQ dynamic allocation
        self.daq_lists: List[DaqList] = []

        # Current DAQ write pointer (SET_DAQ_PTR / WRITE_DAQ)
        self.daq_ptr_list: int = 0
        self.daq_ptr_odt: int = 0
        self.daq_ptr_entry: int = 0

    def reset(self) -> None:
        """Reset to disconnected state, preserving no session data."""
        self.__init__(max_cto=self.max_cto, max_dto=self.max_dto)

    @property
    def daq_running(self) -> bool:
        return any(dl.running for dl in self.daq_lists)

    def update_status_byte(self) -> None:
        if self.daq_running:
            self.session_status |= self.STATUS_DAQ_RUNNING
        else:
            self.session_status &= ~self.STATUS_DAQ_RUNNING
