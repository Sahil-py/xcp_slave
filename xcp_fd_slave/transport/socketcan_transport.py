"""
SocketCAN / CAN FD transport layer for the XCP slave.

Responsibilities:
  - Receive CAN FD frames from the master (arbitration ID = rx_id)
  - Pass the payload to the XcpDispatcher
  - Send CAN FD response frames (arbitration ID = tx_id)
  - Run a background thread that ticks the DAQ engine and fires DTOs

The main thread calls run(), which blocks until stop() is called from a
signal handler or another thread.
"""
from __future__ import annotations

import logging
import threading
import time
from typing import TYPE_CHECKING

import can

if TYPE_CHECKING:
    from ..protocol.dispatcher import XcpDispatcher

log = logging.getLogger(__name__)

# DAQ DTO transmission interval in seconds.  10 ms gives a 100 Hz maximum rate;
# individual DAQ lists can run slower via their prescaler.
_DAQ_TICK_INTERVAL = 0.010


class XcpSlaveTransport:
    """CAN FD transport for the XCP slave — one rx ID, one tx ID."""

    def __init__(
        self,
        interface: str = "socketcan",
        channel: str = "vcan0",
        rx_id: int = 0x650,
        tx_id: int = 0x651,
        bitrate: int = 500_000,
        data_bitrate: int = 2_000_000,
        fd: bool = True,
    ) -> None:
        self._interface = interface
        self._channel = channel
        self._rx_id = rx_id
        self._tx_id = tx_id
        self._bitrate = bitrate
        self._data_bitrate = data_bitrate
        self._fd = fd
        self._bus: can.Bus | None = None
        self._stop_event = threading.Event()
        self._daq_thread: threading.Thread | None = None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def run(self, dispatcher: XcpDispatcher) -> None:
        """Open the CAN interface and block until stop() is called."""
        bus_kwargs: dict = dict(
            interface=self._interface,
            channel=self._channel,
            bitrate=self._bitrate,
        )
        if self._fd:
            bus_kwargs["fd"] = True
            bus_kwargs["data_bitrate"] = self._data_bitrate
        self._bus = can.Bus(**bus_kwargs)
        mode = "FD" if self._fd else "classic"
        log.info("CAN %s bus opened: %s/%s  RX=0x%03X  TX=0x%03X", mode,
                 self._interface, self._channel, self._rx_id, self._tx_id)
        if self._fd:
            log.info("FD mode: interface must be up with 'fd on dbitrate %d'", self._data_bitrate)
        else:
            log.info("Classic CAN mode: MAX_CTO=8, DTOs max 8 bytes, 2 ODTs per DAQ list")

        self._stop_event.clear()
        self._daq_thread = threading.Thread(
            target=self._daq_worker,
            args=(dispatcher,),
            daemon=True,
            name="daq-tx",
        )
        self._daq_thread.start()

        try:
            self._rx_loop(dispatcher)
        finally:
            self._bus.shutdown()
            self._bus = None
            log.info("CAN bus closed")

    def stop(self) -> None:
        """Signal the transport to stop (call from signal handler or another thread)."""
        self._stop_event.set()

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _rx_loop(self, dispatcher: XcpDispatcher) -> None:
        while not self._stop_event.is_set():
            try:
                msg = self._bus.recv(timeout=0.1)
            except can.CanOperationError as exc:
                log.error("CAN receive error: %s", exc)
                continue

            if msg is None:
                continue
            if msg.arbitration_id != self._rx_id:
                continue
            log.info("RX 0x%03X [%d] %s  (fd=%s ext=%s)",
                     msg.arbitration_id, len(msg.data), bytes(msg.data).hex(),
                     msg.is_fd, msg.is_extended_id)
            if self._fd and not msg.is_fd:
                log.warning("RX classic CAN frame on FD bus — slave will reply as FD; if TX fails use --no-fd")

            response = dispatcher.process(bytes(msg.data))
            if response is not None:
                self._send(response)

    def _daq_worker(self, dispatcher: XcpDispatcher) -> None:
        """Background thread: update simulation variables and transmit DAQ DTOs."""
        t0 = time.monotonic()
        while not self._stop_event.is_set():
            t = time.monotonic() - t0
            dispatcher.memory.simulate_tick(t)
            for dto in dispatcher.collect_daq_dtos():
                self._send(dto)
            time.sleep(_DAQ_TICK_INTERVAL)

    def _send(self, data: bytes) -> None:
        if self._bus is None:
            return
        try:
            msg = can.Message(
                arbitration_id=self._tx_id,
                is_extended_id=False,
                is_fd=self._fd,
                bitrate_switch=self._fd,
                data=data,
            )
            self._bus.send(msg)
            log.info("TX 0x%03X [%d] %s  (fd=%s)", self._tx_id, len(data), data.hex(), self._fd)
        except can.CanOperationError as exc:
            log.error("TX FAILED 0x%03X [%d] %s  fd=%s  error=%s",
                      self._tx_id, len(data), data.hex(), self._fd, exc)
