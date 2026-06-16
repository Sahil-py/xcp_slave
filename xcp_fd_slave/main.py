#!/usr/bin/env python3
"""
XCP 1.3 CAN FD Slave Simulator — entry point.

Usage:
    python -m xcp_fd_slave.main
    python -m xcp_fd_slave.main --channel vcan0 --log-level DEBUG
"""
import argparse
import logging
import signal
import sys

from .memory.memory_map import MemoryMap
from .protocol.dispatcher import XcpDispatcher
from .protocol.session import XcpSession
from .transport.socketcan_transport import XcpSlaveTransport


def _build_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="XCP 1.3 CAN FD Slave Simulator",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--interface", default="socketcan",
                   help="python-can interface type")
    p.add_argument("--channel", default="vcan0",
                   help="CAN channel name")
    p.add_argument("--rx-id", metavar="HEX", default="0x650",
                   help="Master→Slave CAN arbitration ID (hex)")
    p.add_argument("--tx-id", metavar="HEX", default="0x651",
                   help="Slave→Master CAN arbitration ID (hex)")
    p.add_argument("--bitrate", type=int, default=500_000,
                   help="Nominal CAN bitrate (bps)")
    p.add_argument("--data-bitrate", type=int, default=2_000_000,
                   help="CAN FD data phase bitrate (bps)")
    p.add_argument("--no-fd", action="store_true",
                   help="Use classic CAN frames instead of CAN FD (for non-FD masters)")
    p.add_argument("--log-level", default="INFO",
                   choices=["DEBUG", "INFO", "WARNING", "ERROR"])
    return p.parse_args()


def main() -> None:
    args = _build_args()

    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s.%(msecs)03d [%(levelname)-7s] %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    rx_id = int(args.rx_id, 16)
    tx_id = int(args.tx_id, 16)

    session = XcpSession()
    memory = MemoryMap()
    dispatcher = XcpDispatcher(session, memory)

    transport = XcpSlaveTransport(
        interface=args.interface,
        channel=args.channel,
        rx_id=rx_id,
        tx_id=tx_id,
        bitrate=args.bitrate,
        data_bitrate=args.data_bitrate,
        fd=not args.no_fd,
    )

    def _handle_signal(sig: int, frame: object) -> None:
        logging.getLogger(__name__).info("Signal %d — shutting down", sig)
        transport.stop()

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    logging.getLogger(__name__).info(
        "XCP FD slave starting  channel=%s  RX=0x%03X  TX=0x%03X",
        args.channel, rx_id, tx_id,
    )
    transport.run(dispatcher)


if __name__ == "__main__":
    main()
