# models/dm_serial.py
# -*- coding: utf-8 -*-
"""
DM_Serial: Serial port reader for Damiang IMU (supports background read thread + main thread fetches latest on demand)
- Fixed timeout=0 (non-blocking)
- read(): Drain+ParseAll+Latest (can also be called from single thread)
- start_reader()/stop_reader(): Starts internal thread that only refreshes data; no logging
- get_latest(): Main thread fetches latest frame with timestamp/count at any time
- destory()/reopen(): Resource management
- CRC: By default includes frame header 0x55,0xAA; falls back to excluding header if verification fails

Frame format:
[0,1]=0x55,0xAA | [2]=? | [3]=RID | [4:16]=3*float32(LE) | [16:18]=CRC16(LE) | [18]=0x0A
"""

from __future__ import annotations

import struct
import threading
import time
from typing import Optional, Tuple, List

import serial  # pip install pyserial

from .dm_crc import dm_crc16

HDR = b"\x55\xaa"
TAIL = 0x0A
FRAME_LEN = 19
VALID_RIDS = {0x01, 0x02, 0x03}

# Firmware: CRC should include frame header; toggle for other versions; fallback is available.
SKIP_HDR_IN_CRC = False


class DM_Serial:
    def __init__(self, port: str, baudrate: int):
        self.port = port
        self.baudrate = int(baudrate)
        self.timeout = 0.0  # non-blocking
        self.ser: Optional[serial.Serial] = None
        self._buf = bytearray()

        # Statistics
        self.cnt_ok = 0
        self.cnt_crc = 0
        self.cnt_short = 0
        self.cnt_nohdr = 0

        # Background reader thread state
        self._th: Optional[threading.Thread] = None
        self._stop_evt: Optional[threading.Event] = None
        self._read_sleep = 0.001  # Reader thread sleep to control CPU usage

        # Latest data (thread-safe)
        self._latest_lock = threading.Lock()
        self._latest_pkt: Optional[Tuple[int, Tuple[float, float, float]]] = None
        self._latest_ts: float = 0.0
        self._latest_count: int = 0
        self._last_error: Optional[str] = None

        self._open()

    # ------------ Public API ------------
    def read(
        self, max_bytes: int | None = None
    ) -> Optional[Tuple[int, Tuple[float, float, float]]]:
        """Read all currently available bytes from serial, parse complete frames, return only the latest frame."""
        if not self.ser or not self.ser.is_open:
            return None
        self._read_into_buf(max_bytes)
        frames = self._parse_all()
        return frames[-1] if frames else None

    def start_reader(self, read_sleep: float = 0.001) -> bool:
        """Start background thread that only refreshes data; no logging."""
        if self._th and self._th.is_alive():
            self._read_sleep = read_sleep
            return True
        if not self.is_open:
            if not self._open():
                return False
        self._stop_evt = threading.Event()
        self._read_sleep = read_sleep
        self._th = threading.Thread(target=self._reader_loop, daemon=True)
        self._th.start()
        return True

    def stop_reader(self) -> None:
        """Stop the background reader thread."""
        if self._stop_evt:
            self._stop_evt.set()
        if self._th:
            self._th.join(timeout=1.0)
        self._th = None
        self._stop_evt = None

    def get_latest(
        self,
    ) -> Tuple[Optional[Tuple[int, Tuple[float, float, float]]], float, int]:
        """Thread-safely get (pkt, timestamp, count)."""
        with self._latest_lock:
            return self._latest_pkt, self._latest_ts, self._latest_count

    def last_error(self) -> Optional[str]:
        return self._last_error

    def destory(self) -> None:
        """Immediately close the serial port (keeps original spelling)."""
        self.stop_reader()
        if self.ser:
            try:
                self.ser.close()
            finally:
                self.ser = None

    # Alias
    def destroy(self) -> None:
        self.destory()

    def reopen(self) -> bool:
        """Close and reopen the serial port."""
        self.destory()
        return self._open()

    @property
    def is_open(self) -> bool:
        return bool(self.ser and self.ser.is_open)

    # ------------ Internal implementation ------------
    def _open(self) -> bool:
        try:
            self.ser = serial.Serial(
                self.port, self.baudrate, timeout=self.timeout, write_timeout=0
            )
            try:
                self.ser.reset_input_buffer()
            except Exception:
                pass
            return True
        except Exception as e:
            self._last_error = str(e)
            self.ser = None
            return False

    def _reader_loop(self):
        """Background thread: continuously refreshes latest data; no logging."""
        evt = self._stop_evt
        try:
            while evt and not evt.is_set():
                pkt = self.read(None)
                if pkt is not None:
                    with self._latest_lock:
                        self._latest_pkt = pkt
                        self._latest_ts = time.time()
                        self._latest_count += 1
                if self._read_sleep > 0.0:
                    time.sleep(self._read_sleep)
        except Exception as e:
            # No logging; store error string for main thread to query
            self._last_error = f"reader_loop: {e!r}"

    def _read_into_buf(self, max_bytes: Optional[int]) -> int:
        """Read currently available bytes from serial into buffer; returns number of bytes read."""
        n = getattr(self.ser, "in_waiting", 0) if self.ser else 0
        if max_bytes is not None and n > max_bytes:
            n = max_bytes
        if n <= 0:
            return 0
        self._buf.extend(self.ser.read(n))
        return n

    def _parse_all(self) -> List[Tuple[int, Tuple[float, float, float]]]:
        """Parse as many complete frames as possible; return list."""
        results: List[Tuple[int, Tuple[float, float, float]]] = []
        buf = self._buf
        start = 0

        while True:
            j = buf.find(HDR, start)
            if j < 0:
                # Keep only last 1 byte to avoid frame header spanning packets
                keep = buf[-1:] if buf else b""
                self._buf = bytearray(keep)
                if buf:
                    self.cnt_nohdr += 1
                break

            if len(buf) - j < FRAME_LEN:
                # Incomplete frame; keep remainder from frame header onward
                self._buf = bytearray(buf[j:])
                self.cnt_short += 1
                break

            frame = bytes(buf[j : j + FRAME_LEN])
            start = j + 1  # Advance quickly

            # Tail byte check
            if frame[-1] != TAIL:
                continue

            rid = frame[3]
            if rid not in VALID_RIDS:
                continue

            # CRC (default includes header; retry without header if verification fails)
            if SKIP_HDR_IN_CRC:
                crc_calc = dm_crc16(frame[2:16])
            else:
                crc_calc = dm_crc16(frame[0:16])
            crc_wire = frame[16] | (frame[17] << 8)
            if crc_calc != crc_wire:
                alt = (
                    dm_crc16(frame[2:16])
                    if not SKIP_HDR_IN_CRC
                    else dm_crc16(frame[0:16])
                )
                if alt != crc_wire:
                    self.cnt_crc += 1
                    continue

            # Unpack 3 x float32 (LE)
            f1 = struct.unpack("<f", frame[4:8])[0]
            f2 = struct.unpack("<f", frame[8:12])[0]
            f3 = struct.unpack("<f", frame[12:16])[0]
            results.append((rid, (f1, f2, f3)))

            # Discard consumed data (up to frame end) and continue searching from start
            buf = buf[j + FRAME_LEN :]
            start = 0

        if isinstance(buf, (bytes, bytearray)) and buf is not self._buf:
            self._buf = bytearray(buf)

        self.cnt_ok += len(results)
        return results
