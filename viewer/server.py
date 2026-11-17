#!/usr/bin/env python3
"""
High-performance Serial -> Binary WebSocket bridge & static HTTP server
for NUCLEO-F767ZI (BMP280 I2C + MPU-6050 I2C) Three.js Flight Visualizer.
"""

from __future__ import annotations

import argparse
import asyncio
from dataclasses import dataclass
from functools import partial
import http.server
import json
import os
from pathlib import Path
import re
import socketserver
import struct
import threading
import time
from typing import Optional

import serial
from serial.tools import list_ports
import websockets
from websockets.asyncio.server import ServerConnection, serve as ws_serve

# Exact 56-byte IEEE-754 little-endian binary packet layout:
# [0]rollDeg, [1]pitchDeg, [2]yawDeg,
# [3]accelXG, [4]accelYG, [5]accelZG,
# [6]gyroXDps, [7]gyroYDps, [8]gyroZDps,
# [9]mpuTempC, [10]bmpTempC, [11]bmpPressHpa, [12]bmpAltM, [13]packetRateHz
FRAME_STRUCT = struct.Struct("<14f")

_FLOAT_RE = r"([+-]?\d+(?:\.\d+)?)"
BMP_LINE_RE = re.compile(
    rf"BMP280[^\]]*\]\s*Temp:\s*{_FLOAT_RE}\s*(?:degC|°C)\s*\|\s*"
    rf"Press:\s*{_FLOAT_RE}\s*hPa.*?\|\s*Hypsometric Alt:\s*{_FLOAT_RE}\s*m"
)
MPU_LINE_RE = re.compile(
    rf"MPU6050[^\]]*\]\s*Acc:\s*{_FLOAT_RE}\s*,\s*{_FLOAT_RE}\s*,\s*{_FLOAT_RE}\s*g\s*\|\s*"
    rf"Gyro:\s*{_FLOAT_RE}\s*,\s*{_FLOAT_RE}\s*,\s*{_FLOAT_RE}\s*dps\s*\|\s*"
    rf"Temp:\s*{_FLOAT_RE}\s*(?:degC|°C)\s*\|\s*"
    rf"Pose\s*R:\s*{_FLOAT_RE}\s*P:\s*{_FLOAT_RE}\s*Y:\s*{_FLOAT_RE}\s*deg"
)


@dataclass
class TelemetryState:
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0
    accel_x_g: float = 0.0
    accel_y_g: float = 0.0
    accel_z_g: float = 1.0
    gyro_x_dps: float = 0.0
    gyro_y_dps: float = 0.0
    gyro_z_dps: float = 0.0
    mpu_temp_c: float = 25.0
    bmp_temp_c: float = 25.0
    bmp_press_hpa: float = 1013.25
    bmp_alt_m: float = 0.0
    packet_rate_hz: float = 0.0

    def pack(self) -> bytes:
        return FRAME_STRUCT.pack(
            self.roll_deg,
            self.pitch_deg,
            self.yaw_deg,
            self.accel_x_g,
            self.accel_y_g,
            self.accel_z_g,
            self.gyro_x_dps,
            self.gyro_y_dps,
            self.gyro_z_dps,
            self.mpu_temp_c,
            self.bmp_temp_c,
            self.bmp_press_hpa,
            self.bmp_alt_m,
            self.packet_rate_hz,
        )


def parse_telemetry_line(line: str, state: TelemetryState) -> Optional[str]:
    """Parse a single ASCII serial line into TelemetryState. Returns 'mpu', 'bmp', or None."""
    mpu_match = MPU_LINE_RE.search(line)
    if mpu_match:
        vals = [float(g) for g in mpu_match.groups()]
        state.accel_x_g, state.accel_y_g, state.accel_z_g = vals[0], vals[1], vals[2]
        state.gyro_x_dps, state.gyro_y_dps, state.gyro_z_dps = vals[3], vals[4], vals[5]
        state.mpu_temp_c = vals[6]
        state.roll_deg, state.pitch_deg, state.yaw_deg = vals[7], vals[8], vals[9]
        return "mpu"

    bmp_match = BMP_LINE_RE.search(line)
    if bmp_match:
        vals = [float(g) for g in bmp_match.groups()]
        state.bmp_temp_c = vals[0]
        state.bmp_press_hpa = vals[1]
        state.bmp_alt_m = vals[2]
        return "bmp"

    return None


def discover_serial_ports() -> list[dict[str, str]]:
    ports = []
    for p in sorted(list_ports.comports(), key=lambda x: x.device):
        ports.append({
            "device": p.device,
            "description": p.description or "",
        })
    return ports


def select_default_port(ports: list[dict[str, str]]) -> Optional[str]:
    for p in ports:
        dev = p["device"]
        if "ttyACM" in dev or "ttyUSB" in dev or "usbmodem" in dev:
            return dev
    return ports[0]["device"] if ports else None


class QuietHTTPHandler(http.server.SimpleHTTPRequestHandler):
    def log_message(self, format: str, *args: object) -> None:
        return


class ThreadedHTTPServer(socketserver.ThreadingMixIn, http.server.HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


class SerialBridge:
    def __init__(self, loop: asyncio.AbstractEventLoop, initial_baud: int = 115200) -> None:
        self.loop = loop
        self.clients: set[ServerConnection] = set()
        self.state = TelemetryState()
        self.baud = initial_baud
        self.port: Optional[str] = None
        self.connected = False
        self.last_error: Optional[str] = None
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self._rate_window_start = time.monotonic()
        self._rate_count = 0

    def status_payload(self) -> str:
        return json.dumps({
            "type": "status",
            "connected": self.connected,
            "port": self.port,
            "baud": self.baud,
            "ports": discover_serial_ports(),
            "error": self.last_error,
        })

    def broadcast_text(self, text: str) -> None:
        if not self.clients:
            return
        asyncio.run_coroutine_threadsafe(self._async_broadcast(text), self.loop)

    def broadcast_binary(self, payload: bytes) -> None:
        if not self.clients:
            return
        asyncio.run_coroutine_threadsafe(self._async_broadcast(payload), self.loop)

    async def _async_broadcast(self, payload: bytes | str) -> None:
        if not self.clients:
            return
        dead: list[ServerConnection] = []
        for client in list(self.clients):
            try:
                await client.send(payload)
            except websockets.WebSocketException:
                dead.append(client)
        for client in dead:
            self.clients.discard(client)

    def connect(self, port: Optional[str], baud: int = 115200) -> None:
        self.disconnect()
        if not port:
            ports = discover_serial_ports()
            port = select_default_port(ports)
        if not port:
            self.last_error = "No serial port detected"
            self.broadcast_text(self.status_payload())
            return

        with self._lock:
            self.port = port
            self.baud = baud
            self.last_error = None
            self._stop_event.clear()
            self._thread = threading.Thread(target=self._reader_loop, args=(port, baud), daemon=True)
            self._thread.start()

    def disconnect(self) -> None:
        with self._lock:
            self._stop_event.set()
            thread = self._thread
            self._thread = None
        if thread and thread.is_alive():
            thread.join(timeout=1.0)
        self.connected = False
        self.port = None
        self.broadcast_text(self.status_payload())

    def _reader_loop(self, port: str, baud: int) -> None:
        ser: Optional[serial.Serial] = None
        try:
            ser = serial.Serial(port=port, baudrate=baud, timeout=0.05)
            self.connected = True
            self.last_error = None
            self._rate_window_start = time.monotonic()
            self._rate_count = 0
            self.broadcast_text(self.status_payload())

            while not self._stop_event.is_set():
                raw = ser.readline()
                if not raw:
                    continue
                line = raw.decode("ascii", errors="replace").strip()
                if not line:
                    continue

                kind = parse_telemetry_line(line, self.state)
                if kind is not None:
                    now = time.monotonic()
                    self._rate_count += 1
                    elapsed = now - self._rate_window_start
                    if elapsed >= 0.5:
                        self.state.packet_rate_hz = self._rate_count / elapsed
                        self._rate_count = 0
                        self._rate_window_start = now
                    self.broadcast_binary(self.state.pack())
                elif line.startswith("[") or line.startswith("Check"):
                    self.broadcast_text(json.dumps({"type": "log", "line": line}))
        except serial.SerialException as exc:
            self.last_error = str(exc)
        finally:
            if ser and ser.is_open:
                ser.close()
            self.connected = False
            self.broadcast_text(self.status_payload())


async def ws_handler(websocket: ServerConnection, bridge: SerialBridge) -> None:
    bridge.clients.add(websocket)
    try:
        await websocket.send(bridge.status_payload())
        async for message in websocket:
            if not isinstance(message, str):
                continue
            try:
                cmd = json.loads(message)
            except json.JSONDecodeError:
                continue

            ctype = cmd.get("type")
            if ctype == "list_ports":
                await websocket.send(bridge.status_payload())
            elif ctype == "connect":
                port = cmd.get("port")
                baud = int(cmd.get("baud", bridge.baud))
                bridge.connect(port, baud)
            elif ctype == "disconnect":
                bridge.disconnect()
    finally:
        bridge.clients.discard(websocket)


def start_http_server(dist_dir: Path, host: str, port: int) -> ThreadedHTTPServer:
    handler = partial(QuietHTTPHandler, directory=str(dist_dir))
    httpd = ThreadedHTTPServer((host, port), handler)
    thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    thread.start()
    return httpd


async def async_main(args: argparse.Namespace) -> None:
    dist_dir = Path(__file__).resolve().parent / "dist"
    if not dist_dir.is_dir():
        dist_dir = Path(__file__).resolve().parent

    httpd = start_http_server(dist_dir, args.host, args.http_port)
    loop = asyncio.get_running_loop()
    bridge = SerialBridge(loop, initial_baud=args.baud)

    if not args.no_auto:
        default_port = args.port or select_default_port(discover_serial_ports())
        if default_port:
            bridge.connect(default_port, args.baud)

    print(f"[VIEWER] HTTP: http://{args.host}:{args.http_port}")
    print(f"[VIEWER] WS:   ws://{args.host}:{args.ws_port}")
    if bridge.port:
        print(f"[VIEWER] Serial: {bridge.port} @ {args.baud} bps")
    else:
        print("[VIEWER] Serial: standby (no board detected; Simulator mode ready in browser)")

    try:
        async with ws_serve(partial(ws_handler, bridge=bridge), args.host, args.ws_port):
            await asyncio.Future()
    finally:
        bridge.disconnect()
        httpd.shutdown()
        httpd.server_close()


def main() -> None:
    parser = argparse.ArgumentParser(description="NUCLEO-F767ZI 3D Flight Visualizer Bridge")
    parser.add_argument("--port", default=None, help="Serial device path (e.g. /dev/ttyACM0)")
    parser.add_argument("--baud", type=int, default=115200, help="UART baudrate (default: 115200)")
    parser.add_argument("--host", default="127.0.0.1", help="Bind address (default: 127.0.0.1)")
    parser.add_argument("--http-port", type=int, default=8080, help="HTTP port (default: 8080)")
    parser.add_argument("--ws-port", type=int, default=8765, help="WebSocket port (default: 8765)")
    parser.add_argument("--no-auto", action="store_true", help="Do not auto-connect serial on startup")
    args = parser.parse_args()

    try:
        asyncio.run(async_main(args))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
