"""Unit and PTY+WebSocket+HTTP integration tests for viewer/server.py."""

import asyncio
from functools import partial
import json
import os
from pathlib import Path
import pty
import struct
import sys
import termios
import tty
import unittest
import urllib.request

from websockets.asyncio.client import connect as ws_connect
from websockets.asyncio.server import serve as ws_serve

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import server


class TestSerialBridgeParser(unittest.TestCase):
    def test_parse_bmp280_ascii_and_utf8(self) -> None:
        state = server.TelemetryState()
        line_ascii = (
            "[NUCLEO-F767ZI BMP280 I2C] Temp:  24.85 degC | "
            "Press:  1012.30 hPa ( 101230 Pa) | Hypsometric Alt:    7.84 m"
        )
        kind = server.parse_telemetry_line(line_ascii, state)
        self.assertEqual(kind, "bmp")
        self.assertAlmostEqual(state.bmp_temp_c, 24.85, places=3)
        self.assertAlmostEqual(state.bmp_press_hpa, 1012.30, places=3)
        self.assertAlmostEqual(state.bmp_alt_m, 7.84, places=3)

        line_utf8 = (
            "[NUCLEO-F767ZI BMP280 I2C] Temp:  26.10 °C | "
            "Press:  1008.50 hPa ( 100850 Pa) | Hypsometric Alt:   -3.42 m"
        )
        kind2 = server.parse_telemetry_line(line_utf8, state)
        self.assertEqual(kind2, "bmp")
        self.assertAlmostEqual(state.bmp_temp_c, 26.10, places=3)
        self.assertAlmostEqual(state.bmp_alt_m, -3.42, places=3)

    def test_parse_mpu6050_line(self) -> None:
        state = server.TelemetryState()
        line = (
            "[NUCLEO-F767ZI MPU6050 I2C] Acc:  +0.12, -0.34, +0.93 g | "
            "Gyro:   +12.50,  -4.25, +45.00 dps | Temp: 29.45 degC | "
            "Pose R: -18.5 P:  +7.2 Y:+123.4 deg"
        )
        kind = server.parse_telemetry_line(line, state)
        self.assertEqual(kind, "mpu")
        self.assertAlmostEqual(state.accel_x_g, 0.12, places=3)
        self.assertAlmostEqual(state.accel_y_g, -0.34, places=3)
        self.assertAlmostEqual(state.accel_z_g, 0.93, places=3)
        self.assertAlmostEqual(state.gyro_x_dps, 12.50, places=3)
        self.assertAlmostEqual(state.gyro_y_dps, -4.25, places=3)
        self.assertAlmostEqual(state.gyro_z_dps, 45.00, places=3)
        self.assertAlmostEqual(state.mpu_temp_c, 29.45, places=3)
        self.assertAlmostEqual(state.roll_deg, -18.5, places=3)
        self.assertAlmostEqual(state.pitch_deg, 7.2, places=3)
        self.assertAlmostEqual(state.yaw_deg, 123.4, places=3)

    def test_ignore_banner_and_invalid_lines(self) -> None:
        state = server.TelemetryState()
        self.assertIsNone(server.parse_telemetry_line("[OK] BMP280 detected over I2C at 0x76!", state))
        self.assertIsNone(server.parse_telemetry_line("garbage bytes", state))

    def test_binary_frame_pack_56_bytes(self) -> None:
        state = server.TelemetryState(
            roll_deg=15.5,
            pitch_deg=-4.2,
            yaw_deg=90.0,
            accel_x_g=0.1,
            accel_y_g=-0.2,
            accel_z_g=0.98,
            gyro_x_dps=5.0,
            gyro_y_dps=-3.0,
            gyro_z_dps=12.0,
            mpu_temp_c=30.5,
            bmp_temp_c=24.5,
            bmp_press_hpa=1010.25,
            bmp_alt_m=25.4,
            packet_rate_hz=50.0,
        )
        payload = state.pack()
        self.assertEqual(len(payload), 56)
        unpacked = struct.unpack("<14f", payload)
        self.assertAlmostEqual(unpacked[0], 15.5, places=4)
        self.assertAlmostEqual(unpacked[11], 1010.25, places=2)
        self.assertAlmostEqual(unpacked[12], 25.4, places=3)
        self.assertAlmostEqual(unpacked[13], 50.0, places=3)

    def test_select_default_port(self) -> None:
        ports = [
            {"device": "/dev/ttyS0", "description": "Serial"},
            {"device": "/dev/ttyACM0", "description": "STM32 STLink"},
        ]
        self.assertEqual(server.select_default_port(ports), "/dev/ttyACM0")
        self.assertIsNone(server.select_default_port([]))

    def test_pty_to_binary_websocket_and_http_e2e(self) -> None:
        master_fd, slave_fd = pty.openpty()
        tty.setraw(slave_fd, when=termios.TCSANOW)
        slave_name = os.ttyname(slave_fd)
        dist_dir = Path(__file__).resolve().parents[1] / "dist"
        if not dist_dir.is_dir():
            dist_dir = Path(__file__).resolve().parents[1]

        httpd = server.start_http_server(dist_dir, "127.0.0.1", 0)
        http_port = httpd.server_address[1]

        async def run_e2e() -> None:
            loop = asyncio.get_running_loop()
            bridge = server.SerialBridge(loop, initial_baud=115200)
            async with ws_serve(partial(server.ws_handler, bridge=bridge), "127.0.0.1", 0) as ws_srv:
                ws_port = ws_srv.sockets[0].getsockname()[1]
                async with ws_connect(f"ws://127.0.0.1:{ws_port}") as ws:
                    initial_status = json.loads(await ws.recv())
                    self.assertEqual(initial_status["type"], "status")
                    self.assertFalse(initial_status["connected"])

                    bridge.connect(slave_name, 115200)
                    for _ in range(10):
                        msg = await asyncio.wait_for(ws.recv(), timeout=2.0)
                        if isinstance(msg, str):
                            st = json.loads(msg)
                            if st.get("type") == "status" and st.get("connected"):
                                break

                    os.write(
                        master_fd,
                        b"[NUCLEO-F767ZI BMP280 I2C] Temp:  23.40 degC | Press:  1010.50 hPa ( 101050 Pa) | Hypsometric Alt:   22.80 m\r\n"
                        b"[NUCLEO-F767ZI MPU6050 I2C] Acc:  +0.05, -0.10, +0.99 g | Gyro:   +5.50,  -2.25,  +8.00 dps | Temp: 28.10 degC | Pose R: -12.3 P:  +4.5 Y: +67.8 deg\r\n",
                    )

                    got_binary = None
                    for _ in range(10):
                        msg = await asyncio.wait_for(ws.recv(), timeout=2.0)
                        if isinstance(msg, bytes):
                            got_binary = msg
                            vals_check = struct.unpack("<14f", msg)
                            if abs(vals_check[0] - (-12.3)) < 0.05 and abs(vals_check[12] - 22.80) < 0.05:
                                break
                    self.assertIsNotNone(got_binary)
                    assert got_binary is not None
                    self.assertEqual(len(got_binary), 56)
                    vals = struct.unpack("<14f", got_binary)
                    self.assertAlmostEqual(vals[0], -12.3, places=2)
                    self.assertAlmostEqual(vals[1], 4.5, places=2)
                    self.assertAlmostEqual(vals[2], 67.8, places=2)
                    self.assertAlmostEqual(vals[11], 1010.50, places=2)
                    self.assertAlmostEqual(vals[12], 22.80, places=2)

                    bridge.disconnect()

        try:
            with urllib.request.urlopen(f"http://127.0.0.1:{http_port}/", timeout=2.0) as resp:
                self.assertEqual(resp.status, 200)
                body = resp.read().decode("utf-8", errors="ignore")
                self.assertIn("NUCLEO-F767ZI", body)
            asyncio.run(run_e2e())
        finally:
            httpd.shutdown()
            httpd.server_close()
            os.close(master_fd)
            os.close(slave_fd)


if __name__ == "__main__":
    unittest.main()
