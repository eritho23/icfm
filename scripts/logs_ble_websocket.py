#!/usr/bin/env python3
"""
Rocket telemetry WebSocket server over BLE.
Reads BLE notify lines from ESP32 and broadcasts JSON to browser clients.
Accepts command messages from websocket clients and forwards commands via BLE.
"""

import asyncio
import csv
import datetime as dt
import json
import logging
import pathlib
from typing import Optional, Tuple, List

from bleak import BleakClient, BleakScanner
import websockets

DEVICE_NAME = "Rocket-FC"
SERVICE_UUID = "12345678-1234-1234-1234-123456789abc"
CMD_CHAR_UUID = "abcdefab-1234-1234-1234-abcdefabcdef"
LOG_CHAR_UUID = "fedcbafe-4321-4321-4321-abcdefabcdef"

WS_HOST = "localhost"
WS_PORT = 8765
LOG_DIR = pathlib.Path("logs")
BLE_RECONNECT_SEC = 1.0

KALMAN_COLS = [
    "t_ms",
    "wx",
    "wy",
    "wz",
    "p_x",
    "p_y",
    "p_z",
    "v_x",
    "v_y",
    "v_z",
    "a_x",
    "a_y",
    "a_z",
    "d_theta",
    "d_alpha",
    "d_beta",
    "q_w",
    "q_x",
    "q_y",
    "q_z",
]

CTRL_COLS = [
    "t_ms",
    "qd_w",
    "qd_x",
    "qd_y",
    "qd_z",
    "fin0",
    "fin1",
    "fin2",
    "fin3",
]

COMBINED_COLS = [
    "t_ms",
    "wx",
    "wy",
    "wz",
    "p_x",
    "p_y",
    "p_z",
    "v_x",
    "v_y",
    "v_z",
    "a_x",
    "a_y",
    "a_z",
    "d_theta",
    "d_alpha",
    "d_beta",
    "q_w",
    "q_x",
    "q_y",
    "q_z",
    "fin0",
    "fin1",
    "fin2",
    "fin3",
]

HEADER_PREFIXES = (
    "Commands:",
    "STATE,",
    "FSM,",
    "DUMP_BEGIN",
    "DUMP_END",
    "LOG:",
    "ERROR:",
)


def parse_row(line: str) -> Tuple[Optional[str], Optional[List[float]]]:
    parts = line.strip().split(",")
    try:
        vals = [float(p) for p in parts]
    except ValueError:
        return None, None

    if len(vals) == len(KALMAN_COLS):
        return "kalman", vals
    if len(vals) == len(CTRL_COLS):
        return "ctrl", vals
    if len(vals) == len(COMBINED_COLS):
        return "combined", vals
    return None, None


async def telemetry_server():
    logging.getLogger("asyncio").setLevel(logging.CRITICAL)
    ws_logger = logging.getLogger("websockets.server")
    ws_logger.setLevel(logging.CRITICAL)
    ws_logger.propagate = False

    LOG_DIR.mkdir(exist_ok=True)
    stamp = dt.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    fp_k = open(LOG_DIR / f"kalman_{stamp}.csv", "w", newline="", encoding="utf-8")
    fp_c = open(LOG_DIR / f"ctrl_{stamp}.csv", "w", newline="", encoding="utf-8")
    writer_k = csv.writer(fp_k)
    writer_c = csv.writer(fp_c)
    writer_k.writerow(KALMAN_COLS)
    writer_c.writerow(CTRL_COLS)

    clients: set = set()
    ble_client: Optional[BleakClient] = None
    ble_connected = asyncio.Event()
    ble_write_lock = asyncio.Lock()
    notify_line_buf = ""
    first_t_ms = None

    async def broadcast(msg: str):
        if not clients:
            return
        dead = set()
        for ws in clients:
            try:
                await ws.send(msg)
            except Exception:
                dead.add(ws)
        clients.difference_update(dead)

    async def handle_data_line(line: str):
        nonlocal first_t_ms
        await broadcast(json.dumps({"type": "serial_raw", "line": line}))

        if any(line.startswith(p) for p in HEADER_PREFIXES):
            return

        kind, row = parse_row(line)
        if kind is None or row is None:
            return

        if first_t_ms is None:
            first_t_ms = row[0]
        t_s = (row[0] - first_t_ms) / 1000.0

        if kind == "kalman":
            data = dict(zip(KALMAN_COLS, row))
            data["t_s"] = t_s
            writer_k.writerow(row)
            fp_k.flush()
            await broadcast(json.dumps({"type": "kalman", "data": data}))
            return

        if kind == "ctrl":
            data = dict(zip(CTRL_COLS, row))
            writer_c.writerow(row)
            fp_c.flush()
            await broadcast(json.dumps({"type": "ctrl", "data": data}))
            return

        # combined row
        krow = row[: len(KALMAN_COLS)]
        crow = [row[0], 1.0, 0.0, 0.0, 0.0] + row[20:24]
        data_k = dict(zip(KALMAN_COLS, krow))
        data_k["t_s"] = t_s
        data_c = dict(zip(CTRL_COLS, crow))

        writer_k.writerow(krow)
        writer_c.writerow(crow)
        fp_k.flush()
        fp_c.flush()

        await broadcast(json.dumps({"type": "kalman", "data": data_k}))
        await broadcast(json.dumps({"type": "ctrl", "data": data_c}))

    async def on_notify(_: int, data: bytearray):
        nonlocal notify_line_buf
        chunk = bytes(data).decode("utf-8", errors="replace")
        notify_line_buf += chunk
        while True:
            i = notify_line_buf.find("\n")
            if i < 0:
                break
            line = notify_line_buf[:i].rstrip("\r")
            notify_line_buf = notify_line_buf[i + 1 :]
            if line:
                await handle_data_line(line)

    async def ws_handler(ws):
        nonlocal ble_client
        clients.add(ws)
        print(f"[ws] client connected ({len(clients)})")
        try:
            async for raw in ws:
                try:
                    msg = json.loads(raw)
                except json.JSONDecodeError:
                    continue

                if msg.get("type") != "command":
                    continue

                cmd = str(msg.get("cmd", "")).strip().upper()
                if cmd not in ("CALIBRATE", "RESET", "DUMP"):
                    await ws.send(
                        json.dumps(
                            {
                                "type": "command_ack",
                                "ok": False,
                                "cmd": cmd,
                                "reason": "invalid_command",
                            }
                        )
                    )
                    continue

                if not ble_connected.is_set() or ble_client is None:
                    await ws.send(
                        json.dumps(
                            {
                                "type": "command_ack",
                                "ok": False,
                                "cmd": cmd,
                                "reason": "ble_not_ready",
                            }
                        )
                    )
                    continue

                try:
                    async with ble_write_lock:
                        await ble_client.write_gatt_char(
                            CMD_CHAR_UUID, cmd.encode("utf-8")
                        )
                    await ws.send(
                        json.dumps(
                            {
                                "type": "command_ack",
                                "ok": True,
                                "cmd": cmd,
                            }
                        )
                    )
                except Exception as e:
                    await ws.send(
                        json.dumps(
                            {
                                "type": "command_ack",
                                "ok": False,
                                "cmd": cmd,
                                "reason": f"ble_write_error:{e}",
                            }
                        )
                    )
        finally:
            clients.discard(ws)
            print(f"[ws] client disconnected ({len(clients)})")

    async def ble_loop():
        nonlocal ble_client
        while True:
            try:
                print(f"[ble] scanning for '{DEVICE_NAME}'...")
                device = await BleakScanner.find_device_by_name(
                    DEVICE_NAME, timeout=10.0
                )
                if device is None:
                    print("[ble] device not found")
                    await asyncio.sleep(BLE_RECONNECT_SEC)
                    continue

                print(f"[ble] connecting {device.name} ({device.address})")
                async with BleakClient(device) as client:
                    ble_client = client
                    ble_connected.set()
                    print("[ble] connected")
                    await client.start_notify(LOG_CHAR_UUID, on_notify)
                    while client.is_connected:
                        await asyncio.sleep(0.2)
            except Exception as e:
                print(f"[ble] disconnected/error: {e}")
            finally:
                ble_connected.clear()
                ble_client = None
                await asyncio.sleep(BLE_RECONNECT_SEC)

    ws_server = await websockets.serve(ws_handler, WS_HOST, WS_PORT, logger=ws_logger)
    print(f"[ws] listening on ws://{WS_HOST}:{WS_PORT}")

    await ble_loop()

    ws_server.close()
    await ws_server.wait_closed()


if __name__ == "__main__":
    try:
        asyncio.run(telemetry_server())
    except KeyboardInterrupt:
        print("\nServer stopped.")
