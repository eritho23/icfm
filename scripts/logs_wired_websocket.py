#!/usr/bin/env python3
"""
Simplified Rocket Telemetry WebSocket server.
Reads Kalman + Controller CSV rows from serial and broadcasts JSON to browser clients.
Logs CSV under logs/.
"""

import asyncio
import csv
import datetime as dt
import json
import logging
import pathlib
import sys
from typing import Optional, Tuple, List

SERIAL_PORT = "COM9"
BAUD_RATE = 115200
SERIAL_TIMEOUT_SEC = 0.02
WS_HOST = "localhost"
WS_PORT = 8765
LOG_DIR = pathlib.Path("logs")
SERIAL_RECONNECT_SEC = 1.0

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

HEADER_PREFIXES = ("t_ms,", "Commands:", "STATE,", "FSM,")


# ---------------------------------------------------------------------------
# Serial helpers
# ---------------------------------------------------------------------------
def open_serial(port, baud, timeout):
    try:
        import serial
    except ImportError:
        sys.exit("Missing pyserial — pip install pyserial")
    try:
        ser = serial.Serial(port, baud, timeout=timeout)
        print(f"[serial] opened {port} @ {baud}")
        return ser
    except Exception as e:
        sys.exit(f"Cannot open {port}: {e}")


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
    return None, None


# ---------------------------------------------------------------------------
# WebSocket + Serial server
# ---------------------------------------------------------------------------
async def telemetry_server():
    import serial
    import serial_asyncio
    import websockets

    logging.getLogger("serial_asyncio").setLevel(logging.CRITICAL)
    ws_logger = logging.getLogger("websockets.server")
    ws_logger.setLevel(logging.CRITICAL)
    ws_logger.propagate = False

    # Logs
    LOG_DIR.mkdir(exist_ok=True)
    stamp = dt.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    fp_k = open(LOG_DIR / f"kalman_{stamp}.csv", "w", newline="", encoding="utf-8")
    fp_c = open(LOG_DIR / f"ctrl_{stamp}.csv", "w", newline="", encoding="utf-8")
    writer_k = csv.writer(fp_k)
    writer_k.writerow(KALMAN_COLS)
    writer_c = csv.writer(fp_c)
    writer_c.writerow(CTRL_COLS)

    # WebSocket clients
    clients: set = set()
    serial_writer = None
    serial_write_lock = asyncio.Lock()

    async def broadcast(msg):
        if not clients:
            return
        dead = set()
        for ws in clients:
            try:
                await ws.send(msg)
            except:
                dead.add(ws)
        clients.difference_update(dead)

    async def ws_handler(ws):
        nonlocal serial_writer
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
                if cmd not in ("CALIBRATE", "RESET"):
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

                if serial_writer is None:
                    await ws.send(
                        json.dumps(
                            {
                                "type": "command_ack",
                                "ok": False,
                                "cmd": cmd,
                                "reason": "serial_not_ready",
                            }
                        )
                    )
                    continue

                try:
                    async with serial_write_lock:
                        serial_writer.write((cmd + "\n").encode("utf-8"))
                        await serial_writer.drain()
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
                                "reason": f"serial_write_error:{e}",
                            }
                        )
                    )
        finally:
            clients.discard(ws)
            print(f"[ws] client disconnected ({len(clients)})")

    async def serial_reader():
        nonlocal serial_writer
        reader = None
        writer = None
        opened = False
        try:
            ser = await serial_asyncio.open_serial_connection(
                url=SERIAL_PORT, baudrate=BAUD_RATE
            )
            reader, writer = ser
            serial_writer = writer
            opened = True
            print(f"[serial] opened {SERIAL_PORT} @ {BAUD_RATE}")
            first_t_ms = None
            while True:
                line = (
                    (await reader.readline()).decode("utf-8", errors="replace").strip()
                )
                if not line:
                    continue

                await broadcast(json.dumps({"type": "serial_raw", "line": line}))

                if any(
                    line.startswith(p) for p in HEADER_PREFIXES
                ) or line.upper().startswith("ERR,"):
                    continue
                kind, row = parse_row(line)
                if kind is None or row is None:
                    continue
                if first_t_ms is None:
                    first_t_ms = row[0]
                t_s = (row[0] - first_t_ms) / 1000.0
                data = dict(zip(KALMAN_COLS if kind == "kalman" else CTRL_COLS, row))
                data["t_s"] = t_s

                # Write CSV
                if kind == "kalman":
                    writer_k.writerow(row)
                    fp_k.flush()
                else:
                    writer_c.writerow(row)
                    fp_c.flush()

                # Broadcast JSON
                msg = json.dumps({"type": kind, "data": data})
                await broadcast(msg)
        except (serial.SerialException, OSError, asyncio.IncompleteReadError):
            if opened:
                print("[serial] disconnected")
            return
        finally:
            serial_writer = None
            if writer is not None:
                writer.close()
                try:
                    await writer.wait_closed()
                except Exception:
                    pass

    # Start WebSocket server
    ws_server = await websockets.serve(ws_handler, WS_HOST, WS_PORT, logger=ws_logger)
    print(f"[ws] listening on ws://{WS_HOST}:{WS_PORT}")

    try:
        # Keep websocket alive; reconnect serial when unplugged/replugged.
        while True:
            await serial_reader()
            await asyncio.sleep(SERIAL_RECONNECT_SEC)
    finally:
        ws_server.close()
        await ws_server.wait_closed()
        fp_k.close()
        fp_c.close()


if __name__ == "__main__":
    try:
        asyncio.run(telemetry_server())
    except KeyboardInterrupt:
        print("\nServer stopped.")
