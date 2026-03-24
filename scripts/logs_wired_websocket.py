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
import pathlib
import sys

SERIAL_PORT = "COM9"
BAUD_RATE = 115200
SERIAL_TIMEOUT_SEC = 0.02
WS_HOST = "localhost"
WS_PORT = 8765
LOG_DIR = pathlib.Path("logs")

KALMAN_COLS = [
    "t_ms", "wx", "wy", "wz",
    "p_x", "p_y", "p_z",
    "v_x", "v_y", "v_z",
    "a_x", "a_y", "a_z",
    "d_theta", "d_alpha", "d_beta",
    "q_w", "q_x", "q_y", "q_z",
]
CTRL_COLS = [
    "t_ms", "qd_w", "qd_x", "qd_y", "qd_z",
    "fin0", "fin1", "fin2", "fin3",
]

HEADER_PREFIXES = ("t_ms,",)

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


def parse_row(line):
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
    import serial_asyncio
    import websockets

    # Logs
    LOG_DIR.mkdir(exist_ok=True)
    stamp = dt.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    fp_k = open(LOG_DIR / f"kalman_{stamp}.csv", "w", newline="", encoding="utf-8")
    fp_c = open(LOG_DIR / f"ctrl_{stamp}.csv", "w", newline="", encoding="utf-8")
    writer_k = csv.writer(fp_k); writer_k.writerow(KALMAN_COLS)
    writer_c = csv.writer(fp_c); writer_c.writerow(CTRL_COLS)

    # WebSocket clients
    clients: set = set()

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
        clients.add(ws)
        print(f"[ws] client connected ({len(clients)})")
        try:
            await ws.wait_closed()
        finally:
            clients.discard(ws)
            print(f"[ws] client disconnected ({len(clients)})")

    async def serial_reader():
        ser = await serial_asyncio.open_serial_connection(url=SERIAL_PORT, baudrate=BAUD_RATE)
        reader, _ = ser
        first_t_ms = None
        while True:
            line = (await reader.readline()).decode("utf-8", errors="replace").strip()
            if not line or any(line.startswith(p) for p in HEADER_PREFIXES) or line.startswith("err,"):
                continue
            kind, row = parse_row(line)
            if kind is None:
                continue
            if first_t_ms is None:
                first_t_ms = row[0]
            t_s = (row[0] - first_t_ms) / 1000.0
            data = dict(zip(KALMAN_COLS if kind == "kalman" else CTRL_COLS, row))
            data["t_s"] = t_s

            # Write CSV
            if kind == "kalman":
                writer_k.writerow(row); fp_k.flush()
            else:
                writer_c.writerow(row); fp_c.flush()

            # Broadcast JSON
            msg = json.dumps({"type": kind, "data": data})
            await broadcast(msg)

    # Start WebSocket server
    ws_server = await websockets.serve(ws_handler, WS_HOST, WS_PORT)
    print(f"[ws] listening on ws://{WS_HOST}:{WS_PORT}")

    # Run serial reader forever
    await serial_reader()
    await ws_server.wait_closed()


if __name__ == "__main__":
    try:
        asyncio.run(telemetry_server())
    except KeyboardInterrupt:
        print("\nServer stopped.")
