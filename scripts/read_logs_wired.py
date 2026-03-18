#!/usr/bin/env python3
from __future__ import annotations

import csv
import datetime as dt
from collections import deque
import pathlib
import sys
import time


COLUMNS = [
        "t_ms",
        "ax_mps2",
        "ay_mps2",
        "az_mps2",
        "roll_rad",
        "pitch_rad",
        "yaw_rad",
        "p_x",
        "p_y",
        "p_z",
        "v_x",
        "v_y",
        "v_z",
        "a_x",
        "a_y",
        "a_z",
        "roll",
        "pitch",
        "yaw",
        ]


SERIAL_PORT = "COM9"
BAUD_RATE = 115200
SERIAL_TIMEOUT_SEC = 0.05

MAX_POINTS = 2000
REFRESH_MS = 100
PLOT_CHUNK_SEC = 0.25
MAX_LINES_PER_UPDATE = 300

LOG_DIR = pathlib.Path("logs")


def open_serial(port: str, baud: int, timeout: float):
    try:
        import serial  # type: ignore
    except ImportError:
        print("Missing dependency: pyserial", file=sys.stderr)
        print("Install it with: pip install pyserial", file=sys.stderr)
        sys.exit(2)

    try:
        ser = serial.Serial(port=port, baudrate=baud, timeout=timeout)
        time.sleep(2.0)
        ser.reset_input_buffer()
        return ser
    except Exception as exc:  # pragma: no cover
        print(f"Failed to open serial port {port}: {exc}", file=sys.stderr)
        sys.exit(1)


def create_log_file(log_dir: pathlib.Path) -> pathlib.Path:
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = dt.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    return log_dir / f"kalman_{stamp}.csv"


def parse_data_row(line: str) -> list[float] | None:
    parts = [p.strip() for p in line.split(",")]
    if len(parts) != len(COLUMNS):
        return None
    try:
        return [float(x) for x in parts]
    except ValueError:
        return None


def main() -> int:
    try:
        import matplotlib.pyplot as plt  # type: ignore
        from matplotlib.animation import FuncAnimation  # type: ignore
    except ImportError:
        print("Missing dependency: matplotlib", file=sys.stderr)
        print("Install it with: pip install matplotlib", file=sys.stderr)
        return 2

    ser = open_serial(SERIAL_PORT, BAUD_RATE, SERIAL_TIMEOUT_SEC)
    log_path = create_log_file(LOG_DIR)

    print(f"Listening on {SERIAL_PORT} @ {BAUD_RATE}")
    print(f"Logging to {log_path}")
    print("Close the plot window or press Ctrl+C to stop")

    data = {name: deque(maxlen=MAX_POINTS) for name in COLUMNS}
    first_t_ms: float | None = None
    header_received = False
    last_plot_time = time.monotonic()
    last_flush_time = time.monotonic()
    y_rescale_counter = 0

    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

    line_map = {
            "ax_meas": axes[0].plot([], [], label="ax_meas", alpha=0.75)[0],
            "ay_meas": axes[0].plot([], [], label="ay_meas", alpha=0.75)[0],
            "az_meas": axes[0].plot([], [], label="az_meas", alpha=0.75)[0],
            "a_x_kalman": axes[0].plot([], [], label="a_x_kalman", linewidth=2)[0],
            "a_y_kalman": axes[0].plot([], [], label="a_y_kalman", linewidth=2)[0],
            "a_z_kalman": axes[0].plot([], [], label="a_z_kalman", linewidth=2)[0],
            "roll_meas": axes[1].plot([], [], label="roll_meas", alpha=0.75)[0],
            "pitch_meas": axes[1].plot([], [], label="pitch_meas", alpha=0.75)[0],
            "yaw_meas": axes[1].plot([], [], label="yaw_meas", alpha=0.75)[0],
            "roll_kalman": axes[1].plot([], [], label="roll_kalman", linewidth=2)[0],
            "pitch_kalman": axes[1].plot([], [], label="pitch_kalman", linewidth=2)[0],
            "yaw_kalman": axes[1].plot([], [], label="yaw_kalman", linewidth=2)[0],
            "p_x": axes[2].plot([], [], label="p_x", linewidth=2)[0],
            "p_y": axes[2].plot([], [], label="p_y", linewidth=2)[0],
            "p_z": axes[2].plot([], [], label="p_z", linewidth=2)[0],
            "v_x": axes[2].plot([], [], label="v_x", linestyle="--")[0],
            "v_y": axes[2].plot([], [], label="v_y", linestyle="--")[0],
            "v_z": axes[2].plot([], [], label="v_z", linestyle="--")[0],
            }

    axes[0].set_ylabel("Accel")
    axes[0].set_title("Acceleration (m/s^2): measurement vs Kalman")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc="upper right", ncol=2)

    axes[1].set_ylabel("Angle (rad)")
    axes[1].set_title("Attitude: measurement vs Kalman")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc="upper right", ncol=2)

    axes[2].set_ylabel("Position / Velocity")
    axes[2].set_xlabel("Time (s)")
    axes[2].set_title("Kalman position and velocity states")
    axes[2].grid(True, alpha=0.3)
    axes[2].legend(loc="upper right", ncol=2)

    fig.suptitle("Kalman Realtime Capture")
    fig.tight_layout()

    fp = log_path.open("w", newline="", encoding="utf-8")
    writer = csv.writer(fp)
    writer.writerow(COLUMNS)
    fp.flush()

    def update(_frame):
        nonlocal first_t_ms
        nonlocal header_received
        nonlocal last_plot_time
        nonlocal last_flush_time
        nonlocal y_rescale_counter

        lines_processed = 0
        while lines_processed < MAX_LINES_PER_UPDATE:
            raw = ser.readline()
            if not raw:
                break
            lines_processed += 1

            line = raw.decode("utf-8", errors="replace").strip()
            if not line:
                continue

            if line.startswith("err,"):
                print(f"Device error: {line}")
                continue

            if line.startswith("t_ms,"):
                header_received = True
                continue

            row = parse_data_row(line)
            if row is None:
                continue

            if not header_received:
                # Allow stream data even if startup header was missed.
                header_received = True

            writer.writerow(row)

            if first_t_ms is None:
                first_t_ms = row[0]

            for idx, col in enumerate(COLUMNS):
                if col == "t_ms":
                    data[col].append((row[idx] - first_t_ms) / 1000.0)
                else:
                    data[col].append(row[idx])

        now = time.monotonic()
        should_plot = (now - last_plot_time) >= PLOT_CHUNK_SEC

        if should_plot and data["t_ms"]:
            t = list(data["t_ms"])

            line_map["ax_meas"].set_data(t, list(data["ax_mps2"]))
            line_map["ay_meas"].set_data(t, list(data["ay_mps2"]))
            line_map["az_meas"].set_data(t, list(data["az_mps2"]))
            line_map["a_x_kalman"].set_data(t, list(data["a_x"]))
            line_map["a_y_kalman"].set_data(t, list(data["a_y"]))
            line_map["a_z_kalman"].set_data(t, list(data["a_z"]))

            line_map["roll_meas"].set_data(t, list(data["roll_rad"]))
            line_map["pitch_meas"].set_data(t, list(data["pitch_rad"]))
            line_map["yaw_meas"].set_data(t, list(data["yaw_rad"]))
            line_map["roll_kalman"].set_data(t, list(data["roll"]))
            line_map["pitch_kalman"].set_data(t, list(data["pitch"]))
            line_map["yaw_kalman"].set_data(t, list(data["yaw"]))

            line_map["p_x"].set_data(t, list(data["p_x"]))
            line_map["p_y"].set_data(t, list(data["p_y"]))
            line_map["p_z"].set_data(t, list(data["p_z"]))
            line_map["v_x"].set_data(t, list(data["v_x"]))
            line_map["v_y"].set_data(t, list(data["v_y"]))
            line_map["v_z"].set_data(t, list(data["v_z"]))

            latest_t = t[-1]
            axes[2].set_xlim(0.0, latest_t + 0.1)

            y_rescale_counter += 1
            if y_rescale_counter >= 4:
                for ax in axes:
                    ax.relim()
                    ax.autoscale_view(scalex=False)
                y_rescale_counter = 0

            fig.canvas.draw_idle()
            last_plot_time = now

        if (now - last_flush_time) >= 1.0:
            fp.flush()
            last_flush_time = now
        return tuple(line_map.values())

    ani = FuncAnimation(fig, update, interval=REFRESH_MS, blit=False, cache_frame_data=False)

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        # keep reference alive for matplotlib
        _ = ani
        fp.flush()
        fp.close()
        ser.close()
        print(f"Saved: {log_path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
