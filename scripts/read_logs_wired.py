#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import datetime as dt
from collections import deque
import pathlib
import sys
import time


COLUMNS = [
        "t_ms",
        "ax",
        "ay",
        "az",
        "gx",
        "gy",
        "gz",
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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
            description="Capture ESP Kalman CSV stream, log it, and plot live"
            )
    parser.add_argument("--port", default="COM9", help="Serial port (default: COM9)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument(
            "--timeout",
            type=float,
            default=0.05,
            help="Serial read timeout in seconds",
            )
    parser.add_argument(
            "--max-points",
            type=int,
            default=2000,
            help="Max plotted samples kept in memory",
            )
    parser.add_argument(
            "--refresh-ms",
            type=int,
            default=100,
            help="Plot refresh interval in milliseconds",
            )
    return parser.parse_args()


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
    args = parse_args()

    try:
        import matplotlib.pyplot as plt  # type: ignore
        from matplotlib.animation import FuncAnimation  # type: ignore
    except ImportError:
        print("Missing dependency: matplotlib", file=sys.stderr)
        print("Install it with: pip install matplotlib", file=sys.stderr)
        return 2

    ser = open_serial(args.port, args.baud, args.timeout)
    log_path = create_log_file(pathlib.Path("logs"))

    print(f"Listening on {args.port} @ {args.baud}")
    print(f"Logging to {log_path}")
    print("Close the plot window or press Ctrl+C to stop")

    data = {name: deque(maxlen=args.max_points) for name in COLUMNS}
    first_t_ms: float | None = None
    header_received = False

    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

    line_map = {
            "ax_raw": axes[0].plot([], [], label="ax_raw", alpha=0.75)[0],
            "ay_raw": axes[0].plot([], [], label="ay_raw", alpha=0.75)[0],
            "az_raw": axes[0].plot([], [], label="az_raw", alpha=0.75)[0],
            "a_x_kalman": axes[0].plot([], [], label="a_x_kalman", linewidth=2)[0],
            "a_y_kalman": axes[0].plot([], [], label="a_y_kalman", linewidth=2)[0],
            "a_z_kalman": axes[0].plot([], [], label="a_z_kalman", linewidth=2)[0],
            "gx_raw": axes[1].plot([], [], label="gx_raw", alpha=0.75)[0],
            "gy_raw": axes[1].plot([], [], label="gy_raw", alpha=0.75)[0],
            "gz_raw": axes[1].plot([], [], label="gz_raw", alpha=0.75)[0],
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
    axes[0].set_title("Acceleration: raw vs Kalman")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc="upper right", ncol=2)

    axes[1].set_ylabel("Gyro / Angle")
    axes[1].set_title("Gyro raw and attitude states")
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

        while True:
            raw = ser.readline()
            if not raw:
                break

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

        if data["t_ms"]:
            t = list(data["t_ms"])

            line_map["ax_raw"].set_data(t, list(data["ax"]))
            line_map["ay_raw"].set_data(t, list(data["ay"]))
            line_map["az_raw"].set_data(t, list(data["az"]))
            line_map["a_x_kalman"].set_data(t, list(data["a_x"]))
            line_map["a_y_kalman"].set_data(t, list(data["a_y"]))
            line_map["a_z_kalman"].set_data(t, list(data["a_z"]))

            line_map["gx_raw"].set_data(t, list(data["gx"]))
            line_map["gy_raw"].set_data(t, list(data["gy"]))
            line_map["gz_raw"].set_data(t, list(data["gz"]))
            line_map["roll_kalman"].set_data(t, list(data["roll"]))
            line_map["pitch_kalman"].set_data(t, list(data["pitch"]))
            line_map["yaw_kalman"].set_data(t, list(data["yaw"]))

            line_map["p_x"].set_data(t, list(data["p_x"]))
            line_map["p_y"].set_data(t, list(data["p_y"]))
            line_map["p_z"].set_data(t, list(data["p_z"]))
            line_map["v_x"].set_data(t, list(data["v_x"]))
            line_map["v_y"].set_data(t, list(data["v_y"]))
            line_map["v_z"].set_data(t, list(data["v_z"]))

            for ax in axes:
                ax.relim()
                ax.autoscale_view()

            fig.canvas.draw_idle()

        fp.flush()
        return tuple(line_map.values())

    ani = FuncAnimation(fig, update, interval=args.refresh_ms, blit=False, cache_frame_data=False)

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        # keep reference alive for matplotlib
        _ = ani
        fp.close()
        ser.close()
        print(f"Saved: {log_path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
