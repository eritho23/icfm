#!/usr/bin/env python3
from __future__ import annotations

import csv
import datetime as dt
from collections import deque
import pathlib
import sys
import time

import numpy as np


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


def rpy_to_rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Build a ZYX rotation matrix from roll, pitch, yaw (radians)."""
    cr, sr = np.cos(roll),  np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw),   np.sin(yaw)

    Rz = np.array([[ cy, -sy,  0],
                   [ sy,  cy,  0],
                   [  0,   0,  1]])
    Ry = np.array([[ cp,   0, sp],
                   [  0,   1,  0],
                   [-sp,   0, cp]])
    Rx = np.array([[  1,   0,   0],
                   [  0,  cr, -sr],
                   [  0,  sr,  cr]])
    return Rz @ Ry @ Rx


def draw_orientation_axes(ax3d, R: np.ndarray, R_meas: np.ndarray | None = None) -> None:
    """
    Redraw the 3-D orientation axes on *ax3d*.

    Kalman axes  → solid, thick:   X=red, Y=green, Z=blue
    Measured axes → dashed, thin:  same colours, lighter
    """
    ax3d.cla()

    origin = np.zeros(3)
    colors = ["#E24B4A", "#1D9E75", "#378ADD"]      # X, Y, Z
    meas_colors = ["#F09595", "#9FE1CB", "#85B7EB"]  # lighter versions

    labels = ["X", "Y", "Z"]

    # Draw measured (ghost) axes first so they sit behind
    if R_meas is not None:
        for i in range(3):
            vec = R_meas[:, i]
            ax3d.quiver(*origin, *vec,
                        color=meas_colors[i], alpha=0.5,
                        linewidth=1.5, linestyle="dashed",
                        arrow_length_ratio=0.15)

    # Draw Kalman axes
    for i in range(3):
        vec = R[:, i]
        ax3d.quiver(*origin, *vec,
                    color=colors[i], alpha=1.0,
                    linewidth=2.5, linestyle="solid",
                    arrow_length_ratio=0.15)
        ax3d.text(vec[0] * 1.15, vec[1] * 1.15, vec[2] * 1.15,
                  labels[i], color=colors[i], fontsize=10, fontweight="bold",
                  ha="center", va="center")

    # World frame reference (thin grey)
    for i, (dx, dy, dz) in enumerate([(1,0,0),(0,1,0),(0,0,1)]):
        ax3d.quiver(0, 0, 0, dx, dy, dz,
                    color="grey", alpha=0.18, linewidth=1,
                    arrow_length_ratio=0.1)

    ax3d.set_xlim(-1.3, 1.3)
    ax3d.set_ylim(-1.3, 1.3)
    ax3d.set_zlim(-1.3, 1.3)
    ax3d.set_xlabel("X", fontsize=8, labelpad=2)
    ax3d.set_ylabel("Y", fontsize=8, labelpad=2)
    ax3d.set_zlabel("Z", fontsize=8, labelpad=2)
    ax3d.set_title("Orientation (3-D)", fontsize=9, pad=4)
    ax3d.tick_params(labelsize=7)
    ax3d.set_box_aspect([1, 1, 1])


def main() -> int:
    try:
        import matplotlib.pyplot as plt  # type: ignore
        from matplotlib.animation import FuncAnimation  # type: ignore
        from mpl_toolkits.mplot3d import Axes3D  # type: ignore  # noqa: F401
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

    # ------------------------------------------------------------------ layout
    # 3 time-series plots on the left column; 3-D orientation on the right.
    fig = plt.figure(figsize=(14, 10))
    fig.suptitle("Kalman Realtime Capture")

    gs = fig.add_gridspec(3, 2, width_ratios=[2, 1], hspace=0.45, wspace=0.35)

    ax_accel  = fig.add_subplot(gs[0, 0])
    ax_att    = fig.add_subplot(gs[1, 0], sharex=ax_accel)
    ax_pos    = fig.add_subplot(gs[2, 0], sharex=ax_accel)
    ax3d      = fig.add_subplot(gs[:, 1], projection="3d")

    # ---------------------------------------------------------------- 2-D lines
    line_map = {
            "ax_meas":    ax_accel.plot([], [], label="ax_meas",    alpha=0.75)[0],
            "ay_meas":    ax_accel.plot([], [], label="ay_meas",    alpha=0.75)[0],
            "az_meas":    ax_accel.plot([], [], label="az_meas",    alpha=0.75)[0],
            "a_x_kalman": ax_accel.plot([], [], label="a_x_kalman", linewidth=2)[0],
            "a_y_kalman": ax_accel.plot([], [], label="a_y_kalman", linewidth=2)[0],
            "a_z_kalman": ax_accel.plot([], [], label="a_z_kalman", linewidth=2)[0],

            "roll_meas":   ax_att.plot([], [], label="roll_meas",   alpha=0.75)[0],
            "pitch_meas":  ax_att.plot([], [], label="pitch_meas",  alpha=0.75)[0],
            "yaw_meas":    ax_att.plot([], [], label="yaw_meas",    alpha=0.75)[0],
            "roll_kalman": ax_att.plot([], [], label="roll_kalman", linewidth=2)[0],
            "pitch_kalman":ax_att.plot([], [], label="pitch_kalman",linewidth=2)[0],
            "yaw_kalman":  ax_att.plot([], [], label="yaw_kalman",  linewidth=2)[0],

            "p_x": ax_pos.plot([], [], label="p_x", linewidth=2)[0],
            "p_y": ax_pos.plot([], [], label="p_y", linewidth=2)[0],
            "p_z": ax_pos.plot([], [], label="p_z", linewidth=2)[0],
            "v_x": ax_pos.plot([], [], label="v_x", linestyle="--")[0],
            "v_y": ax_pos.plot([], [], label="v_y", linestyle="--")[0],
            "v_z": ax_pos.plot([], [], label="v_z", linestyle="--")[0],
            }

    ax_accel.set_ylabel("Accel")
    ax_accel.set_title("Acceleration (m/s²): measurement vs Kalman")
    ax_accel.grid(True, alpha=0.3)
    ax_accel.legend(loc="upper right", ncol=2, fontsize=7)

    ax_att.set_ylabel("Angle (rad)")
    ax_att.set_title("Attitude: measurement vs Kalman")
    ax_att.grid(True, alpha=0.3)
    ax_att.legend(loc="upper right", ncol=2, fontsize=7)

    ax_pos.set_ylabel("Position / Velocity")
    ax_pos.set_xlabel("Time (s)")
    ax_pos.set_title("Kalman position and velocity states")
    ax_pos.grid(True, alpha=0.3)
    ax_pos.legend(loc="upper right", ncol=2, fontsize=7)

    # Draw the 3-D panel once at identity so it isn't blank on startup
    R0 = np.eye(3)
    draw_orientation_axes(ax3d, R0)

    fp = log_path.open("w", newline="", encoding="utf-8")
    writer = csv.writer(fp)
    writer.writerow(COLUMNS)
    fp.flush()

    # ----------------------------------------------------------------- update
    def update(_frame):
        nonlocal first_t_ms, header_received
        nonlocal last_plot_time, last_flush_time, y_rescale_counter

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

            # --- time-series updates ---
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

            ax_pos.set_xlim(0.0, t[-1] + 0.1)

            y_rescale_counter += 1
            if y_rescale_counter >= 4:
                for ax in (ax_accel, ax_att, ax_pos):
                    ax.relim()
                    ax.autoscale_view(scalex=False)
                y_rescale_counter = 0

            # --- 3-D orientation update (latest sample) ---
            roll_k  = data["roll"][-1]
            pitch_k = data["pitch"][-1]
            yaw_k   = data["yaw"][-1]

            roll_m  = data["roll_rad"][-1]
            pitch_m = data["pitch_rad"][-1]
            yaw_m   = data["yaw_rad"][-1]

            R_kalman = rpy_to_rotation_matrix(roll_k, pitch_k, yaw_k)
            R_meas   = rpy_to_rotation_matrix(roll_m, pitch_m, yaw_m)
            draw_orientation_axes(ax3d, R_kalman, R_meas)

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
        _ = ani  # keep reference alive for matplotlib
        fp.flush()
        fp.close()
        ser.close()
        print(f"Saved: {log_path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
