#!/usr/bin/env python3
"""
Kalman + Controller Realtime Capture — low-latency version.

Firmware emits two CSV rows per log tick on Serial:

  Kalman row  (20 columns):
    t_ms, wx, wy, wz,
    p_x, p_y, p_z, v_x, v_y, v_z, a_x, a_y, a_z,
    d_theta, d_alpha, d_beta,
    q_w, q_x, q_y, q_z

  Controller row  (9 columns):
    t_ms,
    qd_w, qd_x, qd_y, qd_z,
    fin0, fin1, fin2, fin3

Rows are distinguished by column count.
Both are logged to separate CSV files under logs/.
"""
from __future__ import annotations

import csv
import datetime as dt
from collections import deque
import pathlib
import sys
import time

import numpy as np

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------
SERIAL_PORT        = "COM9"
BAUD_RATE          = 115200
SERIAL_TIMEOUT_SEC = 0.02

MAX_POINTS           = 500
PLOT_INTERVAL_SEC    = 0.10    # ~10 fps
MAX_LINES_PER_UPDATE = 500
ORIENTATION_EVERY_N  = 5

LOG_DIR = pathlib.Path("logs")

# ── Column definitions ──────────────────────────────────────────────────────
KALMAN_COLS = [
    "t_ms",
    "wx", "wy", "wz",
    "p_x", "p_y", "p_z",
    "v_x", "v_y", "v_z",
    "a_x", "a_y", "a_z",
    "d_theta", "d_alpha", "d_beta",
    "q_w", "q_x", "q_y", "q_z",
]
CTRL_COLS = [
    "t_ms",
    "qd_w", "qd_x", "qd_y", "qd_z",
    "fin0", "fin1", "fin2", "fin3",
]

K_IDX = {col: i for i, col in enumerate(KALMAN_COLS)}
C_IDX = {col: i for i, col in enumerate(CTRL_COLS)}

N_KALMAN = len(KALMAN_COLS)   # 20
N_CTRL   = len(CTRL_COLS)     #  9

HEADER_PREFIXES = ("t_ms,",)


# ---------------------------------------------------------------------------
# Math helpers
# ---------------------------------------------------------------------------
def quat_to_rotation_matrix(q_w, q_x, q_y, q_z) -> np.ndarray:
    n = (q_w**2 + q_x**2 + q_y**2 + q_z**2) ** 0.5
    if n < 1e-9:
        return np.eye(3)
    w, x, y, z = q_w / n, q_x / n, q_y / n, q_z / n
    return np.array([
        [1 - 2*(y*y + z*z),   2*(x*y - w*z),   2*(x*z + w*y)],
        [    2*(x*y + w*z), 1 - 2*(x*x + z*z),   2*(y*z - w*x)],
        [    2*(x*z - w*y),   2*(y*z + w*x), 1 - 2*(x*x + y*y)],
    ])


def quat_to_euler(q_w, q_x, q_y, q_z):
    n = (q_w**2 + q_x**2 + q_y**2 + q_z**2) ** 0.5
    if n < 1e-9:
        return 0.0, 0.0, 0.0
    w, x, y, z = q_w / n, q_x / n, q_y / n, q_z / n
    roll  = np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    sinp  = max(-1.0, min(1.0, 2*(w*y - z*x)))
    pitch = np.arcsin(sinp)
    yaw   = np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return roll, pitch, yaw


# ---------------------------------------------------------------------------
# Serial helpers
# ---------------------------------------------------------------------------
def open_serial(port, baud, timeout):
    try:
        import serial
    except ImportError:
        sys.exit("Missing pyserial — pip install pyserial")
    try:
        ser = serial.Serial(port=port, baudrate=baud, timeout=timeout)
        time.sleep(2.0)
        ser.reset_input_buffer()
        return ser
    except Exception as exc:
        sys.exit(f"Cannot open {port}: {exc}")


def create_log_files(log_dir: pathlib.Path):
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = dt.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    return (
        log_dir / f"kalman_{stamp}.csv",
        log_dir / f"controller_{stamp}.csv",
        log_dir / f"unknown_{stamp}.txt",   # capture unparsed lines for debug
    )


def parse_row(line: str):
    """
    Return (kind, values) where kind is 'kalman', 'ctrl', or None.
    Distinguished purely by column count: 20 = kalman, 9 = ctrl.
    """
    parts = line.split(",")
    n = len(parts)
    try:
        values = [float(x) for x in parts]
    except ValueError:
        return None, None
    if n == N_KALMAN:
        return "kalman", values
    if n == N_CTRL:
        return "ctrl", values
    return None, None


# ---------------------------------------------------------------------------
# 3-D orientation widget
# ---------------------------------------------------------------------------
class OrientationAxes:
    COLORS = ["#E24B4A", "#1D9E75", "#378ADD"]

    def __init__(self, ax3d, title="Orientation"):
        self.ax = ax3d
        ax3d.set_xlim(-1.4, 1.4); ax3d.set_ylim(-1.4, 1.4); ax3d.set_zlim(-1.4, 1.4)
        ax3d.set_xlabel("X", fontsize=8); ax3d.set_ylabel("Y", fontsize=8)
        ax3d.set_zlabel("Z", fontsize=8)
        ax3d.set_box_aspect([1, 1, 1])
        ax3d.tick_params(labelsize=7)
        ax3d.set_title(title, fontsize=9)

        R = np.eye(3)
        o = np.zeros(3)
        self._q = [
            ax3d.quiver(*o, *R[:, i], color=self.COLORS[i],
                        linewidth=2.5, arrow_length_ratio=0.15)
            for i in range(3)
        ]
        self._txt = [
            ax3d.text(*(R[:, i] * 1.18), lbl, color=self.COLORS[i],
                      fontsize=10, fontweight="bold", ha="center", va="center")
            for i, lbl in enumerate("XYZ")
        ]
        for dx, dy, dz in [(1, 0, 0), (0, 1, 0), (0, 0, 1)]:
            ax3d.quiver(0, 0, 0, dx, dy, dz, color="grey",
                        alpha=0.18, linewidth=1, arrow_length_ratio=0.1)

    def update(self, R: np.ndarray) -> None:
        o = np.zeros(3)
        for i in range(3):
            self._q[i].remove()
            self._q[i] = self.ax.quiver(*o, *R[:, i], color=self.COLORS[i],
                                        linewidth=2.5, arrow_length_ratio=0.15)
            self._txt[i].set_position_3d(R[:, i] * 1.18)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main() -> int:
    try:
        import matplotlib
        matplotlib.use("TkAgg")
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
    except ImportError:
        sys.exit("Missing matplotlib — pip install matplotlib")

    ser = open_serial(SERIAL_PORT, BAUD_RATE, SERIAL_TIMEOUT_SEC)
    kalman_log, ctrl_log, unknown_log = create_log_files(LOG_DIR)
    print(f"Listening on {SERIAL_PORT} @ {BAUD_RATE}")
    print(f"  Kalman log     → {kalman_log}")
    print(f"  Controller log → {ctrl_log}")
    print(f"  Unknown log    → {unknown_log}  (non-empty = parse problem)")
    print("Close the window or Ctrl-C to stop.\n")

    # ── Ring buffers ─────────────────────────────────────────────────────────
    kbuf: dict[str, deque] = {col: deque(maxlen=MAX_POINTS) for col in KALMAN_COLS}
    cbuf: dict[str, deque] = {col: deque(maxlen=MAX_POINTS) for col in CTRL_COLS}
    roll_buf   : deque = deque(maxlen=MAX_POINTS)
    pitch_buf  : deque = deque(maxlen=MAX_POINTS)
    yaw_buf    : deque = deque(maxlen=MAX_POINTS)
    droll_buf  : deque = deque(maxlen=MAX_POINTS)
    dpitch_buf : deque = deque(maxlen=MAX_POINTS)
    dyaw_buf   : deque = deque(maxlen=MAX_POINTS)

    # Shared time origin — set on the very first parseable row of either type
    first_t_ms: float | None = None

    # Diagnostic counters
    counts = {"kalman": 0, "ctrl": 0, "skip": 0, "bad": 0}

    last_plot_time = 0.0
    orient_counter = 0

    # ── Figure layout ────────────────────────────────────────────────────────
    fig = plt.figure(figsize=(16, 9))
    fig.suptitle("Kalman + Controller Realtime", fontsize=11)

    gs = fig.add_gridspec(
        3, 3,
        width_ratios=[2.2, 1.3, 1.3],
        hspace=0.52, wspace=0.32,
    )

    ax_acc   = fig.add_subplot(gs[0, 0])
    ax_att   = fig.add_subplot(gs[1, 0], sharex=ax_acc)
    ax_pos   = fig.add_subplot(gs[2, 0], sharex=ax_acc)
    ax3d_cur = fig.add_subplot(gs[0:2, 1], projection="3d")
    ax_fins  = fig.add_subplot(gs[2, 1])
    ax3d_des = fig.add_subplot(gs[0:2, 2], projection="3d")
    ax_datt  = fig.add_subplot(gs[2, 2], sharex=ax_acc)

    # ── Line factory ─────────────────────────────────────────────────────────
    def lines(ax, specs):
        return {k: ax.plot([], [], **kw)[0] for k, kw in specs.items()}

    La = lines(ax_acc, {
        "ax_k": dict(label="a_x", lw=2),
        "ay_k": dict(label="a_y", lw=2),
        "az_k": dict(label="a_z", lw=2),
    })
    Lb = lines(ax_att, {
        "roll":  dict(label="roll",  lw=2),
        "pitch": dict(label="pitch", lw=2),
        "yaw":   dict(label="yaw",   lw=2),
        "dth":   dict(label="dθ", ls=":", alpha=0.7, lw=1),
        "dal":   dict(label="dα", ls=":", alpha=0.7, lw=1),
        "dbe":   dict(label="dβ", ls=":", alpha=0.7, lw=1),
    })
    Lc = lines(ax_pos, {
        "px": dict(label="p_x", lw=2),
        "py": dict(label="p_y", lw=2),
        "pz": dict(label="p_z", lw=2),
        "vx": dict(label="v_x", ls="--", lw=1),
        "vy": dict(label="v_y", ls="--", lw=1),
        "vz": dict(label="v_z", ls="--", lw=1),
    })
    Lf = lines(ax_fins, {
        "f0": dict(label="fin 0", lw=2),
        "f1": dict(label="fin 1", lw=2),
        "f2": dict(label="fin 2", lw=2),
        "f3": dict(label="fin 3", lw=2),
    })
    Ld = lines(ax_datt, {
        "droll":  dict(label="des roll",  lw=2, ls="--"),
        "dpitch": dict(label="des pitch", lw=2, ls="--"),
        "dyaw":   dict(label="des yaw",   lw=2, ls="--"),
    })

    # ── Axis labels / grid ───────────────────────────────────────────────────
    for ax, title, ylabel in [
        (ax_acc,  "Kalman acceleration (world frame, m/s²)", "m/s²"),
        (ax_att,  "Attitude (ZYX Euler) + Δθ error",         "rad"),
        (ax_pos,  "Position & velocity",                      "m / m/s"),
        (ax_fins, "Fin deflections",                          "deg"),
        (ax_datt, "Desired attitude (Euler from q_desired)",  "rad"),
    ]:
        ax.set_title(title, fontsize=8, pad=3)
        ax.set_ylabel(ylabel, fontsize=8)
        ax.grid(True, alpha=0.25)
        ax.legend(loc="upper right", ncol=3, fontsize=6)
    ax_pos.set_xlabel("Time (s)", fontsize=8)
    ax_fins.set_xlabel("Time (s)", fontsize=8)
    ax_datt.set_xlabel("Time (s)", fontsize=8)

    # ── Diagnostic text overlay ───────────────────────────────────────────────
    # Shows live row counts — if ctrl stays 0, the rows aren't arriving/parsing
    diag_text = fig.text(
        0.01, 0.005,
        "waiting for data…",
        fontsize=7, family="monospace",
        va="bottom", ha="left",
        color="#888888",
    )

    # ── 3-D orientation widgets ──────────────────────────────────────────────
    orient_cur = OrientationAxes(ax3d_cur, title="Current orientation")
    orient_des = OrientationAxes(ax3d_des, title="Desired orientation")

    fig.tight_layout(rect=[0, 0, 1, 0.96])

    # ── Log files ────────────────────────────────────────────────────────────
    fp_k = kalman_log.open("w", newline="", encoding="utf-8")
    fp_c = ctrl_log.open("w", newline="", encoding="utf-8")
    fp_u = unknown_log.open("w", encoding="utf-8")
    writer_k = csv.writer(fp_k); writer_k.writerow(KALMAN_COLS)
    writer_c = csv.writer(fp_c); writer_c.writerow(CTRL_COLS)

    all_lines = (
        list(La.values()) + list(Lb.values()) + list(Lc.values()) +
        list(Lf.values()) + list(Ld.values())
    )

    # ── Animation callback ───────────────────────────────────────────────────
    def update(_frame):
        nonlocal first_t_ms, last_plot_time, orient_counter

        new_k = new_c = 0

        for _ in range(MAX_LINES_PER_UPDATE):
            raw = ser.readline()
            if not raw:
                break
            line = raw.decode("utf-8", errors="replace").strip()
            if not line:
                continue
            if line.startswith("err,"):
                counts["skip"] += 1
                continue
            # Firmware may re-emit a CSV header row — skip it
            if any(line.startswith(p) for p in HEADER_PREFIXES):
                counts["skip"] += 1
                continue

            kind, row = parse_row(line)

            if kind == "kalman":
                writer_k.writerow(row)
                counts["kalman"] += 1
                new_k += 1

                t_raw = row[K_IDX["t_ms"]]
                if first_t_ms is None:
                    first_t_ms = t_raw
                t_s = (t_raw - first_t_ms) / 1000.0

                kbuf["t_ms"].append(t_s)
                for col in KALMAN_COLS[1:]:
                    kbuf[col].append(row[K_IDX[col]])

                r, p, y = quat_to_euler(
                    row[K_IDX["q_w"]], row[K_IDX["q_x"]],
                    row[K_IDX["q_y"]], row[K_IDX["q_z"]],
                )
                roll_buf.append(r); pitch_buf.append(p); yaw_buf.append(y)

            elif kind == "ctrl":
                writer_c.writerow(row)
                counts["ctrl"] += 1
                new_c += 1

                t_raw = row[C_IDX["t_ms"]]
                if first_t_ms is None:
                    first_t_ms = t_raw
                t_s = (t_raw - first_t_ms) / 1000.0

                cbuf["t_ms"].append(t_s)
                for col in CTRL_COLS[1:]:
                    cbuf[col].append(row[C_IDX[col]])

                dr, dp, dy = quat_to_euler(
                    row[C_IDX["qd_w"]], row[C_IDX["qd_x"]],
                    row[C_IDX["qd_y"]], row[C_IDX["qd_z"]],
                )
                droll_buf.append(dr); dpitch_buf.append(dp); dyaw_buf.append(dy)

            else:
                # Log unparsed lines — helps diagnose column-count mismatches
                n_cols = len(line.split(","))
                fp_u.write(f"[cols={n_cols}] {line}\n")
                counts["bad"] += 1

        if new_k: fp_k.flush()
        if new_c: fp_c.flush()

        # Update diagnostic overlay every frame (cheap)
        diag_text.set_text(
            f"kalman={counts['kalman']}  ctrl={counts['ctrl']}  "
            f"skip={counts['skip']}  bad={counts['bad']}"
        )

        now = time.monotonic()
        if (now - last_plot_time) < PLOT_INTERVAL_SEC or not kbuf["t_ms"]:
            return all_lines
        last_plot_time = now

        t_k = list(kbuf["t_ms"])

        # ── Kalman plots ──
        La["ax_k"].set_data(t_k, list(kbuf["a_x"]))
        La["ay_k"].set_data(t_k, list(kbuf["a_y"]))
        La["az_k"].set_data(t_k, list(kbuf["a_z"]))

        Lb["roll"].set_data(t_k,  list(roll_buf))
        Lb["pitch"].set_data(t_k, list(pitch_buf))
        Lb["yaw"].set_data(t_k,   list(yaw_buf))
        Lb["dth"].set_data(t_k, list(kbuf["d_theta"]))
        Lb["dal"].set_data(t_k, list(kbuf["d_alpha"]))
        Lb["dbe"].set_data(t_k, list(kbuf["d_beta"]))

        Lc["px"].set_data(t_k, list(kbuf["p_x"]))
        Lc["py"].set_data(t_k, list(kbuf["p_y"]))
        Lc["pz"].set_data(t_k, list(kbuf["p_z"]))
        Lc["vx"].set_data(t_k, list(kbuf["v_x"]))
        Lc["vy"].set_data(t_k, list(kbuf["v_y"]))
        Lc["vz"].set_data(t_k, list(kbuf["v_z"]))

        ax_pos.set_xlim(t_k[0], t_k[-1] + 0.5)
        for ax in (ax_acc, ax_att, ax_pos):
            ax.relim(); ax.autoscale_view(scalex=False)

        # ── Controller plots ──
        if cbuf["t_ms"]:
            t_c = list(cbuf["t_ms"])
            Lf["f0"].set_data(t_c, list(cbuf["fin0"]))
            Lf["f1"].set_data(t_c, list(cbuf["fin1"]))
            Lf["f2"].set_data(t_c, list(cbuf["fin2"]))
            Lf["f3"].set_data(t_c, list(cbuf["fin3"]))

            Ld["droll"].set_data(t_c,  list(droll_buf))
            Ld["dpitch"].set_data(t_c, list(dpitch_buf))
            Ld["dyaw"].set_data(t_c,   list(dyaw_buf))

            # Force a visible y-range even when fins are near zero,
            # so "flat at zero" is distinguishable from "no data"
            fin_vals = (
                list(cbuf["fin0"]) + list(cbuf["fin1"]) +
                list(cbuf["fin2"]) + list(cbuf["fin3"])
            )
            if fin_vals:
                lo = min(min(fin_vals), -1.0)
                hi = max(max(fin_vals),  1.0)
                margin = max((hi - lo) * 0.1, 0.5)
                ax_fins.set_ylim(lo - margin, hi + margin)

            for ax in (ax_fins, ax_datt):
                ax.relim(); ax.autoscale_view(scalex=False)

        # ── 3-D orientation (throttled) ──
        orient_counter += 1
        if orient_counter >= ORIENTATION_EVERY_N:
            orient_counter = 0
            R_cur = quat_to_rotation_matrix(
                kbuf["q_w"][-1], kbuf["q_x"][-1],
                kbuf["q_y"][-1], kbuf["q_z"][-1],
            )
            orient_cur.update(R_cur)
            if cbuf["qd_w"]:
                R_des = quat_to_rotation_matrix(
                    cbuf["qd_w"][-1], cbuf["qd_x"][-1],
                    cbuf["qd_y"][-1], cbuf["qd_z"][-1],
                )
                orient_des.update(R_des)

        fig.canvas.draw_idle()
        return all_lines

    ani = FuncAnimation(fig, update, interval=50, blit=False,
                        cache_frame_data=False)
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        _ = ani
        for fp in (fp_k, fp_c, fp_u):
            fp.flush(); fp.close()
        ser.close()
        print(f"\nSaved kalman log    : {kalman_log}")
        print(f"Saved controller log: {ctrl_log}")
        print(f"Unparsed lines      : {unknown_log}  (bad={counts['bad']})")
        print(f"Final counts — {counts}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
