#!/usr/bin/env python3
"""Plot visual-servo CSV exports with a shared time axis.

Expected input is one export directory produced by export_vs_rosbag_data.py:

  <export_dir>/csv/sc2_visual_servo_centroid_error.csv
  <export_dir>/csv/sc2_velocity_controller_node_outer_console_signals.csv
  <export_dir>/csv/sc2_odometry_ekf.csv

or a parent directory containing multiple *_csv export directories.

Output per bag:
  plot 1: bbox/centroid center error
  plot 2: x/y control command and x/y EKF-estimated velocity
  plot 3: DVL altitude/range, when available

Use --split-cmd-est when the command signal and estimated velocity do not
share physical units.
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Iterable


CENTROID_CSV = "sc2_visual_servo_centroid_error.csv"
CMD_CSV = "sc2_velocity_controller_node_outer_console_signals.csv"
EKF_ODOM_CSV = "sc2_odometry_ekf.csv"
DVL_CSV = "sc2_dvl_dvl.csv"
WATERLINKED_DVL_CSV = "waterlinked_dvl_driver_velocity_report.csv"


def read_csv(path: Path) -> tuple[list[str], list[dict[str, str]]]:
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        return list(reader.fieldnames or []), list(reader)


def to_float(value: str | None) -> float:
    if value is None or value == "":
        return math.nan
    try:
        return float(value)
    except ValueError:
        return math.nan


def numeric_score(rows: list[dict[str, str]], col: str, sample_n: int = 200) -> int:
    score = 0
    for row in rows[:sample_n]:
        v = to_float(row.get(col))
        if math.isfinite(v):
            score += 1
    return score


def numeric_cols(headers: Iterable[str], rows: list[dict[str, str]]) -> list[str]:
    skip = {
        "bag_time_ns",
        "bag_time_sec",
        "t_rel_sec",
        "header_stamp_ns",
        "header_stamp_sec",
    }
    cols = []
    for col in headers:
        if col in skip:
            continue
        if "covariance" in col.lower():
            continue
        if numeric_score(rows, col) > 0:
            cols.append(col)
    return cols


def pick_first_existing(headers: Iterable[str], candidates: Iterable[str]) -> str | None:
    header_set = set(headers)
    for col in candidates:
        if col in header_set:
            return col
    return None


def pick_centroid_cols(headers: list[str], rows: list[dict[str, str]]) -> list[str]:
    picked = [
        col
        for col in (
            "x",
            "y",
            "z",
            "vector.x",
            "vector.y",
            "vector.z",
            "point.x",
            "point.y",
            "point.z",
        )
        if col in headers
    ]
    if picked:
        return picked[:3]
    return numeric_cols(headers, rows)[:3]


def pick_cmd_cols(
    headers: list[str],
    rows: list[dict[str, str]],
    explicit_x: str | None,
    explicit_y: str | None,
) -> tuple[str | None, str | None]:
    if explicit_x or explicit_y:
        return explicit_x, explicit_y

    x = pick_first_existing(
        headers,
        (
            "x",
            "vx",
            "v_x",
            "linear.x",
            "linear_x",
            "cmd_vx",
            "cmd_x",
            "data.0",
            "values.0",
            "vector.0",
        ),
    )
    y = pick_first_existing(
        headers,
        (
            "y",
            "vy",
            "v_y",
            "linear.y",
            "linear_y",
            "cmd_vy",
            "cmd_y",
            "data.1",
            "values.1",
            "vector.1",
        ),
    )
    if x and y:
        return x, y

    cols = numeric_cols(headers, rows)
    if len(cols) >= 2:
        return cols[0], cols[1]
    if len(cols) == 1:
        return cols[0], None
    return None, None


def pick_velocity_cols(
    headers: list[str],
    rows: list[dict[str, str]],
    explicit_x: str | None,
    explicit_y: str | None,
) -> tuple[str | None, str | None]:
    if explicit_x or explicit_y:
        return explicit_x, explicit_y

    x = pick_first_existing(
        headers,
        (
            "twist.twist.linear.x",
            "twist.linear.x",
            "linear.x",
            "velocity.x",
            "velocity_x",
            "vx",
            "v_x",
            "x",
        ),
    )
    y = pick_first_existing(
        headers,
        (
            "twist.twist.linear.y",
            "twist.linear.y",
            "linear.y",
            "velocity.y",
            "velocity_y",
            "vy",
            "v_y",
            "y",
        ),
    )
    if x and y:
        return x, y

    cols = numeric_cols(headers, rows)
    if len(cols) >= 2:
        return cols[0], cols[1]
    if len(cols) == 1:
        return cols[0], None
    return None, None


def pick_est_velocity_csv(csv_dir: Path, explicit_csv: str | None) -> Path:
    if explicit_csv:
        return csv_dir / explicit_csv
    ekf_path = csv_dir / EKF_ODOM_CSV
    if ekf_path.exists():
        return ekf_path
    return csv_dir / DVL_CSV


def pick_altitude_col(headers: list[str], rows: list[dict[str, str]], explicit_altitude: str | None) -> str | None:
    if explicit_altitude:
        return explicit_altitude

    col = pick_first_existing(
        headers,
        (
            "altitude",
            "altitude_m",
            "range",
            "range_m",
            "distance",
            "distance_m",
            "bottom_distance",
            "bottom_distance_m",
            "beam_range",
            "beams.0.range",
            "beams.0.range_m",
        ),
    )
    if col:
        return col

    numeric = numeric_cols(headers, rows)
    altitude_like = [
        c
        for c in numeric
        if any(token in c.lower() for token in ("alt", "range", "distance", "dist"))
    ]
    if altitude_like:
        return altitude_like[0]
    return None


def series(
    rows: list[dict[str, str]],
    y_col: str,
    *,
    start_sec: float | None = None,
    end_sec: float | None = None,
    time_offset: float = 0.0,
) -> tuple[list[float], list[float]]:
    t: list[float] = []
    y: list[float] = []
    for row in rows:
        tt = to_float(row.get("t_rel_sec"))
        yy = to_float(row.get(y_col))
        if math.isfinite(tt):
            if start_sec is not None and tt < start_sec:
                continue
            if end_sec is not None and tt > end_sec:
                continue
            t.append(tt - time_offset)
            y.append(yy)
    return t, y


def finite_range(values: Iterable[float]) -> tuple[float, float] | None:
    vals = [v for v in values if math.isfinite(v)]
    if not vals:
        return None
    return min(vals), max(vals)


def find_export_dirs(path: Path) -> list[Path]:
    if (path / "csv").is_dir():
        return [path]
    dirs = []
    for child in sorted(path.iterdir()):
        if child.is_dir() and (child / "csv").is_dir():
            dirs.append(child)
    return dirs


def is_centroid_z_col(col: str) -> bool:
    parts = col.replace("[", ".").replace("]", "").split(".")
    return bool(parts and parts[-1] == "z")


def plot_one_export(
    export_dir: Path,
    output_dir: Path,
    *,
    cmd_x_col: str | None,
    cmd_y_col: str | None,
    est_velocity_csv: str | None,
    est_x_col: str | None,
    est_y_col: str | None,
    altitude_col: str | None,
    centroid_cols_arg: list[str] | None,
    no_centroid_z: bool,
    start_sec: float | None,
    end_sec: float | None,
    zero_time: bool,
    split_cmd_est: bool,
    dpi: int,
) -> None:
    import matplotlib.pyplot as plt

    csv_dir = export_dir / "csv"
    centroid_path = csv_dir / CENTROID_CSV
    cmd_path = csv_dir / CMD_CSV
    est_velocity_path = pick_est_velocity_csv(csv_dir, est_velocity_csv)
    waterlinked_path = csv_dir / WATERLINKED_DVL_CSV

    missing = [str(p) for p in (centroid_path, cmd_path, est_velocity_path) if not p.exists()]
    if missing:
        print(f"[SKIP] {export_dir.name}: missing {missing}")
        return

    c_headers, c_rows = read_csv(centroid_path)
    cmd_headers, cmd_rows = read_csv(cmd_path)
    est_headers, est_rows = read_csv(est_velocity_path)
    waterlinked_headers: list[str] = []
    waterlinked_rows: list[dict[str, str]] = []
    if waterlinked_path.exists():
        waterlinked_headers, waterlinked_rows = read_csv(waterlinked_path)

    if centroid_cols_arg:
        centroid_cols = [col for col in centroid_cols_arg if col in c_headers]
        missing_centroid_cols = [col for col in centroid_cols_arg if col not in c_headers]
        if missing_centroid_cols:
            print(f"[WARN] {export_dir.name}: centroid columns not found: {missing_centroid_cols}")
    else:
        centroid_cols = pick_centroid_cols(c_headers, c_rows)
    if no_centroid_z:
        centroid_cols = [col for col in centroid_cols if not is_centroid_z_col(col)]
    auto_cmd_x, auto_cmd_y = pick_cmd_cols(cmd_headers, cmd_rows, cmd_x_col, cmd_y_col)
    auto_est_x, auto_est_y = pick_velocity_cols(est_headers, est_rows, est_x_col, est_y_col)
    auto_altitude = pick_altitude_col(waterlinked_headers, waterlinked_rows, altitude_col)

    if not centroid_cols:
        print(f"[SKIP] {export_dir.name}: no centroid numeric columns found")
        return

    time_offset = start_sec if zero_time and start_sec is not None else 0.0
    all_t: list[float] = []
    if split_cmd_est:
        fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True, constrained_layout=True)
        ax1, ax_cmd, ax_est, ax_altitude = axes
    else:
        fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True, constrained_layout=True)
        ax1, ax_cmd, ax_altitude = axes
        ax_est = ax_cmd

    for col in centroid_cols:
        t, y = series(c_rows, col, start_sec=start_sec, end_sec=end_sec, time_offset=time_offset)
        all_t.extend(t)
        ax1.plot(t, y, marker=".", markersize=2, linewidth=1.0, label=f"centroid {col}")

    ax1.axhline(0.0, color="black", linewidth=0.8, alpha=0.4)
    ax1.set_ylabel("bbox center error")
    ax1.set_title(export_dir.name)
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc="best", fontsize="small")

    plotted_cmd = False
    plotted_est = False
    for label, rows, col, style, ax in (
        ("cmd vx", cmd_rows, auto_cmd_x, "-", ax_cmd),
        ("cmd vy", cmd_rows, auto_cmd_y, "-", ax_cmd),
        ("est vx", est_rows, auto_est_x, "--", ax_est),
        ("est vy", est_rows, auto_est_y, "--", ax_est),
    ):
        if col is None:
            continue
        t, y = series(rows, col, start_sec=start_sec, end_sec=end_sec, time_offset=time_offset)
        all_t.extend(t)
        ax.plot(t, y, style, linewidth=1.2, label=f"{label}: {col}")
        if label.startswith("cmd"):
            plotted_cmd = True
        else:
            plotted_est = True

    ax_cmd.axhline(0.0, color="black", linewidth=0.8, alpha=0.4)
    ax_cmd.set_ylabel("console command [-]" if split_cmd_est else "velocity / command")
    ax_cmd.grid(True, alpha=0.3)
    if plotted_cmd:
        ax_cmd.legend(loc="best", fontsize="small")

    if split_cmd_est:
        ax_est.axhline(0.0, color="black", linewidth=0.8, alpha=0.4)
        ax_est.set_ylabel("EKF estimated velocity [m/s]")
        ax_est.grid(True, alpha=0.3)
        if plotted_est:
            ax_est.legend(loc="best", fontsize="small")
    elif plotted_est:
        ax_est.legend(loc="best", fontsize="small")

    if auto_altitude is not None:
        t, y = series(waterlinked_rows, auto_altitude, start_sec=start_sec, end_sec=end_sec, time_offset=time_offset)
        all_t.extend(t)
        ax_altitude.plot(t, y, color="tab:purple", linewidth=1.2, label=f"altitude: {auto_altitude}")
        ax_altitude.legend(loc="best", fontsize="small")
    else:
        ax_altitude.text(
            0.5,
            0.5,
            "No altitude column detected",
            transform=ax_altitude.transAxes,
            ha="center",
            va="center",
            color="tab:red",
        )
    ax_altitude.set_xlabel("time from window start [s]" if zero_time else "time from bag start [s]")
    ax_altitude.set_ylabel("altitude / range [m]")
    ax_altitude.grid(True, alpha=0.3)

    if start_sec is not None or end_sec is not None:
        x_start = 0.0 if zero_time else (0.0 if start_sec is None else start_sec)
        if end_sec is not None:
            x_end = end_sec - time_offset
        else:
            tr = finite_range(all_t)
            x_end = tr[1] if tr is not None else x_start + 1.0
        ax_altitude.set_xlim(x_start, x_end)
    else:
        tr = finite_range(all_t)
        if tr is not None:
            ax_altitude.set_xlim(*tr)

    output_dir.mkdir(parents=True, exist_ok=True)
    out_png = output_dir / f"{export_dir.name}_vs_summary.png"
    fig.savefig(out_png, dpi=dpi)
    plt.close(fig)

    print(f"[OK] {export_dir.name}")
    print(f"     centroid cols: {centroid_cols}")
    print(f"     cmd cols: x={auto_cmd_x}, y={auto_cmd_y}")
    print(f"     est velocity csv: {est_velocity_path.name}")
    print(f"     est velocity cols: x={auto_est_x}, y={auto_est_y}")
    print(f"     altitude col: {auto_altitude}")
    print(f"     plot: {out_png}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", help="One *_csv export dir, or parent directory containing *_csv dirs.")
    parser.add_argument("--output-dir", default=None)
    parser.add_argument("--cmd-x-col", default=None)
    parser.add_argument("--cmd-y-col", default=None)
    parser.add_argument(
        "--est-velocity-csv",
        default=None,
        help=f"Estimated-velocity CSV filename under csv/. Defaults to {EKF_ODOM_CSV} if present, otherwise {DVL_CSV}.",
    )
    parser.add_argument("--est-x-col", default=None)
    parser.add_argument("--est-y-col", default=None)
    parser.add_argument("--dvl-x-col", default=None, help="Backward-compatible alias for --est-x-col.")
    parser.add_argument("--dvl-y-col", default=None, help="Backward-compatible alias for --est-y-col.")
    parser.add_argument("--altitude-col", default=None)
    parser.add_argument("--centroid-cols", nargs="+", default=None)
    parser.add_argument("--no-centroid-z", action="store_true")
    parser.add_argument("--start-sec", type=float, default=None)
    parser.add_argument("--end-sec", type=float, default=None)
    parser.add_argument("--zero-time", action="store_true", help="Plot time as seconds from --start-sec.")
    parser.add_argument(
        "--split-cmd-est",
        action="store_true",
        help="Plot console command and estimated velocity in separate subplots.",
    )
    parser.add_argument("--dpi", type=int, default=160)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    input_path = Path(args.input).expanduser().resolve()
    output_dir = (
        Path(args.output_dir).expanduser().resolve()
        if args.output_dir
        else input_path / "plots"
        if (input_path / "csv").is_dir()
        else input_path / "_plots"
    )
    export_dirs = find_export_dirs(input_path)
    if not export_dirs:
        raise SystemExit(f"No export dirs with csv/ found under {input_path}")

    for export_dir in export_dirs:
        plot_one_export(
            export_dir,
            output_dir,
            cmd_x_col=args.cmd_x_col,
            cmd_y_col=args.cmd_y_col,
            est_velocity_csv=args.est_velocity_csv,
            est_x_col=args.est_x_col or args.dvl_x_col,
            est_y_col=args.est_y_col or args.dvl_y_col,
            altitude_col=args.altitude_col,
            centroid_cols_arg=args.centroid_cols,
            no_centroid_z=args.no_centroid_z,
            start_sec=args.start_sec,
            end_sec=args.end_sec,
            zero_time=args.zero_time,
            split_cmd_est=args.split_cmd_est,
            dpi=args.dpi,
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
