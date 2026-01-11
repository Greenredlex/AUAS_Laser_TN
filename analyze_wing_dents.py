"""
Analyze dents in wing scans.
Generates an overlay plot of Z-span along Robot Y for multiple scans.
Detects dent extents via large Z-span changes and extracts statistics around known dent locations.

Usage examples:
  python analyze_wing_dents.py --scans "wing_meetingen/meting*.csv"
  python analyze_wing_dents.py --scans "wing_meetingen/meting*.csv" --y-bin 0.5 --save-plot results.png
"""
import argparse
import csv
import json
from pathlib import Path
from typing import Iterable, List, Tuple

import matplotlib.pyplot as plt
import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Wing dent Z-span overlay analysis")
    parser.add_argument(
        "--scans",
        nargs="+",
        default=["wing_meetingen/meting*.csv"],
        help="Glob pattern(s) for scan CSV files",
    )
    parser.add_argument(
        "--y-bin",
        type=float,
        default=0.2,
        help="Bin size in mm when aggregating min/max along robotY",
    )
    parser.add_argument(
        "--dent-change-threshold",
        type=float,
        default=0.8,
        help="Threshold (mm) for detecting large Z-span changes (dent edges)",
    )
    parser.add_argument(
        "--save-plot",
        type=str,
        default=None,
        help="Optional file path to save the plot (e.g., results.png)",
    )
    parser.add_argument(
        "--max-rows",
        type=int,
        default=None,
        help="Optional cap on total frames read per file (for quick previews)",
    )
    return parser.parse_args()


def iter_rows(files: Iterable[Path]) -> Iterable[Tuple[float, float, float, np.ndarray, np.ndarray]]:
    """Yield robot pose and laser arrays for each frame."""
    for csv_path in files:
        with csv_path.open("r", newline="") as f:
            reader = csv.DictReader(f)
            for row in reader:
                try:
                    rx = float(row.get("robot_x", 0.0))
                    ry = float(row.get("robot_y", 0.0))
                    rz = float(row.get("robot_z", 0.0))
                    lx = np.array(json.loads(row["x_values"]), dtype=float)
                    lz = np.array(json.loads(row["z_values"]), dtype=float)
                except Exception:
                    continue
                if lx.size == 0 or lz.size == 0:
                    continue
                # Transform: actual height is robot_z + laser_z; X along scanner line.
                actual_z = rz + lz
                global_x = rx + lx
                yield rx, ry, rz, global_x, actual_z


def load_points(patterns: List[str], max_rows: int | None) -> Tuple[np.ndarray, np.ndarray, np.ndarray, List[Tuple[float, np.ndarray, np.ndarray]]]:
    matched: List[Path] = []
    for pat in patterns:
        matched.extend(Path().glob(pat))
    matched = sorted(set(matched))
    if not matched:
        raise FileNotFoundError(f"No CSV files matched: {patterns}")

    xs: List[float] = []
    ys: List[float] = []
    zs: List[float] = []
    per_frame: List[Tuple[float, np.ndarray, np.ndarray]] = []

    for idx, (rx, ry, rz, gx, az) in enumerate(iter_rows(matched)):
        if max_rows is not None and idx >= max_rows:
            break
        xs.append(gx)
        ys.append(np.full_like(gx, ry))
        zs.append(az)
        per_frame.append((ry, gx, az))

    if not xs:
        raise RuntimeError("No points loaded from the provided scans")

    x_arr = np.concatenate(xs)
    y_arr = np.concatenate(ys)
    z_arr = np.concatenate(zs)
    return x_arr, y_arr, z_arr, per_frame


def load_per_file(patterns: List[str], max_rows: int | None) -> List[Tuple[str, List[Tuple[float, np.ndarray, np.ndarray]]]]:
    """Load scans per file separately (don't concatenate)."""
    matched: List[Path] = []
    for pat in patterns:
        matched.extend(Path().glob(pat))
    matched = sorted(set(matched))
    if not matched:
        raise FileNotFoundError(f"No CSV files matched: {patterns}")

    results: List[Tuple[str, List[Tuple[float, np.ndarray, np.ndarray]]]] = []
    
    for csv_path in matched:
        per_frame: List[Tuple[float, np.ndarray, np.ndarray]] = []
        with csv_path.open("r", newline="") as f:
            reader = csv.DictReader(f)
            for idx, row in enumerate(reader):
                if max_rows is not None and idx >= max_rows:
                    break
                try:
                    rx = float(row.get("robot_x", 0.0))
                    ry = float(row.get("robot_y", 0.0))
                    rz = float(row.get("robot_z", 0.0))
                    lx = np.array(json.loads(row["x_values"]), dtype=float)
                    lz = np.array(json.loads(row["z_values"]), dtype=float)
                except Exception:
                    continue
                if lx.size == 0 or lz.size == 0:
                    continue
                actual_z = rz + lz
                global_x = rx + lx
                per_frame.append((ry, global_x, actual_z))
        
        if per_frame:
            results.append((csv_path.name, per_frame))
    
    if not results:
        raise RuntimeError("No points loaded from the provided scans")
    return results


def compute_span(per_frame: List[Tuple[float, np.ndarray, np.ndarray]], y_bin: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Compute min, max, and span along robotY with binning."""
    y_vals: List[float] = []
    min_z: List[float] = []
    max_z: List[float] = []

    for ry, gx, az in per_frame:
        if az.size == 0:
            continue
        y_vals.append(ry)
        min_z.append(np.min(az))
        max_z.append(np.max(az))

    if not y_vals:
        return np.array([]), np.array([]), np.array([]), np.array([])

    y_vals = np.array(y_vals)
    min_z = np.array(min_z)
    max_z = np.array(max_z)

    y_min, y_max = float(y_vals.min()), float(y_vals.max())
    bins = np.arange(y_min, y_max + y_bin, y_bin)
    bin_idx = np.digitize(y_vals, bins) - 1

    agg_y: List[float] = []
    agg_min: List[float] = []
    agg_max: List[float] = []

    for b in range(int(bin_idx.min()), int(bin_idx.max()) + 1):
        mask = bin_idx == b
        if not np.any(mask):
            continue
        agg_y.append(np.mean(y_vals[mask]))
        agg_min.append(np.mean(min_z[mask]))
        agg_max.append(np.mean(max_z[mask]))

    agg_y = np.array(agg_y)
    agg_min = np.array(agg_min)
    agg_max = np.array(agg_max)
    agg_span = agg_max - agg_min
    
    return agg_y, agg_min, agg_max, agg_span


def detect_dent_changes(agg_span: np.ndarray, threshold: float) -> List[Tuple[int, float]]:
    """Detect dent edges by finding large changes in Z-span."""
    if len(agg_span) < 2:
        return []
    changes = np.abs(np.diff(agg_span))
    edges: List[Tuple[int, float]] = []
    for i, change in enumerate(changes):
        if change > threshold:
            edges.append((i, change))
    return edges


def extract_dent_stats(
    file_data: List[Tuple[str, np.ndarray, np.ndarray, np.ndarray, np.ndarray]],
    y_centers: List[float],
    y_range: float = 30.0
) -> dict:
    """Extract Z-span statistics around known dent locations."""
    stats: dict = {}
    for y_center in y_centers:
        y_min, y_max = y_center - y_range, y_center + y_range
        dent_stats = {
            "center": y_center,
            "range": (y_min, y_max),
            "values": {},
        }
        
        for filename, agg_y, agg_min, agg_max, agg_span in file_data:
            mask = (agg_y >= y_min) & (agg_y <= y_max)
            if np.any(mask):
                spans = agg_span[mask]
                dent_stats["values"][filename] = {
                    "mean": np.mean(spans),
                    "std": np.std(spans),
                    "spans": spans,
                }
        
        # Compute overall stats
        all_spans = []
        for v in dent_stats["values"].values():
            all_spans.extend(v["spans"])
        if all_spans:
            dent_stats["overall_mean"] = np.mean(all_spans)
            dent_stats["overall_std"] = np.std(all_spans)
        
        stats[f"Dent@Y{y_center}"] = dent_stats
    
    return stats


def detect_dent_edges(per_frame: List[Tuple[float, np.ndarray, np.ndarray]], depth_threshold: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    y_list: List[float] = []
    start_x: List[float] = []
    width: List[float] = []

    for ry, gx, az in per_frame:
        if az.size < 4:
            continue
        baseline = np.percentile(az, 90)
        depth = baseline - az
        mask = depth > depth_threshold
        if not np.any(mask):
            continue
        dent_x = gx[mask]
        y_list.append(ry)
        start_x.append(np.min(dent_x))
        width.append(np.max(dent_x) - np.min(dent_x))

    return np.array(y_list), np.array(start_x), np.array(width)


def plot_dent_edges(ax: plt.Axes, y_pos: np.ndarray, x_start: np.ndarray, widths: np.ndarray) -> None:
    if y_pos.size == 0:
        ax.text(0.5, 0.5, "No dents detected", ha="center", va="center")
        return
    ax.plot(y_pos, x_start, label="Dent start X", color="tab:green")
    ax.plot(y_pos, x_start + widths, label="Dent end X", color="tab:red")
    ax.fill_between(y_pos, x_start, x_start + widths, color="tab:purple", alpha=0.2, label="Width")
    ax.set_xlabel("Robot Y (mm)")
    ax.set_ylabel("RobotX + LaserX (mm)")
    ax.set_title("Dent extent along scanner line")
    ax.legend()


def main() -> None:
    args = parse_args()
    try:
        file_data_list = load_per_file(args.scans, args.max_rows)
    except Exception as exc:
        print(f"Error: {exc}")
        return

    # Compute spans for each file
    file_processed: List[Tuple[str, np.ndarray, np.ndarray, np.ndarray]] = []
    for filename, per_frame in file_data_list:
        agg_y, agg_min, agg_max, agg_span = compute_span(per_frame, args.y_bin)
        file_processed.append((filename, agg_y, agg_min, agg_max))

    # Align all to the same starting point
    y_offsets = [data[1][0] if len(data[1]) > 0 else 0 for data in file_processed]
    min_y_start = min(y_offsets)
    
    aligned_data: List[Tuple[str, np.ndarray, np.ndarray, np.ndarray, np.ndarray]] = []
    for (filename, agg_y, agg_min, agg_max), offset in zip(file_processed, y_offsets):
        agg_y_aligned = agg_y - offset + min_y_start
        agg_span = agg_max - agg_min
        aligned_data.append((filename, agg_y_aligned, agg_min, agg_max, agg_span))

    # Create subplots: one per file
    fig, axes = plt.subplots(len(aligned_data), 1, figsize=(14, 5 * len(aligned_data)), constrained_layout=True)
    if len(aligned_data) == 1:
        axes = [axes]  # Make it iterable if only one subplot
    
    for ax1, (filename, agg_y, agg_min, agg_max, agg_span) in zip(axes, aligned_data):
        # Plot min/max on left axis (blue)
        ax1.plot(agg_y, agg_min, color="tab:blue", linestyle="-", linewidth=2, label="Min height")
        ax1.plot(agg_y, agg_max, color="tab:blue", linestyle="--", linewidth=2, label="Max height")
        ax1.fill_between(agg_y, agg_min, agg_max, color="tab:blue", alpha=0.1)
        
        ax1.set_xlabel("Gantry Y (mm)", fontsize=16)
        ax1.set_ylabel("height (mm)", fontsize=16, color="tab:blue")
        ax1.tick_params(axis="y", labelcolor="tab:blue", labelsize=14)
        ax1.tick_params(axis="x", labelsize=14)
        ax1.grid(True, alpha=0.3)
        ax1.legend(bbox_to_anchor=(0, 2/4), loc="lower left", fontsize=14)
        label = filename.replace("_", " ").replace(".csv", "")
        ax1.set_title(label, fontsize=12, fontweight="bold")

        # Create secondary y-axis for Z-span (orange)
        ax2 = ax1.twinx()
        # Filter out Z-span values > 5
        agg_span_filtered = np.where(agg_span > 5, np.nan, agg_span)
        ax2.plot(agg_y, agg_span_filtered, color="tab:orange", linestyle="-", linewidth=2.5, label="height span")
        ax2.set_ylabel("height span (mm)", fontsize=16, color="tab:orange")
        ax2.tick_params(axis="y", labelcolor="tab:orange", labelsize=14)
        ax2.legend(loc="upper right", fontsize=14)

    if args.save_plot:
        plt.savefig(args.save_plot, dpi=150, bbox_inches="tight")
        print(f"Plot saved to {args.save_plot}")

    plt.show()


if __name__ == "__main__":
    main()
