#!/usr/bin/env python3
"""Plot images from unified inference CSV logs."""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path
from xml.sax.saxutils import escape


Series = list[tuple[str, list[float]]]


def default_log_dir() -> Path:
    return Path(__file__).resolve().parents[1] / "log"


def find_latest_csv(log_dir: Path) -> Path:
    pattern = "[0-9][0-9][0-9][0-9]_[0-9][0-9][0-9][0-9].csv"
    candidates = sorted(log_dir.glob(pattern), key=lambda path: path.stat().st_mtime)
    if not candidates:
        raise FileNotFoundError(f"no MMDD_HHMM.csv files under {log_dir}")
    return candidates[-1]


def load_rows(csv_path: Path) -> tuple[list[dict[str, str]], list[str]]:
    with csv_path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        rows = list(reader)
        fieldnames = reader.fieldnames or []

    if not rows:
        raise ValueError(f"{csv_path} has no data rows")
    return rows, fieldnames


def available_motors(fieldnames: list[str], prefix: str) -> list[int]:
    motors: list[int] = []
    for name in fieldnames:
        if not name.startswith(prefix):
            continue
        try:
            motors.append(int(name[len(prefix):]))
        except ValueError:
            continue
    return sorted(motors)


def available_position_error_motors(fieldnames: list[str]) -> list[int]:
    target = set(available_motors(fieldnames, "target_pos_rad_M"))
    rx = set(available_motors(fieldnames, "rx_pos_rad_M"))
    return sorted(target & rx)


def parse_motor_selection(value: str | None, available: list[int]) -> list[int]:
    if value is None or value.strip().lower() == "all":
        return available

    selected: list[int] = []
    for token in value.split(","):
        token = token.strip()
        if not token:
            continue
        try:
            motor_id = int(token)
        except ValueError as exc:
            raise ValueError(f"invalid motor id: {token}") from exc
        if motor_id not in available:
            raise ValueError(f"motor {motor_id} not found in CSV")
        if motor_id not in selected:
            selected.append(motor_id)

    if not selected:
        raise ValueError("empty motor selection")
    return selected


def x_axis(rows: list[dict[str, str]]) -> tuple[list[float], str]:
    if "elapsed_us" in rows[0]:
        return [float(row["elapsed_us"]) / 1_000_000.0 for row in rows], "time (s)"
    return [float(row["frame_index"]) for row in rows], "frame"


def position_error_series(
    rows: list[dict[str, str]], motors: list[int], use_abs: bool
) -> Series:
    series: Series = []
    for motor_id in motors:
        target_col = f"target_pos_rad_M{motor_id}"
        rx_col = f"rx_pos_rad_M{motor_id}"
        values = [
            math.degrees(float(row[target_col]) - float(row[rx_col]))
            for row in rows
        ]
        if use_abs:
            values = [abs(value) for value in values]
        series.append((f"M{motor_id}", values))
    return series


def torque_series(rows: list[dict[str, str]], motors: list[int]) -> Series:
    series: Series = []
    for motor_id in motors:
        column = f"torque_pct_M{motor_id}"
        series.append((f"M{motor_id}", [float(row[column]) for row in rows]))
    return series


def nice_ticks(lo: float, hi: float, count: int) -> list[float]:
    if count <= 1:
        return [lo]
    if lo == hi:
        return [lo]
    step = (hi - lo) / float(count - 1)
    return [lo + step * i for i in range(count)]


def svg_polyline(points: list[tuple[float, float]]) -> str:
    return " ".join(f"{x:.2f},{y:.2f}" for x, y in points)


def plot_svg(
    csv_path: Path,
    output_path: Path,
    title: str,
    y_label: str,
    x_values: list[float],
    x_label: str,
    series: Series,
    include_zero: bool,
) -> None:
    width = 1200
    height = 650
    left = 80
    right = 190
    top = 55
    bottom = 75
    plot_width = width - left - right
    plot_height = height - top - bottom

    y_all = [value for _, values in series for value in values]
    x_min = min(x_values)
    x_max = max(x_values)
    y_min = min(y_all)
    y_max = max(y_all)
    if x_min == x_max:
        x_min -= 0.5
        x_max += 0.5
    if y_min == y_max:
        y_min -= 0.5
        y_max += 0.5
    if include_zero:
        y_min = min(y_min, 0.0)
        y_max = max(y_max, 0.0)

    y_pad = (y_max - y_min) * 0.08
    if y_pad > 0:
        y_min -= y_pad
        y_max += y_pad

    def sx(value: float) -> float:
        return left + (value - x_min) / (x_max - x_min) * plot_width

    def sy(value: float) -> float:
        return top + (y_max - value) / (y_max - y_min) * plot_height

    colors = [
        "#1f77b4", "#d62728", "#2ca02c", "#9467bd",
        "#ff7f0e", "#17becf", "#8c564b", "#e377c2",
        "#7f7f7f", "#bcbd22", "#004c6d", "#8f2d56",
    ]

    lines = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        "<style>text{font-family:Arial,sans-serif;font-size:13px;fill:#222}.tick{fill:#555}.grid{stroke:#ddd;stroke-width:1}.axis{stroke:#222;stroke-width:1.3}.line{fill:none;stroke-width:1.8}</style>",
        f'<rect width="{width}" height="{height}" fill="white"/>',
        f'<text x="{left}" y="28" font-size="18">{escape(csv_path.name)} - {escape(title)}</text>',
    ]

    for tick in nice_ticks(x_min, x_max, 7):
        x = sx(tick)
        lines.append(f'<line class="grid" x1="{x:.2f}" y1="{top}" x2="{x:.2f}" y2="{top + plot_height}"/>')
        lines.append(f'<text class="tick" x="{x:.2f}" y="{height - 35}" text-anchor="middle">{tick:.2f}</text>')

    for tick in nice_ticks(y_min, y_max, 7):
        y = sy(tick)
        lines.append(f'<line class="grid" x1="{left}" y1="{y:.2f}" x2="{left + plot_width}" y2="{y:.2f}"/>')
        lines.append(f'<text class="tick" x="{left - 10}" y="{y + 4:.2f}" text-anchor="end">{tick:.2f}</text>')

    lines.extend(
        [
            f'<line class="axis" x1="{left}" y1="{top + plot_height}" x2="{left + plot_width}" y2="{top + plot_height}"/>',
            f'<line class="axis" x1="{left}" y1="{top}" x2="{left}" y2="{top + plot_height}"/>',
            f'<text x="{left + plot_width / 2:.2f}" y="{height - 10}" text-anchor="middle">{escape(x_label)}</text>',
            f'<text x="20" y="{top + plot_height / 2:.2f}" transform="rotate(-90 20 {top + plot_height / 2:.2f})" text-anchor="middle">{escape(y_label)}</text>',
        ]
    )

    for idx, (label, values) in enumerate(series):
        color = colors[idx % len(colors)]
        points = [(sx(x), sy(y)) for x, y in zip(x_values, values)]
        lines.append(f'<polyline class="line" stroke="{color}" points="{svg_polyline(points)}"/>')
        legend_y = top + 22 + idx * 22
        legend_x = left + plot_width + 25
        lines.append(f'<line x1="{legend_x}" y1="{legend_y}" x2="{legend_x + 28}" y2="{legend_y}" stroke="{color}" stroke-width="2"/>')
        lines.append(f'<text x="{legend_x + 36}" y="{legend_y + 4}">{escape(label)}</text>')

    lines.append("</svg>")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def plot_matplotlib(
    output_path: Path,
    title: str,
    y_label: str,
    x_values: list[float],
    x_label: str,
    series: Series,
    show: bool,
) -> None:
    import matplotlib

    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(12, 6))
    for label, values in series:
        ax.plot(x_values, values, linewidth=1.2, label=label)
    ax.set_title(title)
    ax.set_xlabel(x_label)
    ax.set_ylabel(y_label)
    ax.grid(True, alpha=0.3)
    ax.legend(ncol=4, fontsize=8)
    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=150)
    if show:
        plt.show()


def has_matplotlib() -> bool:
    try:
        import matplotlib  # noqa: F401
    except ImportError:
        return False
    return True


def write_plot(
    csv_path: Path,
    output_dir: Path,
    stem_suffix: str,
    title: str,
    y_label: str,
    x_values: list[float],
    x_label: str,
    series: Series,
    fmt: str,
    show: bool,
    include_zero: bool,
) -> Path:
    use_matplotlib = fmt == "png" or (fmt == "auto" and has_matplotlib())
    suffix = ".png" if use_matplotlib else ".svg"
    output_path = output_dir / f"{csv_path.stem}_{stem_suffix}{suffix}"
    if use_matplotlib:
        plot_matplotlib(output_path, title, y_label, x_values, x_label, series, show)
    else:
        if show:
            print("--show is ignored for SVG output", file=sys.stderr)
        plot_svg(csv_path, output_path, title, y_label, x_values, x_label, series, include_zero)
    return output_path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plot images from src/inference/log unified inference CSV logs."
    )
    parser.add_argument(
        "csv",
        nargs="?",
        type=Path,
        help="CSV file to plot. Defaults to the latest MMDD_HHMM.csv in src/inference/log.",
    )
    parser.add_argument(
        "-p",
        "--plot",
        choices=("all", "pos-error", "torque"),
        default="all",
        help="Which plot to generate. Defaults to all.",
    )
    parser.add_argument(
        "-m",
        "--motors",
        help="Comma-separated motor IDs, for example 0,1,5. Defaults to all available motors.",
    )
    parser.add_argument(
        "-o",
        "--output-dir",
        type=Path,
        help="Directory for generated images. Defaults to the CSV directory.",
    )
    parser.add_argument(
        "--format",
        choices=("auto", "png", "svg"),
        default="auto",
        help="Image format. auto writes PNG when matplotlib is installed, otherwise SVG.",
    )
    parser.add_argument(
        "--abs",
        action="store_true",
        help="For pos-error, plot absolute error instead of signed target-rx error.",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Open matplotlib windows after saving PNG output.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    csv_path = args.csv or find_latest_csv(default_log_dir())
    csv_path = csv_path.resolve()
    output_dir = (args.output_dir or csv_path.parent).resolve()

    rows, fieldnames = load_rows(csv_path)
    x_values, x_label = x_axis(rows)
    saved: list[Path] = []

    if args.plot in ("all", "pos-error"):
        available = available_position_error_motors(fieldnames)
        if not available:
            raise ValueError(f"{csv_path} has no target_pos_rad_M*/rx_pos_rad_M* column pairs")
        motors = parse_motor_selection(args.motors, available)
        title = "abs target-rx error" if args.abs else "target-rx error"
        saved.append(
            write_plot(
                csv_path,
                output_dir,
                "pos_error",
                title,
                "error (deg)",
                x_values,
                x_label,
                position_error_series(rows, motors, args.abs),
                args.format,
                args.show,
                include_zero=True,
            )
        )

    if args.plot in ("all", "torque"):
        available = available_motors(fieldnames, "torque_pct_M")
        if not available:
            raise ValueError(f"{csv_path} has no torque_pct_M* columns")
        motors = parse_motor_selection(args.motors, available)
        saved.append(
            write_plot(
                csv_path,
                output_dir,
                "torque",
                "torque_pct",
                "torque (%)",
                x_values,
                x_label,
                torque_series(rows, motors),
                args.format,
                args.show,
                include_zero=False,
            )
        )

    for path in saved:
        print(f"saved {path}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(1)
