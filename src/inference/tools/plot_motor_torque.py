#!/usr/bin/env python3
"""Plot RX_TQ_PCT curves from motor_torque CSV logs."""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path
from xml.sax.saxutils import escape


def default_log_dir() -> Path:
    return Path(__file__).resolve().parents[1] / "log" / "motor_torque"


def find_latest_csv(log_dir: Path) -> Path:
    candidates = sorted(
        log_dir.glob("motor_torque_*.csv"),
        key=lambda path: path.stat().st_mtime,
    )
    if not candidates:
        raise FileNotFoundError(f"no motor_torque_*.csv files under {log_dir}")
    return candidates[-1]


def parse_motor_selection(
    value: str | None, available: list[int]
) -> list[int]:
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


def load_rows(csv_path: Path) -> tuple[list[dict[str, str]], list[int]]:
    with csv_path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        rows = list(reader)
        fieldnames = reader.fieldnames or []

    if not rows:
        raise ValueError(f"{csv_path} has no data rows")

    motors: list[int] = []
    prefix = "M"
    suffix = "_tq_pct"
    for name in fieldnames:
        if name.startswith(prefix) and name.endswith(suffix):
            try:
                motors.append(int(name[len(prefix):-len(suffix)]))
            except ValueError:
                continue
    motors.sort()
    if not motors:
        raise ValueError(f"{csv_path} has no M*_tq_pct columns")
    return rows, motors


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plot RX_TQ_PCT curves from src/inference/log/motor_torque CSV logs."
    )
    parser.add_argument(
        "csv",
        nargs="?",
        type=Path,
        help="CSV file to plot. Defaults to the latest motor_torque_*.csv.",
    )
    parser.add_argument(
        "-m",
        "--motors",
        help="Comma-separated motor IDs to plot, for example 1,3,7,9. Defaults to all.",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        help="Image output path. Defaults to .png with matplotlib, otherwise .svg.",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Open an interactive matplotlib window after saving.",
    )
    return parser.parse_args()


def build_plot_data(
    rows: list[dict[str, str]], selected_motors: list[int]
) -> tuple[list[float], str, list[tuple[str, list[float]]]]:
    if "elapsed_us" in rows[0]:
        x_values = [float(row["elapsed_us"]) / 1_000_000.0 for row in rows]
        x_label = "time (s)"
    else:
        x_values = [float(row["frame_index"]) for row in rows]
        x_label = "frame"

    series: list[tuple[str, list[float]]] = []
    for motor_id in selected_motors:
        column = f"M{motor_id}_tq_pct"
        y_values = [float(row[column]) for row in rows]
        series.append((f"M{motor_id}", y_values))

    return x_values, x_label, series


def plot_with_matplotlib(
    csv_path: Path,
    output_path: Path,
    x_values: list[float],
    x_label: str,
    series: list[tuple[str, list[float]]],
    show: bool,
) -> None:
    import matplotlib

    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(12, 6))
    for label, y_values in series:
        ax.plot(x_values, y_values, linewidth=1.2, label=label)

    ax.set_title(f"{csv_path.name} - RX_TQ_PCT")
    ax.set_xlabel(x_label)
    ax.set_ylabel("torque (%)")
    ax.grid(True, alpha=0.3)
    ax.legend(ncol=4, fontsize=8)
    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=150)

    if show:
        plt.show()


def nice_ticks(lo: float, hi: float, count: int) -> list[float]:
    if count <= 1:
        return [lo]
    if lo == hi:
        return [lo]
    step = (hi - lo) / float(count - 1)
    return [lo + step * i for i in range(count)]


def svg_polyline(points: list[tuple[float, float]]) -> str:
    return " ".join(f"{x:.2f},{y:.2f}" for x, y in points)


def plot_with_svg(
    csv_path: Path,
    output_path: Path,
    x_values: list[float],
    x_label: str,
    series: list[tuple[str, list[float]]],
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

    lines: list[str] = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        "<style>text{font-family:Arial,sans-serif;font-size:13px;fill:#222}.tick{fill:#555}.grid{stroke:#ddd;stroke-width:1}.axis{stroke:#222;stroke-width:1.3}.line{fill:none;stroke-width:1.8}</style>",
        f'<rect width="{width}" height="{height}" fill="white"/>',
        f'<text x="{left}" y="28" font-size="18">{escape(csv_path.name)} - RX_TQ_PCT</text>',
    ]

    for tick in nice_ticks(x_min, x_max, 7):
        x = sx(tick)
        lines.append(
            f'<line class="grid" x1="{x:.2f}" y1="{top}" x2="{x:.2f}" y2="{top + plot_height}"/>'
        )
        lines.append(
            f'<text class="tick" x="{x:.2f}" y="{height - 35}" text-anchor="middle">{tick:.2f}</text>'
        )

    for tick in nice_ticks(y_min, y_max, 7):
        y = sy(tick)
        lines.append(
            f'<line class="grid" x1="{left}" y1="{y:.2f}" x2="{left + plot_width}" y2="{y:.2f}"/>'
        )
        lines.append(
            f'<text class="tick" x="{left - 10}" y="{y + 4:.2f}" text-anchor="end">{tick:.2f}</text>'
        )

    lines.extend(
        [
            f'<line class="axis" x1="{left}" y1="{top + plot_height}" x2="{left + plot_width}" y2="{top + plot_height}"/>',
            f'<line class="axis" x1="{left}" y1="{top}" x2="{left}" y2="{top + plot_height}"/>',
            f'<text x="{left + plot_width / 2:.2f}" y="{height - 10}" text-anchor="middle">{escape(x_label)}</text>',
            f'<text x="20" y="{top + plot_height / 2:.2f}" transform="rotate(-90 20 {top + plot_height / 2:.2f})" text-anchor="middle">torque (%)</text>',
        ]
    )

    for idx, (label, y_values) in enumerate(series):
        color = colors[idx % len(colors)]
        points = [(sx(x), sy(y)) for x, y in zip(x_values, y_values)]
        lines.append(
            f'<polyline class="line" stroke="{color}" points="{svg_polyline(points)}"/>'
        )

        legend_y = top + 22 + idx * 22
        legend_x = left + plot_width + 25
        lines.append(
            f'<line x1="{legend_x}" y1="{legend_y}" x2="{legend_x + 28}" y2="{legend_y}" stroke="{color}" stroke-width="2"/>'
        )
        lines.append(
            f'<text x="{legend_x + 36}" y="{legend_y + 4}">{escape(label)}</text>'
        )

    lines.append("</svg>")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    csv_path = args.csv or find_latest_csv(default_log_dir())
    csv_path = csv_path.resolve()

    rows, available_motors = load_rows(csv_path)
    selected_motors = parse_motor_selection(args.motors, available_motors)
    x_values, x_label, series = build_plot_data(rows, selected_motors)

    has_matplotlib = True
    try:
        import matplotlib  # noqa: F401
    except ImportError:
        has_matplotlib = False

    if args.output:
        output_path = args.output.resolve()
    else:
        output_path = csv_path.with_suffix(
            ".png" if has_matplotlib else ".svg"
        ).resolve()

    if has_matplotlib:
        plot_with_matplotlib(
            csv_path, output_path, x_values, x_label, series, args.show
        )
    else:
        if output_path.suffix.lower() != ".svg":
            fallback_path = output_path.with_suffix(".svg")
            print(
                f"matplotlib is not installed; writing SVG instead: {fallback_path}",
                file=sys.stderr,
            )
            output_path = fallback_path
        if args.show:
            print(
                "matplotlib is not installed; --show is ignored for SVG output",
                file=sys.stderr,
            )
        plot_with_svg(csv_path, output_path, x_values, x_label, series)

    print(f"saved {output_path}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(1)
