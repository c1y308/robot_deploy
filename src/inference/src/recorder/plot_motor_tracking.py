#!/usr/bin/env python3
"""Plot motor target/rx tracking from inference CSV logs."""

from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
from dataclasses import dataclass
from pathlib import Path
from xml.sax.saxutils import escape


EXPECTED_COMMAND_INTERVAL_MS = 20.0
COMMAND_INTERVAL_WARN_TOLERANCE_MS = 2.0
EXCLUDED_MOTOR_IDS = {4, 5, 10, 11}
OVERLAY_Y_LIMIT_DEG = (-45.0, 45.0)
POLICY_TARGET_PERIOD_S = 0.02


@dataclass
class LogData:
    path: Path
    rows: list[dict[str, str]]
    fieldnames: list[str]
    motor_sample_time_column: str


@dataclass
class MotorTrackingSeries:
    motor_id: int
    target_time_s: list[float]
    target_pos_rad: list[float]
    rx_time_s: list[float]
    rx_pos_rad: list[float]
    target_delta_time_s: list[float]
    target_delta_q_rad: list[float]
    target_rate_rad_s: list[float]
    aligned_time_s: list[float]
    aligned_target_rad: list[float]
    aligned_rx_rad: list[float]
    error_deg: list[float]
    skipped_before_first_command: int


@dataclass
class MotorTrackingMetric:
    motor_id: int
    sample_count: int
    skipped_before_first_command: int
    rms_error_deg: float
    mean_error_deg: float
    p95_abs_error_deg: float
    max_abs_error_deg: float
    p95_abs_target_rate_rad_s: float
    max_abs_target_rate_rad_s: float
    p95_abs_delta_q_rad: float
    max_abs_delta_q_rad: float
    max_abs_delta_q_time_s: float


@dataclass
class PlotSeries:
    label: str
    x: list[float]
    y: list[float]
    color: str
    dashed: bool = False


def inference_root() -> Path:
    return Path(__file__).resolve().parents[2]


def default_log_dir() -> Path:
    return inference_root() / "log"


def find_latest_csv(log_dir: Path) -> Path:
    pattern = "[0-9][0-9][0-9][0-9]_[0-9][0-9][0-9][0-9].csv"
    candidates = sorted(log_dir.glob(pattern), key=lambda path: path.stat().st_mtime)
    if not candidates:
        raise FileNotFoundError(f"no MMDD_HHMM.csv files under {log_dir}")
    return candidates[-1]


def load_csv_rows(csv_path: Path) -> tuple[list[dict[str, str]], list[str]]:
    with csv_path.open(newline="") as stream:
        reader = csv.reader(stream)
        try:
            raw_fieldnames = next(reader)
        except StopIteration as exc:
            raise ValueError(f"{csv_path} is empty") from exc

        fieldnames = [field.strip() for field in raw_fieldnames]
        duplicate_names = {
            name for name in fieldnames if fieldnames.count(name) > 1
        }
        if duplicate_names:
            raise ValueError(
                f"{csv_path} has duplicate columns after stripping header whitespace: "
                + ", ".join(sorted(duplicate_names))
            )

        rows: list[dict[str, str]] = []
        malformed_rows = 0
        for line_number, values in enumerate(reader, start=2):
            if not values or all(not value.strip() for value in values):
                continue
            if len(values) != len(fieldnames):
                malformed_rows += 1
                if malformed_rows <= 3:
                    print(
                        f"warning: skipping malformed CSV row {csv_path}:{line_number}; "
                        f"has {len(values)} values, expected {len(fieldnames)}",
                        file=sys.stderr,
                    )
                continue
            rows.append(
                {
                    fieldnames[index]: values[index].strip()
                    for index in range(len(fieldnames))
                }
            )

    if malformed_rows > 3:
        print(
            f"warning: skipped {malformed_rows} malformed CSV rows in {csv_path}",
            file=sys.stderr,
        )

    if not rows:
        raise ValueError(f"{csv_path} has no data rows")
    return rows, fieldnames


def require_column(fieldnames: list[str], column: str) -> None:
    if column not in fieldnames:
        raise ValueError(f"missing required CSV column: {column}")


def choose_motor_sample_time_column(fieldnames: list[str]) -> str:
    if "motor_sample_timestamp_ns" in fieldnames:
        return "motor_sample_timestamp_ns"
    if "state_timestamp_ns" in fieldnames:
        return "state_timestamp_ns"
    raise ValueError("missing required CSV column: motor_sample_timestamp_ns or state_timestamp_ns")


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


def available_tracking_motors(fieldnames: list[str]) -> list[int]:
    target = set(available_motors(fieldnames, "target_pos_rad_M"))
    rx = set(available_motors(fieldnames, "rx_pos_rad_M"))
    return sorted(target & rx)


def filter_plot_motors(motors: list[int]) -> list[int]:
    return [motor_id for motor_id in motors if motor_id not in EXCLUDED_MOTOR_IDS]


def parse_motor_selection(value: str | None, available: list[int]) -> list[int]:
    available = filter_plot_motors(available)
    if not available:
        excluded = ",".join(f"M{motor_id}" for motor_id in sorted(EXCLUDED_MOTOR_IDS))
        raise ValueError(f"no plottable motors after excluding {excluded}")

    if value is None or value.strip().lower() == "all":
        return available

    selected: list[int] = []
    excluded_requested: list[int] = []
    for token in value.split(","):
        token = token.strip()
        if not token:
            continue
        try:
            motor_id = int(token)
        except ValueError as exc:
            raise ValueError(f"invalid motor id: {token}") from exc
        if motor_id in EXCLUDED_MOTOR_IDS:
            excluded_requested.append(motor_id)
            continue
        if motor_id not in available:
            raise ValueError(f"motor {motor_id} not found in target/rx CSV column pairs")
        if motor_id not in selected:
            selected.append(motor_id)

    if excluded_requested:
        print(
            "warning: skipping excluded motors: "
            + ",".join(f"M{motor_id}" for motor_id in sorted(set(excluded_requested))),
            file=sys.stderr,
        )
    if not selected:
        excluded = ",".join(f"M{motor_id}" for motor_id in sorted(EXCLUDED_MOTOR_IDS))
        raise ValueError(f"empty motor selection after excluding {excluded}")
    return selected


def parse_ns(value: str, column: str) -> int:
    try:
        return int(value)
    except ValueError:
        try:
            return int(float(value))
        except ValueError as exc:
            raise ValueError(f"invalid ns timestamp in {column}: {value!r}") from exc


def parse_float(value: str, column: str) -> float:
    try:
        parsed = float(value)
    except ValueError as exc:
        raise ValueError(f"invalid float in {column}: {value!r}") from exc
    if not math.isfinite(parsed):
        raise ValueError(f"non-finite float in {column}: {value!r}")
    return parsed


def extract_ns_series(rows: list[dict[str, str]], column: str) -> list[int]:
    return [parse_ns(row[column], column) for row in rows]


def extract_float_series(rows: list[dict[str, str]], column: str) -> list[float]:
    return [parse_float(row[column], column) for row in rows]


def require_monotonic_non_decreasing(name: str, values: list[int]) -> None:
    for index in range(1, len(values)):
        if values[index] < values[index - 1]:
            raise ValueError(
                f"{name} is not monotonic at row {index + 1}: "
                f"{values[index - 1]} -> {values[index]}"
            )


def load_log_data(csv_path: Path) -> LogData:
    rows, fieldnames = load_csv_rows(csv_path)
    require_column(fieldnames, "command_timestamp_ns")
    motor_sample_time_column = choose_motor_sample_time_column(fieldnames)

    motors = available_tracking_motors(fieldnames)
    if not motors:
        raise ValueError(f"{csv_path} has no target_pos_rad_M*/rx_pos_rad_M* column pairs")

    command_times_ns = extract_ns_series(rows, "command_timestamp_ns")
    motor_sample_times_ns = extract_ns_series(rows, motor_sample_time_column)
    require_monotonic_non_decreasing("command_timestamp_ns", command_times_ns)
    require_monotonic_non_decreasing(motor_sample_time_column, motor_sample_times_ns)

    return LogData(csv_path, rows, fieldnames, motor_sample_time_column)


def zoh_align(
    target_times_ns: list[int],
    target_values: list[float],
    rx_times_ns: list[int],
    rx_values: list[float],
) -> tuple[list[int], list[float], list[float], int]:
    aligned_times_ns: list[int] = []
    aligned_target_values: list[float] = []
    aligned_rx_values: list[float] = []
    skipped = 0
    target_index = 0

    for rx_time_ns, rx_value in zip(rx_times_ns, rx_values):
        while (
            target_index + 1 < len(target_times_ns)
            and target_times_ns[target_index + 1] <= rx_time_ns
        ):
            target_index += 1

        if target_times_ns[target_index] > rx_time_ns:
            skipped += 1
            continue

        aligned_times_ns.append(rx_time_ns)
        aligned_target_values.append(target_values[target_index])
        aligned_rx_values.append(rx_value)

    return aligned_times_ns, aligned_target_values, aligned_rx_values, skipped


def build_tracking_series(log_data: LogData, motors: list[int]) -> list[MotorTrackingSeries]:
    rows = log_data.rows
    command_times_ns = extract_ns_series(rows, "command_timestamp_ns")
    motor_sample_times_ns = extract_ns_series(rows, log_data.motor_sample_time_column)
    time_origin_ns = min(command_times_ns[0], motor_sample_times_ns[0])

    series: list[MotorTrackingSeries] = []
    for motor_id in motors:
        target_col = f"target_pos_rad_M{motor_id}"
        rx_col = f"rx_pos_rad_M{motor_id}"
        require_column(log_data.fieldnames, target_col)
        require_column(log_data.fieldnames, rx_col)

        target_values = extract_float_series(rows, target_col)
        rx_values = extract_float_series(rows, rx_col)
        target_delta_q_rad = [
            target_values[index] - target_values[index - 1]
            for index in range(1, len(target_values))
        ]
        target_rate_rad_s = [
            delta_q / POLICY_TARGET_PERIOD_S
            for delta_q in target_delta_q_rad
        ]
        target_delta_time_s = [
            (command_times_ns[index] - time_origin_ns) / 1_000_000_000.0
            for index in range(1, len(command_times_ns))
        ]
        aligned_times_ns, aligned_targets, aligned_rx, skipped = zoh_align(
            command_times_ns,
            target_values,
            motor_sample_times_ns,
            rx_values,
        )
        errors_deg = [
            math.degrees(rx - target)
            for target, rx in zip(aligned_targets, aligned_rx)
        ]

        series.append(
            MotorTrackingSeries(
                motor_id=motor_id,
                target_time_s=[
                    (time_ns - time_origin_ns) / 1_000_000_000.0
                    for time_ns in command_times_ns
                ],
                target_pos_rad=target_values,
                rx_time_s=[
                    (time_ns - time_origin_ns) / 1_000_000_000.0
                    for time_ns in motor_sample_times_ns
                ],
                rx_pos_rad=rx_values,
                target_delta_time_s=target_delta_time_s,
                target_delta_q_rad=target_delta_q_rad,
                target_rate_rad_s=target_rate_rad_s,
                aligned_time_s=[
                    (time_ns - time_origin_ns) / 1_000_000_000.0
                    for time_ns in aligned_times_ns
                ],
                aligned_target_rad=aligned_targets,
                aligned_rx_rad=aligned_rx,
                error_deg=errors_deg,
                skipped_before_first_command=skipped,
            )
        )

    if not any(item.error_deg for item in series):
        raise ValueError("no rx samples could be aligned to an earlier command timestamp")
    return series


def percentile_nearest_rank(values: list[float], percentile: float) -> float:
    if not values:
        return math.nan
    if percentile < 0.0 or percentile > 100.0:
        raise ValueError(f"invalid percentile: {percentile}")
    ordered = sorted(values)
    index = math.ceil(percentile / 100.0 * len(ordered)) - 1
    index = max(0, min(index, len(ordered) - 1))
    return ordered[index]


def max_abs_with_time(values: list[float], times_s: list[float]) -> tuple[float, float]:
    if not values:
        return math.nan, math.nan
    index = max(range(len(values)), key=lambda item_index: abs(values[item_index]))
    time_s = times_s[index] if index < len(times_s) else math.nan
    return abs(values[index]), time_s


def compute_metrics(series: list[MotorTrackingSeries]) -> list[MotorTrackingMetric]:
    metrics: list[MotorTrackingMetric] = []
    for item in series:
        abs_errors = [abs(error) for error in item.error_deg]
        if abs_errors:
            rms = math.sqrt(
                sum(error * error for error in item.error_deg) / len(item.error_deg)
            )
            mean_error = sum(item.error_deg) / len(item.error_deg)
            p95_abs_error = percentile_nearest_rank(abs_errors, 95.0)
            max_abs = max(abs_errors)
        else:
            rms = math.nan
            mean_error = math.nan
            p95_abs_error = math.nan
            max_abs = math.nan

        abs_target_rates = [abs(value) for value in item.target_rate_rad_s]
        p95_abs_target_rate = percentile_nearest_rank(abs_target_rates, 95.0)
        max_abs_target_rate = max(abs_target_rates) if abs_target_rates else math.nan
        abs_delta_q = [abs(value) for value in item.target_delta_q_rad]
        p95_abs_delta_q = percentile_nearest_rank(abs_delta_q, 95.0)
        max_abs_delta_q, max_abs_delta_q_time_s = max_abs_with_time(
            item.target_delta_q_rad,
            item.target_delta_time_s,
        )
        metrics.append(
            MotorTrackingMetric(
                motor_id=item.motor_id,
                sample_count=len(item.error_deg),
                skipped_before_first_command=item.skipped_before_first_command,
                rms_error_deg=rms,
                mean_error_deg=mean_error,
                p95_abs_error_deg=p95_abs_error,
                max_abs_error_deg=max_abs,
                p95_abs_target_rate_rad_s=p95_abs_target_rate,
                max_abs_target_rate_rad_s=max_abs_target_rate,
                p95_abs_delta_q_rad=p95_abs_delta_q,
                max_abs_delta_q_rad=max_abs_delta_q,
                max_abs_delta_q_time_s=max_abs_delta_q_time_s,
            )
        )
    return metrics


def command_frequency_stats(command_times_ns: list[int]) -> tuple[float, float, float] | None:
    intervals_ms = [
        (command_times_ns[index] - command_times_ns[index - 1]) / 1_000_000.0
        for index in range(1, len(command_times_ns))
        if command_times_ns[index] > command_times_ns[index - 1]
    ]
    if not intervals_ms:
        return None
    return (
        statistics.median(intervals_ms),
        min(intervals_ms),
        max(intervals_ms),
    )


def print_timing_report(log_data: LogData, series: list[MotorTrackingSeries]) -> None:
    command_times_ns = extract_ns_series(log_data.rows, "command_timestamp_ns")
    stats = command_frequency_stats(command_times_ns)
    if stats is None:
        print("command interval: unavailable; fewer than two unique command timestamps")
    else:
        median_ms, min_ms, max_ms = stats
        approx_hz = 1000.0 / median_ms if median_ms > 0 else math.nan
        print(
            "command interval ms: "
            f"median={median_ms:.3f}, min={min_ms:.3f}, max={max_ms:.3f}, "
            f"approx_hz={approx_hz:.2f}"
        )
        if abs(median_ms - EXPECTED_COMMAND_INTERVAL_MS) > COMMAND_INTERVAL_WARN_TOLERANCE_MS:
            print(
                "warning: median command interval differs from the expected 20.0 ms "
                f"by more than {COMMAND_INTERVAL_WARN_TOLERANCE_MS:.1f} ms",
                file=sys.stderr,
            )

    skipped = sum(item.skipped_before_first_command for item in series)
    valid = sum(len(item.error_deg) for item in series)
    print(
        f"time alignment: target=command_timestamp_ns, "
        f"rx={log_data.motor_sample_time_column}, method=ZOH"
    )
    print(f"aligned rx samples: valid={valid}, skipped_before_first_command={skipped}")


def has_matplotlib() -> bool:
    try:
        import matplotlib  # noqa: F401
    except ImportError:
        return False
    return True


def choose_output_format(fmt: str) -> str:
    if fmt == "auto":
        return "png" if has_matplotlib() else "svg"
    if fmt == "png" and not has_matplotlib():
        raise ValueError("--format png requires matplotlib; use --format svg instead")
    return fmt


def step_post_points(
    x_values: list[float],
    y_values: list[float],
    end_x: float,
) -> tuple[list[float], list[float]]:
    if not x_values:
        return [], []
    x_out = [x_values[0]]
    y_out = [y_values[0]]
    for index in range(1, len(x_values)):
        x_out.append(x_values[index])
        y_out.append(y_values[index - 1])
        x_out.append(x_values[index])
        y_out.append(y_values[index])
    if end_x > x_values[-1]:
        x_out.append(end_x)
        y_out.append(y_values[-1])
    return x_out, y_out


def nice_ticks(lo: float, hi: float, count: int) -> list[float]:
    if count <= 1:
        return [lo]
    if lo == hi:
        return [lo]
    step = (hi - lo) / float(count - 1)
    return [lo + step * index for index in range(count)]


def svg_polyline(points: list[tuple[float, float]]) -> str:
    return " ".join(f"{x:.2f},{y:.2f}" for x, y in points)


def plot_lines_svg(
    output_path: Path,
    title: str,
    x_label: str,
    y_label: str,
    series: list[PlotSeries],
    include_zero: bool,
    y_bounds: tuple[float, float] | None = None,
    y_tick_count: int = 7,
    y_ticks: list[float] | None = None,
) -> None:
    plotted_series = [item for item in series if item.x and item.y]
    if not plotted_series:
        raise ValueError(f"no data to plot for {title}")

    x_all = [value for item in plotted_series for value in item.x]
    y_all = [value for item in plotted_series for value in item.y]
    x_min = min(x_all)
    x_max = max(x_all)
    if y_bounds is None:
        y_min = min(y_all)
        y_max = max(y_all)
    else:
        y_min, y_max = y_bounds
        if y_min >= y_max:
            raise ValueError(f"invalid y-axis bounds for {title}: {y_bounds}")
    if x_min == x_max:
        x_min -= 0.5
        x_max += 0.5
    if y_min == y_max:
        y_min -= 0.5
        y_max += 0.5
    if include_zero and y_bounds is None:
        y_min = min(y_min, 0.0)
        y_max = max(y_max, 0.0)

    y_pad = (y_max - y_min) * 0.08
    if y_pad > 0 and y_bounds is None:
        y_min -= y_pad
        y_max += y_pad

    resolved_y_ticks = y_ticks if y_ticks is not None else nice_ticks(y_min, y_max, y_tick_count)

    width = 1300
    height = max(720, 145 + len(plotted_series) * 18)
    if y_ticks is not None:
        height = max(height, 185 + len(resolved_y_ticks) * 14)
    left = 85
    right = 300
    top = 58
    bottom = 75
    plot_width = width - left - right
    plot_height = height - top - bottom

    def sx(value: float) -> float:
        return left + (value - x_min) / (x_max - x_min) * plot_width

    def sy(value: float) -> float:
        return top + (y_max - value) / (y_max - y_min) * plot_height

    lines = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        "<style>text{font-family:Arial,sans-serif;font-size:12px;fill:#222}.title{font-size:18px}.tick{fill:#555}.grid{stroke:#ddd;stroke-width:1}.axis{stroke:#222;stroke-width:1.3}.line{fill:none;stroke-width:1.5}</style>",
        f'<defs><clipPath id="plot_clip"><rect x="{left}" y="{top}" width="{plot_width}" height="{plot_height}"/></clipPath></defs>',
        f'<rect width="{width}" height="{height}" fill="white"/>',
        f'<text class="title" x="{left}" y="30">{escape(title)}</text>',
    ]

    for tick in nice_ticks(x_min, x_max, 7):
        x = sx(tick)
        lines.append(f'<line class="grid" x1="{x:.2f}" y1="{top}" x2="{x:.2f}" y2="{top + plot_height}"/>')
        lines.append(f'<text class="tick" x="{x:.2f}" y="{height - 35}" text-anchor="middle">{tick:.2f}</text>')

    for tick in resolved_y_ticks:
        y = sy(tick)
        lines.append(f'<line class="grid" x1="{left}" y1="{y:.2f}" x2="{left + plot_width}" y2="{y:.2f}"/>')
        lines.append(f'<text class="tick" x="{left - 10}" y="{y + 4:.2f}" text-anchor="end">{tick:.2f}</text>')

    lines.extend(
        [
            f'<line class="axis" x1="{left}" y1="{top + plot_height}" x2="{left + plot_width}" y2="{top + plot_height}"/>',
            f'<line class="axis" x1="{left}" y1="{top}" x2="{left}" y2="{top + plot_height}"/>',
            f'<text x="{left + plot_width / 2:.2f}" y="{height - 10}" text-anchor="middle">{escape(x_label)}</text>',
            f'<text x="22" y="{top + plot_height / 2:.2f}" transform="rotate(-90 22 {top + plot_height / 2:.2f})" text-anchor="middle">{escape(y_label)}</text>',
        ]
    )

    for index, item in enumerate(plotted_series):
        points = [(sx(x), sy(y)) for x, y in zip(item.x, item.y)]
        dash = ' stroke-dasharray="6 4"' if item.dashed else ""
        lines.append(
            f'<polyline class="line" stroke="{item.color}"{dash} '
            f'clip-path="url(#plot_clip)" points="{svg_polyline(points)}"/>'
        )
        legend_y = top + 18 + index * 18
        legend_x = left + plot_width + 24
        lines.append(
            f'<line x1="{legend_x}" y1="{legend_y}" x2="{legend_x + 26}" y2="{legend_y}" '
            f'stroke="{item.color}" stroke-width="2"{dash}/>'
        )
        lines.append(f'<text x="{legend_x + 34}" y="{legend_y + 4}">{escape(item.label)}</text>')

    lines.append("</svg>")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def plot_motor_overlay_svg(
    output_path: Path,
    csv_path: Path,
    item: MotorTrackingSeries,
) -> None:
    end_x = max(item.rx_time_s[-1], item.target_time_s[-1])
    target_pos_deg = [math.degrees(value) for value in item.target_pos_rad]
    rx_pos_deg = [math.degrees(value) for value in item.rx_pos_rad]
    step_x, step_y = step_post_points(item.target_time_s, target_pos_deg, end_x)
    plot_lines_svg(
        output_path,
        f"{csv_path.name} M{item.motor_id} target/rx overlay",
        "time (s)",
        "position (deg)",
        [
            PlotSeries("target", step_x, step_y, "#1f77b4", dashed=True),
            PlotSeries("rx", item.rx_time_s, rx_pos_deg, "#d62728"),
        ],
        include_zero=True,
        y_bounds=OVERLAY_Y_LIMIT_DEG,
        y_ticks=[float(deg) for deg in range(-45, 46)],
    )


def format_metric(value: float) -> str:
    if math.isnan(value):
        return "n/a"
    return f"{value:.4f}"


def plot_summary_svg(
    output_path: Path,
    csv_path: Path,
    metrics: list[MotorTrackingMetric],
) -> None:
    row_height = 30
    width = 1780
    height = 118 + row_height * (len(metrics) + 1)
    left = 28
    top = 70
    columns = [
        ("motor", 70),
        ("samples", 88),
        ("skipped", 82),
        ("RMSE(deg)", 100),
        ("mean error(deg)", 140),
        ("P95(|e|)(deg)", 128),
        ("max |e|(deg)", 118),
        ("P95(|qdot_target|)(rad/s)", 218),
        ("max |qdot_target|(rad/s)", 218),
        ("P95(|delta q|)(rad)", 172),
        ("max |delta q|(rad)", 162),
        ("max |delta q| time(s)", 176),
    ]

    lines = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        "<style>text{font-family:Arial,sans-serif;font-size:12px;fill:#222}.title{font-size:18px}.note{fill:#555}.head{font-weight:bold}.grid{stroke:#ddd;stroke-width:1}</style>",
        f'<rect width="{width}" height="{height}" fill="white"/>',
        f'<text class="title" x="{left}" y="32">{escape(csv_path.name)} motor tracking summary</text>',
        f'<text class="note" x="{left}" y="52">tracking error e = rx - aligned target; target rate uses fixed T={POLICY_TARGET_PERIOD_S:.2f}s</text>',
    ]

    x = left
    for label, width_px in columns:
        lines.append(f'<text class="head" x="{x + 8}" y="{top}">{escape(label)}</text>')
        lines.append(f'<line class="grid" x1="{x}" y1="{top - 20}" x2="{x}" y2="{height - 32}"/>')
        x += width_px
    lines.append(f'<line class="grid" x1="{x}" y1="{top - 20}" x2="{x}" y2="{height - 32}"/>')
    lines.append(f'<line class="grid" x1="{left}" y1="{top + 8}" x2="{x}" y2="{top + 8}"/>')

    for row_index, metric in enumerate(metrics):
        y = top + row_height * (row_index + 1)
        values = [
            f"M{metric.motor_id}",
            str(metric.sample_count),
            str(metric.skipped_before_first_command),
            format_metric(metric.rms_error_deg),
            format_metric(metric.mean_error_deg),
            format_metric(metric.p95_abs_error_deg),
            format_metric(metric.max_abs_error_deg),
            format_metric(metric.p95_abs_target_rate_rad_s),
            format_metric(metric.max_abs_target_rate_rad_s),
            format_metric(metric.p95_abs_delta_q_rad),
            format_metric(metric.max_abs_delta_q_rad),
            format_metric(metric.max_abs_delta_q_time_s),
        ]
        x = left
        for value, (_, width_px) in zip(values, columns):
            lines.append(f'<text x="{x + 8}" y="{y}">{escape(value)}</text>')
            x += width_px
        lines.append(f'<line class="grid" x1="{left}" y1="{y + 8}" x2="{x}" y2="{y + 8}"/>')

    lines.append("</svg>")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def plot_motor_summary_svg(
    output_path: Path,
    csv_path: Path,
    metric: MotorTrackingMetric,
) -> None:
    plot_summary_svg(output_path, csv_path, [metric])


def plot_motor_overlay_matplotlib(
    output_path: Path,
    csv_path: Path,
    item: MotorTrackingSeries,
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(12, 14))
    target_pos_deg = [math.degrees(value) for value in item.target_pos_rad]
    rx_pos_deg = [math.degrees(value) for value in item.rx_pos_rad]
    ax.step(item.target_time_s, target_pos_deg, where="post", label="target", linewidth=1.4)
    ax.plot(item.rx_time_s, rx_pos_deg, label="rx", linewidth=1.0)
    ax.set_title(f"{csv_path.name} M{item.motor_id} motor target/rx overlay")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("position (deg)")
    ax.set_ylim(*OVERLAY_Y_LIMIT_DEG)
    ax.set_yticks([float(deg) for deg in range(-45, 46)])
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=9, loc="upper right")
    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=150)
    plt.close(fig)


def plot_summary_matplotlib(
    output_path: Path,
    csv_path: Path,
    metrics: list[MotorTrackingMetric],
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(18, max(3.0, 0.38 * len(metrics) + 1.7)))
    ax.axis("off")
    table_rows = [
        [
            f"M{metric.motor_id}",
            str(metric.sample_count),
            str(metric.skipped_before_first_command),
            format_metric(metric.rms_error_deg),
            format_metric(metric.mean_error_deg),
            format_metric(metric.p95_abs_error_deg),
            format_metric(metric.max_abs_error_deg),
            format_metric(metric.p95_abs_target_rate_rad_s),
            format_metric(metric.max_abs_target_rate_rad_s),
            format_metric(metric.p95_abs_delta_q_rad),
            format_metric(metric.max_abs_delta_q_rad),
            format_metric(metric.max_abs_delta_q_time_s),
        ]
        for metric in metrics
    ]
    table = ax.table(
        cellText=table_rows,
        colLabels=[
            "motor",
            "samples",
            "skipped",
            "RMSE(deg)",
            "mean error(deg)",
            "P95(|e|)(deg)",
            "max |e|(deg)",
            "P95(|qdot_target|)(rad/s)",
            "max |qdot_target|(rad/s)",
            "P95(|delta q|)(rad)",
            "max |delta q|(rad)",
            "max |delta q| time(s)",
        ],
        loc="center",
        cellLoc="center",
    )
    table.auto_set_font_size(False)
    table.set_fontsize(8)
    table.scale(1.0, 1.35)
    ax.set_title(
        f"{csv_path.name} motor tracking summary; error = rx - aligned target; "
        f"target rate T={POLICY_TARGET_PERIOD_S:.2f}s",
        pad=12,
    )
    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=150)
    plt.close(fig)


def plot_motor_summary_matplotlib(
    output_path: Path,
    csv_path: Path,
    metric: MotorTrackingMetric,
) -> None:
    plot_summary_matplotlib(output_path, csv_path, [metric])


def default_output_dir(csv_path: Path) -> Path:
    return default_log_dir() / csv_path.stem


def resolve_output_dir(csv_path: Path, output_dir_arg: Path | None) -> Path:
    if output_dir_arg is None:
        return default_output_dir(csv_path).resolve()
    return output_dir_arg.resolve() / csv_path.stem


def write_plots(
    csv_path: Path,
    output_dir: Path,
    series: list[MotorTrackingSeries],
    metrics: list[MotorTrackingMetric],
    output_format: str,
    abs_error: bool,
) -> list[Path]:
    suffix = f".{output_format}"
    metric_by_motor = {metric.motor_id: metric for metric in metrics}
    saved: list[Path] = []

    for item in series:
        motor_prefix = f"M{item.motor_id}"
        overlay_path = output_dir / f"{motor_prefix}_motor_tracking_overlay{suffix}"
        summary_path = output_dir / f"{motor_prefix}_motor_tracking_summary{suffix}"
        metric = metric_by_motor[item.motor_id]

        if output_format == "png":
            plot_motor_overlay_matplotlib(overlay_path, csv_path, item)
            plot_motor_summary_matplotlib(summary_path, csv_path, metric)
        else:
            plot_motor_overlay_svg(overlay_path, csv_path, item)
            plot_motor_summary_svg(summary_path, csv_path, metric)

        saved.extend([overlay_path, summary_path])
    return saved


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plot motor target/rx tracking from inference CSV logs."
    )
    parser.add_argument(
        "csv",
        nargs="?",
        type=Path,
        help="CSV file to plot. Defaults to the latest MMDD_HHMM.csv in src/inference/log.",
    )
    parser.add_argument(
        "-m",
        "--motors",
        help="Comma-separated motor IDs, or all. Defaults to all available motors.",
    )
    parser.add_argument(
        "-o",
        "--output-dir",
        type=Path,
        help=(
            "Base directory for generated images. A CSV-stem subdirectory is "
            "always created. Defaults to src/inference/log."
        ),
    )
    parser.add_argument(
        "--format",
        choices=("auto", "png", "svg"),
        default="auto",
        help="Image format. auto writes PNG when matplotlib is installed, otherwise SVG.",
    )
    parser.add_argument(
        "--abs-error",
        action="store_true",
        help="Accepted for compatibility; separate error plots are no longer generated.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    csv_path = args.csv or find_latest_csv(default_log_dir())
    csv_path = csv_path.resolve()
    output_dir = resolve_output_dir(csv_path, args.output_dir)
    output_format = choose_output_format(args.format)

    log_data = load_log_data(csv_path)
    motors = parse_motor_selection(args.motors, available_tracking_motors(log_data.fieldnames))
    series = build_tracking_series(log_data, motors)
    metrics = compute_metrics(series)
    print_timing_report(log_data, series)

    for path in write_plots(
        csv_path,
        output_dir,
        series,
        metrics,
        output_format,
        args.abs_error,
    ):
        print(f"saved {path}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(1)
