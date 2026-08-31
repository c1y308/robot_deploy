#!/usr/bin/env python3
"""Tests for plot_motor_tracking.py."""

from __future__ import annotations

import importlib.util
import math
import sys
import tempfile
from pathlib import Path


MODULE_PATH = Path(__file__).with_name("plot_motor_tracking.py")
spec = importlib.util.spec_from_file_location("plot_motor_tracking", MODULE_PATH)
assert spec is not None
plot_motor_tracking = importlib.util.module_from_spec(spec)
assert spec.loader is not None
sys.modules[spec.name] = plot_motor_tracking
spec.loader.exec_module(plot_motor_tracking)


def expect(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def expect_near(actual: float, expected: float, message: str) -> None:
    if abs(actual - expected) > 1e-9:
        raise AssertionError(f"{message}: expected {expected}, got {actual}")


def write_csv(path: Path, header: str, rows: list[str]) -> None:
    path.write_text(header + "\n" + "\n".join(rows) + "\n", encoding="utf-8")


def test_zoh_alignment() -> None:
    with tempfile.TemporaryDirectory() as temp_dir:
        csv_path = Path(temp_dir) / "zoh.csv"
        write_csv(
            csv_path,
            "command_timestamp_ns,motor_sample_timestamp_ns,target_pos_rad_M0,rx_pos_rad_M0",
            [
                "0,10000000,1.0,0.5",
                "20000000,30000000,2.0,1.0",
                "40000000,50000000,4.0,3.5",
            ],
        )

        log_data = plot_motor_tracking.load_log_data(csv_path)
        series = plot_motor_tracking.build_tracking_series(log_data, [0])[0]

        expect(series.aligned_target_rad == [1.0, 2.0, 4.0], "ZOH target mismatch")
        expect(series.aligned_rx_rad == [0.5, 1.0, 3.5], "aligned rx mismatch")
        expect(series.skipped_before_first_command == 0, "unexpected skipped samples")
        expect_near(series.error_deg[0], math.degrees(-0.5), "error 0")
        expect_near(series.error_deg[1], math.degrees(-1.0), "error 1")
        expect_near(series.error_deg[2], math.degrees(-0.5), "error 2")


def test_old_header_and_state_timestamp_fallback() -> None:
    with tempfile.TemporaryDirectory() as temp_dir:
        csv_path = Path(temp_dir) / "old_header.csv"
        write_csv(
            csv_path,
            "frame_index, command_timestamp_ns, state_timestamp_ns, target_pos_rad_M0, rx_pos_rad_M0",
            [
                "0, 1000000, 500000, 1.0, 0.1",
                "1, 2000000, 1500000, 2.0, 0.2",
            ],
        )

        log_data = plot_motor_tracking.load_log_data(csv_path)
        expect(
            log_data.motor_sample_time_column == "state_timestamp_ns",
            "old state timestamp fallback was not selected",
        )
        series = plot_motor_tracking.build_tracking_series(log_data, [0])[0]

        expect(series.skipped_before_first_command == 1, "first rx sample should be skipped")
        expect(series.aligned_target_rad == [1.0], "fallback ZOH target mismatch")
        expect(series.aligned_rx_rad == [0.2], "fallback rx mismatch")


def test_excluded_motor_selection() -> None:
    available = [0, 1, 4, 5, 10, 11]
    expect(
        plot_motor_tracking.parse_motor_selection(None, available) == [0, 1],
        "default motor selection should exclude ankle parallel motors",
    )
    expect(
        plot_motor_tracking.parse_motor_selection("4,0,11", available) == [0],
        "explicit motor selection should skip excluded motors",
    )

    try:
        plot_motor_tracking.parse_motor_selection("4,5,10,11", available)
    except ValueError as exc:
        expect("after excluding" in str(exc), "unexpected excluded-only error")
    else:
        raise AssertionError("excluded-only selection should fail")


def test_target_rate_and_delta_metrics() -> None:
    with tempfile.TemporaryDirectory() as temp_dir:
        csv_path = Path(temp_dir) / "target_rate.csv"
        write_csv(
            csv_path,
            "command_timestamp_ns,motor_sample_timestamp_ns,target_pos_rad_M0,rx_pos_rad_M0",
            [
                "0,10000000,0.0,0.0",
                "20000000,30000000,0.2,0.2",
                "40000000,50000000,0.1,0.1",
            ],
        )

        log_data = plot_motor_tracking.load_log_data(csv_path)
        series = plot_motor_tracking.build_tracking_series(log_data, [0])[0]
        metrics = plot_motor_tracking.compute_metrics([series])[0]

        expect(series.target_delta_q_rad == [0.2, -0.1], "target delta q mismatch")
        expect(series.target_rate_rad_s == [10.0, -5.0], "target rate mismatch")
        expect_near(metrics.p95_abs_target_rate_rad_s, 10.0, "P95 target rate")
        expect_near(metrics.max_abs_target_rate_rad_s, 10.0, "max target rate")
        expect_near(metrics.p95_abs_delta_q_rad, 0.2, "P95 delta q")
        expect_near(metrics.max_abs_delta_q_rad, 0.2, "max delta q")
        expect_near(metrics.max_abs_delta_q_time_s, 0.02, "max delta q time")


def make_test_series(motor_id: int) -> object:
    return plot_motor_tracking.MotorTrackingSeries(
        motor_id=motor_id,
        target_time_s=[0.0, 0.02],
        target_pos_rad=[0.1, 0.2],
        rx_time_s=[0.01, 0.03],
        rx_pos_rad=[0.08, 0.18],
        target_delta_time_s=[0.02],
        target_delta_q_rad=[0.1],
        target_rate_rad_s=[5.0],
        aligned_time_s=[0.01, 0.03],
        aligned_target_rad=[0.1, 0.2],
        aligned_rx_rad=[0.08, 0.18],
        error_deg=[math.degrees(-0.02), math.degrees(-0.02)],
        skipped_before_first_command=0,
    )


def test_per_motor_svg_outputs_and_output_dir() -> None:
    with tempfile.TemporaryDirectory() as temp_dir:
        root = Path(temp_dir)
        csv_path = root / "0831_1757.csv"
        output_dir = plot_motor_tracking.resolve_output_dir(csv_path, root)
        expect(output_dir == root / "0831_1757", "CSV-stem output directory mismatch")

        series = [make_test_series(0), make_test_series(1)]
        metrics = plot_motor_tracking.compute_metrics(series)
        saved = plot_motor_tracking.write_plots(
            csv_path,
            output_dir,
            series,
            metrics,
            "svg",
            abs_error=False,
        )

        expected_names = {
            "M0_motor_tracking_overlay.svg",
            "M0_motor_tracking_summary.svg",
            "M1_motor_tracking_overlay.svg",
            "M1_motor_tracking_summary.svg",
        }
        expect({path.name for path in saved} == expected_names, "unexpected output filenames")
        for path in saved:
            expect(path.exists(), f"missing generated image: {path}")
        overlay_text = (output_dir / "M0_motor_tracking_overlay.svg").read_text(
            encoding="utf-8"
        )
        expect("position (deg)" in overlay_text, "overlay should plot degrees")
        expect(">-45.00<" in overlay_text, "overlay should include fixed -45 deg tick")
        expect(">-44.00<" in overlay_text, "overlay should include every-degree tick")
        expect(">0.00<" in overlay_text, "overlay should include zero degree tick")
        expect(">44.00<" in overlay_text, "overlay should include every-degree tick")
        expect(">45.00<" in overlay_text, "overlay should include fixed +45 deg tick")
        summary_text = (output_dir / "M0_motor_tracking_summary.svg").read_text(
            encoding="utf-8"
        )
        expected_summary_labels = [
            "RMSE(deg)",
            "mean error(deg)",
            "P95(|e|)(deg)",
            "max |e|(deg)",
            "P95(|qdot_target|)(rad/s)",
            "max |qdot_target|(rad/s)",
            "P95(|delta q|)(rad)",
            "max |delta q|(rad)",
        ]
        for label in expected_summary_labels:
            expect(label in summary_text, f"summary missing label: {label}")
        expect(
            not (output_dir / "0831_1757_motor_tracking_overlay.svg").exists(),
            "combined overlay image should not be generated",
        )
        expect(
            not any(output_dir.glob("*_motor_tracking_error.svg")),
            "per-motor error images should not be generated",
        )


def main() -> int:
    test_zoh_alignment()
    test_old_header_and_state_timestamp_fallback()
    test_excluded_motor_selection()
    test_target_rate_and_delta_metrics()
    test_per_motor_svg_outputs_and_output_dir()
    print("plot_motor_tracking_test passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
