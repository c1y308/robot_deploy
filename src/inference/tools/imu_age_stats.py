#!/usr/bin/env python3
"""Measure IMU age at policy inference entry from an existing policy_test CSV.

Uses host monotonic timestamps only. The device sample clock is not synchronized
to the host, so this is NOT sensor-sampling-to-inference latency.
"""

import argparse
import bisect
import csv
import json
from pathlib import Path


REQUIRED_COLUMNS = ("frame_index", "inference_start_ns", "imu_receive_timestamp_ns")
PERCENTILES = (("p50", 500), ("p90", 900), ("p95", 950), ("p99", 990), ("p99_9", 999))
BIN_EDGES_NS = (0, 500_000, 1_000_000, 2_000_000, 3_000_000, 4_000_000,
                5_000_000, 10_000_000, 20_000_000, 50_000_000)


def analyze(path):
    ages = []
    frames = set()
    first_start = None
    last_start = None
    repeated_snapshots = 0
    receive_times = set()
    dropped_results = 0
    max_sample = None
    with Path(path).open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        missing = set(REQUIRED_COLUMNS) - set(reader.fieldnames or ())
        if missing:
            raise ValueError("missing required columns: " + ", ".join(sorted(missing)))
        for line, row in enumerate(reader, 2):
            try:
                frame, start, receive = (int(row[key]) for key in REQUIRED_COLUMNS)
                dropped = int(row.get("policy_result_dropped", "0"))
            except (ValueError, TypeError) as error:
                raise ValueError(f"line {line}: invalid integer or incomplete row") from error
            if frame < 0 or start <= 0 or receive <= 0 or start < receive:
                raise ValueError(f"line {line}: invalid host timestamps or frame index")
            if dropped not in (0, 1):
                raise ValueError(f"line {line}: policy_result_dropped must be 0 or 1")
            if frame in frames:
                raise ValueError(f"line {line}: duplicate policy frame {frame}")
            frames.add(frame)
            age = start - receive  # Subtract integer ns BEFORE converting units.
            ages.append(age)
            dropped_results += dropped
            repeated_snapshots += receive in receive_times
            receive_times.add(receive)
            first_start = start if first_start is None else min(first_start, start)
            last_start = start if last_start is None else max(last_start, start)
            if max_sample is None or age > max_sample["age_ns"]:
                max_sample = {"age_ns": age, "frame_index": frame, "csv_line": line,
                              "inference_start_ns": start,
                              "imu_receive_timestamp_ns": receive}
    if not ages:
        raise ValueError("CSV contains no policy frames")

    ages.sort()
    count = len(ages)
    age_ms = {"min": ages[0] / 1e6, "mean": sum(ages) / count / 1e6}
    # Nearest rank: sorted[ceil(N*p)-1], using exact integer ranks.
    for name, permille in PERCENTILES:
        age_ms[name] = ages[(count * permille + 999) // 1000 - 1] / 1e6
    age_ms["max"] = ages[-1] / 1e6

    histogram = []
    for index, lower in enumerate(BIN_EDGES_NS):
        upper = BIN_EDGES_NS[index + 1] if index + 1 < len(BIN_EDGES_NS) else None
        lo = bisect.bisect_left(ages, lower)
        hi = count if upper is None else bisect.bisect_left(ages, upper)
        histogram.append({"lower_ms_inclusive": lower / 1e6,
                          "upper_ms_exclusive": None if upper is None else upper / 1e6,
                          "count": hi - lo, "percent": 100 * (hi - lo) / count})

    return {
        "input_csv": str(Path(path).resolve()),
        "metric": "inference_start_ns - imu_receive_timestamp_ns",
        "percentile_method": "nearest rank: sorted[ceil(N*p)-1]",
        "scope": "Logged policy frames, including dropped policy results; repeated IMU snapshots retained.",
        "limitations": [
            "Excludes sensor internal delay and transport before the host CAN RX timestamp.",
            "Device sample_timestamp_ns has a different clock and is not subtracted from host time.",
            "Frames failing before recording (including stale-sensor guard failures) are absent.",
            "No missing frame indices does not prove the final attempted frame was recorded.",
            "Historical RX timestamp accuracy depends on the driver version used for that run.",
        ],
        "samples": count,
        "inference_span_s": (last_start - first_start) / 1e9,
        "first_frame_index": min(frames),
        "last_frame_index": max(frames),
        "missing_frames_between_first_and_last": max(frames) - min(frames) + 1 - count,
        "unlogged_initial_frames": min(frames),
        "dropped_policy_results_included": dropped_results,
        "unique_imu_receive_timestamps": len(receive_times),
        "reused_imu_snapshot_frames": repeated_snapshots,
        "age_ms": age_ms,
        "max_sample": dict(max_sample, seconds_from_first_inference=
                           (max_sample["inference_start_ns"] - first_start) / 1e9),
        "histogram": histogram,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_path", nargs="?", type=Path,
                        help="policy_test CSV; defaults to newest CSV in src/inference/log")
    parser.add_argument("--output-dir", type=Path,
                        help="report directory; defaults to <CSV stem>.imu_age beside input")
    args = parser.parse_args()
    path = args.csv_path
    if path is None:
        paths = list((Path(__file__).resolve().parents[1] / "log").glob("*.csv"))
        if not paths:
            parser.error("no policy CSV found; specify a CSV path")
        path = max(paths, key=lambda item: item.stat().st_mtime_ns)
    try:
        report = analyze(path)
        output = args.output_dir or path.with_suffix(".imu_age")
        output.mkdir(parents=True, exist_ok=True)
        (output / "summary.json").write_text(
            json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        with (output / "histogram.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(report["histogram"][0]))
            writer.writeheader()
            writer.writerows(report["histogram"])
    except (OSError, ValueError) as error:
        parser.exit(1, f"ERROR: {error}\n")

    print(f"Input: {path}")
    print("Age = inference_start_ns - imu_receive_timestamp_ns (host clock)")
    print(f"Samples: {report['samples']}; inference span: {report['inference_span_s']:.3f} s")
    print("Age (ms): " + ", ".join(f"{key}={value:.6f}"
                                       for key, value in report["age_ms"].items()))
    sample = report["max_sample"]
    print(f"Max: frame={sample['frame_index']}, CSV line={sample['csv_line']}, "
          f"time=+{sample['seconds_from_first_inference']:.6f} s")
    print(f"Missing frames within range: {report['missing_frames_between_first_and_last']}; "
          f"unlogged initial frames: {report['unlogged_initial_frames']}; "
          f"dropped results included: {report['dropped_policy_results_included']}; "
          f"reused snapshots: {report['reused_imu_snapshot_frames']}")
    print("Distribution [lower, upper) ms:")
    for bucket in report["histogram"]:
        upper = bucket["upper_ms_exclusive"]
        label = "inf" if upper is None else f"{upper:g}"
        print(f"  [{bucket['lower_ms_inclusive']:g}, {label}): "
              f"{bucket['count']:7d}  {bucket['percent']:7.3f}%")
    print("Scope: logged frames only; pre-recording failures are absent. "
          "This is not sensor-sampling-to-inference latency.")
    print(f"Reports: {output / 'summary.json'}, {output / 'histogram.csv'}")


if __name__ == "__main__":
    main()
