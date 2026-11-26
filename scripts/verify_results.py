#!/usr/bin/env python3
import argparse
import csv
import sys
from pathlib import Path


def as_bool(value):
    return value.strip().lower() == "true"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_path", type=Path)
    parser.add_argument("--attack-point", choices=("before_gate", "after_gate"), required=True)
    parser.add_argument("--hold-count", type=int, default=30)
    args = parser.parse_args()

    with args.csv_path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) != 1:
        raise SystemExit(f"FAIL: expected one result row, got {len(rows)}")

    row = rows[0]
    checks = {
        "attack point": row["attack_point"] == args.attack_point,
        "configured count": int(row["configured_hold_count"]) == args.hold_count,
        "released count": int(row["released_count"]) == args.hold_count,
        "stale command age": float(row["min_released_age_ms"]) > 100.0,
        "timestamp reversal": int(row["out_of_order_count"]) >= 1,
        "recipient accepted stale input": as_bool(row["stale_accepted_by_recipient"]),
        "physical reacceleration": as_bool(row["physical_reacceleration"]),
        "forward movement": float(row["max_forward_distance_m"]) > 0.01,
        "attack result": as_bool(row["attack_success"]),
    }
    for label, passed in checks.items():
        print(f"{'PASS' if passed else 'FAIL'}: {label}")
    if not all(checks.values()):
        return 1
    print(f"REPRODUCED: {args.attack_point} N-command delay attack")
    return 0


if __name__ == "__main__":
    sys.exit(main())
