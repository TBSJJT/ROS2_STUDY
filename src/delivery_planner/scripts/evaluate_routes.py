#!/usr/bin/env python3
"""读取冻结距离/实测时间缓存，对手动任务配置精确枚举；拒绝不完整标定。"""
import argparse
import hashlib
from pathlib import Path
import shutil
from delivery_planner.costs import load_distance_cache, write_json, digest
from delivery_planner.calibration import load_time_cache
from delivery_planner.experiment import load_bundle
from delivery_planner.model import load_instance, enumerate_routes


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bundle", type=Path)
    parser.add_argument("--distance", type=Path)
    parser.add_argument("--time", type=Path)
    parser.add_argument("--instance", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if args.bundle:
        if any((args.distance, args.time, args.instance)):
            parser.error("--bundle cannot be mixed with individual inputs")
        args.distance, args.time, args.instance = load_bundle(args.bundle)
    elif not all((args.distance, args.time, args.instance)):
        parser.error("provide --bundle or all three individual inputs")
    distance = load_distance_cache(args.distance)
    time = load_time_cache(args.time, distance)
    instance = load_instance(args.instance, distance["stations"], distance["depot"])
    result = enumerate_routes(
        instance,
        distance["stations"],
        distance["depot"],
        distance["distance_m"],
        time["travel_time_sec"],
    )
    result.update(
        objective_semantics="planned_roundtrip_distance_and_tardiness_at_mean_travel_times",
        expected_actual_tardiness_claimed=False,
        evaluation_input_sha256=digest(
            {
                "distance": distance["content_sha256"],
                "time": time["content_sha256"],
                "instance": hashlib.sha256(args.instance.read_bytes()).hexdigest(),
                "model": "completion_tardiness_mean_time_v1",
            }
        ),
        distance_content_sha256=distance["content_sha256"],
        time_content_sha256=time["content_sha256"],
        instance_sha256=hashlib.sha256(args.instance.read_bytes()).hexdigest(),
        instance_id=instance["instance_id"],
        time_source=time["time_source"],
    )
    args.output.mkdir(parents=True, exist_ok=False)
    shutil.copyfile(args.instance, args.output / "instance.yaml")
    write_json(args.output / "enumeration.json", result)
    print({k: result[k] for k in ("permutation_count", "feasible_count", "pareto_count")})


if __name__ == "__main__":
    main()
