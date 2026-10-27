#!/usr/bin/env python3
"""生成独立的冻结实验输入目录；失败或不足五次的时间样本不得发布为正式输入。"""
import argparse
from delivery_planner.experiment import freeze


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ("distance", "time", "instance", "output"):
        parser.add_argument("--" + key, required=True)
    args = parser.parse_args()
    print(freeze(args.distance, args.time, args.instance, args.output)["evaluation_input_sha256"])


if __name__ == "__main__":
    main()
