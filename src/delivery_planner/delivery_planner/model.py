"""纯 Python 配送目标模型：完整回程距离与完成卸货后的累计迟交。

时间统一使用从出发开始的秒数。软截止时间只增加迟交目标，不把迟到
判成不可行。不可达边则返回明确的不可行结果，不能用零或直线补齐。
"""

from itertools import permutations
import math

from delivery_tasks.config import mapping, number


def load_instance(path, names, depot):
    raw = mapping(path)
    tasks = raw.get("tasks")
    if not isinstance(tasks, dict) or not tasks:
        raise ValueError("tasks must be a nonempty mapping")
    if any(name not in names or name == depot for name in tasks):
        raise ValueError("unknown task station or depot in tasks")
    for name, task in tasks.items():
        if not isinstance(task, dict):
            raise ValueError(f"{name}: task must be a mapping")
        for key in ("due_sec", "service_sec"):
            number(task.get(key), f"{name}.{key}")
    if not isinstance(raw.get("instance_id"), str) or not raw["instance_id"].strip():
        raise ValueError("instance_id must be nonempty")
    return raw


def validate_matrix(matrix, names, field):
    n = len(names)
    if len(set(names)) != n or not n or len(matrix) != n:
        raise ValueError(f"{field}: invalid names or matrix size")
    for i, row in enumerate(matrix):
        if len(row) != n:
            raise ValueError(f"{field}: matrix must be square")
        for j, value in enumerate(row):
            if i == j and value != 0:
                raise ValueError(f"{field}: diagonal must be zero")
            if value is not None:
                number(value, field)


def evaluate(route, instance, names, depot, distances, times):
    """调用前矩阵须经 validate_matrix 验证；该热点函数不做重复 O(n²) 校验。"""
    tasks = instance["tasks"]
    if not tasks or len(route) != len(tasks) or set(route) != set(tasks):
        raise ValueError("route must contain each task exactly once")
    index = {name: i for i, name in enumerate(names)}
    current, completion, distance, tardiness = depot, 0.0, 0.0, 0.0
    visits = []
    for target in [*route, depot]:
        i, j = index[current], index[target]
        length, travel = distances[i][j], times[i][j]
        if length is None or travel is None:
            return {
                "route": list(route),
                "feasible": False,
                "unavailable_edge": [current, target],
                "distance_m": None,
                "tardiness_sec": None,
                "duration_sec": None,
                "visits": visits,
            }
        distance += length
        arrival = completion + travel
        if target != depot:
            completion = arrival + tasks[target]["service_sec"]
            late = max(0.0, completion - tasks[target]["due_sec"])
            tardiness += late
            visits.append(
                {
                    "station": target,
                    "arrival_sec": arrival,
                    "completion_sec": completion,
                    "due_sec": tasks[target]["due_sec"],
                    "tardiness_sec": late,
                }
            )
        else:
            completion = arrival
        current = target
    return {
        "route": list(route),
        "feasible": True,
        "distance_m": distance,
        "tardiness_sec": tardiness,
        "duration_sec": completion,
        "visits": visits,
    }


def dominates(a, b):
    """两个目标均不差且至少一个严格更小；目标相同的不同顺序都保留。"""
    keys = ("distance_m", "tardiness_sec")
    return all(a[k] <= b[k] for k in keys) and any(a[k] < b[k] for k in keys)


def enumerate_routes(instance, names, depot, distances, times):
    """六任务枚举 720 种顺序，用来建立后续优化算法的精确参照。"""
    if len(instance["tasks"]) > 8:
        raise ValueError("exact enumeration is limited to at most 8 tasks")
    validate_matrix(distances, names, "distance_m")
    validate_matrix(times, names, "travel_time_sec")
    results = [
        evaluate(route, instance, names, depot, distances, times)
        for route in permutations(instance["tasks"])
    ]
    front = []
    # 对小规模问题用增量非支配集合，保留所有并列解，不做结果四舍五入。
    for row in results:
        if not row["feasible"] or any(dominates(other, row) for other in front):
            continue
        front = [other for other in front if not dominates(row, other)]
        front.append(row)
    return {
        "permutation_count": len(results),
        "feasible_count": sum(r["feasible"] for r in results),
        "pareto_count": len(front),
        "pareto": front,
        "results": results,
    }
