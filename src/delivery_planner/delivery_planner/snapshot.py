"""为静态点间规划保存输入快照，并派生隔离的 Nav2 参数。

离线矩阵使用保存地图的静态层和现有膨胀层。实时激光障碍不进入本次
矩阵，以免一次采集中的不同点对采用不同的环境。实际配送仍沿用原
Nav2 参数，包含实时障碍层；两者用途在结果中明确区分。
"""

from copy import deepcopy
import hashlib
from pathlib import Path
import shutil

from ament_index_python.packages import get_package_share_directory
import yaml

from delivery_tasks.config import load_stations
from delivery_planner.costs import digest, write_json


def prepare(output, map_file, stations_file, params_file):
    """创建新目录，拒绝覆盖已有实验；返回规划参数、工位和来源清单。"""
    output = Path(output).resolve()
    map_file, stations_file, params_file = map(Path, (map_file, stations_file, params_file))
    book = load_stations(stations_file, map_file)
    nav = yaml.safe_load(params_file.read_text())
    output.mkdir(parents=True, exist_ok=False)

    def copy(source, target):
        destination = output / "snapshots" / target
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, destination)
        return destination

    # 地图 YAML 和图像一起保存。原文件可以来自工作空间，也可以来自安装目录。
    saved_map = copy(map_file, "map/" + map_file.name)
    image = Path(yaml.safe_load(map_file.read_text())["image"])
    if image.is_absolute() or ".." in image.parts:
        raise ValueError("map image must be a relative path inside the map directory")
    copy(map_file.parent / image, "map/" + str(image))
    saved_stations = copy(stations_file, "stations.yaml")
    copy(params_file, "navigation.yaml")
    description = Path(get_package_share_directory("delivery_description"))
    for source in sorted((description / "urdf/delivery_robot").rglob("*.xacro")):
        copy(source, "description/" + str(source.relative_to(description)))
    simulation = Path(get_package_share_directory("delivery_simulation"))
    for relative in ("config/delivery_ros2_controller.yaml", "worlds/workshop_delivery_v1.world"):
        copy(simulation / relative, "simulation/" + relative)

    planner = deepcopy(nav["planner_server"]["ros__parameters"])
    costmap = deepcopy(nav["global_costmap"]["global_costmap"]["ros__parameters"])
    if costmap.get("global_frame") != "map" or costmap.get("rolling_window", False):
        raise ValueError("static matrix requires a nonrolling map-frame costmap")
    costmap.pop("obstacle_layer", None)
    costmap["plugins"] = ["static_layer", "inflation_layer"]
    costmap["static_layer"].update(map_topic="/cost_matrix/map", footprint_clearing_enabled=False)
    costmap.update(use_sim_time=False, robot_base_frame="cost_matrix_origin")
    planner["use_sim_time"] = False
    # 绝对节点名确保只作用于隔离规划器；不修改主导航的参数或 TF。
    runtime = {
        "/cost_matrix/planner_server": {"ros__parameters": planner},
        "/cost_matrix/global_costmap/global_costmap": {"ros__parameters": costmap},
        "/cost_matrix/map_server": {
            "ros__parameters": {
                "use_sim_time": False,
                "yaml_filename": str(saved_map),
                "topic_name": "map",
                "frame_id": "map",
            }
        },
    }
    runtime_file = output / "runtime_nav2.yaml"
    runtime_file.write_text(yaml.safe_dump(runtime, sort_keys=False))
    hashes = {
        str(path.relative_to(output)): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in sorted((output / "snapshots").rglob("*"))
        if path.is_file()
    }
    # 文件里的地图绝对位置不影响规划结果。规范化后参与绑定，目录搬移不改变身份。
    normalized = deepcopy(runtime)
    normalized["/cost_matrix/map_server"]["ros__parameters"]["yaml_filename"] = (
        "snapshots/map/" + map_file.name
    )
    normalized_file = output / "snapshots/runtime_nav2.normalized.yaml"
    normalized_file.write_text(yaml.safe_dump(normalized, sort_keys=False))
    hashes[str(normalized_file.relative_to(output))] = hashlib.sha256(
        normalized_file.read_bytes()
    ).hexdigest()
    manifest = {
        "version": 1,
        "sha256": hashes,
        "config_sha256": digest(hashes),
        "method": "Nav2 static layer + inflation layer; explicit start; directed pairs",
        "scope": "planned geometric paths, not driven trajectories or measured travel time",
        "runtime": normalized,
    }
    write_json(output / "provenance.json", manifest)
    return runtime_file, saved_stations, book, manifest
