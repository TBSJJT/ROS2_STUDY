#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
基于 Nav2 单点导航动作的自主气体巡检任务管理节点.

================================================================================
这个节点是整个气体巡检系统的"大脑"——负责任务编排和生命周期管理.
它本身不做导航(交给 Nav2), 不做气体检测(交给传感器驱动), 
而是像一个"指挥官": 加载路线、逐点派发导航任务、监控风险、处理异常.

================================================================================
系统架构概览

  ┌──────────────────────────────────────────────────────────┐
  │                 InspectionManager(本节点)               │
  │                                                          │
  │  输入:                                                    │
  │  - YAML 路线配置文件                                      │
  │  - Nav2 NavigateToPose Action(导航能力)                 │
  │  - ExecuteInspection Action(来自后端的巡检请求)          │
  │  - RiskEvent 话题(气体风险事件)                          │
  │  - GasSensorArray 话题(实时气体浓度)                     │
  │  - ROS 2 服务(启动/取消/暂停/初始化/重载路线)            │
  │                                                          │
  │  输出:                                                    │
  │  - 状态发布(IDLE/NAVIGATING/DWELLING/...)               │
  │  - 巡检进度(current/total waypoints)                    │
  │  - ExecuteInspection Action 结果                          │
  │                                                          │
  │  协作节点:                                                │
  │  - Nav2(导航栈): 接收 NavigateToPose 目标, 路径规划和运动控制 │
  │  - AMCL(定位): 提供机器人在地图上的实时位姿               │
  │  - gasrobot_base(底盘): 接收最终的速度指令                │
  │  - gas_sensor_driver(传感器): 发布气体浓度数据            │
  └──────────────────────────────────────────────────────────┘

================================================================================
任务状态机

  IDLE ──(接受巡检任务)──> WAITING_NAV2 ──(Nav2就绪)──> NAVIGATING
    ^                                                     │
    │                                          ┌──────────┤
    │                                          │  到达航点  │  导航失败
    │                                          ▼          │
    │                                       DWELLING      │
    │                                          │          │
    │                              停留结束     │          │
    │                                          ▼          ▼
    │                                    下一个航点?   FAILED/CANCELLED
    │                                          │
    │                              全部完成     │
    │                                          ▼
    └──────────────────────────────────── COMPLETED

  任何状态在收到取消请求或安全停机信号时 → CANCELLING → CANCELLED/SAFETY_STOP

================================================================================
关键技术点(对 Python 初学者): 

1. async/await(异步编程): 
   Python 的 async/await 让函数可以在等待时不阻塞线程.
   - async def: 定义一个协程函数(coroutine)
   - await: 暂停当前协程, 等待另一个异步操作完成
   - 这里的很多方法是 async 的, 因为它们需要等待 Nav2 的异步响应

2. Action(动作): 
   ROS 2 的 Action 是一种"带反馈的长时间任务".
   - 普通 Service: 一问一答, 短时间完成
   - Action: 可以持续几分钟, 期间持续汇报进度, 还可以被取消
   - Action 三要素: Goal(目标)、Feedback(反馈)、Result(结果)

3. Callback Group(回调组): 
   决定回调函数在哪个线程中执行.
   - MutuallyExclusive: 同一组的回调不会并发执行(默认)
   - Reentrant: 同一组的回调可以并发执行(本节点使用这个)

4. MultiThreadedExecutor(多线程执行器): 
   ROS 2 默认是单线程的.使用多线程执行器后, 
   多个回调可以同时在不同的线程中运行, 提高响应性.

5. QoS(Quality of Service, 服务质量): 
   控制消息传递的可靠性、持久性等策略.
   - RELIABLE: 保证送达(不丢消息)
   - TRANSIENT_LOCAL: 后订阅的节点也能拿到最后一条消息
"""

# =========================================================================
# 标准库导入
# =========================================================================
# math: 数学函数(atan2 用于四元数→偏航角转换)
import math
# threading: 线程锁(RLock 是可重入锁, 同一线程可以多次获取)
import threading
# time: 时间函数(monotonic() 是单调时钟, 不受系统时间调整影响)
import time
# Enum: 枚举类型, 定义一组有名称的常量
from enum import Enum
# List: 列表的类型标注
from typing import List

# =========================================================================
# ROS 2 客户端库导入
# =========================================================================
# rclpy: ROS 2 Python 客户端库的主入口
import rclpy
# GoalStatus: Nav2 Action 的目标状态枚举
#   值包括 STATUS_SUCCEEDED, STATUS_ABORTED, STATUS_CANCELED 等
from action_msgs.msg import GoalStatus
# PoseStamped: 带时间戳和坐标系的位姿消息(导航目标点用)
# PoseWithCovarianceStamped: 带协方差的位姿消息(AMCL 初始位姿用)
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
# NavigateToPose: Nav2 的导航 Action 类型
#   发送一个目标位姿, Nav2 负责规划路径并驱动机器人到达
from nav2_msgs.action import NavigateToPose
# SetInitialPose: AMCL 的初始位姿设置服务
#   告诉 AMCL"机器人的初始位置大概在这里"
from nav2_msgs.srv import SetInitialPose
# SetParametersResult: 参数修改回调的返回值
from rcl_interfaces.msg import SetParametersResult
# ActionClient: Action 的客户端(发送目标的一方)
# ActionServer: Action 的服务器(接收目标、执行任务的一方)
# CancelResponse: Action 取消请求的响应
# GoalResponse: Action 目标请求的响应
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
# ReentrantCallbackGroup: 可重入回调组(允许多个回调并发执行)
from rclpy.callback_groups import ReentrantCallbackGroup
# MultiThreadedExecutor: 多线程执行器
from rclpy.executors import MultiThreadedExecutor
# Node: ROS 2 节点的基类
from rclpy.node import Node
# Parameter: ROS 2 参数类型
from rclpy.parameter import Parameter
# QoSProfile: 服务质量配置
# DurabilityPolicy: 持久性策略(消息在发布后是否保留)
# ReliabilityPolicy: 可靠性策略(是否保证送达)
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
# Future: 异步操作的结果占位符(类似 JavaScript 的 Promise)
from rclpy.task import Future
# Bool, String: 标准消息类型
from std_msgs.msg import Bool, String
# SetBool, Trigger: 标准服务类型
#   SetBool: 设置一个布尔值
#   Trigger: 无参数触发(只需 success/message 响应)
from std_srvs.srv import SetBool, Trigger

# =========================================================================
# 业务接口导入(自定义消息和 Action)
# =========================================================================
# ExecuteInspection: 巡检任务的 Action 类型
#   这是我们自己定义的 Action, 在 gasrobot_interfaces 包中
from gasrobot_interfaces.action import ExecuteInspection
# GasSensorArray: 气体传感器阵列读数
# RiskEvent: 气体风险事件
from gasrobot_interfaces.msg import GasSensorArray, RiskEvent

# =========================================================================
# 路线配置模块导入
# =========================================================================
from gasrobot_navigation.map_route_validator import (
    validate_route_book_against_map,
)
from gasrobot_navigation.route_config import (
    InspectionRoute,
    InspectionWaypoint,
    RouteBook,
    RouteConfigError,
    load_route_book,
)


# =========================================================================
# MissionState: 任务状态枚举
# =========================================================================
class MissionState(str, Enum):
    """
    巡检任务对外发布的稳定状态集合.

    继承自 str + Enum, 这样状态值既可以用作枚举比较, 
    又可以直接作为字符串发布(如 String 消息).

    状态说明: 
    - IDLE:          空闲, 等待新任务
    - INITIALIZING:  正在初始化(如设置 AMCL 初始位姿)
    - WAITING_NAV2:  等待 Nav2 导航栈就绪
    - NAVIGATING:    正在向一个航点导航
    - DWELLING:      已到达航点, 正在执行可选的静止观察
    - PAUSED:        任务已暂停(可继续)
    - CANCELLING:    正在取消任务(等待当前操作安全停止)
    - COMPLETED:     任务已成功完成
    - FAILED:        任务已失败(如导航超时且不允许跳过)
    - CANCELLED:     任务已被取消
    - SAFETY_STOP:   因安全原因紧急停机(如严重气体泄漏)

    """

    IDLE = "IDLE"
    INITIALIZING = "INITIALIZING"
    WAITING_NAV2 = "WAITING_NAV2"
    NAVIGATING = "NAVIGATING"
    DWELLING = "DWELLING"
    PAUSED = "PAUSED"
    CANCELLING = "CANCELLING"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"
    CANCELLED = "CANCELLED"
    SAFETY_STOP = "SAFETY_STOP"


# =========================================================================
# InspectionManager: 巡检任务管理节点
# =========================================================================
class InspectionManager(Node):
    """
    加载命名巡检路线, 向 Nav2 逐点派发导航目标并管理任务生命周期.

    这个类是一个 ROS 2 节点, 继承自 rclpy.node.Node.
    作为节点, 它拥有: 
    - 参数(parameters): 可通过 YAML 文件或命令行配置
    - 发布者(publishers): 向外发送消息
    - 订阅者(subscriptions): 接收外部消息
    - 服务(services): 响应外部调用
    - Action 客户端/服务器: 管理长时间运行的任务
    - 定时器(timers): 周期性地执行某些操作

    """

    def __init__(self) -> None:
        """
        初始化配置、Action 客户端/服务器、服务、话题和运行状态.

        初始化过程(按顺序): 
        1. 调用父类 Node.__init__("inspection_manager") 注册节点
        2. 声明并读取所有 ROS 参数
        3. 加载并校验路线文件
        4. 创建 Nav2 和巡检 Action 客户端
        5. 创建巡检 Action 服务器(响应后端请求)
        6. 创建 5 个 ROS 2 服务(启动、取消、暂停、初始化、重载)
        7. 创建状态发布者(QoS=TRANSIENT_LOCAL, 后启动也能拿到最新状态)
        8. 创建气体传感器订阅者
        9. 初始化运行状态变量(线程安全)
        10. 注册参数变更回调
        11. 按配置设置自动初始化和自动启动定时器

        """
        super().__init__("inspection_manager")

        # 创建可重入回调组
        # 可重入(Reentrant)意味着同一个回调组的回调函数可以并发执行
        # 这对于 Action 服务器/客户端尤其重要: 
        #   当 Action 回调正在等待 Nav2 响应时, 其他回调(如风险事件)仍然可以执行
        self.callback_group = ReentrantCallbackGroup()

        # declare_parameter(name, default_value): 
        #   声明一个参数并设置默认值
        #   用户可以在 launch 文件中或命令行用 --ros-args -p name:=value 覆盖

        # route_file: 巡检路线 YAML 文件路径
        #   默认空字符串 → 必须在参数文件或命令行中提供
        self.declare_parameter("route_file", "")
        # map_file: 与 Nav2 使用同一份地图，用于启动前校验所有航点。
        self.declare_parameter("map_file", "")
        # 巡检点中心到障碍物、未知区域或地图边界的最小安全距离。
        self.declare_parameter("minimum_waypoint_clearance_m", 0.30)
        # default_route: 默认执行的路线名称
        self.declare_parameter("default_route", "standard_route")
        # auto_set_initial_pose: 启动后是否自动向 AMCL 发送初始位姿
        self.declare_parameter("auto_set_initial_pose", True)
        # auto_start: 启动后是否自动开始巡检
        self.declare_parameter("auto_start", False)
        # nav2_wait_timeout_sec: 等待 Nav2 就绪的超时时间(秒)
        self.declare_parameter("nav2_wait_timeout_sec", 60.0)
        # critical_risk_level: 触发安全停机的风险等级阈值
        self.declare_parameter("critical_risk_level", 3)
        # initial_pose_topic: AMCL 初始位姿的话题名
        self.declare_parameter("initial_pose_topic", "/initialpose")

        # get_parameter(name).value: 读取参数值
        # 注意需要做类型转换(str/int/float/bool), 
        # 因为 rclpy 参数系统返回的是 ParameterValue 类型
        self.route_file = str(self.get_parameter("route_file").value)
        self.map_file = str(self.get_parameter("map_file").value)
        self.minimum_waypoint_clearance_m = float(
            self.get_parameter("minimum_waypoint_clearance_m").value
        )
        self.default_route = str(self.get_parameter("default_route").value)
        self.auto_set_initial_pose = bool(
            self.get_parameter("auto_set_initial_pose").value
        )
        self.auto_start = bool(self.get_parameter("auto_start").value)
        self.nav2_wait_timeout_sec = float(
            self.get_parameter("nav2_wait_timeout_sec").value
        )
        self.critical_risk_level = int(
            self.get_parameter("critical_risk_level").value
        )
        self.initial_pose_topic = str(
            self.get_parameter("initial_pose_topic").value
        )

        # ============================================================
        # 步骤 3: 加载并校验路线文件
        # ============================================================
        # 同时校验路线格式、场地确认标志和地图栅格安全距离。
        self.route_book = self._load_and_validate_route_book()

        # ============================================================
        # 步骤 4: 创建 Action 客户端
        # ============================================================
        # ActionClient: 向其他节点提供的 Action 服务器发送目标
        # 参数: 
        #   self:          本节点
        #   NavigateToPose: Action 类型(Nav2 的导航 Action)
        #   "navigate_to_pose": Action 的话题名
        #   callback_group: 使用可重入回调组确保并发性

        # 导航客户端: 向 Nav2 发送导航目标
        self.navigation_client = ActionClient(
            self,
            NavigateToPose,
            "navigate_to_pose",
            callback_group=self.callback_group,
        )

        # 巡检客户端: 用于从默认路线服务内部启动巡检 Action
        self.inspection_client = ActionClient(
            self,
            ExecuteInspection,
            "execute_inspection",
            callback_group=self.callback_group,
        )

        # AMCL 初始位姿服务客户端
        # create_client 用于普通的 ROS 2 服务(非 Action)
        self.initial_pose_client = self.create_client(
            SetInitialPose,
            "/set_initial_pose",
            callback_group=self.callback_group,
        )

        # ============================================================
        # 步骤 5: 创建 Action 服务器
        # ============================================================
        # ActionServer: 接收来自外部的巡检任务请求
        # ExecuteInspection Action 允许后端系统(如 Web 控制台、定时任务)
        # 直接下发临时巡检路线, 不依赖修改本地 YAML 文件
        self.inspection_server = ActionServer(
            self,
            ExecuteInspection,
            "execute_inspection",
            # execute_callback: 当目标被接受后, 执行实际任务
            execute_callback=self._execute_action,
            # goal_callback: 收到新目标时的验证(接受或拒绝)
            goal_callback=self._goal_callback,
            # cancel_callback: 收到取消请求时的处理
            cancel_callback=self._cancel_callback,
            callback_group=self.callback_group,
        )

        # ============================================================
        # 步骤 6: 创建 ROS 2 服务
        # ============================================================
        # 每个服务提供一个操作接口, 外部可以通过 ros2 service call 调用
        # 服务名用 ~/ 前缀表示私有命名空间: 
        #   实际服务名为 /inspection_manager/start_default

        # ~/start_default: 启动默认路线巡检
        self.start_service = self.create_service(
            Trigger,
            "~/start_default",
            self._start_default_callback,
            callback_group=self.callback_group,
        )
        # ~/cancel: 取消当前巡检任务
        self.cancel_service = self.create_service(
            Trigger,
            "~/cancel",
            self._cancel_service_callback,
            callback_group=self.callback_group,
        )
        # ~/pause: 暂停或继续巡检
        self.pause_service = self.create_service(
            SetBool,
            "~/pause",
            self._pause_callback,
            callback_group=self.callback_group,
        )
        # ~/set_initial_pose: 重新设置 AMCL 初始位姿
        self.initialize_service = self.create_service(
            Trigger,
            "~/set_initial_pose",
            self._set_initial_pose_callback,
            callback_group=self.callback_group,
        )
        # ~/reload_routes: 重新加载路线文件(现场调整航点后使用)
        self.reload_service = self.create_service(
            Trigger,
            "~/reload_routes",
            self._reload_routes_callback,
            callback_group=self.callback_group,
        )

        # ============================================================
        # 步骤 7: 创建状态发布者
        # ============================================================
        # QoS 配置说明: 
        # - depth=1: 只保留最新一条消息(Queue Size = 1)
        # - RELIABLE: 保证消息送达(不会因网络拥塞丢消息)
        # - TRANSIENT_LOCAL: 后启动的订阅者也能收到最后发布的一条消息
        #   这对于 UI 界面很重要: 界面后启动时能立即看到当前任务状态

        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        # ~/state: 发布任务状态字符串(格式: "MISSION_STATE|详细信息")
        self.state_publisher = self.create_publisher(
            String, "~/state", state_qos
        )
        # ~/active: 发布任务是否正在执行(True/False)
        self.active_publisher = self.create_publisher(
            Bool, "~/active", state_qos
        )
        # ~/current_waypoint: 发布当前正在执行的航点 ID
        self.waypoint_publisher = self.create_publisher(
            String, "~/current_waypoint", 10
        )

        # AMCL 初始位姿发布者(备用方式: 不是所有 AMCL 版本都有服务接口)
        self.initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            self.initial_pose_topic,
            10,
        )

        # ============================================================
        # 步骤 8: 创建气体传感器订阅者
        # ============================================================
        # 风险事件订阅(紧急安全信息)
        self.risk_subscription = self.create_subscription(
            RiskEvent,
            "/gas/risk_event",
            self._risk_event_callback,
            20,  # Queue Size = 20
            callback_group=self.callback_group,
        )
        # 气体浓度读数订阅(实时浓度数据)
        self.gas_subscription = self.create_subscription(
            GasSensorArray,
            "/gas/readings",
            self._gas_readings_callback,
            20,
            callback_group=self.callback_group,
        )

        # ============================================================
        # 步骤 9: 初始化运行状态变量
        # ============================================================
        # threading.RLock(): 可重入锁(Reentrant Lock)
        # 与普通 Lock 的区别: 
        #   同一个线程可以多次 acquire() 而不会死锁
        # 用于保护 _state、_mission_active 等共享变量
        self._state_lock = threading.RLock()

        # 任务控制标志
        self._mission_active = False     # 是否有巡检任务正在执行
        self._paused = False             # 任务是否已暂停
        self._cancel_requested = False   # 是否收到了取消请求
        self._safety_stop_requested = False  # 是否触发了安全停机

        # 导航状态
        self._current_nav_goal = None    # 当前活跃的 Nav2 目标句柄

        # 统计计数器
        self._risk_event_count = 0       # 累计风险事件数
        self._completed_waypoints = 0    # 已完成的航点数
        self._current_waypoint_id = ""   # 当前航点 ID

        # 气体监测状态
        self._target_gas = ""            # 本次任务的目标气体类型
        self._alarm_threshold = 0.0      # 报警浓度阈值
        self._current_concentration = 0.0  # 当前实测浓度
        self._current_risk_level = 0     # 当前风险等级
        self._stop_on_critical_risk = False  # 严重风险时是否停机

        # 初始状态发布
        self._state = MissionState.IDLE
        self._detail = "节点已启动, 等待巡检任务"
        self._publish_state()

        # ============================================================
        # 步骤 10: 注册参数变更回调
        # ============================================================
        # add_on_set_parameters_callback: 
        #   当用户通过 ros2 param set 动态修改参数时触发
        #   这里只允许在空闲状态下切换默认路线
        self.add_on_set_parameters_callback(self._parameter_callback)

        # ============================================================
        # 步骤 11: 按配置设置启动定时器
        # ============================================================
        # 启动定时器是单次触发的(回调中会取消自身)
        self._initial_pose_startup_timer = None
        self._auto_startup_timer = None

        if self.auto_set_initial_pose:
            # 2 秒后自动发送 AMCL 初始位姿
            # 延迟是为了确保 Nav2/AMCL 节点已经启动完毕
            self._initial_pose_startup_timer = self.create_timer(
                2.0, self._initial_pose_timer
            )
        if self.auto_start:
            # 5 秒后自动开始巡检
            # 比 initial_pose 更长的延迟, 确保定位已完成
            self._auto_startup_timer = self.create_timer(
                5.0, self._auto_start_timer
            )

        # 启动日志
        self.get_logger().info(
            f"巡检任务管理节点已启动: 路线文件={self.route_file}, "
            f"默认路线={self.default_route}"
        )

    # =====================================================================
    # 静态方法: 场地配置校验
    # =====================================================================
    @staticmethod
    def _validate_site_configuration(route_book: RouteBook) -> None:
        """
        拒绝把模板中的零坐标误用于真实巡检.

        这是一个安全检查机制.
        模板 YAML 文件中的航点坐标通常是占位值(如 x:0, y:0), 
        需要用户根据实际场地测量并修改.如果忘记修改就启动, 
        机器人会导航到错误的位置.

        防护方式: 
        配置文件中有一个 site_configured 字段(默认为 false).
        用户标定完所有航点坐标后, 手动将其改为 true.
        如果 site_configured 仍是 false, 节点拒绝启动.

        参数:
            route_book: 已加载的路线手册

        异常:
            RouteConfigError: 如果场地尚未标定

        """
        if not route_book.site_configured:
            raise RouteConfigError(
                "巡检点仍是模板值: 请标定 routes.yaml 后将 "
                "site_configured 改为 true"
            )

    def _load_and_validate_route_book(self) -> RouteBook:
        """加载路线，并拒绝不属于指定地图已知自由区的点位。"""

        if not self.map_file:
            raise RouteConfigError("必须通过 map_file 指定 Nav2 使用的地图")
        route_book = load_route_book(self.route_file)
        self._validate_site_configuration(route_book)
        route_book.route(self.default_route)
        validate_route_book_against_map(
            route_book,
            self.map_file,
            self.minimum_waypoint_clearance_m,
        )
        return route_book

    # =====================================================================
    # 参数变更回调
    # =====================================================================
    def _parameter_callback(self, parameters: List[Parameter]):
        """
        仅允许在空闲状态下切换默认路线名称.

        这个回调在用户通过 ros2 param set 修改参数时触发.
        安全限制: 巡检任务执行中不允许切换路线, 
        避免正在执行的路线被意外改变导致逻辑混乱.

        参数:
            parameters: 被修改的参数列表

        返回:
            SetParametersResult: successful=True 表示接受修改, 
                                  successful=False 表示拒绝并说明原因

        """
        for parameter in parameters:
            # 只关心 default_route 参数的修改
            if parameter.name != "default_route":
                continue
            # 线程安全地检查任务状态
            with self._state_lock:
                if self._mission_active:
                    # 任务执行中: 拒绝修改
                    return SetParametersResult(
                        successful=False,
                        reason="巡检执行中不能切换默认路线",
                    )
            # 验证新路线名是否存在
            try:
                self.route_book.route(str(parameter.value))
            except RouteConfigError as exc:
                return SetParametersResult(successful=False, reason=str(exc))
            # 接受修改
            self.default_route = str(parameter.value)
        return SetParametersResult(successful=True)

    # =====================================================================
    # Action 服务器回调(目标/取消/执行)
    # =====================================================================
    def _goal_callback(self, _goal_request) -> GoalResponse:
        """
        同一时刻只接受一项巡检任务.

        这个方法是 Action 服务器的目标过滤器.
        当外部发送 ExecuteInspection Goal 时, 先经过这个方法判断
        是否接受.如果已有任务在执行, 则拒绝新的.

        参数:
            _goal_request: 目标请求(这里用 _ 前缀表示不使用此参数)

        返回:
            GoalResponse.ACCEPT: 接受任务
            GoalResponse.REJECT: 拒绝任务

        """
        with self._state_lock:
            if self._mission_active:
                self.get_logger().warning("已有巡检任务运行, 拒绝新任务")
                return GoalResponse.REJECT
            # 接受任务: 设置活动标志, 重置所有计数器
            self._mission_active = True
            self._reset_mission_counters()
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle) -> CancelResponse:
        """
        接受任务取消请求, 并转发给当前正在执行的 Nav2 导航目标.

        Action 的取消是协作式的: 取消请求只是一个建议, 
        需要当前 Action 目标主动响应.

        返回:
            CancelResponse.ACCEPT: 总是接受取消请求

        """
        self._request_cancel("收到 Action 取消请求")
        return CancelResponse.ACCEPT

    async def _execute_action(self, goal_handle):
        """
        执行通过 ExecuteInspection Action 下发的临时巡检路线.

        这是 Action 服务器的核心执行回调, 在任务被接受后运行.
        作为 async 函数, 它可以使用 await 等待异步操作, 
        同时不阻塞 ROS 2 执行器线程.

        有两种路线来源: 
        1. goal_handle.request.route_name 不为空 → 从 RouteBook 查找已配置路线
        2. goal_handle.request.route_name 为空 → 使用 request 中携带的航点构造临时路线

        参数:
            goal_handle: Action 目标句柄, 通过它可以获取 request 和发布 feedback/result

        返回:
            ExecuteInspection.Result: 任务结果

        """
        try:
            # 获取路线名称(去除空白)
            route_name = str(goal_handle.request.route_name).strip()
            if route_name:
                # 方式 1: 从配置文件加载已有路线
                route = self.route_book.route(route_name)
            else:
                # 方式 2: 从 Action 请求中解析临时路线
                route = self._route_from_action_goal(goal_handle.request)
        except RouteConfigError as exc:
            # 路线配置有问题 → 任务失败
            result = ExecuteInspection.Result()
            goal_handle.abort()  # 通知客户端任务中止
            with self._state_lock:
                self._mission_active = False
            self._set_state(MissionState.FAILED, str(exc))
            self._publish_active(False)
            return self._fill_result(result, False, str(exc))

        # 执行路线
        return await self._execute_route(route, goal_handle)

    def _route_from_action_goal(self, request) -> InspectionRoute:
        """
        把 ExecuteInspection Action 的 Goal 请求转换为内部 InspectionRoute 模型.

        这允许后端动态下发巡检路线, 不需要提前写入 YAML 文件.
        适用于远程控制台或定时任务等场景.

        参数:
            request: ExecuteInspection.Goal 请求对象

        返回:
            对应的 InspectionRoute 对象

        异常:
            RouteConfigError: 如果请求中没有任何航点

        """
        if not request.waypoints:
            raise RouteConfigError("Action 巡检路线至少需要一个目标点")

        waypoints = []
        # default_dwell_sec: 路线级默认静止观察时间
        default_dwell = max(0.0, float(request.default_dwell_sec))

        # enumerate 同时获取索引和值
        for index, pose in enumerate(request.waypoints):
            frame_id = str(pose.header.frame_id).strip()
            if frame_id and frame_id != self.route_book.frame_id:
                raise RouteConfigError(
                    f"临时巡检点 {index + 1} 的坐标系必须是 "
                    f"{self.route_book.frame_id}"
                )
            # 从四元数中提取偏航角(yaw)
            # 四元数 (x, y, z, w) 到偏航角的简化公式: 
            #   yaw = atan2(2*(w*z), 1 - 2*z²)
            # 这是四元数→欧拉角公式在 2D 平面(只有 yaw)的特例
            quaternion = pose.pose.orientation
            yaw = math.atan2(
                2.0 * (quaternion.w * quaternion.z),
                1.0 - 2.0 * quaternion.z * quaternion.z,
            )

            # 创建航点, ID 格式: remote_001, remote_002, ...
            waypoints.append(
                InspectionWaypoint(
                    waypoint_id=f"remote_{index + 1:03d}",
                    description="后端临时下发的巡检点",
                    x=pose.pose.position.x,
                    y=pose.pose.position.y,
                    yaw=yaw,
                    dwell_sec=default_dwell,
                )
            )

        # 构造 InspectionRoute, 使用请求中的参数或默认值
        route = InspectionRoute(
            name="remote_action_route",
            description="通过 ExecuteInspection Action 下发",
            target_gas=request.target_gas or "unknown",
            alarm_threshold=max(0.0, float(request.alarm_threshold)),
            stop_on_critical_risk=bool(request.stop_on_critical_risk),
            repeat_count=max(1, int(request.repeat_count)),
            continue_on_failure=bool(request.continue_on_failure),
            max_retries=int(request.max_retries),
            navigation_timeout_sec=(
                float(request.navigation_timeout_sec)
                if request.navigation_timeout_sec > 0.0
                else 300.0  # 默认 5 分钟超时
            ),
            waypoints=waypoints,
        )
        # 后端临时下发的点也必须经过与本地 YAML 相同的地图安全校验。
        temporary_book = RouteBook(
            frame_id=self.route_book.frame_id,
            site_configured=True,
            initial_pose=None,
            routes={route.name: route},
        )
        validate_route_book_against_map(
            temporary_book,
            self.map_file,
            self.minimum_waypoint_clearance_m,
        )
        return route

    # =====================================================================
    # 核心: 路线执行引擎
    # =====================================================================
    async def _execute_route(self, route, goal_handle=None):
        """
        按圈次和航点顺序执行巡检路线, 统一处理失败、取消和安全停机.

        这是本节点最核心的方法——"巡检任务引擎".
        流程如下: 

        for 每一圈 (repeat_count 次):
            for 每个航点:
                1. 检查是否应该停止(取消/安全事件)
                2. 向 Nav2 发送导航目标(带重试)
                3. 等待到达或超时
                4. 如果不成功且 continue_on_failure=False → 任务失败
                5. 如果配置了 dwell_sec → 执行可选的静止观察
                6. 继续下一个航点

        参数:
            route:       要执行的巡检路线
            goal_handle: Action 目标句柄(可通过外部 Action 调用时为非 None)

        返回:
            ExecuteInspection.Result

        """
        result = ExecuteInspection.Result()
        try:
            # 记录本次巡检的气体目标和报警参数
            self._target_gas = route.target_gas
            self._alarm_threshold = route.alarm_threshold
            self._stop_on_critical_risk = route.stop_on_critical_risk

            # 等待 Nav2 导航栈就绪
            self._set_state(
                MissionState.WAITING_NAV2,
                f'等待 Nav2, 准备执行路线 "{route.name}"',
            )
            if not await self._wait_for_nav2():
                return self._finish_result(
                    result, goal_handle, False, "等待 Nav2 超时"
                )

            # 双层循环: 外层是圈数, 内层是航点
            for repeat_index in range(route.repeat_count):
                for waypoint_index, waypoint in enumerate(route.waypoints):
                    # --- 检查点: 每次循环开始前检查是否需要停止 ---
                    if self._must_stop(goal_handle):
                        return self._finish_cancelled(result, goal_handle)

                    # --- 导航到航点(带重试)---
                    succeeded = await self._navigate_with_retries(
                        route,
                        waypoint,
                        repeat_index,
                        waypoint_index,
                        goal_handle,
                    )

                    if not succeeded:
                        # 检查是否是取消/安全停机导致的失败
                        if self._must_stop(goal_handle):
                            return self._finish_cancelled(result, goal_handle)
                        # 如果配置了 continue_on_failure, 跳过失败的航点
                        if route.continue_on_failure:
                            self.get_logger().warning(
                                f"跳过失败巡检点: {waypoint.waypoint_id}"
                            )
                            continue
                        # 否则整个任务失败
                        return self._finish_result(
                            result,
                            goal_handle,
                            False,
                            f"巡检点 {waypoint.waypoint_id} 导航失败",
                        )

                    # --- 导航成功 ---
                    self._completed_waypoints += 1
                    self._publish_feedback(
                        goal_handle,
                        len(route.waypoints) * route.repeat_count,
                    )

                    # 气体节点在整个任务期间连续采样；这里的停留不是采样开关，
                    # 只用于响应实验或重点区域需要静止观察的情况。
                    if waypoint.dwell_sec > 0.0:
                        self._set_state(
                            MissionState.DWELLING,
                            f"在 {waypoint.waypoint_id} 静止观察",
                        )
                        # 可中断的等待(支持暂停/取消/安全停机)
                        if not await self._interruptible_wait(
                            waypoint.dwell_sec,
                            goal_handle,
                        ):
                            return self._finish_cancelled(result, goal_handle)

            # 全部航点完成 → 任务成功
            return self._finish_result(
                result,
                goal_handle,
                True,
                f'路线 "{route.name}" 巡检完成',
            )
        except Exception as exc:
            # 大 catch: 任何未预期的异常都要被捕获
            # 不能让异常直接传播到 Action 服务器层, 否则会丢失任务状态
            self.get_logger().exception(f"巡检任务异常: {exc}")
            return self._finish_result(result, goal_handle, False, str(exc))
        finally:
            # finally 块: 无论成功、失败还是异常, 都会执行
            # 确保任务结束后正确清理状态
            with self._state_lock:
                self._mission_active = False
                self._paused = False
                self._cancel_requested = False
                self._safety_stop_requested = False
                self._current_nav_goal = None
                self._stop_on_critical_risk = False
            self._publish_active(False)

    async def _navigate_with_retries(
        self,
        route,
        waypoint,
        repeat_index,
        waypoint_index,
        goal_handle,
    ) -> bool:
        """
        向 Nav2 发送单点导航目标, 并按路线策略进行有限重试.

        重试逻辑: 
        - 每次导航失败后, 如果仍有重试次数, 重新发起导航
        - 如果导航过程中收到取消/暂停/安全信号, 停止重试
        - 暂停不是导航失败: 恢复后继续用原来的重试次数

        参数:
            route:           巡检路线
            waypoint:        目标航点
            repeat_index:    当前圈数索引(0-based)
            waypoint_index:  当前航点索引(0-based)
            goal_handle:     Action 目标句柄

        返回:
            True: 导航成功到达
            False: 导航失败且重试次数已用完, 或任务被中断

        """
        # 计算总航点数(航点数 × 圈数), 用于进度显示
        total_waypoints = len(route.waypoints) * route.repeat_count

        self._current_waypoint_id = waypoint.waypoint_id
        self._publish_waypoint(waypoint)

        attempt = 0
        # 尝试次数 = max_retries + 1(首次算尝试 0)
        while attempt <= route.max_retries:
            # --- 等待暂停结束 ---
            if not await self._wait_while_paused(goal_handle):
                return False

            # 计算全局进度序号(跨圈)
            sequence = repeat_index * len(route.waypoints) + waypoint_index + 1
            self._set_state(
                MissionState.NAVIGATING,
                f"前往 {waypoint.waypoint_id}, 进度 {sequence}/{total_waypoints}, "
                f"尝试 {attempt + 1}/{route.max_retries + 1}",
            )
            self._publish_feedback(goal_handle, total_waypoints)

            # --- 执行一次导航 ---
            status = await self._navigate_once(
                waypoint,
                route.navigation_timeout_sec,
                goal_handle,
            )

            # 成功 → 直接返回
            if status == GoalStatus.STATUS_SUCCEEDED:
                return True

            # 被取消或安全停机 → 停止重试
            if self._must_stop(goal_handle):
                return False

            # 暂停状态 → 等待继续后重新尝试(不算一次失败)
            if self._paused:
                if not await self._wait_while_paused(goal_handle):
                    return False
                continue

            # 其他失败 → 记录日志, 增加重试计数
            self.get_logger().warning(
                f"巡检点 {waypoint.waypoint_id} 导航未成功, 状态={status}"
            )
            attempt += 1

        # 所有重试已用完 → 失败
        return False

    async def _navigate_once(self, waypoint, timeout_sec, goal_handle) -> int:
        """
        执行一次 NavigateToPose Action, 并监视超时、暂停、取消和风险停机.

        这是单次导航的核心方法.流程: 
        1. 构造 NavigateToPose.Goal
        2. 异步发送目标到 Nav2
        3. 进入循环等待结果: 
           - 检查是否需要停止(取消/暂停/安全)
           - 检查是否超时
           - 短暂 sleep(不阻塞线程)
        4. 返回最终状态

        参数:
            waypoint:    目标航点
            timeout_sec: 超时时间(秒)
            goal_handle: Action 目标句柄

        返回:
            GoalStatus 状态码(STATUS_SUCCEEDED/ABORTED/CANCELED 等)

        """
        # 构造导航目标
        goal = NavigateToPose.Goal()
        goal.pose = self._pose_stamped(waypoint)

        # send_goal_async: 异步发送目标
        # 返回一个 Future, 在完成时提供 GoalHandle
        future = self.navigation_client.send_goal_async(goal)
        nav_goal = await future

        # 目标被 Nav2 拒绝
        if not nav_goal.accepted:
            return GoalStatus.STATUS_ABORTED

        # 保存当前导航目标句柄(用于外部取消)
        with self._state_lock:
            self._current_nav_goal = nav_goal

        # get_result_async: 异步等待导航结果
        result_future = nav_goal.get_result_async()
        # 计算截止时间
        deadline = time.monotonic() + timeout_sec

        try:
            # 循环等待导航完成
            while not result_future.done():
                # 检查停止条件
                if self._must_stop(goal_handle) or self._paused:
                    # 取消当前 Nav2 目标
                    await nav_goal.cancel_goal_async()
                    if self._paused and not self._must_stop(goal_handle):
                        # 暂停 → 发布状态说明
                        self._set_state(
                            MissionState.PAUSED,
                            f"已暂停, 将在继续后重新导航到 {waypoint.waypoint_id}",
                        )
                    return GoalStatus.STATUS_CANCELED

                # 检查超时
                if time.monotonic() >= deadline:
                    self.get_logger().error(
                        f"巡检点 {waypoint.waypoint_id} 导航超时(>{timeout_sec:.1f}s)"
                    )
                    await nav_goal.cancel_goal_async()
                    return GoalStatus.STATUS_ABORTED

                # 短暂休眠(100ms), 避免忙等消耗 CPU
                # 使用 _sleep 而不是 time.sleep, 因为 _sleep 创建的是 ROS 定时器
                # 不会阻塞事件循环
                await self._sleep(0.1)

            # 导航完成 → 获取并返回状态
            return (await result_future).status
        finally:
            # 无论导航成功还是失败, 清除当前目标句柄
            with self._state_lock:
                self._current_nav_goal = None

    # =====================================================================
    # 异步等待辅助方法
    # =====================================================================
    async def _wait_for_nav2(self) -> bool:
        """
        异步等待 Nav2 Action 服务器就绪, 期间保持可响应取消.

        在发送导航目标之前, 必须先确认 Nav2 的 navigate_to_pose
        Action 服务器已经启动并准备接收请求.

        server_is_ready(): 非阻塞检查 Action 服务器是否就绪.
        如果没有服务器在监听, 返回 False.

        返回:
            True: Nav2 已就绪
            False: 超时或收到取消请求

        """
        deadline = time.monotonic() + self.nav2_wait_timeout_sec
        while time.monotonic() < deadline:
            if self.navigation_client.server_is_ready():
                return True
            if self._cancel_requested:
                return False
            # 每 200ms 检查一次
            await self._sleep(0.2)
        return False

    async def _wait_while_paused(self, goal_handle) -> bool:
        """
        暂停期间保持等待, 继续后从当前巡检点重新发起导航.

        这个循环在暂停时每 100ms 检查一次: 
        1. 是否继续了(_paused 变为 False)？
        2. 是否被取消了？
        3. 是否触发了安全停机？

        返回:
            True: 暂停已结束, 可以继续
            False: 任务被取消或安全停机

        """
        while self._paused and not self._must_stop(goal_handle):
            self._set_state(MissionState.PAUSED, "巡检任务已暂停")
            await self._sleep(0.1)
        return not self._must_stop(goal_handle)

    async def _interruptible_wait(self, duration, goal_handle) -> bool:
        """
        执行可被暂停、取消或安全事件打断的航点停留.

        与 time.sleep(duration) 不同, 这个方法: 
        - 可以被暂停打断(暂停时冻结倒计时)
        - 可以被取消打断
        - 可以被安全风险打断
        - 每次只睡很小的时间片, 频繁检查中断条件

        参数:
            duration:     停留时长(秒)
            goal_handle:  Action 目标句柄

        返回:
            True: 停留时间完全用完
            False: 被中断

        """
        remaining = duration
        previous = time.monotonic()
        while remaining > 0.0:
            # 检查中断条件
            if self._must_stop(goal_handle):
                return False
            # 被暂停时冻结倒计时
            if self._paused:
                if not await self._wait_while_paused(goal_handle):
                    return False
                # 暂停结束, 重设时间戳避免计算暂停期间的时间
                previous = time.monotonic()
            # 每次最多睡 100ms, 以便及时响应中断
            await self._sleep(min(0.1, remaining))
            now = time.monotonic()
            remaining -= now - previous
            previous = now
        return True

    def _must_stop(self, goal_handle=None) -> bool:
        """
        返回任务是否必须立即停止.

        停止条件(任一成立即返回 True): 
        1. _cancel_requested: 收到取消请求
        2. _safety_stop_requested: 触发安全停机
        3. goal_handle.is_cancel_requested: Action 客户端请求取消

        参数:
            goal_handle: Action 目标句柄(可能为 None)

        返回:
            True: 必须停止
            False: 可以继续

        """
        return (
            self._cancel_requested
            or self._safety_stop_requested
            or (goal_handle is not None and goal_handle.is_cancel_requested)
        )

    # =====================================================================
    # ROS 2 服务回调
    # =====================================================================
    def _start_default_callback(self, _request, response):
        """
        通过 ~/start_default 服务启动配置文件中的默认路线.

        这个服务提供了最简单的启动方式: 
        ros2 service call /inspection_manager/start_default std_srvs/srv/Trigger

        流程: 
        1. 检查是否已有任务在运行
        2. 检查巡检 Action 服务器是否就绪
        3. 从 RouteBook 获取默认路线
        4. 通过 inspection_client 异步发送巡检目标

        参数:
            _request:  Trigger 请求(无内容)
            response:  Trigger 响应(success + message)

        返回:
            Trigger.Response

        """
        if self._mission_active:
            response.success = False
            response.message = "已有巡检任务正在运行"
            return response

        if not self.inspection_client.server_is_ready():
            response.success = False
            response.message = "巡检 Action 服务尚未就绪, 请稍后重试"
            return response

        # 获取默认路线
        route = self.route_book.route(self.default_route)
        goal = ExecuteInspection.Goal()
        goal.route_name = route.name

        # 异步发送巡检目标
        # add_done_callback: 当目标响应到达时调用回调
        future = self.inspection_client.send_goal_async(goal)
        future.add_done_callback(self._default_goal_response_callback)

        response.success = True
        response.message = f"默认路线启动请求已提交: {route.name}"
        return response

    def _default_goal_response_callback(self, future) -> None:
        """
        记录默认路线内部 Action 请求是否被接受.

        这个回调在 inspection_client 收到 Action 服务器的目标响应后触发.
        注意: 这个回调在异步上下文中执行.

        参数:
            future: 包含 GoalHandle 或异常的 Future

        """
        try:
            if not future.result().accepted:
                self.get_logger().error("默认路线启动请求被拒绝")
        except Exception as exc:
            self.get_logger().error(f"默认路线启动失败: {exc}")

    def _cancel_service_callback(self, _request, response):
        """
        通过 ~/cancel 服务取消当前巡检任务.

        ros2 service call /inspection_manager/cancel std_srvs/srv/Trigger

        """
        if not self._mission_active:
            response.success = False
            response.message = "当前没有巡检任务"
            return response
        self._request_cancel("收到取消服务请求")
        response.success = True
        response.message = "取消请求已提交"
        return response

    def _pause_callback(self, request, response):
        """
        通过 ~/pause 服务暂停或继续巡检任务.

        ros2 service call /inspection_manager/pause std_srvs/srv/SetBool "{data: true}"
        data=true  → 暂停
        data=false → 继续

        暂停时会取消当前正在执行的 Nav2 导航目标.
        继续时会从当前航点重新发起导航.

        """
        if not self._mission_active:
            response.success = False
            response.message = "当前没有巡检任务"
            return response

        # 如果状态没有变化, 不需要做任何事
        if bool(request.data) == self._paused:
            response.success = True
            response.message = (
                "巡检任务已暂停" if self._paused else "巡检任务正在运行"
            )
            return response

        # 切换暂停状态
        self._paused = bool(request.data)
        if self._paused:
            # 暂停 → 取消当前的 Nav2 目标
            self._cancel_current_nav_goal()
            response.message = "巡检任务正在暂停"
        else:
            response.message = "巡检任务继续执行"
        response.success = True
        return response

    def _set_initial_pose_callback(self, _request, response):
        """
        通过 ~/set_initial_pose 服务向 AMCL 发送配置文件中的初始位姿.

        这在以下场景很有用: 
        - 机器人被搬到新位置后手动触发重新定位
        - AMCL 定位漂移后手动重置

        """
        with self._state_lock:
            if self._mission_active:
                response.success = False
                response.message = "巡检执行中禁止重新设置 AMCL 初始位姿"
                return response
        success, message = self._send_initial_pose()
        response.success = success
        response.message = message
        return response

    def _reload_routes_callback(self, _request, response):
        """
        通过 ~/reload_routes 服务重新读取路线文件.

        使用场景: 
        - 现场调整了某个航点的坐标
        - 添加了新的巡检路线
        - 修改了报警阈值

        限制: 只能在任务空闲时重载, 巡检执行中不允许.

        """
        with self._state_lock:
            if self._mission_active:
                response.success = False
                response.message = "巡检执行中不能重新加载路线"
                return response
        try:
            self.route_book = self._load_and_validate_route_book()
            response.success = True
            response.message = "巡检路线已重新加载并通过地图安全校验"
        except RouteConfigError as exc:
            response.success = False
            response.message = str(exc)
        return response

    # =====================================================================
    # 启动定时器回调
    # =====================================================================
    def _initial_pose_timer(self) -> None:
        """
        启动 2 秒后自动设置一次 AMCL 初始位姿.

        只触发一次: 执行后取消定时器并清空标志.

        """
        if not self.auto_set_initial_pose:
            return
        self.auto_set_initial_pose = False  # 防止重复触发
        if self._initial_pose_startup_timer is not None:
            self._initial_pose_startup_timer.cancel()
        success, message = self._send_initial_pose()
        if success:
            self.get_logger().info(message)
        else:
            self.get_logger().error(message)

    def _auto_start_timer(self) -> None:
        """
        启动 5 秒后自动执行一次默认巡检路线.

        只触发一次: 执行后取消定时器并清空标志.

        """
        if not self.auto_start:
            return
        self.auto_start = False  # 防止重复触发
        if self._auto_startup_timer is not None:
            self._auto_startup_timer.cancel()
        # 动态创建 Response 对象(不需要完整导入 Trigger.Response)
        response = type("Response", (), {})()
        self._start_default_callback(None, response)
        self.get_logger().info(response.message)

    # =====================================================================
    # AMCL 初始位姿
    # =====================================================================
    def _send_initial_pose(self):
        """
        向 AMCL 发送 RouteBook 中配置的初始位姿.

        尝试两种方式(按优先级): 
        1. 调用 /set_initial_pose 服务(AMCL 标准接口)
        2. 发布到 initial_pose_topic 话题(兼容备用方式)

        返回:
            (success: bool, message: str) 元组

        """
        initial_pose = self.route_book.initial_pose
        if initial_pose is None:
            return False, "路线文件没有配置 initial_pose"

        # 构造 PoseWithCovarianceStamped 消息
        message = PoseWithCovarianceStamped()
        message.header.frame_id = self.route_book.frame_id
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.pose.position.x = initial_pose.x
        message.pose.pose.position.y = initial_pose.y
        # 偏航角 → 四元数: z = sin(yaw/2), w = cos(yaw/2)
        message.pose.pose.orientation.z = math.sin(initial_pose.yaw * 0.5)
        message.pose.pose.orientation.w = math.cos(initial_pose.yaw * 0.5)
        # 协方差矩阵对角元素
        # covariance 是 36 个 float 的一维数组(6×6 矩阵展平)
        # 索引 0, 7, 14, 21, 28, 35 是对角线元素
        message.pose.covariance[0] = initial_pose.covariance_x    # X 方差
        message.pose.covariance[7] = initial_pose.covariance_y    # Y 方差
        message.pose.covariance[35] = initial_pose.covariance_yaw # Yaw 方差

        self._set_state(MissionState.INITIALIZING, "正在设置 AMCL 初始位姿")

        # 方式 1: 尝试调用服务
        if self.initial_pose_client.wait_for_service(timeout_sec=0.2):
            request = SetInitialPose.Request()
            request.pose = message
            self.initial_pose_client.call_async(request)
            result = "AMCL 初始位姿服务请求已提交"
            self._set_state(MissionState.IDLE, result)
            return True, result

        # 方式 2: 服务不可用 → 发布话题(Humble 常见做法)
        self.initial_pose_publisher.publish(message)
        result = f"AMCL 初始位姿已发布到 {self.initial_pose_topic} 话题"
        self._set_state(MissionState.IDLE, result)
        return True, result

    # =====================================================================
    # 气体传感器回调
    # =====================================================================
    def _risk_event_callback(self, event: RiskEvent) -> None:
        """
        统计风险事件, 并在配置要求时触发严重风险安全停机.

        风险事件来自气体传感器驱动节点, 通过 /gas/risk_event 话题发布.
        每个事件包含: 
        - event_id: 事件唯一标识
        - gas_type: 气体类型
        - risk_level: 风险等级(数值越大越严重)
        - message: 人类可读的描述

        安全停机逻辑: 
        1. 检查是否在巡检任务中(不在则忽略)
        2. 累计风险事件计数
        3. 检查气体类型是否匹配当前任务的目标气体
        4. 如果风险等级 >= critical_risk_level, 且 stop_on_critical_risk 为 True
           → 触发安全停机

        参数:
            event: RiskEvent 消息

        """
        if not self._mission_active:
            return

        self._risk_event_count += 1
        self._current_risk_level = int(event.risk_level)

        # 检查气体类型是否匹配(大小写不敏感)
        # casefold() 是比 lower() 更激进的转换, 适合多语言文本比较
        event_matches_target = (
            not self._target_gas  # 未指定目标气体 → 所有事件都匹配
            or event.gas_type.casefold() == self._target_gas.casefold()
        )

        if (
            self._stop_on_critical_risk
            and event_matches_target
            and event.risk_level >= self.critical_risk_level
        ):
            # 触发安全停机
            self._safety_stop_requested = True
            self._set_state(
                MissionState.SAFETY_STOP,
                f"严重气体风险触发停机: {event.event_id}",
            )
            self._cancel_current_nav_goal()

    def _gas_readings_callback(self, message: GasSensorArray) -> None:
        """
        提取当前任务目标气体的浓度数据, 供 Action Feedback 和后端显示.

        气体传感器节点应独立、连续发布 GasSensorArray 消息，不能等机器人
        到达巡检点后才采样。航点只约束运动路线，实际采样位置由消息时间戳
        对应的 TF 历史位姿确定。

        GasSensorArray 可以包含多种气体的读数.
        这个回调只提取与当前任务目标气体匹配的读数.

        如果浓度超过报警阈值, 自动提升当前风险等级(至少为 1), 
        确保通过 Action Feedback 通知到调用方.

        参数:
            message: GasSensorArray 消息, 包含多个 GasReading

        """
        if not self._mission_active or not self._target_gas:
            return

        target = self._target_gas.casefold()
        # 列表推导式: 从所有读数中筛选匹配目标气体且有效的浓度值
        matching = [
            float(reading.concentration)
            for reading in message.readings
            if reading.valid  # 只考虑传感器确认有效的读数
            and reading.gas_type.casefold() == target
        ]
        if matching:
            # 取最大浓度值(如果有多个同类型传感器)
            self._current_concentration = max(matching)
            # 超过报警阈值 → 至少标记为风险等级 1
            if self._current_concentration >= self._alarm_threshold:
                self._current_risk_level = max(self._current_risk_level, 1)

    # =====================================================================
    # 取消与停机处理
    # =====================================================================
    def _request_cancel(self, detail: str) -> None:
        """
        记录取消状态并请求 Nav2 停止当前导航目标.

        参数:
            detail: 取消原因的详细描述(用于状态消息)

        """
        self._cancel_requested = True
        self._set_state(MissionState.CANCELLING, detail)
        self._cancel_current_nav_goal()

    def _cancel_current_nav_goal(self) -> None:
        """
        如果存在活动的 Nav2 导航目标, 异步提交取消请求.

        取消是异步的: cancel_goal_async() 发送取消请求后立即返回, 
        实际取消在 Nav2 端异步完成.

        """
        with self._state_lock:
            nav_goal = self._current_nav_goal
        if nav_goal is not None:
            nav_goal.cancel_goal_async()

    # =====================================================================
    # 消息构造辅助方法
    # =====================================================================
    def _pose_stamped(self, waypoint) -> PoseStamped:
        """
        把二维巡检航点转换为 Nav2 使用的 PoseStamped 消息.

        PoseStamped = Pose(位姿)+ Header(时间戳+坐标系)
        Nav2 需要知道目标位姿在哪个坐标系中(通常是 "map").

        参数:
            waypoint: InspectionWaypoint 对象

        返回:
            PoseStamped 消息

        """
        pose = PoseStamped()
        pose.header.frame_id = self.route_book.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = waypoint.x
        pose.pose.position.y = waypoint.y
        # 偏航角 → 四元数(2D 简化公式)
        pose.pose.orientation.z = math.sin(waypoint.yaw * 0.5)
        pose.pose.orientation.w = math.cos(waypoint.yaw * 0.5)
        return pose

    def _reset_mission_counters(self) -> None:
        """
        开始新任务前清空所有进度、风险和控制标志.

        确保前一次任务的状态不会泄漏影响新任务.

        """
        self._paused = False
        self._cancel_requested = False
        self._safety_stop_requested = False
        self._risk_event_count = 0
        self._completed_waypoints = 0
        self._current_waypoint_id = ""
        self._current_concentration = 0.0
        self._current_risk_level = 0
        self._publish_active(True)

    # =====================================================================
    # Action 结果构造
    # =====================================================================
    def _finish_cancelled(self, result, goal_handle=None):
        """
        根据安全停机或普通取消生成统一的取消结果.

        - 安全停机: 状态 = SAFETY_STOP, 调用 goal_handle.abort()
        - 普通取消: 状态 = CANCELLED, 调用 goal_handle.canceled()

        """
        if self._safety_stop_requested:
            state = MissionState.SAFETY_STOP
            message = "严重气体风险触发巡检停止"
            if goal_handle is not None:
                goal_handle.abort()  # abort 表示异常中止
        else:
            state = MissionState.CANCELLED
            message = "巡检任务已取消"
            if goal_handle is not None:
                goal_handle.canceled()  # canceled 表示正常取消
        self._set_state(state, message)
        return self._fill_result(result, False, message)

    def _finish_result(self, result, goal_handle, success, message):
        """
        结束成功或失败任务, 更新 Action 终态并填充结果.

        参数:
            result:      ExecuteInspection.Result 对象
            goal_handle: Action 目标句柄
            success:     True=成功, False=失败
            message:     结果描述

        返回:
            填充后的 result

        """
        self._set_state(
            MissionState.COMPLETED if success else MissionState.FAILED,
            message,
        )
        if goal_handle is not None:
            # 根据成功/失败调用相应的 Action 终止方法
            goal_handle.succeed() if success else goal_handle.abort()
        return self._fill_result(result, success, message)

    def _fill_result(self, result, success, message):
        """
        填充 ExecuteInspection.Result 的公共字段.

        参数:
            result:  ExecuteInspection.Result 对象
            success: True/False
            message: 结果消息

        返回:
            填充后的 result

        """
        result.success = success
        result.message = message
        result.completed_waypoints = self._completed_waypoints
        result.risk_event_count = self._risk_event_count
        return result

    # =====================================================================
    # Action Feedback 发布
    # =====================================================================
    def _publish_feedback(self, goal_handle, total_waypoints) -> None:
        """
        向 Action 调用方发布任务进度和当前风险统计.

        Action Feedback 让调用方可以实时了解任务进展: 
        - 已完成多少航点 / 总共多少航点
        - 当前气体浓度和风险等级
        - 累计风险事件数
        - 当前任务状态

        参数:
            goal_handle:     Action 目标句柄
            total_waypoints: 总航点数

        """
        if goal_handle is None:
            return
        feedback = ExecuteInspection.Feedback()
        feedback.current_waypoint = self._completed_waypoints
        feedback.total_waypoints = total_waypoints
        feedback.current_concentration = self._current_concentration
        feedback.current_risk_level = self._current_risk_level
        feedback.risk_event_count = self._risk_event_count
        feedback.state = self._state.value
        goal_handle.publish_feedback(feedback)

    # =====================================================================
    # 状态发布
    # =====================================================================
    def _set_state(self, state: MissionState, detail: str) -> None:
        """
        线程安全地原子更新并发布任务状态.

        参数:
            state:  新状态(MissionState 枚举值)
            detail: 人类可读的详细信息

        """
        with self._state_lock:
            self._state = state
            self._detail = detail
        self._publish_state()

    def _publish_state(self) -> None:
        """
        发布状态文本消息, 格式为 "STATE|详细信息".

        这个格式便于后端和命令行工具解析: 
        - 按 "|" 分割可以得到状态码和详情
        - 状态码是固定的枚举值, 适合做状态机判断

        安全检查: 如果 ROS 上下文已经失效(节点正在关闭), 
        不再发布消息, 避免 RCLError.

        """
        # self.context.ok(): 检查 ROS 2 上下文是否正常
        # 在节点关闭过程中(收到 SIGINT/SIGTERM), 上下文会先失效
        if not self.context.ok():
            return
        message = String()
        message.data = f"{self._state.value}|{self._detail}"
        self.state_publisher.publish(message)

    def _publish_active(self, active: bool) -> None:
        """
        发布巡检任务是否处于活跃状态.

        参数:
            active: True=有任务执行中, False=空闲

        """
        if not self.context.ok():
            return
        message = Bool()
        message.data = active
        self.active_publisher.publish(message)

    def _publish_waypoint(self, waypoint) -> None:
        """
        发布当前正在执行的航点 ID.

        参数:
            waypoint: InspectionWaypoint 对象

        """
        if not self.context.ok():
            return
        message = String()
        message.data = waypoint.waypoint_id
        self.waypoint_publisher.publish(message)

    # =====================================================================
    # 非阻塞延时(Action 协程专用)
    # =====================================================================
    def _sleep(self, duration_sec: float) -> Future:
        """
        创建由 ROS 2 定时器完成的 Future, 供 Action 协程非阻塞等待.

        为什么不用 time.sleep()？
        - time.sleep() 会阻塞整个线程
        - 在多线程执行器中, 虽然可以接受, 但不够优雅
        - ROS 2 定时器可以与执行器的事件循环正确集成

        工作原理: 
        1. 创建一个 Future 对象
        2. 创建一个单次触发的 ROS 2 定时器
        3. 定时器到期时, 完成 Future
        4. await future → 等待定时器触发

        参数:
            duration_sec: 等待时长(秒), 最小 0.001 秒

        返回:
            一个在 duration_sec 后完成的 Future 对象

        """
        future = Future(executor=self.executor)
        # 用字典包装定时器引用, 方便在回调中取消
        timer_holder = {}

        def finish_wait() -> None:
            """定时器回调: 取消定时器并完成 Future."""
            timer = timer_holder["timer"]
            timer.cancel()            # 取消定时器(单次触发后自动取消)
            self.destroy_timer(timer) # 销毁定时器释放资源
            if not future.done():
                future.set_result(None)  # 完成 Future(await 解除阻塞)

        # 创建单次定时器(到期后回调 finish_wait)
        # max(0.001, duration_sec): 最小延时 0.001 秒, 避免传入 0 或负数
        timer_holder["timer"] = self.create_timer(
            max(0.001, duration_sec),
            finish_wait,
            callback_group=self.callback_group,
        )
        return future

    # =====================================================================
    # 节点销毁
    # =====================================================================
    def destroy_node(self) -> bool:
        """
        销毁节点前安全取消 Action 服务和当前导航目标.

        重写父类的 destroy_node() 方法, 添加自定义清理逻辑.
        在 ROS 2 关闭流程中, 节点按依赖关系销毁.

        返回:
            True: 销毁成功

        """
        if self.context.ok():
            self._request_cancel("节点正在关闭")
        else:
            # 上下文已失效时直接设置标志, 不能调用需要上下文的方法
            self._cancel_requested = True
        # 销毁 Action 服务器, 释放资源
        self.inspection_server.destroy()
        # 调用父类 destroy_node() 完成标准销毁流程
        return super().destroy_node()


# =========================================================================
# main: 程序入口
# =========================================================================
def main(args=None) -> None:
    """
    使用多线程执行器启动巡检任务管理节点.

    使用 MultiThreadedExecutor(num_threads=4) 而不是默认的单线程执行器, 
    原因: 
    1. Action 回调涉及大量异步等待(await), 单线程容易阻塞
    2. 气体传感器回调需要实时响应, 不应被导航操作阻塞
    3. 多个服务回调可能同时触发, 需要并发处理
    4 个线程提供了足够的并发能力, 同时不会带来过多的线程开销.

    参数:
        args: 命令行参数(默认 None 使用 sys.argv)

    """
    rclpy.init(args=args)
    node = InspectionManager()

    # 创建多线程执行器: 4 个线程同时处理回调
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        # 启动执行器的事件循环(阻塞)
        executor.spin()
    except KeyboardInterrupt:
        # Ctrl+C: 正常退出
        pass
    finally:
        # 安全清理
        executor.shutdown()       # 停止执行器
        node.destroy_node()       # 销毁节点
        if rclpy.ok():
            rclpy.shutdown()      # 关闭 ROS 2


# =========================================================================
# 直接执行入口
# =========================================================================
if __name__ == "__main__":
    main()
