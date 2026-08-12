#!/usr/bin/env python3
"""基于 Nav2 单点动作的自主气体巡检任务管理节点。"""

import math
import threading
import time
from enum import Enum
from typing import List

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import SetInitialPose
from rcl_interfaces.msg import SetParametersResult
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.task import Future
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger

from gasrobot_interfaces.action import ExecuteInspection
from gasrobot_interfaces.msg import GasSensorArray, RiskEvent
from gasrobot_navigation.route_config import (
    InspectionRoute,
    InspectionWaypoint,
    RouteBook,
    RouteConfigError,
    load_route_book,
)


class MissionState(str, Enum):
    """巡检任务对外发布的稳定状态集合。"""

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


class InspectionManager(Node):
    """加载命名路线，向 Nav2 逐点派发目标并管理任务生命周期。"""

    def __init__(self) -> None:
        """初始化配置、Action、服务、话题和运行状态。"""

        super().__init__("inspection_manager")
        self.callback_group = ReentrantCallbackGroup()

        self.declare_parameter("route_file", "")
        self.declare_parameter("default_route", "standard_route")
        self.declare_parameter("auto_set_initial_pose", True)
        self.declare_parameter("auto_start", False)
        self.declare_parameter("nav2_wait_timeout_sec", 60.0)
        self.declare_parameter("critical_risk_level", 3)
        self.declare_parameter("initial_pose_topic", "/initialpose")

        self.route_file = str(self.get_parameter("route_file").value)
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

        self.route_book = load_route_book(self.route_file)
        self._validate_site_configuration(self.route_book)
        self.route_book.route(self.default_route)

        # Nav2 客户端负责实际路径规划和运动控制，本节点只负责业务任务编排。
        self.navigation_client = ActionClient(
            self,
            NavigateToPose,
            "navigate_to_pose",
            callback_group=self.callback_group,
        )
        self.inspection_client = ActionClient(
            self,
            ExecuteInspection,
            "execute_inspection",
            callback_group=self.callback_group,
        )
        self.initial_pose_client = self.create_client(
            SetInitialPose,
            "/set_initial_pose",
            callback_group=self.callback_group,
        )

        # ExecuteInspection 允许后端直接下发临时路线，不依赖修改本地 YAML。
        self.inspection_server = ActionServer(
            self,
            ExecuteInspection,
            "execute_inspection",
            execute_callback=self._execute_action,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self.callback_group,
        )

        self.start_service = self.create_service(
            Trigger,
            "~/start_default",
            self._start_default_callback,
            callback_group=self.callback_group,
        )
        self.cancel_service = self.create_service(
            Trigger,
            "~/cancel",
            self._cancel_service_callback,
            callback_group=self.callback_group,
        )
        self.pause_service = self.create_service(
            SetBool,
            "~/pause",
            self._pause_callback,
            callback_group=self.callback_group,
        )
        self.initialize_service = self.create_service(
            Trigger,
            "~/set_initial_pose",
            self._set_initial_pose_callback,
            callback_group=self.callback_group,
        )
        self.reload_service = self.create_service(
            Trigger,
            "~/reload_routes",
            self._reload_routes_callback,
            callback_group=self.callback_group,
        )

        # 可靠、瞬态本地 QoS 让后启动的界面也能立即获得最近任务状态。
        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.state_publisher = self.create_publisher(
            String, "~/state", state_qos
        )
        self.active_publisher = self.create_publisher(
            Bool, "~/active", state_qos
        )
        self.waypoint_publisher = self.create_publisher(
            String, "~/current_waypoint", 10
        )
        self.initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            self.initial_pose_topic,
            10,
        )
        self.risk_subscription = self.create_subscription(
            RiskEvent,
            "/gas/risk_event",
            self._risk_event_callback,
            20,
            callback_group=self.callback_group,
        )
        self.gas_subscription = self.create_subscription(
            GasSensorArray,
            "/gas/readings",
            self._gas_readings_callback,
            20,
            callback_group=self.callback_group,
        )

        self._state_lock = threading.RLock()
        self._mission_active = False
        self._paused = False
        self._cancel_requested = False
        self._safety_stop_requested = False
        self._current_nav_goal = None
        self._risk_event_count = 0
        self._completed_waypoints = 0
        self._current_waypoint_id = ""
        self._target_gas = ""
        self._alarm_threshold = 0.0
        self._current_concentration = 0.0
        self._current_risk_level = 0
        self._stop_on_critical_risk = False
        self._state = MissionState.IDLE
        self._detail = "节点已启动，等待巡检任务"
        self._publish_state()

        self.add_on_set_parameters_callback(self._parameter_callback)
        self._initial_pose_startup_timer = None
        self._auto_startup_timer = None
        if self.auto_set_initial_pose:
            self._initial_pose_startup_timer = self.create_timer(
                2.0, self._initial_pose_timer
            )
        if self.auto_start:
            self._auto_startup_timer = self.create_timer(
                5.0, self._auto_start_timer
            )

        self.get_logger().info(
            f"巡检任务管理节点已启动：路线文件={self.route_file}，"
            f"默认路线={self.default_route}"
        )

    @staticmethod
    def _validate_site_configuration(route_book: RouteBook) -> None:
        """拒绝把模板零坐标误用于真实巡检。"""

        if not route_book.site_configured:
            raise RouteConfigError(
                "巡检点仍是模板值：请标定 routes.yaml 后将 "
                "site_configured 改为 true"
            )

    def _parameter_callback(self, parameters: List[Parameter]):
        """仅允许空闲状态下切换默认路线。"""

        for parameter in parameters:
            if parameter.name != "default_route":
                continue
            with self._state_lock:
                if self._mission_active:
                    return SetParametersResult(
                        successful=False,
                        reason="巡检执行中不能切换默认路线",
                    )
            try:
                self.route_book.route(str(parameter.value))
            except RouteConfigError as exc:
                return SetParametersResult(successful=False, reason=str(exc))
            self.default_route = str(parameter.value)
        return SetParametersResult(successful=True)

    def _goal_callback(self, _goal_request) -> GoalResponse:
        """同一时刻只接受一项巡检任务。"""

        with self._state_lock:
            if self._mission_active:
                self.get_logger().warning("已有巡检任务运行，拒绝新任务")
                return GoalResponse.REJECT
            self._mission_active = True
            self._reset_mission_counters()
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle) -> CancelResponse:
        """接受任务取消，并转发给当前 Nav2 目标。"""

        self._request_cancel("收到 Action 取消请求")
        return CancelResponse.ACCEPT

    async def _execute_action(self, goal_handle):
        """执行后端通过 Action 下发的临时巡检路线。"""

        try:
            route_name = str(goal_handle.request.route_name).strip()
            if route_name:
                route = self.route_book.route(route_name)
            else:
                route = self._route_from_action_goal(goal_handle.request)
        except RouteConfigError as exc:
            result = ExecuteInspection.Result()
            goal_handle.abort()
            with self._state_lock:
                self._mission_active = False
            self._set_state(MissionState.FAILED, str(exc))
            self._publish_active(False)
            return self._fill_result(result, False, str(exc))
        return await self._execute_route(route, goal_handle)

    def _route_from_action_goal(self, request) -> InspectionRoute:
        """把 ExecuteInspection 目标转换为内部路线模型。"""

        if not request.waypoints:
            raise RouteConfigError("Action 巡检路线至少需要一个目标点")
        waypoints = []
        default_dwell = max(0.0, float(request.default_dwell_sec))
        for index, pose in enumerate(request.waypoints):
            quaternion = pose.pose.orientation
            yaw = math.atan2(
                2.0 * (quaternion.w * quaternion.z),
                1.0 - 2.0 * quaternion.z * quaternion.z,
            )
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
        return InspectionRoute(
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
                else 300.0
            ),
            waypoints=waypoints,
        )

    async def _execute_route(self, route, goal_handle=None):
        """按圈次和航点顺序执行路线，统一处理失败、取消和安全停机。"""

        result = ExecuteInspection.Result()
        try:
            self._target_gas = route.target_gas
            self._alarm_threshold = route.alarm_threshold
            self._stop_on_critical_risk = route.stop_on_critical_risk
            self._set_state(
                MissionState.WAITING_NAV2,
                f"等待 Nav2，准备执行路线“{route.name}”",
            )
            if not await self._wait_for_nav2():
                return self._finish_result(
                    result, goal_handle, False, "等待 Nav2 超时"
                )

            for repeat_index in range(route.repeat_count):
                for waypoint_index, waypoint in enumerate(route.waypoints):
                    if self._must_stop(goal_handle):
                        return self._finish_cancelled(result, goal_handle)

                    succeeded = await self._navigate_with_retries(
                        route,
                        waypoint,
                        repeat_index,
                        waypoint_index,
                        goal_handle,
                    )
                    if not succeeded:
                        if self._must_stop(goal_handle):
                            return self._finish_cancelled(result, goal_handle)
                        if route.continue_on_failure:
                            self.get_logger().warning(
                                f"跳过失败巡检点：{waypoint.waypoint_id}"
                            )
                            continue
                        return self._finish_result(
                            result,
                            goal_handle,
                            False,
                            f"巡检点 {waypoint.waypoint_id} 导航失败",
                        )

                    self._completed_waypoints += 1
                    self._publish_feedback(
                        goal_handle,
                        len(route.waypoints) * route.repeat_count,
                    )
                    if waypoint.dwell_sec > 0.0:
                        self._set_state(
                            MissionState.DWELLING,
                            f"在 {waypoint.waypoint_id} 停留采样",
                        )
                        if not await self._interruptible_wait(
                            waypoint.dwell_sec,
                            goal_handle,
                        ):
                            return self._finish_cancelled(result, goal_handle)

            return self._finish_result(
                result,
                goal_handle,
                True,
                f"路线“{route.name}”巡检完成",
            )
        except Exception as exc:  # 节点边界必须把异常转换成可诊断任务结果。
            self.get_logger().exception(f"巡检任务异常：{exc}")
            return self._finish_result(result, goal_handle, False, str(exc))
        finally:
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
        """向 Nav2 发送单点目标，并按路线策略进行有限重试。"""

        total_waypoints = len(route.waypoints) * route.repeat_count
        self._current_waypoint_id = waypoint.waypoint_id
        self._publish_waypoint(waypoint)
        attempt = 0
        while attempt <= route.max_retries:
            if not await self._wait_while_paused(goal_handle):
                return False
            sequence = repeat_index * len(route.waypoints) + waypoint_index + 1
            self._set_state(
                MissionState.NAVIGATING,
                f"前往 {waypoint.waypoint_id}，进度 {sequence}/{total_waypoints}，"
                f"尝试 {attempt + 1}/{route.max_retries + 1}",
            )
            self._publish_feedback(goal_handle, total_waypoints)
            status = await self._navigate_once(
                waypoint,
                route.navigation_timeout_sec,
                goal_handle,
            )
            if status == GoalStatus.STATUS_SUCCEEDED:
                return True
            if self._must_stop(goal_handle):
                return False
            if self._paused:
                # 暂停不属于导航失败，继续后仍使用原重试次数。
                if not await self._wait_while_paused(goal_handle):
                    return False
                continue
            self.get_logger().warning(
                f"巡检点 {waypoint.waypoint_id} 导航未成功，状态={status}"
            )
            attempt += 1
        return False

    async def _navigate_once(self, waypoint, timeout_sec, goal_handle) -> int:
        """执行一次 NavigateToPose，并监视超时、暂停、取消和风险停机。"""

        goal = NavigateToPose.Goal()
        goal.pose = self._pose_stamped(waypoint)
        future = self.navigation_client.send_goal_async(goal)
        nav_goal = await future
        if not nav_goal.accepted:
            return GoalStatus.STATUS_ABORTED

        with self._state_lock:
            self._current_nav_goal = nav_goal
        result_future = nav_goal.get_result_async()
        deadline = time.monotonic() + timeout_sec
        try:
            while not result_future.done():
                if self._must_stop(goal_handle) or self._paused:
                    await nav_goal.cancel_goal_async()
                    if self._paused and not self._must_stop(goal_handle):
                        self._set_state(
                            MissionState.PAUSED,
                            f"已暂停，将在继续后重新导航到 {waypoint.waypoint_id}",
                        )
                    return GoalStatus.STATUS_CANCELED
                if time.monotonic() >= deadline:
                    self.get_logger().error(
                        f"巡检点 {waypoint.waypoint_id} 超过 {timeout_sec:.1f}s"
                    )
                    await nav_goal.cancel_goal_async()
                    return GoalStatus.STATUS_ABORTED
                await self._sleep(0.1)
            return (await result_future).status
        finally:
            with self._state_lock:
                self._current_nav_goal = None

    async def _wait_for_nav2(self) -> bool:
        """异步等待 Nav2 动作服务器，期间保持节点可响应取消。"""

        deadline = time.monotonic() + self.nav2_wait_timeout_sec
        while time.monotonic() < deadline:
            if self.navigation_client.server_is_ready():
                return True
            if self._cancel_requested:
                return False
            await self._sleep(0.2)
        return False

    async def _wait_while_paused(self, goal_handle) -> bool:
        """暂停时保持任务上下文，继续后从当前巡检点重新发起导航。"""

        while self._paused and not self._must_stop(goal_handle):
            self._set_state(MissionState.PAUSED, "巡检任务已暂停")
            await self._sleep(0.1)
        return not self._must_stop(goal_handle)

    async def _interruptible_wait(self, duration, goal_handle) -> bool:
        """执行可被暂停、取消或安全事件打断的航点停留。"""

        remaining = duration
        previous = time.monotonic()
        while remaining > 0.0:
            if self._must_stop(goal_handle):
                return False
            if self._paused:
                if not await self._wait_while_paused(goal_handle):
                    return False
                previous = time.monotonic()
            await self._sleep(min(0.1, remaining))
            now = time.monotonic()
            remaining -= now - previous
            previous = now
        return True

    def _must_stop(self, goal_handle=None) -> bool:
        """返回任务是否收到取消或严重风险停机请求。"""

        return (
            self._cancel_requested
            or self._safety_stop_requested
            or (goal_handle is not None and goal_handle.is_cancel_requested)
        )

    def _start_default_callback(self, _request, response):
        """从服务启动配置文件中的默认路线。"""

        if self._mission_active:
            response.success = False
            response.message = "已有巡检任务正在运行"
            return response
        if not self.inspection_client.server_is_ready():
            response.success = False
            response.message = "巡检 Action 服务尚未就绪，请稍后重试"
            return response

        route = self.route_book.route(self.default_route)
        goal = ExecuteInspection.Goal()
        goal.route_name = route.name
        future = self.inspection_client.send_goal_async(goal)
        future.add_done_callback(self._default_goal_response_callback)
        response.success = True
        response.message = f"默认路线启动请求已提交：{route.name}"
        return response

    def _default_goal_response_callback(self, future) -> None:
        """记录默认路线的内部 Action 请求是否被接受。"""

        try:
            if not future.result().accepted:
                self.get_logger().error("默认路线启动请求被拒绝")
        except Exception as exc:
            self.get_logger().error(f"默认路线启动失败：{exc}")

    def _cancel_service_callback(self, _request, response):
        """通过服务取消当前巡检任务。"""

        if not self._mission_active:
            response.success = False
            response.message = "当前没有巡检任务"
            return response
        self._request_cancel("收到取消服务请求")
        response.success = True
        response.message = "取消请求已提交"
        return response

    def _pause_callback(self, request, response):
        """暂停或继续巡检；暂停会取消当前 Nav2 目标并保留任务进度。"""

        if not self._mission_active:
            response.success = False
            response.message = "当前没有巡检任务"
            return response
        if bool(request.data) == self._paused:
            response.success = True
            response.message = (
                "巡检任务已暂停" if self._paused else "巡检任务正在运行"
            )
            return response
        self._paused = bool(request.data)
        if self._paused:
            self._cancel_current_nav_goal()
            response.message = "巡检任务正在暂停"
        else:
            response.message = "巡检任务继续执行"
        response.success = True
        return response

    def _set_initial_pose_callback(self, _request, response):
        """通过服务向 AMCL 发送配置文件中的初始化位姿。"""

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
        """在任务空闲时重新读取路线文件，支持现场调整巡检点。"""

        with self._state_lock:
            if self._mission_active:
                response.success = False
                response.message = "巡检执行中不能重新加载路线"
                return response
        try:
            route_book = load_route_book(self.route_file)
            self._validate_site_configuration(route_book)
            route_book.route(self.default_route)
            self.route_book = route_book
            response.success = True
            response.message = "巡检路线已重新加载"
        except RouteConfigError as exc:
            response.success = False
            response.message = str(exc)
        return response

    def _initial_pose_timer(self) -> None:
        """启动后自动设置一次初始化位姿。"""

        if not self.auto_set_initial_pose:
            return
        self.auto_set_initial_pose = False
        if self._initial_pose_startup_timer is not None:
            self._initial_pose_startup_timer.cancel()
        success, message = self._send_initial_pose()
        if success:
            self.get_logger().info(message)
        else:
            self.get_logger().error(message)

    def _auto_start_timer(self) -> None:
        """启动后自动执行一次默认路线。"""

        if not self.auto_start:
            return
        self.auto_start = False
        if self._auto_startup_timer is not None:
            self._auto_startup_timer.cancel()
        response = type("Response", (), {})()
        self._start_default_callback(None, response)
        self.get_logger().info(response.message)

    def _send_initial_pose(self):
        """调用 AMCL 初始化服务，并返回提交结果。"""

        initial_pose = self.route_book.initial_pose
        if initial_pose is None:
            return False, "路线文件没有配置 initial_pose"
        message = PoseWithCovarianceStamped()
        message.header.frame_id = self.route_book.frame_id
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.pose.position.x = initial_pose.x
        message.pose.pose.position.y = initial_pose.y
        message.pose.pose.orientation.z = math.sin(initial_pose.yaw * 0.5)
        message.pose.pose.orientation.w = math.cos(initial_pose.yaw * 0.5)
        message.pose.covariance[0] = initial_pose.covariance_x
        message.pose.covariance[7] = initial_pose.covariance_y
        message.pose.covariance[35] = initial_pose.covariance_yaw
        self._set_state(MissionState.INITIALIZING, "正在设置 AMCL 初始位姿")
        if self.initial_pose_client.wait_for_service(timeout_sec=0.2):
            request = SetInitialPose.Request()
            request.pose = message
            self.initial_pose_client.call_async(request)
            result = "AMCL 初始位姿服务请求已提交"
            self._set_state(MissionState.IDLE, result)
            return True, result

        # Humble 的 AMCL 通常提供 /initialpose 话题；保留服务路径兼容其他版本。
        self.initial_pose_publisher.publish(message)
        result = f"AMCL 初始位姿已发布到 {self.initial_pose_topic}"
        self._set_state(MissionState.IDLE, result)
        return True, result

    def _risk_event_callback(self, event: RiskEvent) -> None:
        """统计风险事件，并在配置要求时触发严重风险安全停机。"""

        if not self._mission_active:
            return
        self._risk_event_count += 1
        self._current_risk_level = int(event.risk_level)
        event_matches_target = (
            not self._target_gas
            or event.gas_type.casefold() == self._target_gas.casefold()
        )
        if (
            self._stop_on_critical_risk
            and event_matches_target
            and event.risk_level >= self.critical_risk_level
        ):
            self._safety_stop_requested = True
            self._set_state(
                MissionState.SAFETY_STOP,
                f"严重气体风险触发停机：{event.event_id}",
            )
            self._cancel_current_nav_goal()

    def _gas_readings_callback(self, message: GasSensorArray) -> None:
        """提取当前任务目标气体浓度，供 Action 反馈和后端显示。"""

        if not self._mission_active or not self._target_gas:
            return
        target = self._target_gas.casefold()
        matching = [
            float(reading.concentration)
            for reading in message.readings
            if reading.valid and reading.gas_type.casefold() == target
        ]
        if matching:
            self._current_concentration = max(matching)
            if self._current_concentration >= self._alarm_threshold:
                self._current_risk_level = max(self._current_risk_level, 1)

    def _request_cancel(self, detail: str) -> None:
        """记录取消状态并请求 Nav2 停止当前目标。"""

        self._cancel_requested = True
        self._set_state(MissionState.CANCELLING, detail)
        self._cancel_current_nav_goal()

    def _cancel_current_nav_goal(self) -> None:
        """若存在活动 Nav2 目标，则异步提交取消。"""

        with self._state_lock:
            nav_goal = self._current_nav_goal
        if nav_goal is not None:
            nav_goal.cancel_goal_async()

    def _pose_stamped(self, waypoint) -> PoseStamped:
        """把二维巡检点转换为 Nav2 使用的 PoseStamped。"""

        pose = PoseStamped()
        pose.header.frame_id = self.route_book.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = waypoint.x
        pose.pose.position.y = waypoint.y
        pose.pose.orientation.z = math.sin(waypoint.yaw * 0.5)
        pose.pose.orientation.w = math.cos(waypoint.yaw * 0.5)
        return pose

    def _reset_mission_counters(self) -> None:
        """开始新任务前清空进度、风险和控制标志。"""

        self._paused = False
        self._cancel_requested = False
        self._safety_stop_requested = False
        self._risk_event_count = 0
        self._completed_waypoints = 0
        self._current_waypoint_id = ""
        self._current_concentration = 0.0
        self._current_risk_level = 0
        self._publish_active(True)

    def _finish_cancelled(self, result, goal_handle=None):
        """根据安全停机或普通取消生成统一结果。"""

        if self._safety_stop_requested:
            state = MissionState.SAFETY_STOP
            message = "严重气体风险触发巡检停止"
            if goal_handle is not None:
                goal_handle.abort()
        else:
            state = MissionState.CANCELLED
            message = "巡检任务已取消"
            if goal_handle is not None:
                goal_handle.canceled()
        self._set_state(state, message)
        return self._fill_result(result, False, message)

    def _finish_result(self, result, goal_handle, success, message):
        """结束成功或失败任务，并更新 Action 终态。"""

        self._set_state(
            MissionState.COMPLETED if success else MissionState.FAILED,
            message,
        )
        if goal_handle is not None:
            goal_handle.succeed() if success else goal_handle.abort()
        return self._fill_result(result, success, message)

    def _fill_result(self, result, success, message):
        """填充 ExecuteInspection 结果的公共字段。"""

        result.success = success
        result.message = message
        result.completed_waypoints = self._completed_waypoints
        result.risk_event_count = self._risk_event_count
        return result

    def _publish_feedback(self, goal_handle, total_waypoints) -> None:
        """向 Action 调用方发布任务进度和风险统计。"""

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

    def _set_state(self, state: MissionState, detail: str) -> None:
        """原子更新并发布任务状态。"""

        with self._state_lock:
            self._state = state
            self._detail = detail
        self._publish_state()

    def _publish_state(self) -> None:
        """发布便于后端和命令行读取的状态文本。"""

        # 收到 SIGINT/SIGTERM 后 ROS 上下文可能先于节点析构失效。
        # 此时不再发布关机状态，避免正常退出被误报为 RCLError。
        if not self.context.ok():
            return
        message = String()
        message.data = f"{self._state.value}|{self._detail}"
        self.state_publisher.publish(message)

    def _publish_active(self, active: bool) -> None:
        """发布巡检任务是否活动。"""

        if not self.context.ok():
            return
        message = Bool()
        message.data = active
        self.active_publisher.publish(message)

    def _publish_waypoint(self, waypoint) -> None:
        """发布当前命名巡检点，供气体数据和后端记录关联。"""

        if not self.context.ok():
            return
        message = String()
        message.data = waypoint.waypoint_id
        self.waypoint_publisher.publish(message)

    def _sleep(self, duration_sec: float) -> Future:
        """创建由 ROS 定时器完成的 Future，供 Action 协程非阻塞等待。"""

        future = Future(executor=self.executor)
        timer_holder = {}

        def finish_wait() -> None:
            timer = timer_holder["timer"]
            timer.cancel()
            self.destroy_timer(timer)
            if not future.done():
                future.set_result(None)

        timer_holder["timer"] = self.create_timer(
            max(0.001, duration_sec),
            finish_wait,
            callback_group=self.callback_group,
        )
        return future

    def destroy_node(self) -> bool:
        """销毁节点前取消动作服务和当前导航目标。"""

        if self.context.ok():
            self._request_cancel("节点正在关闭")
        else:
            self._cancel_requested = True
        self.inspection_server.destroy()
        return super().destroy_node()


def main(args=None) -> None:
    """使用多线程执行器启动巡检任务管理节点。"""

    rclpy.init(args=args)
    node = InspectionManager()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
