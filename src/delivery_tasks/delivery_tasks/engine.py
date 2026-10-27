"""固定顺序配送状态机（不依赖 ROS，可用虚拟时钟进行单元测试）。

commands 是发给 ROS 适配层的动作队列，events 是发给日志层的事件队列。
active 保存唯一在途导航请求的令牌，包括“发送后尚未被服务器接受”的阶段。
暂停、取消和超时都先请求停止，等旧导航进入终态后才能发下一个目标。
因此，取消服务返回成功只表示请求已接收，不表示机器人已经停止。
"""


class DeliveryEngine:
    TERMINAL = {"IDLE", "COMPLETED", "CANCELLED", "FAILED"}

    def __init__(self, batch, depot):
        self.batch, self.depot = batch, depot
        self.state = "IDLE"
        self.index = 0
        self.completed = []
        self.retry = 0
        self.serial = 0
        self.active = None
        self.commands = []
        self.events = []
        self.started = 0.0
        self.last_time = None
        self.remaining_service = 0.0
        self.stop_reason = None
        self.paused_phase = None

    @property
    def target(self):
        return self.batch.route[self.index] if self.index < len(self.batch.route) else self.depot

    @property
    def busy(self):
        return self.state not in self.TERMINAL or self.active is not None

    def event(self, kind, now, **details):
        self.events.append(
            dict(
                event=kind,
                ros_time=now,
                elapsed=max(0, now - self.started),
                station=self.target,
                state=self.state,
                **details
            )
        )

    def start(self, now):
        """确认装货并创建一批任务；令牌计数不清零，以隔离上一批的迟到回调。"""
        if self.busy:
            return False, "Task already active or navigation cancellation pending"
        self.index, self.retry, self.completed = 0, 0, []
        self.state = "IDLE"
        self.started = now
        self.last_time = now
        self.stop_reason = None
        self.paused_phase = None
        self.remaining_service = 0.0
        self.event("departure", now)
        self.send(now)
        return True, "Loading confirmed; delivery started"

    def send(self, now):
        """排队发送当前工位，或所有工位完成后的物料站回程目标。"""
        assert self.active is None
        self.state = "NAVIGATING" if self.index < len(self.batch.route) else "RETURNING"
        self.serial += 1
        self.active = self.serial
        self.nav_started = now
        self.stop_reason = None
        self.commands.append(("navigate", self.active, self.target))
        self.event("navigation_started", now, token=self.active, retry=self.retry)

    def stop_navigation(self, reason, now):
        """建立取消屏障；调用者负责等到 nav_result 才允许继续。"""
        self.stop_reason = reason
        self.stop_started = now
        self.state = "PAUSING" if reason == "pause" else "CANCELLING"
        self.commands.append(("cancel", self.active, self.target))

    def pause(self, now):
        """导航暂停先取消目标；卸货暂停保存尚未消耗的服务时间。"""
        if self.state in ("NAVIGATING", "RETURNING"):
            self.paused_phase = self.state
            self.stop_navigation("pause", now)
        elif self.state == "SERVICING":
            self.tick(now)
            if self.state != "SERVICING":
                return self.pause(now)
            self.paused_phase = "SERVICING"
            self.state = "PAUSED"
        else:
            return False, "Task is not running"
        self.event("pause_requested", now)
        return True, "Pause requested; wait for PAUSED before resuming"

    def resume(self, now):
        """只允许完整暂停后继续；业务暂停时间仍计入整批 elapsed。"""
        if self.state != "PAUSED" or self.active is not None:
            return False, "Task is not fully paused"
        self.last_time = now
        self.event("resumed", now)
        if self.paused_phase == "SERVICING":
            self.state = "SERVICING"
        else:
            self.send(now)
        return True, "Resumed"

    def cancel(self, now):
        """取消整批任务；已完成的工位保留在日志中，不补记未完成工位。"""
        if not self.busy:
            return False, "No active task"
        self.event("cancel_requested", now)
        if self.active is not None:
            self.stop_navigation("cancel", now)
        else:
            self.finish("CANCELLED", now)
        return True, "Cancellation requested"

    def finish(self, state, now, reason=""):
        self.state = state
        self.event(state.lower(), now, reason=reason)

    def retry_or_fail(self, now, reason):
        if self.retry < self.batch.max_retries:
            self.retry += 1
            self.event("retry", now, reason=reason, retry=self.retry)
            self.send(now)
        else:
            self.finish("FAILED", now, reason)

    def nav_result(self, token, success, now, reason="navigation failed"):
        """处理已确认终止且车辆已停车的导航结果，忽略过期令牌。"""
        if token != self.active:
            return  # Stale callback from a previous goal or batch.
        self.active = None
        if self.state == "FAILED":
            return  # A cancellation watchdog failed; never restart automatically.
        stopped, self.stop_reason = self.stop_reason, None
        if stopped == "pause":
            self.state = "PAUSED"
            self.event("paused", now)
        elif stopped == "cancel":
            self.finish("CANCELLED", now)
        elif stopped == "timeout":
            self.retry_or_fail(now, "navigation timeout")
        elif not success:
            self.retry_or_fail(now, reason)
        elif self.index == len(self.batch.route):
            self.event("returned", now)
            self.finish("COMPLETED", now)
        else:
            self.state = "SERVICING"
            self.remaining_service = self.batch.service_sec
            self.last_time = now
            self.event("arrived", now)

    def tick(self, now):
        """按 ROS 时间推进服务与超时；相同时间不会消耗等待时长。"""
        if self.last_time is not None and now < self.last_time:
            if self.active is not None:
                self.commands.append(("cancel", self.active, self.target))
            if self.busy:
                self.finish("FAILED", now, "simulation clock moved backwards")
        delta = max(0, now - (self.last_time if self.last_time is not None else now))
        self.last_time = now
        if self.state in ("NAVIGATING", "RETURNING"):
            if now - self.nav_started >= self.batch.navigation_timeout_sec:
                self.event("navigation_timeout", now)
                self.stop_navigation("timeout", now)
        elif self.state in ("PAUSING", "CANCELLING"):
            if now - self.stop_started > 10.0:
                # Keep active token locked until its actual terminal callback.
                self.finish("FAILED", now, "navigation did not confirm stop within 10 seconds")
        elif self.state == "SERVICING":
            self.remaining_service = max(0, self.remaining_service - delta)
            if self.remaining_service == 0:
                self.completed.append(self.target)
                self.event("service_completed", now)
                self.index += 1
                self.retry = 0
                self.send(now)
