#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from tm_msgs.srv import SendScript, SetIO
from tm_msgs.msg import FeedbackState
from collections import deque

from ROS.robot_state_collector import SingleRobotStateCollector


def joints_close(a, b, tol_deg: float = 1.0) -> bool:
    if a is None or b is None:
        return False
    if len(a) < 6 or len(b) < 6:
        return False
    return all(abs(float(x) - float(y)) < tol_deg for x, y in zip(a[:6], b[:6]))


class TMRobotController(Node):
    def __init__(self, state_collector: SingleRobotStateCollector):
        super().__init__("tm_robot_controller")

        self.script_cli = None
        self.io_cli = None

        self.state_collector = state_collector

        self.tcp_queue = deque()

        self._busy = False
        self._min_send_interval = 0.2
        self._last_send_ts = 0.0

        self._last_joint_cmd = None
        should_append = False

        self.ee_digital_output = [0, 0, 1, 0]
        self.target_ee_output = None
        self.waiting_for_gripper = False
        self._next_gripper_wait_after = 0.0

        self.states_need_to_wait = []

        # add: busy timeout（避免卡死）
        self._busy_started_ts = 0.0
        self._busy_timeout_s = 8.0  # 可以依情況調整

        self.create_timer(0.01, self._process_queue)
        self.create_timer(0.05, self._check_joint_reached)

        self.create_subscription(
            FeedbackState,
            "feedback_states",
            self.feedback_callback,
            10
        )

    # ------------------ ROS 服務初始化 ------------------

    def setup_services(self):
        self.get_logger().info("等待 ROS 2 服務啟動...")
        self.script_cli = self.create_client(SendScript, "send_script")
        while not self.script_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("等待 send_script 服務...")
        self.io_cli = self.create_client(SetIO, "set_io")
        while not self.io_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("等待 set_io 服務...")

    def is_idle(self) -> bool:
        return (
            (not self.tcp_queue) and
            (not self._busy) and
            (not self.waiting_for_gripper) and
            (not self.states_need_to_wait)
        )

    # ------------------ FeedbackState callback ------------------

    def feedback_callback(self, msg: FeedbackState):
        self.ee_digital_output = list(msg.ee_digital_output)

        if self.waiting_for_gripper and self.target_ee_output is not None:
            if self.ee_digital_output[:3] == self.target_ee_output:
                self.get_logger().info(
                    f"----夾爪 DO 狀態達成-----: {self.ee_digital_output[:3]}"
                )
                self.waiting_for_gripper = False
                self.target_ee_output = None

                wait_sec = float(self._next_gripper_wait_after)
                self._next_gripper_wait_after = 0.0
                self._start_gripper_wait_timer(wait_sec)

    # ------------------ Timer helpers ------------------

    def _start_gripper_wait_timer(self, seconds: float):
        """DO 達成後，額外等待秒數，0 代表直接完成"""
        if seconds <= 0.0:
            self._gripper_wait_done()
        else:
            self._wait_timer = self.createTimer(seconds, self._gripper_wait_done)

    def createTimer(self, period_sec, callback):
        return self.create_timer(period_sec, callback)

    def _gripper_wait_done(self):
        # self.get_logger().info("✅ 夾爪動作等待完成")
        self._busy = False
        if hasattr(self, "_wait_timer"):
            self._wait_timer.cancel()
            del self._wait_timer

    def _start_arm_wait_timer(self, seconds: float):
        self._wait_timer_arm = self.create_timer(float(seconds), self._arm_wait_done)

    def _arm_wait_done(self):
        # self.get_logger().info("✅ 手臂動作等待完成")
        self._busy = False
        if hasattr(self, "_wait_timer_arm"):
            self._wait_timer_arm.cancel()
            del self._wait_timer_arm

    # ------------------ joint 到位檢查 ------------------

    def _check_joint_reached(self):
        """用 /joint_states 的 joints_deg 檢查是否到達等待的目標關節角"""
        if not self.states_need_to_wait:
            return

        cur = self.state_collector.joints_deg
        if not cur:
            return

        state = self.states_need_to_wait[0]
        target = state["joints"]

        if joints_close(cur, target, tol_deg=1.0):
            self.get_logger().info(f"✅ 關節到位")
            wait_t = float(state["time_to_wait"])
            self.states_need_to_wait.pop(0)

            if wait_t > 0.0:
                self._start_arm_wait_timer(wait_t)
            else:
                self._arm_wait_done()

    # ------------------ IO / 夾爪控制 ------------------

    def set_io(self, states: list):
        if not (isinstance(states, (list, tuple)) and len(states) == 3):
            self.get_logger().error("IO 狀態必須為長度 3 的 list，例如 [1,0,0]")
            return

        self.target_ee_output = list(states)
        self.waiting_for_gripper = True

        for pin, state in enumerate(states):
            req = SetIO.Request()
            req.module = 1
            req.type = 1  # Digital Output
            req.pin = int(pin)
            req.state = float(state)
            future = self.io_cli.call_async(req)

            def _done(fut, pin=pin):
                try:
                    _ = fut.result()
                    # if result.ok:
                    #     self.get_logger().info(f"✅ End_DO{pin} 設定成功")
                except Exception as e:
                    self.get_logger().error(f"[SetIO 失敗] {e}")
                    # fix: 避免因為 SetIO 失敗卡死
                    self._busy = False
                    self.waiting_for_gripper = False
                    self.target_ee_output = None

            future.add_done_callback(_done)

    def append_gripper_states(self, states, wait_after: float = 0.0):
        if not (isinstance(states, (list, tuple)) and len(states) == 3):
            self.get_logger().error("IO 狀態必須為長度 3 的 list，例如 [1,0,0]")
            return

        self._next_gripper_wait_after = float(wait_after)

        self.tcp_queue.append({
            "script": f"IO:{states[0]},{states[1]},{states[2]}",
            "wait_time": 2.0,
            "need_wait": False,
        })

    def append_gripper_open(self, wait_after: float = 3.5):
        self.append_gripper_states([0, 0, 1], wait_after=wait_after)

    def append_gripper_close(self, wait_after: float = 2.0):
        self.append_gripper_states([1, 0, 0], wait_after=wait_after)

    def append_gripper_half_open(self, wait_after: float = 3.5):
        self.append_gripper_states([0, 1, 0], wait_after=wait_after)

    def append_gripper_close_tight(self, wait_after: float = 3.5):
        self.append_gripper_states([1, 1, 0], wait_after=wait_after)

    # ------------------ Joint 指令排隊 ------------------

    def append_joint(self,
                     joint_values: list,
                     vel: float = 40,
                     acc: float = 20,
                     coord: int = 100,
                     fine: bool = False,
                     wait_time: float = 0.0,
                     need_wait: bool = False,
                     block: bool = False,
                     executor: SingleThreadedExecutor = None):
        if not (isinstance(joint_values, (list, tuple)) and len(joint_values) == 6):
            self.get_logger().error("Joint 必須 6 個數字")
            return False

        JOINT_DELTA_THRESHOLD_DEG = 4
        should_append = False
        if self._last_joint_cmd is None:
            should_append = True
        else:
            for i in range(6):
                if abs(float(joint_values[i]) - float(self._last_joint_cmd[i])) > JOINT_DELTA_THRESHOLD_DEG:
                    should_append = True
                    break
        if not should_append:
            return True

        fine_str = "true" if fine else "false"
        script = (
            f'PTP("JPP",{joint_values[0]:.2f}, {joint_values[1]:.2f}, '
            f'{joint_values[2]:.2f}, {joint_values[3]:.2f}, '
            f'{joint_values[4]:.2f}, {joint_values[5]:.2f},'
            f'{vel},{acc},{coord},{fine_str})'
        )
        self._last_joint_cmd = list(joint_values)

        self.tcp_queue.append({
            "script": script,
            "wait_time": float(wait_time),
            "need_wait": bool(need_wait),
        })

        if need_wait:
            self.states_need_to_wait.append({
                "joints": list(joint_values),
                "time_to_wait": float(wait_time),
            })

        if block:
            if executor is None:
                raise RuntimeError("append_joint(block=True) 需要傳入 executor 才能阻塞等待")
            ok = self.wait_until_idle(executor, timeout_s=60.0)
            if not ok:
                self.get_logger().warn("❌ append_joint block 等待動作失敗或 timeout")
            return ok

        return True

    # ------------------ Queue 處理邏輯 ------------------

    def _process_queue(self):
        # add: 如果 busy，就先檢查是否超時
        if self._busy:
            if self._busy_started_ts > 0 and (time.time() - self._busy_started_ts) > self._busy_timeout_s:
                self.get_logger().warning(
                    f"⚠️ _busy 狀態超過 {self._busy_timeout_s} 秒，"
                    f"自動解鎖以避免卡死（清空等待狀態）"
                )
                self._busy = False
                self.waiting_for_gripper = False
                self.target_ee_output = None
                self.states_need_to_wait.clear()
            else:
                return

        if not self.tcp_queue:
            return

        now = time.time()
        if now - self._last_send_ts < self._min_send_interval:
            return

        item = self.tcp_queue.popleft()
        cmd = item["script"]
        wait_time = item.get("wait_time", 0.05)
        need_wait = item.get("need_wait", True)

        self._last_send_ts = now
        self._busy = True
        self._busy_started_ts = now  # add: 記錄 busy 開始時間

        # IO 指令
        if isinstance(cmd, str) and cmd.startswith("IO:"):
            try:
                _, vals = cmd.split(":")
                a, b, c = map(int, vals.split(","))
                self.get_logger().info(f"執行夾爪指令: {cmd}")
                self.set_io([a, b, c])
            except Exception as e:
                self.get_logger().error(f"IO 指令解析失敗: {e}")
                self._busy = False
            return

        self.get_logger().info(f"執行 PTP(JPP) 腳本: {cmd}")
        self._send_script_async(cmd, wait_time, need_wait)

    def _send_script_async(self, script: str, wait_time: float, need_wait: bool):
        req = SendScript.Request()
        req.id = "auto"
        req.script = script
        future = self.script_cli.call_async(req)

        def _done(_):
            try:
                res = future.result()
                if getattr(res, "ok", False):
                    self.get_logger().info("指令執行成功")
                else:
                    self.get_logger().warn("⚠️ SendScript 回傳 ok=false")
                if not need_wait:
                    self._busy = False
            except Exception as e:
                self.get_logger().error(f"[SendScript 失敗] {e}")
                self._busy = False
                self.states_need_to_wait.clear()

        future.add_done_callback(_done)

    # ------------------ 公用函式 ------------------

    def spin_once(self, executor: SingleThreadedExecutor, timeout_sec: float = 0.05):
        executor.spin_once(timeout_sec=timeout_sec)

    def wait_until_idle(self, executor: SingleThreadedExecutor, timeout_s: float = 30.0) -> bool:
        start = self.get_clock().now()
        while rclpy.ok() and (not self.is_idle()):
            self.spin_once(executor, timeout_sec=0.05)
            now = self.get_clock().now()
            elapsed = (now - start).nanoseconds * 1e-9
            if elapsed > timeout_s:
                self.get_logger().warn(
                    f"⚠️ wait_until_idle 超過 timeout_s={timeout_s} 秒，強制結束等待"
                )
                return False
        if not rclpy.ok():
            self.get_logger().warn("⚠️ rclpy 已關閉，wait_until_idle 提前結束")
            return False
        return True


# ------------------ main ------------------

def main():
    rclpy.init()

    collector = SingleRobotStateCollector(fps=30)
    node = TMRobotController(collector)

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(collector)

    try:
        node.setup_services()
        j1 = [2.01, -10.63, 120.00, 70.83, -87.79, 179.26]
        j2 = [2.01, -10.63, 140.18, 70.83, -87.79, 179.26]
        j3 = [2.01, -10.63, 134.00, 70.83, -87.79, 179.26]

        node.append_joint(j1, block=False)
        node.append_gripper_open()
        node.append_joint(j2, block=False)

        node.append_joint(j1, block=True, executor=executor)
        node.append_gripper_close()
        node.append_joint(j3, block=False)

        print("hi")
        while rclpy.ok():
            node.spin_once(executor, timeout_sec=0.05)
            if node.is_idle():
                node.spin_once(executor, timeout_sec=0.0)
                node.get_logger().info("✅ 佇列處理完成，自動結束程式")
                break

    except KeyboardInterrupt:
        node.get_logger().info("⛔ 手動中斷程式")
    finally:
        executor.shutdown()
        node.destroy_node()
        collector.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
