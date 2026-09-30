"""真机低层控制的失效安全包装层。

实机测试确认：高层运动模式被 release 后，官方手柄的阻尼模式不会自动覆盖持续发送的
``rt/lowcmd``。因此安全循环必须直接读取独立的 ``WirelessController_`` DDS 数据，并在手柄
急停、手柄数据流超时、主循环心跳超时、状态超时或异常时锁存并持续发送零力矩命令。

软件日志只能证明触发和命令发送路径工作；首次上机或命令格式变更后，必须在悬挂状态下另做
物理卸力验收，不能把“已锁存”日志本身当作执行器已经卸力的证据。

本模块的后台线程独立于策略/任务主循环，可处理主循环阻塞或抛异常；它仍然无法防护整个 Python
进程被 SIGKILL、操作系统/电脑崩溃、DDS 网络完全断开等故障。高风险测试仍需吊架和独立硬件
断能方案。
"""

from __future__ import annotations

from dataclasses import dataclass
import threading
import time
from typing import Protocol

import numpy as np

from sugar_deploy import contract
from sugar_deploy.real_robot_io import RealRobotIO, RealRobotState


@dataclass(frozen=True)
class RemoteButtons:
    """只解码安全相关的手柄键位。

    位布局来自 Unitree SDK ``utils/joystick.py::Joystick.extract``：L2/LT 是 byte 2 的
    bit 5，L1/LB 是 byte 2 的 bit 1，B/A 分别是 byte 3 的 bit 1/bit 0。
    """

    l1: bool
    l2: bool
    a: bool
    b: bool
    y: bool

    @classmethod
    def from_wireless_remote(cls, data: bytes | bytearray | list[int]) -> "RemoteButtons":
        if len(data) < 4:
            raise ValueError(f"wireless_remote 至少需要 4 bytes，实际只有 {len(data)}")
        return cls(
            l1=bool(int(data[2]) & 0x02),
            l2=bool(int(data[2]) & 0x20),
            a=bool(int(data[3]) & 0x01),
            b=bool(int(data[3]) & 0x02),
            y=bool(int(data[3]) & 0x08),
        )

    @property
    def emergency_combo(self) -> str | None:
        # 当前真机实测 L2+Y 是官方零力矩；低层安全层也把它作为首选急停组合。
        if self.l2 and self.y:
            return "L2+Y"
        # L2+B 是当前固件的阻尼模式，低层控制时仍接受它并转成软件零力矩急停。
        if self.l2 and self.b:
            return "L2+B"
        # 兼容旧固件的阻尼组合。
        if self.l1 and self.a:
            return "L1+A"
        return None


class EmergencyStopError(RuntimeError):
    """安全控制器已经锁存急停，拒绝继续位置控制。"""


@dataclass(frozen=True)
class SafetyStatus:
    started: bool
    estopped: bool
    reason: str | None
    worker_error: str | None


class _RobotIO(Protocol):
    def release_high_level_control(self, timeout_s: float = 5.0) -> None: ...
    def read_state(self) -> RealRobotState: ...
    def read_wireless_remote(self) -> bytes | None: ...
    def send_command(
        self,
        q_target_contract: np.ndarray,
        kp_contract: np.ndarray | None = None,
        kd_contract: np.ndarray | None = None,
        tau_ff_contract: np.ndarray | None = None,
    ) -> None: ...
    def send_zero_torque(self) -> None: ...
    def close(self) -> None: ...


class SafeRealRobotController:
    """由后台安全线程独占低层发送，并监督手柄与主循环心跳。

    调用方必须以高于 ``command_timeout_s`` 的频率重复调用 :meth:`set_command`。一旦急停
    被触发就不可在同一实例中复位，必须退出测试、检查原因并重新建立控制器，避免意外恢复运动。
    """

    def __init__(
        self,
        io: _RobotIO,
        *,
        control_period_s: float = 0.02,
        command_timeout_s: float = 0.10,
        state_timeout_s: float = 0.20,
        remote_timeout_s: float = 0.50,
        zero_torque_hold_s: float = 1.0,
        max_tilt_rad: float = 1.0,
        max_abs_joint_velocity: float = 10.0,
        max_abs_base_gyro: float = 6.0,
    ) -> None:
        if control_period_s <= 0:
            raise ValueError("control_period_s 必须大于 0")
        if command_timeout_s <= control_period_s:
            raise ValueError("command_timeout_s 必须大于 control_period_s")
        if state_timeout_s <= control_period_s:
            raise ValueError("state_timeout_s 必须大于 control_period_s")
        if remote_timeout_s <= control_period_s:
            raise ValueError("remote_timeout_s 必须大于 control_period_s")
        if zero_torque_hold_s < 0:
            raise ValueError("zero_torque_hold_s 不能小于 0")
        if max_tilt_rad <= 0:
            raise ValueError("max_tilt_rad 必须大于 0")
        if max_abs_joint_velocity <= 0:
            raise ValueError("max_abs_joint_velocity 必须大于 0")
        if max_abs_base_gyro <= 0:
            raise ValueError("max_abs_base_gyro 必须大于 0")

        self._io = io
        self._period = control_period_s
        self._command_timeout = command_timeout_s
        self._state_timeout = state_timeout_s
        self._remote_timeout = remote_timeout_s
        self._zero_hold = zero_torque_hold_s
        self._max_tilt = max_tilt_rad
        self._max_joint_velocity = max_abs_joint_velocity
        self._max_base_gyro = max_abs_base_gyro

        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._started = False
        self._estop_reason: str | None = None
        self._worker_error: str | None = None
        self._command: tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray] | None = None
        self._last_heartbeat = 0.0
        self._last_state_time = 0.0
        self._last_remote_time = 0.0
        self._latest_state: RealRobotState | None = None

    @classmethod
    def connect(
        cls,
        network_interface: str,
        domain_id: int = 0,
        **kwargs: float,
    ) -> "SafeRealRobotController":
        return cls(RealRobotIO(network_interface, domain_id), **kwargs)

    def start(self) -> RealRobotState:
        """读取首帧、release 高层控制并启动安全发送线程。"""
        with self._lock:
            if self._started:
                raise RuntimeError("SafeRealRobotController 已经启动")

        initial_state = self._io.read_state()
        self._io.release_high_level_control()
        now = time.monotonic()
        with self._lock:
            self._started = True
            self._last_heartbeat = now
            self._last_state_time = now
            # 给 DDS 手柄订阅一个有限启动窗口；窗口内仍收不到数据就失败即停。
            self._last_remote_time = now
            self._latest_state = initial_state

        # 非 daemon：主逻辑意外退出但解释器仍存活时，安全线程继续运行并因心跳超时发送零力矩。
        self._thread = threading.Thread(target=self._worker, name="g1-safety-loop", daemon=False)
        self._thread.start()
        return initial_state

    @property
    def status(self) -> SafetyStatus:
        with self._lock:
            return SafetyStatus(
                started=self._started,
                estopped=self._estop_reason is not None,
                reason=self._estop_reason,
                worker_error=self._worker_error,
            )

    @property
    def latest_state(self) -> RealRobotState:
        """返回安全线程最近收到的状态快照。"""
        with self._lock:
            if self._latest_state is None:
                raise RuntimeError("还没有可用的 lowstate")
            state = self._latest_state
            return RealRobotState(
                joint_pos=state.joint_pos.copy(),
                joint_vel=state.joint_vel.copy(),
                joint_tau_est=state.joint_tau_est.copy(),
                base_quat_wxyz=state.base_quat_wxyz.copy(),
                base_gyro=state.base_gyro.copy(),
                wireless_remote=bytes(state.wireless_remote),
            )

    def set_command(
        self,
        q_target_contract: np.ndarray,
        kp_contract: np.ndarray,
        kd_contract: np.ndarray,
        tau_ff_contract: np.ndarray | None = None,
    ) -> None:
        """更新目标并喂一次主循环心跳；急停锁存后拒绝恢复位置控制。"""
        q = self._validated_vector("q_target_contract", q_target_contract)
        kp = self._validated_vector("kp_contract", kp_contract, nonnegative=True)
        kd = self._validated_vector("kd_contract", kd_contract, nonnegative=True)
        tau = self._validated_vector(
            "tau_ff_contract",
            np.zeros(contract.NUM_JOINTS) if tau_ff_contract is None else tau_ff_contract,
        )
        with self._lock:
            if not self._started:
                raise RuntimeError("必须先调用 start()")
            if self._estop_reason is not None:
                raise EmergencyStopError(f"急停已锁存：{self._estop_reason}")
            self._command = (q, kp, kd, tau)
            self._last_heartbeat = time.monotonic()

    def emergency_stop(self, reason: str = "软件请求急停") -> None:
        """锁存急停；后台线程会持续发送零力矩，不能在本实例内复位。"""
        self._latch_estop(reason)

    def raise_if_estopped(self) -> None:
        status = self.status
        if status.estopped:
            raise EmergencyStopError(f"急停已锁存：{status.reason}")

    def close(self) -> None:
        """正常退出也先锁存零力矩，并至少连续发送 ``zero_torque_hold_s``。"""
        with self._lock:
            if not self._started:
                return
        self._latch_estop("控制器正常关闭")
        deadline = time.monotonic() + self._zero_hold
        while time.monotonic() < deadline:
            thread = self._thread
            if thread is None or not thread.is_alive():
                # 安全线程本身失败时由主线程尽力补发零力矩。
                try:
                    self._io.send_zero_torque()
                except Exception:
                    pass
            time.sleep(min(self._period, max(0.0, deadline - time.monotonic())))
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=max(1.0, self._zero_hold + 0.5))
        try:
            self._io.close()
        except Exception as exc:
            with self._lock:
                if self._worker_error is None:
                    self._worker_error = f"关闭 DDS 异常：{exc!r}"
        with self._lock:
            self._started = False

    def __enter__(self) -> "SafeRealRobotController":
        self.start()
        return self

    def __exit__(self, exc_type: object, exc: object, traceback: object) -> None:
        reason = "控制代码异常退出" if exc_type is not None else "控制器正常关闭"
        self._latch_estop(reason)
        self.close()

    @staticmethod
    def _validated_vector(name: str, value: np.ndarray, nonnegative: bool = False) -> np.ndarray:
        arr = np.asarray(value, dtype=np.float64)
        if arr.shape != (contract.NUM_JOINTS,):
            raise ValueError(f"{name} 形状必须是 ({contract.NUM_JOINTS},)，实际是 {arr.shape}")
        if not np.all(np.isfinite(arr)):
            raise ValueError(f"{name} 含 NaN/Inf")
        if nonnegative and np.any(arr < 0):
            raise ValueError(f"{name} 不能包含负数")
        return arr.copy()

    def _latch_estop(self, reason: str) -> None:
        with self._lock:
            if self._estop_reason is None:
                self._estop_reason = reason
                self._command = None

    def _check_state_limits(self, state: RealRobotState) -> None:
        """按 Unitree G1 官方 termination 思路检查姿态和速度软限制。"""
        quat = np.asarray(state.base_quat_wxyz, dtype=np.float64)
        if quat.shape != (4,) or not np.all(np.isfinite(quat)):
            self._latch_estop("IMU 四元数无效")
            return
        norm = float(np.linalg.norm(quat))
        if norm < 1e-6:
            self._latch_estop("IMU 四元数范数过小")
            return
        quat /= norm
        # 与 Unitree 官方 g1::bad_orientation 等价：机身 z 轴与世界 z 轴的夹角。
        w, x, y, z = quat
        del w, z
        tilt = float(np.arccos(np.clip(1.0 - 2.0 * (x * x + y * y), -1.0, 1.0)))
        if tilt > self._max_tilt:
            self._latch_estop(
                f"机身倾角 {tilt:.3f}rad 超过限制 {self._max_tilt:.3f}rad"
            )
            return

        joint_velocity = np.asarray(state.joint_vel, dtype=np.float64)
        if not np.all(np.isfinite(joint_velocity)):
            self._latch_estop("关节速度含 NaN/Inf")
            return
        max_joint_velocity = float(np.max(np.abs(joint_velocity)))
        if max_joint_velocity > self._max_joint_velocity:
            joint_index = int(np.argmax(np.abs(joint_velocity)))
            joint_name = contract.JOINT_NAMES[joint_index]
            self._latch_estop(
                f"关节速度 {max_joint_velocity:.3f}rad/s 超过限制 "
                f"({joint_name}) "
                f"{self._max_joint_velocity:.3f}rad/s"
            )
            return

        base_gyro = np.asarray(state.base_gyro, dtype=np.float64)
        if not np.all(np.isfinite(base_gyro)):
            self._latch_estop("机身角速度含 NaN/Inf")
            return
        max_base_gyro = float(np.max(np.abs(base_gyro)))
        if max_base_gyro > self._max_base_gyro:
            self._latch_estop(
                f"机身角速度 {max_base_gyro:.3f}rad/s 超过限制 "
                f"{self._max_base_gyro:.3f}rad/s"
            )

    def _worker(self) -> None:
        next_tick = time.monotonic()
        while not self._stop_event.is_set():
            now = time.monotonic()
            try:
                state = self._io.read_state()
            except RuntimeError:
                state = None
            except Exception as exc:
                state = None
                self._latch_estop(f"读取 lowstate 异常：{exc!r}")

            if state is not None:
                with self._lock:
                    self._last_state_time = now
                    self._latest_state = state
                self._check_state_limits(state)

            try:
                remote_data = self._io.read_wireless_remote()
            except Exception as exc:
                remote_data = None
                self._latch_estop(f"读取手柄话题异常：{exc!r}")
            if remote_data is not None:
                with self._lock:
                    self._last_remote_time = now
                try:
                    combo = RemoteButtons.from_wireless_remote(remote_data).emergency_combo
                except (TypeError, ValueError) as exc:
                    self._latch_estop(f"手柄数据无效：{exc}")
                else:
                    if combo is not None:
                        self._latch_estop(f"手柄急停 {combo}")

            with self._lock:
                command = self._command
                last_heartbeat = self._last_heartbeat
                last_state_time = self._last_state_time
                last_remote_time = self._last_remote_time
                estopped = self._estop_reason is not None

            if now - last_state_time > self._state_timeout:
                self._latch_estop(f"lowstate 超时超过 {self._state_timeout:.3f}s")
                estopped = True
            if now - last_remote_time > self._remote_timeout:
                self._latch_estop(f"手柄数据流超时超过 {self._remote_timeout:.3f}s")
                estopped = True
            if command is not None and now - last_heartbeat > self._command_timeout:
                self._latch_estop(f"控制心跳超时超过 {self._command_timeout:.3f}s")
                estopped = True

            try:
                if estopped or command is None:
                    self._io.send_zero_torque()
                else:
                    q, kp, kd, tau = command
                    self._io.send_command(q, kp, kd, tau)
            except Exception as exc:
                self._latch_estop(f"发送低层指令异常：{exc!r}")
                with self._lock:
                    self._worker_error = repr(exc)
                try:
                    self._io.send_zero_torque()
                except Exception:
                    pass

            next_tick += self._period
            wait_s = next_tick - time.monotonic()
            if wait_s <= 0:
                next_tick = time.monotonic()
            else:
                self._stop_event.wait(wait_s)
