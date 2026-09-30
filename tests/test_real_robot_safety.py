from __future__ import annotations

import time
import unittest

import numpy as np

from sugar_deploy import contract
from sugar_deploy.real_robot_io import RealRobotState
from sugar_deploy.real_robot_safety import (
    EmergencyStopError,
    RemoteButtons,
    SafeRealRobotController,
)


def _state(remote: bytes = bytes(40)) -> RealRobotState:
    return RealRobotState(
        joint_pos=np.zeros(contract.NUM_JOINTS),
        joint_vel=np.zeros(contract.NUM_JOINTS),
        joint_tau_est=np.zeros(contract.NUM_JOINTS),
        base_quat_wxyz=np.array([1.0, 0.0, 0.0, 0.0]),
        base_gyro=np.zeros(3),
        wireless_remote=remote,
    )


class FakeRobotIO:
    def __init__(self) -> None:
        self.state = _state()
        self.released = False
        self.command_count = 0
        self.zero_count = 0
        self.remote_available = True
        self.closed = False

    def release_high_level_control(self, timeout_s: float = 5.0) -> None:
        self.released = True

    def read_state(self) -> RealRobotState:
        return self.state

    def read_wireless_remote(self) -> bytes | None:
        return self.state.wireless_remote if self.remote_available else None

    def send_command(self, *args: object) -> None:
        self.command_count += 1

    def send_zero_torque(self) -> None:
        self.zero_count += 1

    def close(self) -> None:
        self.closed = True


def _wait_until(predicate, timeout: float = 0.5) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.002)
    return predicate()


class RemoteButtonsTest(unittest.TestCase):
    def test_l2_b(self) -> None:
        data = bytearray(40)
        data[2] = 0x20
        data[3] = 0x02
        self.assertEqual(RemoteButtons.from_wireless_remote(data).emergency_combo, "L2+B")

    def test_l2_y(self) -> None:
        data = bytearray(40)
        data[2] = 0x20
        data[3] = 0x08
        self.assertEqual(RemoteButtons.from_wireless_remote(data).emergency_combo, "L2+Y")

    def test_l1_a(self) -> None:
        data = bytearray(40)
        data[2] = 0x02
        data[3] = 0x01
        self.assertEqual(RemoteButtons.from_wireless_remote(data).emergency_combo, "L1+A")

    def test_short_data_rejected(self) -> None:
        with self.assertRaises(ValueError):
            RemoteButtons.from_wireless_remote(bytes(3))


class SafeControllerTest(unittest.TestCase):
    def make_controller(self, io: FakeRobotIO) -> SafeRealRobotController:
        return SafeRealRobotController(
            io,
            control_period_s=0.005,
            command_timeout_s=0.025,
            state_timeout_s=0.050,
            remote_timeout_s=0.050,
            zero_torque_hold_s=0.01,
        )

    def test_normal_command_then_close_sends_zero(self) -> None:
        io = FakeRobotIO()
        controller = self.make_controller(io)
        controller.start()
        vec = np.zeros(contract.NUM_JOINTS)
        controller.set_command(vec, vec, vec)
        self.assertTrue(_wait_until(lambda: io.command_count > 0))
        controller.close()
        self.assertTrue(io.released)
        self.assertGreater(io.zero_count, 0)
        self.assertTrue(io.closed)

    def test_command_heartbeat_timeout_latches_estop(self) -> None:
        io = FakeRobotIO()
        controller = self.make_controller(io)
        controller.start()
        vec = np.zeros(contract.NUM_JOINTS)
        controller.set_command(vec, vec, vec)
        self.assertTrue(_wait_until(lambda: controller.status.estopped))
        self.assertIn("控制心跳超时", controller.status.reason or "")
        with self.assertRaises(EmergencyStopError):
            controller.set_command(vec, vec, vec)
        controller.close()

    def test_remote_combo_latches_estop(self) -> None:
        io = FakeRobotIO()
        controller = self.make_controller(io)
        controller.start()
        remote = bytearray(40)
        remote[2] = 0x20
        remote[3] = 0x02
        io.state = _state(bytes(remote))
        self.assertTrue(_wait_until(lambda: controller.status.estopped))
        self.assertEqual(controller.status.reason, "手柄急停 L2+B")
        self.assertTrue(_wait_until(lambda: io.zero_count > 0))
        controller.close()

    def test_remote_stream_timeout_latches_estop(self) -> None:
        io = FakeRobotIO()
        controller = self.make_controller(io)
        controller.start()
        io.remote_available = False
        self.assertTrue(_wait_until(lambda: controller.status.estopped))
        self.assertIn("手柄数据流超时", controller.status.reason or "")
        controller.close()

    def test_excessive_tilt_latches_estop(self) -> None:
        io = FakeRobotIO()
        controller = SafeRealRobotController(
            io,
            control_period_s=0.005,
            command_timeout_s=0.025,
            state_timeout_s=0.050,
            remote_timeout_s=0.050,
            zero_torque_hold_s=0.01,
            max_tilt_rad=0.35,
        )
        controller.start()
        # 绕 x 轴倾斜 0.5rad 的 wxyz 四元数。
        io.state = _state()
        io.state.base_quat_wxyz = np.array([np.cos(0.25), np.sin(0.25), 0.0, 0.0])
        self.assertTrue(_wait_until(lambda: controller.status.estopped))
        self.assertIn("机身倾角", controller.status.reason or "")
        controller.close()

    def test_excessive_joint_velocity_latches_estop(self) -> None:
        io = FakeRobotIO()
        controller = self.make_controller(io)
        controller.start()
        io.state = _state()
        io.state.joint_vel[7] = 11.0
        self.assertTrue(_wait_until(lambda: controller.status.estopped))
        self.assertIn("关节速度", controller.status.reason or "")
        self.assertIn("right_hip_yaw_joint", controller.status.reason or "")
        controller.close()

    def test_excessive_base_gyro_latches_estop(self) -> None:
        io = FakeRobotIO()
        controller = self.make_controller(io)
        controller.start()
        io.state = _state()
        io.state.base_gyro[1] = 7.0
        self.assertTrue(_wait_until(lambda: controller.status.estopped))
        self.assertIn("机身角速度", controller.status.reason or "")
        controller.close()


if __name__ == "__main__":
    unittest.main()
