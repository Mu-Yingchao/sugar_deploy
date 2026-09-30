from __future__ import annotations

import unittest

import numpy as np

from sugar_deploy import contract
from sugar_deploy.observation import RobotState, TrackerObsBuilder
from sugar_deploy.real_robot_io import RealRobotState
from sugar_deploy.real_robot_tracker import (
    blend_tracker_torque,
    effort_limited_position_target,
    real_state_for_tracker,
)


class EffortLimitedTargetTest(unittest.TestCase):
    def test_reconstructed_pd_torque_equals_clipped_torque(self) -> None:
        q_des = np.array([2.0, -3.0])
        q = np.array([0.2, -0.4])
        qd = np.array([0.3, -0.2])
        kp = np.array([10.0, 20.0])
        kd = np.array([1.0, 2.0])
        effort = np.array([5.0, 8.0])
        q_cmd, tau_raw, tau_limited = effort_limited_position_target(
            q_des, q, qd, kp, kd, effort, 0.1
        )
        reconstructed = kp * (q_cmd - q) - kd * qd
        np.testing.assert_allclose(reconstructed, tau_limited, atol=1e-12)
        np.testing.assert_allclose(tau_limited, np.clip(tau_raw, -effort * 0.1, effort * 0.1))

    def test_rejects_invalid_effort_scale(self) -> None:
        x = np.ones(2)
        with self.assertRaises(ValueError):
            effort_limited_position_target(x, x, x, x, x, x, 0.0)

    def test_one_percent_mix_preserves_baseline_and_clips_tracker(self) -> None:
        baseline = np.array([4.0, -3.0])
        tracker_raw = np.array([100.0, -100.0])
        effort = np.array([10.0, 20.0])
        tracker_full, applied = blend_tracker_torque(
            baseline, tracker_raw, effort, 0.01
        )
        np.testing.assert_allclose(tracker_full, [10.0, -20.0])
        np.testing.assert_allclose(applied, [4.06, -3.17])

    def test_torque_mix_rejects_out_of_range_ratio(self) -> None:
        x = np.ones(2)
        with self.assertRaises(ValueError):
            blend_tracker_torque(x, x, x, 1.01)


class RealStateConversionTest(unittest.TestCase):
    def test_builder_round_trip_preserves_body_gyro(self) -> None:
        angle = 0.4
        quat = np.array([np.cos(angle / 2), 0.0, np.sin(angle / 2), 0.0])
        gyro_b = np.array([0.1, -0.2, 0.3])
        state = RealRobotState(
            joint_pos=np.zeros(29),
            joint_vel=np.zeros(29),
            joint_tau_est=np.zeros(29),
            base_quat_wxyz=quat,
            base_gyro=gyro_b,
        )
        robot = real_state_for_tracker(state)
        from sugar_deploy.observation import quat_apply_inverse

        np.testing.assert_allclose(
            quat_apply_inverse(robot.base_quat_w, robot.base_ang_vel_w), gyro_b, atol=1e-12
        )


class TrackerHistoryInitializationTest(unittest.TestCase):
    def test_reset_repeats_first_state_across_full_history(self) -> None:
        joint_pos = np.asarray(contract.DEFAULT_JOINT_POS, dtype=np.float64) + 0.1
        joint_vel = np.linspace(-0.2, 0.2, contract.NUM_JOINTS)
        robot = RobotState(
            joint_pos=joint_pos,
            joint_vel=joint_vel,
            base_quat_w=np.array([1.0, 0.0, 0.0, 0.0]),
            base_ang_vel_w=np.array([0.1, 0.2, 0.3]),
            anchor_pos_w=np.zeros(3),
            anchor_quat_w=np.array([1.0, 0.0, 0.0, 0.0]),
        )
        builder = TrackerObsBuilder()
        builder.reset(robot)

        np.testing.assert_allclose(
            builder.base_ang_vel_hist.flatten().reshape(5, 3),
            np.tile(robot.base_ang_vel_w, (5, 1)),
        )
        np.testing.assert_allclose(
            builder.joint_pos_hist.flatten().reshape(5, contract.NUM_JOINTS),
            np.full((5, contract.NUM_JOINTS), 0.1),
        )
        np.testing.assert_allclose(
            builder.joint_vel_hist.flatten().reshape(5, contract.NUM_JOINTS),
            np.tile(joint_vel, (5, 1)),
        )
        np.testing.assert_allclose(
            builder.gravity_hist.flatten().reshape(5, 3),
            np.tile([0.0, 0.0, -1.0], (5, 1)),
        )
        np.testing.assert_array_equal(
            builder.action_hist.flatten(), np.zeros(5 * contract.NUM_JOINTS)
        )


if __name__ == "__main__":
    unittest.main()
