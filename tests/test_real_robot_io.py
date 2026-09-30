from __future__ import annotations

from types import SimpleNamespace
import unittest

from sugar_deploy.real_robot_io import _set_hg_zero_torque


class HgZeroTorqueCommandTest(unittest.TestCase):
    def test_matches_official_hg_zero_command(self) -> None:
        commands = [
            SimpleNamespace(mode=0, q=12.0, dq=13.0, kp=14.0, kd=15.0, tau=16.0)
            for _ in range(35)
        ]

        _set_hg_zero_torque(commands)

        for command in commands:
            self.assertEqual(command.mode, 1)
            self.assertEqual(command.q, 0.0)
            self.assertEqual(command.dq, 0.0)
            self.assertEqual(command.kp, 0.0)
            self.assertEqual(command.kd, 0.0)
            self.assertEqual(command.tau, 0.0)


if __name__ == "__main__":
    unittest.main()
