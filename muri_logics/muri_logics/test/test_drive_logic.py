# Written, maintained and owned by Louis Moser (MURI DEVELOPMENT TEAM)

import unittest
import math
from types import SimpleNamespace
from unittest.mock import patch

from logic_action_server_drive import DriveLogic, DriveStates
import config


class TestDriveLogic(unittest.TestCase):

    def setUp(self):
        self.logic = DriveLogic()

    def test_initial_state(self):
        """Test that object starts in INIT and moves to IDLE"""
        self.assertEqual(self.logic.getActiveState(), DriveStates.IDLE)

    def test_set_active_from_idle(self):
        """Activation should work only from IDLE --> READY."""
        self.logic.reset()
        self.assertEqual(self.logic.getActiveState(), DriveStates.IDLE)

        self.assertTrue(self.logic.setActive())
        self.assertEqual(self.logic.getActiveState(), DriveStates.READY)

    def test_set_active_wrong_state(self):
        """Activation fails if not in IDLE."""
        self.logic._DriveLogic__state = DriveStates.INIT
        self.assertFalse(self.logic.setActive())

    def test_reset(self):
        """Reset should clean output and set state to IDLE."""
        expected_out = 0.0

        self.logic.reset()
        self.assertEqual(self.logic.getActiveState(), DriveStates.IDLE)
        out = self.logic.getOut()

        self.assertFalse(out.outValid())
        self.assertEqual(out.values['linear_velocity_x'], expected_out)
        self.assertEqual(out.values['linear_velocity_y'], expected_out)
        self.assertEqual(out.values['angular_velocity_z'], expected_out)
        self.assertEqual(out.values['distance_remaining'], expected_out)

    def test_setOdomData(self):
        """Tests if setOdomData updates the position values correctly."""
        expected_theta = 1.87
        expected_x = 1.0
        expected_y = 2.0
        
        q = SimpleNamespace(x = 0.0, y = 0.0, z = 0.803, w = 0.583)
        self.logic.setOdomData(1.0, 2.0, q)

        self.assertAlmostEqual(self.logic._DriveLogic__position_Theta, expected_theta, delta = 1e-3)
        self.assertEqual(self.logic._DriveLogic__position_x, expected_x)
        self.assertEqual(self.logic._DriveLogic__position_y, expected_y)

    def test_state_progression_to_drivemove(self):
        """Moves from READY to DRIVEMOVE if firstTheta not set."""
        expected_active_state = DriveStates.DRIVEMOVE

        self.logic.reset()
        self.logic.setActive()
        self.logic.state_machine()
        self.assertEqual(self.logic.getActiveState(), expected_active_state)

    def test_state_progression_to_intmove_with_first_theta(self):
        """Moves from READY to DRIVEMOVE when firstTheta is set"""
        expected_theta = 1.87
        expected_active_state = DriveStates.DRIVEMOVE

        self.logic.reset()
        self.logic.setActive()
        q = SimpleNamespace(x = 0.0, y = 0.0, z = 0.803, w = 0.583)
        self.logic.setOdomData(0.0, 0.0, q)
        self.logic.state_machine()
        self.assertAlmostEqual(self.logic._DriveLogic__position_Theta, expected_theta, delta = 1e-3)
        self.assertEqual(self.logic.getActiveState(), expected_active_state)

    def test_calculate_rotation(self):
        """calculate() should generate angular velocity and linear velocety if misaligned / Goal not reached ."""
        expected_angle_velocity = 0.0

        self.logic.reset()
        self.logic.setCameraData(config.ANGLE_TOLLERANCE_DRIVE + 1, 2.0)

        avz, lv = self.logic.calculate()

        self.assertNotEqual(avz, expected_angle_velocity)

    def test_calculate_stop_at_goal(self):
        """calculate() should stop linear motion and angular motion when near goal."""
        expected_angle_velocity = 0.0
        expected_linear_velocity = 0.0

        self.logic.reset()
        self.logic.setCameraData(config.ANGLE_TOLLERANCE_DRIVE / 2, config.GOAL_DISTANCE - 0.5)

        avz, lv = self.logic.calculate()

        self.assertEqual(avz, expected_angle_velocity)
        self.assertEqual(lv, expected_linear_velocity)

    def test_drive_to_success(self):
        """State should reach SUCCESS when distance < GOALDISTANCE."""
        expected_active_state_Ready_to_DriveMove = DriveStates.DRIVEMOVE
        expected_active_state_DriveMove_to_Success = DriveStates.SUCCESS

        self.logic.reset()
        self.logic.setActive()
        q = SimpleNamespace(x = 0.0, y = 0.0, z = 0.0, w = 1)
        self.logic.setOdomData(0.0, 0.0, q)
        self.logic.setCameraData(config.ANGLE_TOLLERANCE_DRIVE + 1, 2.0)
        
        self.logic.state_machine() # READY → DRIVEMOVE
        self.assertEqual(self.logic.getActiveState(), expected_active_state_Ready_to_DriveMove)

        # close enough to goal
        self.logic.setCameraData(config.ANGLE_TOLLERANCE_DRIVE / 2, config.GOAL_DISTANCE - 0.5)
        self.logic.state_machine() # DDRIVEMOVE runs callculate()

        self.assertEqual(self.logic.getActiveState(), expected_active_state_DriveMove_to_Success)

    def test_output_validity_in_drivemove(self):
        """Output should be valid in DRIVEMOVE state."""
        self.logic.reset()
        self.logic.setActive()
        q = SimpleNamespace(x = 0.0, y = 0.0, z = 0.0, w = 0.0)
        self.logic.setOdomData(0.0, 0.0, q)
        self.logic.setCameraData(config.ANGLE_TOLLERANCE_DRIVE + 1, 2.0)

        
        self.logic.state_machine() # READY → DRIVEMOVE
        self.logic.state_machine() # DDRIVEMOVE runs callculate()

        out = self.logic.getOut()

        self.assertTrue(out.outValid())
        self.assertIn('linear_velocity_x', out.values)
        self.assertIn('angular_velocity_z', out.values)
        self.assertIn('distance_remaining', out.values)

if __name__ == '__main__':
    unittest.main()