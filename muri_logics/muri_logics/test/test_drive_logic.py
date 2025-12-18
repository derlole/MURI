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
        self.logic.reset()
        self.assertEqual(self.logic.getActiveState(), DriveStates.IDLE)
        out = self.logic.getOut()

        self.assertFalse(out.outValid())
        self.assertEqual(out.values['linear_velocity_x'], 0.0)
        self.assertEqual(out.values['linear_velocity_y'], 0.0)
        self.assertEqual(out.values['angular_velocity_z'], 0.0)
        self.assertEqual(out.values['distance_remaining'], 0.0)

    def test_setOdomData(self):
        """Tests if setOdomData updates the position values correctly."""
        
        q = SimpleNamespace(x = 0.0, y = 0.0, z = 0.803, w = 0.583)
        self.logic.setOdomData(1.0, 2.0, q)

        self.assertAlmostEqual(self.logic._DriveLogic__position_Theta, 1.87, delta = 1e-3)
        self.assertEqual(self.logic._DriveLogic__position_X, 1.0)
        self.assertEqual(self.logic._DriveLogic__position_Y, 2.0)

    def test_state_progression_to_drivemove(self):
        """Moves from READY to DRIVEMOVE if firstTheta not set."""
        self.logic.reset()
        self.logic.setActive()
        self.logic.state_machine()
        self.assertEqual(self.logic.getActiveState(), DriveStates.DRIVEMOVE)

    def test_state_progression_to_intmove_with_first_theta(self):
        """Moves from READY to DRIVEMOVE when firstTheta is set"""
        self.logic.reset()
        self.logic.setActive()
        q = SimpleNamespace(x = 0.0, y = 0.0, z = 0.803, w = 0.583)
        self.logic.setOdomData(0.0, 0.0, q)
        self.logic.state_machine()
        self.assertAlmostEqual(self.logic._DriveLogic__position_Theta, 1.87, delta = 1e-3)
        self.assertEqual(self.logic.getActiveState(), DriveStates.DRIVEMOVE)

    def test_calculate_rotation(self):
        """calculate() should generate angular velocity and linear velocety if misaligned / Goal not reached ."""
        self.logic.reset()
        self.logic.setCameraData(config.ANGLE_TOLLERANCE_DRIVE + 1, 2.0)

        avz, lv = self.logic.calculate()

        self.assertNotEqual(avz, 0.0)

    def test_calculate_stop_at_goal(self):
        """calculate() should stop linear motion and angular motion when near goal."""
        self.logic.reset()
        self.logic.setCameraData(config.ANGLE_TOLLERANCE_DRIVE / 2, config.GOAL_DISTANCE - 0.5)

        avz, lv = self.logic.calculate()

        self.assertEqual(avz, 0.0)
        self.assertEqual(lv, 0.0)

    def test_drive_to_success(self):
        """State should reach SUCCESS when distance < GOALDISTANCE."""
        self.logic.reset()
        self.logic.setActive()
        q = SimpleNamespace(x = 0.0, y = 0.0, z = 0.0, w = 1)
        self.logic.setOdomData(0.0, 0.0, q)
        self.logic.setCameraData(config.ANGLE_TOLLERANCE_DRIVE + 1, 2.0)
        
        self.logic.state_machine() # READY → DRIVEMOVE
        self.assertEqual(self.logic.getActiveState(), DriveStates.DRIVEMOVE)

        # close enough to goal
        self.logic.setCameraData(config.ANGLE_TOLLERANCE_DRIVE / 2, config.GOAL_DISTANCE - 0.5)
        self.logic.state_machine() # DDRIVEMOVE runs callculate()

        self.assertEqual(self.logic.getActiveState(), DriveStates.SUCCESS)

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