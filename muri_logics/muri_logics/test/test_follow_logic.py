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
        self.assertEqual(self.logic.getActiveState(), DriveStates.RAEDY)

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

   

if __name__ == '__main__':
    unittest.main()