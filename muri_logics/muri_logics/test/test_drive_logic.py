import unittest
import math
from types import SimpleNamespace
from unittest.mock import patch
#from muri_logics.general_funcs import quaternion_to_yaw, p_regulator


from logic_action_server_drive import DriveLogic, DriveStates, Constants


class TestDriveLogic(unittest.TestCase):

    def setUp(self):
        self.logic = DriveLogic()

    def test_initial_state(self):
        """INIT should execute and remain in INIT until external event."""
        self.assertEqual(self.logic.getActiveState(), DriveStates.INIT)

    def test_set_active_from_idle(self):
        """Activation should work only from IDLE."""
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
        out = self.logic.getOut()

        self.assertEqual(self.logic.getActiveState(), DriveStates.IDLE)
        self.assertFalse(out.outValid())
        self.assertIn('linear_velocity_x', out.values)
        self.assertEqual(out.values['linear_velocity_x'], 0.0)
        self.assertEqual(out.values['linear_velocity_y'], 0.0)
        self.assertEqual(out.values['angular_velocity_z'], 0.0)
        self.assertEqual(out.values['distance_remaining'], 0.0)

    def test_setOdomData(self):
        
        q = SimpleNamespace(x = 0, y = 0, z = 0, w = 1)
        self.logic.setOdomData(1.0, 2.0, q)

        self.assertEqual(self.logic._DriveLogic__position_Theta, 0.0)
        self.assertEqual(self.logic._DriveLogic__position_X, 1.0)
        self.assertEqual(self.logic._DriveLogic__position_Y, 2.0)

if __name__ == '__main__':
    unittest.main()