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

    def test_setOdomData(self): #TODO Funktion Überprüfung
        
        q = SimpleNamespace(x = 0, y = 0, z = 0, w = 1)
        self.logic.setOdomData(1.0, 2.0, q)

        self.assertEqual(self.logic._DriveLogic__position_Theta, 0.0)
        self.assertEqual(self.logic._DriveLogic__position_X, 1.0)
        self.assertEqual(self.logic._DriveLogic__position_Y, 2.0)

    def test_state_progression_to_drivemove(self):
        """READY should transition to DRIVEMOVE."""
        self.logic.reset()
        self.logic.setActive()
        q = SimpleNamespace(x = 0, y = 0, z = 0, w = 1)
        self.logic.setOdomData(0, 0, q)

        # Call state machine once → RAEDY
        self.logic.state_machine()
        self.assertEqual(self.logic.getActiveState(), DriveStates.DRIVEMOVE)

    def test_calculate_rotation(self):
        """calculate() should generate angular velocity if misaligned."""
        self.logic.reset()
        self.logic.setCameraData(angleIR=0.5, distanceIM=1.0)

        angular, linear = self.logic.calculate()

        self.assertGreater(angular, 0.0)
        self.assertAlmostEqual(linear, Constants.MAXVELOSETY)

    def test_calculate_stop_at_goal(self):
        """calculate() should stop linear motion when near goal."""
        self.logic.reset()
        self.logic.setCameraData(angleIR=0.0, distanceIM=0.1)

        angular, linear = self.logic.calculate()

        self.assertEqual(linear, 0.0)

    def test_drive_to_success(self):
        """State should reach SUCCESS when distance < GOALDISTANCE."""
        self.logic.reset()
        self.logic.setActive()
        q = SimpleNamespace(x = 0, y = 0, z = 0, w = 1)
        self.logic.setOdomData(0, 0, q)

        # READY → DRIVEMOVE
        self.logic.state_machine()
        self.assertEqual(self.logic.getActiveState(), DriveStates.DRIVEMOVE)

        # close enough to goal
        self.logic.setCameraData(0.0, 0.1)
        self.logic.state_machine()

        self.assertEqual(self.logic.getActiveState(), DriveStates.SUCCESS)

    def test_output_validity_in_drivemove(self):
        """Output should be valid in DRIVEMOVE state."""
        self.logic.reset()
        self.logic.setActive()
        q = SimpleNamespace(x = 0.0, y = 0.0, z = 0.0, w = 0.0)
        self.logic.setOdomData(0, 0, q)

        # READY → DRIVEMOVE
        self.logic.state_machine()
        self.logic.setCameraData(0.5, 1.0)
        self.logic.state_machine()

        out = self.logic.getOut()

        self.assertTrue(out.outValid())
        self.assertIn('linear_velocity_x', out.values)
        self.assertIn('angular_velocity_z', out.values)
        self.assertIn('distance_remaining', out.values)

if __name__ == '__main__':
    unittest.main()