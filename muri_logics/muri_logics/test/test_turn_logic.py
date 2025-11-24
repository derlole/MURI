import unittest
import math
from types import SimpleNamespace
from unittest.mock import patch

from logic_action_server_turn import TurnLogic, TurnStates, Constants


class TestTurnLogic(unittest.TestCase):
     
    def setUp(self):
        self.logic = TurnLogic()

    def test_initial_state(self):
        """INIT state should transition to IDLE after state_machine()"""
        self.assertEqual(self.logic.getActiveState(), TurnStates.IDLE)

    def test_set_active_from_idle(self):
        """Activation from IDLE should move to RAEDY"""
        self.logic.reset()
        self.assertEqual(self.logic.getActiveState(), TurnStates.IDLE)

        activated = self.logic.setActive()
        self.assertTrue(activated)
        self.assertEqual(self.logic.getActiveState(), TurnStates.RAEDY)

    def test_set_active_wrong_state(self):
        """Activation fails if not in IDLE"""
        self.logic._TurnLogic__state = TurnStates.INIT
        self.assertFalse(self.logic.setActive())

    def test_reset(self):
        """Reset should clean output and set state to IDLE"""
        self.logic.reset()
        out = self.logic.getOut()

        self.assertEqual(self.logic.getActiveState(), TurnStates.IDLE)
        self.assertFalse(out.outValid())
        self.assertIn('angular_velocity_z', out.values)
        self.assertEqual(out.values['angular_velocity_z'], 0.0)

    def test_set_odom_data(self):
        """Odom data should be converted via quaternion → yaw"""
        with patch("your_module_file.quaternion_to_yaw", return_value=1.234):
            self.logic.setOdomData(1.0, 2.0, (0, 0, 0, 1))
            self.assertAlmostEqual(
                self.logic._TurnLogic__position_Theta, 1.234, delta=1e-6
            )

    def test_state_progression_to_turnmove(self):
        """RAEDY state should transition to TURNMOVE with first Theta set"""
        self.logic.reset()
        self.logic.setActive()
        self.logic.setOdomData(0, 0, 0)

        # Call state_machine once → RAEDY → TURNMOVE
        self.logic.state_machine()
        self.assertEqual(self.logic.getActiveState(), TurnStates.TURNMOVE)

    def test_calculate_rotation(self):
        """calculate() should return angular velocity if angle is large"""
        self.logic.reset()
        self.logic.setCameraData(angleTM=0.5, distanceIM=2.0)

        angular, turned_angle = self.logic.calculate()
        self.assertGreater(angular, 0.0)
        self.assertIsInstance(turned_angle, float)

    def test_calculate_no_rotation_if_aligned(self):
        """calculate() should return zero angular velocity if angle < tolerance"""
        self.logic.reset()
        self.logic.setCameraData(angleTM=0.05, distanceIM=2.0)

        angular, _ = self.logic.calculate()
        self.assertEqual(angular, 0.0)

    def test_turn_to_success(self):
        """TURNMOVE should transition to SUCCESS if angle within tolerance"""
        self.logic.reset()
        self.logic.setActive()
        self.logic.setOdomData(0, 0, 0)

        # READY → TURNMOVE
        self.logic.state_machine()
        self.logic.setCameraData(angleTM=0.05, distanceIM=2.0)

        # Run state_machine to trigger TURNMOVE
        self.logic.state_machine()
        self.assertEqual(self.logic.getActiveState(), TurnStates.SUCCESS)

    def test_output_validity_in_turnmove(self):
        """Output should be valid during TURNMOVE"""
        self.logic.reset()
        self.logic.setActive()
        self.logic.setOdomData(0, 0, 0)
        self.logic.setCameraData(angleTM=0.5, distanceIM=2.0)

        # READY → TURNMOVE
        self.logic.state_machine()
        self.logic.state_machine()  # run calculate in TURNMOVE

        out = self.logic.getOut()
        self.assertTrue(out.outValid())
        self.assertIn('angular_velocity_z', out.values)
        self.assertIn('turened_angle', out.values)


if __name__ == '__main__':
    unittest.main()