import unittest
import math
from types import SimpleNamespace
from unittest.mock import patch

from logic_action_server_turn import TurnLogic, TurnStates
import config


class TestTurnLogic(unittest.TestCase):
     
    def setUp(self):
        self.logic = TurnLogic()

    def test_initial_state(self):
        """Test that object starts in INIT and moves to IDLE"""
        self.assertEqual(self.logic.getActiveState(), TurnStates.IDLE)

    def test_set_active_from_idle(self):
        """Activation should work only from IDLE --> READY."""
        self.logic.reset()
        self.assertEqual(self.logic.getActiveState(), TurnStates.IDLE)

        activated = self.logic.setActive()
        self.assertEqual(self.logic.getActiveState(), TurnStates.RAEDY)

    def test_set_active_wrong_state(self):
        """Activation fails if not in IDLE"""
        self.logic._TurnLogic__state = TurnStates.INIT
        self.assertFalse(self.logic.setActive())

    def test_reset(self):
        """Test that reset returns system to clean IDLE state"""
        expected_out = 0.0

        self.logic.reset()
        self.assertEqual(self.logic.getActiveState(), TurnStates.IDLE)
        out = self.logic.getOut()

        self.assertFalse(out.outValid())
        self.assertEqual(out.values['linear_velocity_x'], expected_out)
        self.assertEqual(out.values['linear_velocity_y'], expected_out)
        self.assertEqual(out.values['angular_velocity_z'], expected_out)
        self.assertEqual(out.values['turened_angle'], expected_out)

    def test_setOdomData(self):
        """Tests if setOdomData updates the position values correctly."""
        expected_theta = 1.87
        expected_x = 1.0
        expected_y = 2.0
        
        q = SimpleNamespace(x = 0.0, y = 0.0, z = 0.803, w = 0.583)
        self.logic.setOdomData(1.0, 2.0, q)

        self.assertAlmostEqual(self.logic._TurnLogic__position_Theta, expected_theta, delta = 1e-3)
        self.assertEqual(self.logic._TurnLogic__position_x, expected_x)
        self.assertEqual(self.logic._TurnLogic__position_y, expected_y)

    def test_state_progression_to_turnmove(self):
        """Moves from READY to TURNEMOVE if firstTheta not set."""
        expected_active_state = TurnStates.TURNMOVE

        self.logic.reset()
        self.logic.setActive()
        self.logic.state_machine()
        self.assertEqual(self.logic.getActiveState(), expected_active_state)

    def test_state_progression_to_intmove_with_first_theta(self):
        """Moves from READY to TURNEMOVE when firstTheta is set"""
        expected_theta = 1.87
        expected_active_state = TurnStates.TURNMOVE

        self.logic.reset()
        self.logic.setActive()
        q = SimpleNamespace(x = 0.0, y = 0.0, z = 0.803, w = 0.583)
        self.logic.setOdomData(0.0, 0.0, q)
        self.logic.state_machine()
        self.assertAlmostEqual(self.logic._TurnLogic__position_Theta, expected_theta, delta = 1e-3)
        self.assertEqual(self.logic.getActiveState(), expected_active_state)
    
    def test_calculate_turn(self):
        """Test calculate logic gives max angular Velocity when no camera data"""
        expected_angle_velocity = config.MAX_ANGLE_VELOCITY_TURN_INIT
        expected_turned_angle = 0.0

        self.logic.reset()
        self.logic.setActive()
        q = SimpleNamespace(x = 0.0, y = 0.0, z = 0.0, w = 0.0)
        self.logic.setOdomData(0.0, 0.0, q)
        
        self.logic.state_machine()  # Ready → TurnMove

        avz, turned_angle = self.logic.calculate()

        self.assertEqual(avz, expected_angle_velocity)
        self.assertEqual(turned_angle, expected_turned_angle)

    def test_success_condition(self):
        """When angle is in the Tollerance the state should switch to SUCCESS"""
        expected_active_state = TurnStates.SUCCESS

        self.logic.reset()
        self.logic.setActive()
        q = SimpleNamespace(x = 0.0, y = 0.0, z = 0.0, w = 0.0)
        self.logic.setOdomData(0.0, 0.0, q)
        self.logic.setCameraData(config.ANGLE_TOLLERANCE_TURN / 2, 1.5)

        self.logic.state_machine() # Ready → TurnMove

        self.logic.state_machine() # TurnMove runs calculate()

        self.assertEqual(self.logic.getActiveState(), expected_active_state)

    def test_success_condition(self):
        """When angle is not in the Tollerance the state should not switch to SUCCESS, should stay in TURNMOVE"""
        expected_active_state = TurnStates.TURNMOVE

        self.logic.reset()
        self.logic.setActive()
        q = SimpleNamespace(x = 0.0, y = 0.0, z = 0.0, w = 0.0)
        self.logic.setOdomData(0.0, 0.0, q)
        self.logic.setCameraData(config.ANGLE_TOLLERANCE_TURN + 1, 1.5)

        self.logic.state_machine() # Ready → TurnMove

        self.logic.state_machine() # TurnMove runs calculate()

        self.assertEqual(self.logic.getActiveState(), expected_active_state)

    def test_output_validity(self):
        """In TURNMOVE mode output should be valid"""
        self.logic.reset()
        self.logic.setActive()
        q = SimpleNamespace(x = 0.0, y = 0.0, z = 0.0, w = 0.0)
        self.logic.setOdomData(0.0, 0.0, q)
        self.logic.setCameraData(0.5, 2.0)

        self.logic.state_machine()  # Ready → TurnMove

        self.logic.state_machine()  # TurnMove runs calculate()

        out = self.logic.getOut()
        self.assertTrue(out.outValid())
        self.assertIn('linear_velocity_x', out.values)
        self.assertIn('linear_velocity_y', out.values)
        self.assertIn('angular_velocity_z', out.values)
        self.assertIn('turned_angle', out.values)


if __name__ == '__main__':
    unittest.main()
