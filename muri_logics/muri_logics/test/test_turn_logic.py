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
        self.assertIn('linear_velocity_x', out.values)
        self.assertEqual(out.values['linear_velocity_x'], 0.0)
        self.assertEqual(out.values['linear_velocity_y'], 0.0)
        self.assertEqual(out.values['angular_velocity_z'], 0.0)
        self.assertEqual(out.values['turened_angle'], 0.0)


    def test_setOdomData(self): #TODO Funktion Überprüfung
        
        q = SimpleNamespace(x = 0, y = 0, z = 0, w = 1)
        self.logic.setOdomData(1.0, 2.0, q)

        self.assertEqual(self.logic._TurnLogic__position_Theta, 0.0)
        self.assertEqual(self.logic._TurnLogic__position_X, 1.0)
        self.assertEqual(self.logic._TurnLogic__position_Y, 2.0)

    def test_state_progression_to_turnmove(self):
        """RAEDY state should transition to TURNMOVE with first Theta set"""
        self.logic.reset()
        self.logic.setActive()
        q = SimpleNamespace(x = 0, y = 0, z = 0, w = 1)
        self.logic.setOdomData(0, 0, q)

        # Call state_machine once → RAEDY → TURNMOVE
        self.logic.state_machine()
        self.assertEqual(self.logic.getActiveState(), TurnStates.TURNMOVE)

    