import unittest
import math
from types import SimpleNamespace
from unittest.mock import patch

from logic_action_server_init import InitLogic, InitStates,Constants


class TestInitLogic(unittest.TestCase):

    def setUp(self):
        self.logic = InitLogic()

    def test_initial_state(self):
        """Test that object starts in INIT and moves to IDLE after first state_machine() call"""
        self.assertEqual(self.logic.getActiveState(), InitStates.IDLE)

    def test_set_active_from_idle(self):
        """Test switching into READY state works"""
        self.assertTrue(self.logic.setActive())
        self.assertEqual(self.logic.getActiveState(), InitStates.RAEDY)

    def test_set_active_wrong_state(self):
        """Test activation fails in any other state"""
        self.logic._InitLogic__state = InitStates.SUCCESS
        self.assertFalse(self.logic.setActive())

    def test_reset(self):
        """Test that reset returns system to clean IDLE state"""
        self.logic.reset()
        self.assertEqual(self.logic.getActiveState(), InitStates.IDLE)
        out = self.logic.getOut()
        self.assertFalse(out.outValid())
        self.assertEqual(out.values['angular_velocity_z'], 0.0)

    def test_state_progression_to_initmove(self):
        """Moves from READY to INITMOVE if firstTheta not set"""
        self.logic.reset()
        self.logic.setActive()
        self.logic.state_machine()
        self.assertEqual(self.logic.getActiveState(), InitStates.INITMOVE)

    def test_state_progression_to_intmove_with_first_theta(self):
        """Moces from READY to INTMOVE when firstTheta is set"""
        self.logic.reset()
        self.logic.setActive()
        q = SimpleNamespace(x = 1.0, y = 2.5, z = 0.0, w = 5.5)
        self.logic.setOdomData(0.0, 0.0, q)
        self.logic.state_machine()
        #self.assertEqual(self.logic.__firstTheta, )
        self.assertEqual(self.logic.getActiveState(), InitStates.INITMOVE)

    def test_calculate_turn(self):
        """Test calculate logic gives max rotation when no camera data"""
        self.logic.reset()
        self.logic.setActive()
        q = SimpleNamespace(x = 0.0, y = 0.0, z = 0.0, w = 0.0)
        self.logic.setOdomData(0.0, 0.0, q)
        self.logic.setCameraData(angleIR = -1.0, distanceIM = 2.0)

        self.logic.state_machine()  # To enter INITMOVE

        avz, turned_angle = self.logic.calculate()

        self.assertAlmostEqual(avz, Constants.MAXANGLEVELOSETY, delta=0.00001)
        self.assertAlmostEqual(turned_angle, 0.0, delta=0.00001)

    def test_success_condition(self):
        """When angle is small the state should switch to SUCCESS"""
        self.logic.reset()
        self.logic.setActive()
        q = SimpleNamespace(x = 0.0, y = 0.0, z = 0.0, w = 0.0)
        self.logic.setOdomData(0.0, 0.0, q)
        self.logic.setCameraData(angleIR=Constants.ANGLETOLLERAMCE / 2, distanceIM=2.0)

        # Enter INITMOVE
        self.logic.state_machine()

        # Run second step (INITMOVE logic)
        self.logic.state_machine()

        self.assertEqual(self.logic.getActiveState(), InitStates.SUCCESS)

    def test_output_validity(self):
        """In INITMOVE mode output should be valid"""
        self.logic.reset()
        self.logic.setActive()
        q = SimpleNamespace(x = 0.0, y = 0.0, z = 0.0, w = 0.0)
        self.logic.setOdomData(0.0, 0.0, q)
        self.logic.setCameraData(0.5, 2.0)
        self.logic.state_machine()  # Ready → InitMove
        self.logic.state_machine()  # InitMove runs calculate()

        out = self.logic.getOut()
        self.assertTrue(out.outValid())
        self.assertIn('angular_velocity_z', out.values)
        self.assertIn('turned_angle', out.values)


if __name__ == '__main__':
    unittest.main()
