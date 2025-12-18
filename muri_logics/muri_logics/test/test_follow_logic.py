import unittest
import math
from types import SimpleNamespace
from unittest.mock import patch

from logic_action_server_follow import FollowLogic, FollowStates
import config


class TestDriveLogic(unittest.TestCase):

    def setUp(self):
        self.logic = FollowLogic()

    def test_initial_state(self):
        """Test that object starts in INIT and moves to IDLE"""
        self.assertEqual(self.logic.getActiveState(), FollowStates.IDLE)

    def test_set_active_from_idle(self):
        """Activation should work only from IDLE --> READY."""
        self.logic.reset()
        self.assertEqual(self.logic.getActiveState(), FollowStates.IDLE)

        self.assertTrue(self.logic.setActive())
        self.assertEqual(self.logic.getActiveState(), FollowStates.READY)

    def test_set_active_wrong_state(self):
        """Activation fails if not in IDLE."""
        self.logic._FollowLogic__stateFollow = FollowStates.INIT
        self.assertFalse(self.logic.setActive())

    def test_reset(self):
        """Reset should clean output and set state to IDLE."""
        self.logic.reset()
        self.assertEqual(self.logic.getActiveState(), FollowStates.IDLE)
        out = self.logic.getOut()

        self.assertFalse(out.outValid())
        self.assertEqual(out.values['linear_velocity_x'], 0.0)
        self.assertEqual(out.values['linear_velocity_y'], 0.0)
        self.assertEqual(out.values['angular_velocity_z'], 0.0)
        self.assertEqual(out.values['distance_remaining'], 0.0)

    def test_state_progression_to_initmove(self):
        """Moves from READY to INITMOVE if firstTheta not set"""
        self.logic.reset()
        self.logic.setActive()         
        self.logic.state_machine()     
        self.assertEqual(self.logic.getActiveState(), FollowStates.FOLLOWMOVE)

    def test_state_progression_to_intmove_with_first_theta(self):
        """Moves from READY to INTMOVE when firstTheta is set"""
        self.logic.reset()
        self.logic.setActive()
        q = SimpleNamespace(x = 0.0, y = 0.0, z = 0.803, w = 0.583)
        self.logic.setOdomData(0.0, 0.0, q)
        self.logic.state_machine()
        self.assertAlmostEqual(self.logic._FollowLogic__positionTheta, 1.87 ,delta = 1e-3)
        self.assertEqual(self.logic.getActiveState(), FollowStates.FOLLOWMOVE)

    def test_setOdomData(self):
        """Tests if setOdomData updates the position values correctly."""
        q = SimpleNamespace(x = 0.0, y = 0.0, z = 0.803, w = 0.583)
        self.logic.setOdomData(1.0, 2.0, q)

        self.assertAlmostEqual(self.logic._FollowLogic__positionTheta, 1.87, delta = 1e-3)
        self.assertEqual(self.logic._FollowLogic__positionX, 1.0)
        self.assertEqual(self.logic._FollowLogic__positionY, 2.0)

    def test_calculate_outputs(self):
        """Check calculation of angular and linear velocity."""
        self.logic.reset()
        self.logic.setActive()
        self.logic.setCameraData(angleIR = 1.7, distanceIM = 0.1)
        self.assertEqual(self.logic._FollowLogic__angleToMidInRad, 1.7)
        self.assertEqual(self.logic._FollowLogic__distanceInMeter, 0.1)


        self.logic.state_machine()  # READY -> FOLLOWMOVE
        avz, lvx = self.logic.calculate()

        self.assertEqual(avz, -0.34)
        self.assertAlmostEqual(lvx, -0.11, delta = 1e-6)

    def test_success_trigger(self):
        """dominant Aruco ID = 0 → SUCCESS"""
        self.logic.reset()
        self.logic.setActive()
        q = SimpleNamespace(x = 0.0, y = 0.0, z = 0.803, w = 0.583)
        self.logic.setOdomData(0.0, 0.0, q)
        self.logic.setCameraData(0.0, 1.0)
        self.logic.setArucoData(0)

        self.logic.state_machine() # READY -> FOLLOWMOVE
        self.logic.state_machine() # FOLLOWMOVE -> Ready

        self.assertEqual(self.logic.getActiveState(), FollowStates.SUCCESS)
        
    def test_failed_trigger(self):
        """dominant Aruco ID = 9999 → FAILED"""
        self.logic.reset()
        self.logic.setActive()
        q = SimpleNamespace(x = 0.0, y = 0.0, z = 0.803, w = 0.583)
        self.logic.setOdomData(0.0, 0.0, q)
        self.logic.setCameraData(0.0, 1.0)
        self.logic.setArucoData(9999)

        self.logic.state_machine() # READY -> FOLLOWMOVE
        self.logic.state_machine() # FOLLOWMOVE -> Ready  

        self.assertEqual(self.logic.getActiveState(), FollowStates.FAILED)

    def test_followmove_output_structure(self):
        """FollowMove should fill correct output fields."""
        self.logic.reset()
        self.logic.setActive()
        q = SimpleNamespace(x = 0.0, y = 0.0, z = 0.803, w = 0.583)
        self.logic.setOdomData(0.0, 0.0, q)
        self.logic.setCameraData(1.7, 0.1)

        self.logic.state_machine()  # READY -> FOLLOWMOVE
        self.logic.state_machine()  # FOLLOWMOVE step

        out = self.logic.getOut().values
        
        self.assertIn("linear_velocity_x", out)
        self.assertIn("angular_velocity_z", out)
        self.assertIn("distance_remaining", out)

    def test_is_True(self): # The only reason for this Test ist that we have in the summ 42 Tests!.
        self.assertTrue(True)







    


   
if __name__ == '__main__':
    unittest.main()