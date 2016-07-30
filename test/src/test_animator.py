#!/usr/bin/env python

import actionlib
import rospy
import sys
import rostest
import unittest

from needybot_msgs.msg import AnimatorAction, AnimatorGoal, AnimatorFeedback, AnimatorResult

class TestAnimator(unittest.TestCase): 

    def setUp(self):
        self.goal = AnimatorGoal(animation=AnimatorGoal.SPIN)
        self.client = actionlib.SimpleActionClient('animator', AnimatorAction)
        self.client.wait_for_server()

    def test_send(self):
        self.client.send_goal(self.goal)
        self.client.wait_for_result()
        self.assertTrue(self.client.get_result().success)

    def tearDown(self):
        pass


if __name__ == '__main__':
    rospy.loginfo("-I- needybot animator test started")
    rospy.init_node('test_animator')
    rostest.rosrun('needybot_choreography_test', 'test_animator', TestAnimator, sys.argv)
