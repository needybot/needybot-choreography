#!/usr/bin/env python

import math

import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Twist
import rospy

from needybot.lib.logger import *
from needybot_msgs.msg import AnimatorAction, AnimatorGoal, AnimatorFeedback, AnimatorResult

class Animation(object):
    
    def __init__(self):
        self.hz = 50
        self.rate = rospy.Rate(self.hz)
        self.vel_pub = rospy.Publisher(
          '/cmd_vel_mux/input/navi',
          Twist,
          queue_size=10
        )

class AnimateSpin(Animation):
    
    def __init__(self):
        super(AnimateSpin, self).__init__()
        self.vel = 4.0
        self.running = False
        self.stopped = False
        self.cmd_vel = Twist() 
        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.angular.z = self.vel

    def start(self, done_cb):
        self.stopped = False 
        self.running = True 
        dist_per_tick = (1.0 / self.hz) * self.vel
        num_ticks = (math.pi * 2) / dist_per_tick 
        tick = 0

        while self.running and tick < num_ticks:
            tick += 1
            self.vel_pub.publish(self.cmd_vel)
            self.rate.sleep()

        if not self.stopped:
            self.cmd_vel.angular.z = 0.0
            self.vel_pub.publish(self.cmd_vel)
            done_cb()

    def stop(self):
        self.stopped = True 
        self.running = False


class AnimatorActionService(object):

    def __init__(self):
        self.rate = rospy.Rate(10)
        self._feedback = AnimatorFeedback()
        self._result = AnimatorResult()
        self._server = actionlib.SimpleActionServer(
            'animator',
            AnimatorAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self._server.start()

    def done_cb(self):
        self.done = True

    def execute_cb(self, goal):
        animation_type = goal.animation

        self.done = False

        animation = None
        if animation_type == AnimatorGoal.SPIN:
            animation = AnimateSpin()
        else:
            self._result.success = False 
            self._server.set_aborted(self._result)
            return

        animation.start(self.done_cb)

        success = True 
        while not self.done and not rospy.is_shutdown():
            if self._server.is_preempt_requested():
                success = False
                animation.stop()
                self._server.set_preempted()
                break

            self._feedback.playing = True
            self._server.publish_feedback(self._feedback)
            self.rate.sleep()

        if success:
            self._result.success = True 
            self._server.set_succeeded(self._result)


if __name__ == '__main__':
    rospy.init_node('animator_service')
    action_server = AnimatorActionService() 
    rospy.spin()
