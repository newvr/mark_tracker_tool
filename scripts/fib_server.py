#! /usr/bin/env python

import roslib; roslib.load_manifest('mark_tracker_tools')
import rospy
import actionlib
import mark_tracker_tools.msg

class FibonacciAction(object):
  # create messages that are used to publish feedback/result
  _feedback = mark_tracker_tools.msg.FibonacciFeedback()
  _result   = mark_tracker_tools.msg.FibonacciResult()

  def __init__(self, name):
    self._action_name = name
    self._as = actionlib.SimpleActionServer(self._action_name,mark_tracker_tools.msg.GoToAction, execute_cb=self.execute_cb, auto_start = False)
    self._as.start()
    
  def execute_cb(self, goal):
    # helper variables
    r = rospy.Rate(1)
    success = False
    rospy.loginfo(' starting to move to %s', goal.order)
    # append the seeds for the fibonacci sequence
    self._feedback.sequence = []
    self._feedback.sequence.append(0)
    self._feedback.sequence.append(1)
    
   
    #r.sleep()
      
    if success:
      self._result.sequence = self._feedback.sequence
      rospy.loginfo('%s: Succeeded' % self._action_name)
      self._as.set_succeeded(self._result)


def myhook():
  print "shutdown time!"
      
if __name__ == '__main__':
  rospy.init_node('fibonacci')
  FibonacciAction(rospy.get_name())
  rospy.spin()
  if rospy.is_shutdown() == True:
            myhook()