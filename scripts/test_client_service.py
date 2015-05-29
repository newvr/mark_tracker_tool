#!/usr/bin/env python
import sys
import rospy
from mark_tracker_tools.srv import AddMark, HowToGo, HowToGoToMark
from mark_tracker_tools.srv import InitMarkToRobot, InitPlan, WhereIs
import time
if __name__ == "__main__":

    try:
        # give postion robot
        print "debut"

        rospy.wait_for_service('init_plan')
        init_plan = rospy.ServiceProxy('init_plan', InitPlan)
        resp = init_plan(1)
        print "##Initplan", resp.result

        # time.sleep(1)  # delay needed to broadcast of tf

        # rospy.wait_for_service('init_mark_to_robot')
        # init_mark_to_robot = rospy.ServiceProxy(
        #     'init_mark_to_robot', InitMarkToRobot)
        # resp = init_mark_to_robot(2, 0.57, 0, 0, "HeadTouchFront_frame")
        # print "##Initplan", resp.result

        # rospy.wait_for_service('where_is')
        # where_is = rospy.ServiceProxy('where_is', WhereIs)
        # resp = where_is("ar_marker_2")
        # print "##Iam", resp.x, resp.y, resp.theta

        # give the relative command to send to the robot to go to absolute
        # coord

        # rospy.wait_for_service('how_to_go')
        # how_to_go = rospy.ServiceProxy('how_to_go', HowToGo)
        # resp = how_to_go(0, 0, 0)
        # print "##I need to ", resp.x, resp.y, resp.theta

        # publish tf_coord of a mark

        rospy.wait_for_service('add_mark')
        add_mark = rospy.ServiceProxy('add_mark', AddMark)
        resp = add_mark(1, True)
        if resp.result == True:
            print"##new mark learnt!"
        else:
            print "##mark overwrited!"

        # time.sleep(1)  # delay needed to broadcast of tf

        # rospy.wait_for_service('how_to_go_to_mark')
        # how_to_go_to_mark = rospy.ServiceProxy(
        #     'how_to_go_to_mark', HowToGoToMark)
        # resp = how_to_go_to_mark(1)

        # print "##I need to :  ", resp.x, resp.y, resp.theta

        """
        rospy.wait_for_service('how_to_GoToMark')
        how_to_GoToMark = rospy.ServiceProxy('how_to_GoToMark', AddMark)
        resp = how_to_GoToMark(1)
        print "I need to ", resp.x, resp.y, resp.theta
        """

    except rospy.ServiceException, e:
        print "Service call failed: %s" % e
