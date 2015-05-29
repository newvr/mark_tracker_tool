#!/usr/bin/env python
import roslib
import rospy
import naoqi
import time
import sys
from math import pi, atan, sqrt, cos, sin
import numpy as np
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from std_msgs.msg import Empty
from std_msgs.msg import String
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist  # for sending commands to the drone
from naoqi import ALProxy
import tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import actionlib
from mark_tracker_tools.msg import *


# -- variables magiques
FINESSE = 0.1  # a 1 cm pres
FINESSE_ANGLE = 0.01
DURATION_LOST = 0.3
TIMER_ALMOTION = 0.8
HAUTEUR_ERREUR = 0.5
PITCH_MAX = 30
VISI_MAX = 3
temps_erreur = 1
HEAD_MAX_SPEED = 0.08
# IP="127.0.0.1"
IP = "10.0.206.111"
PORT = 9559
BACK = 1
FRONT = -1
robot_body_part = "/HeadTouchFront_frame"
ID = 2

# -- variables magiques


def sign(x):
    if x > 0:
        return 1
    if x == 0:
        return 0
    return -1


class move_pepper:

    def __init__(self):
        # =================
        self.shutdown = False
        self.move = False
        self.coord = []
        self.motionProxy = ALProxy("ALMotion", IP, PORT)
        self.postureProxy = ALProxy("ALRobotPosture", IP, PORT)
        self.move = False
        self.command_head = [0, 0]
        self.tosend = [0, 0, 0]
        self.detection = 0
        self.id_markeur_tete = ID
        self.clock = rospy.Time.now()
        self.flag_first = 0
        self.result = False

        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()
        rospy.Subscriber("/result_position", Odometry, self.position_callback)
        rospy.Timer(rospy.Duration(TIMER_ALMOTION), self.timer_callback)
        rospy.Subscriber(
            "/cam0/visualization_marker", Marker, self.mark_callback)

        self.server = actionlib.SimpleActionServer(
            "GoToPlace", mark_tracker_tools.msg.GoToAction, execute_cb=self.GoToExec, auto_start=False)
        self.server.start()

    def GoToExec(self, goal):

        size = len(goal.order)
        self.coord = []
        for i in range(size):
            self.coord.append(goal.order[i])
        print goal
        success = False
        while success == False:
            success = self.result
            # print self.server.is_active()
            r.sleep
        self.send_feedback(success)

    def mark_callback(self, data):
        if data.id == self.id_markeur_tete:
            self.clock = rospy.Time.now() + rospy.Duration(temps_erreur)
            self.detection = 1
        elif rospy.Time.now() > self.clock:
            self.detection = 0
            print " markclb "

    def send_feedback(self, success):
        _feedback = mark_tracker_tools.msg.GoToFeedback()
        _result = mark_tracker_tools.msg.GoToResult()
        print "1111"
        _feedback.sequence = []
        _feedback.sequence.append(0)
        _feedback.sequence.append(1)
        self.server.publish_feedback(_feedback)
        if success:
            _result.sequence = [0.0, 0.0]
            self.server.set_succeeded(_result)
            print "1111"

    def position_callback(self, data):
        print "callback"
        if self.server.is_active() != True:
            print " nooooooooooooooooooooooooooooooooooooot truuuuuuuuuuuuuuuuuuuuuuuuuue"

        if len(self.coord) > 0:
            # 3 comportements differents en fonction du nombre de param rentres
            if len(self.coord) == 3:
                quat = quaternion_from_euler(0, 0, self.coord[2])
                self.broadcaster.sendTransform(
                    [self.coord[0], self.coord[1], 0], quat, rospy.Time.now(), "/mon_tf/mygoal", "/map")
                (trans, rot) = self.listener.lookupTransform(
                    "/base_footprint", "/mon_tf/mygoal", rospy.Time(0))  # equivaut a un changement de repere
                euler = euler_from_quaternion(rot)
                # print "trans", trans
                self.tosend = [trans[0], trans[1], euler[2]]
                # print "euler", euler
                self.tosend[2] = euler[2]

            elif len(self.coord) == 2:  # on a entre un x,y
                self.broadcaster.sendTransform([self.coord[0], self.coord[1], 0], [
                                               0, 0, 0, 1], rospy.Time.now(), "/mon_tf/mygoal", "/map")
                (trans, rot) = self.listener.lookupTransform(
                    "/base_footprint", "/mon_tf/mygoal", rospy.Time(0))  # equivaut a un changement de repere
                euler = euler_from_quaternion(rot)
                # print "trans", trans
                self.tosend = [trans[0], trans[1], 0]

            elif len(self.coord) == 1:
                quat = quaternion_from_euler(0, 0, self.coord[2])
                self.broadcaster.sendTransform(
                    [self.coord[0], self.coord[1], 0], quat, rospy.Time.now(), "/mon_tf/mygoal", "/map")
                (trans, rot) = self.listener.lookupTransform(
                    "/base_footprint", "/mon_tf/mygoal", rospy.Time(0))  # equivaut a un changement de repere
                euler = euler_from_quaternion(rot)
                self.tosend = [0, 0, euler[2]]

            if (abs(self.tosend[0]) > FINESSE or abs(self.tosend[1]) > FINESSE or abs(self.tosend[2]) > FINESSE):
                self.move = True
                print "True"
                self.result = False

            else:
                self.move = False
                self.motionProxy.waitUntilMoveIsFinished()
                print "goal reached"
                self.coord = []
                self.detection == 0
                self.result = True

        else:
            self.detection = 0

    def fix_bug_tete(self):
        angle = self.motionProxy.getAngles("HeadYaw", False)
       # print "ang", angle
        angle[0] = angle[0] + 0.1 * sign(angle[0])
        if abs(angle[0]) > 1.05:
            angle[0] = 0
        names = ["HeadYaw"]

        fractionMaxSpeed = HEAD_MAX_SPEED
        self.motionProxy.setAngles(
            names, angle, fractionMaxSpeed)  # iciiiiiiiiiii
        self.motionProxy.waitUntilMoveIsFinished()

    def timer_callback(self, data):
        print "timer"
        (trans, rot) = self.listener.lookupTransform(
            "/base_footprint", "/map", rospy.Time(0))  # equivaut a un changement de repere
        if trans[2] > HAUTEUR_ERREUR:
            self.move = False
            self.fix_bug_tete()

        if self.detection == 1:
            if self.move == True:
                print "******commande to send to almotion *******"
                print self.tosend
                print "******************************************"
                # sens axes repere pepper = sens axes repere rviz
                # ICIIIIIIIIIIIIIIIIIIIIIIII
                self.motionProxy.post.moveTo(
                    self.tosend[0], self.tosend[1], self.tosend[2])
                # wait is useful because moveTo is not blocking function
                self.motionProxy.waitUntilMoveIsFinished()
        # else:
            # print "000000"
            # if self.flag_first==0:
               # self.fix_bug_tete()


def main(args):

    rospy.init_node('move_pepper', anonymous=True)
    move_pepper()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"

        print "Finished."


if __name__ == '__main__':
    main(sys.argv)
