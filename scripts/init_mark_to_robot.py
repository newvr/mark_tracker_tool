#!/usr/bin/env python
import roslib
import rospy
import numpy
import math
import time
import sys
from gazebo_msgs.msg import ModelState
from visualization_msgs.msg import Marker
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty, EmptyResponse
import tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from mark_tracker_tools.srv import *
from os import chdir
import functools
from naoqi import ALProxy

# -- variables magiques TODO

CAMERA_NAME = "axis_camera"
# ROBOT_REF = "base_link"
ROBOT_FOOT = "/base_footprint"
MAP = "/map"
FILE_SAVED_MARK = "saved_mark.txt"
FILE_SAVED_INIT_PLAN = "saved_init_plan.txt"
FILE_SAVED_MARK_TO_ROBOT = "saved_mark_to_"
TIMER = 0.2
# MARKER_NAME = "4x4_"
MARKER_NAME = "ar_marker_"
# NAMESPACE = ["robot1"]
IP = "10.0.206.111"
PORT = 9559


def sign(var):
    """
    signe function
    """
    if var > 0:
        return 1
    if var == 0:
        return 0
    return -1


class ToolsPepper:

    """
    contains all services that can be called by the client
    """

    def __init__(self, value, namespace, mark, mark_init, robot_part, i, p):

        self.moy_it = 0
        self.moy_vec = []
        self.path = value
        self.namespace = namespace
        self.mark = MARKER_NAME + str(mark)
        self.mark_init = MARKER_NAME + str(mark_init)
        self.angle = 0
        self.delta_angle = 0
        self.robotpart = robot_part
        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()
        self.iter = 0
        self.x_old = 0
        self.y_old = 0
        self.FLAG_init = False
        self.FLAG_old = False
        self.motionProxy = ALProxy("ALMotion", IP, PORT)

        # mon_tf / monica / HeadTouchFront_frame
        rospy.Timer(rospy.Duration(5), self.timer_callback)
        self.vect_tf = [[CAMERA_NAME, MAP, [0, 0, 0], [0, 0, 0, 1]],
                        ["mon_tf/" + self.namespace + "/" + self.robotpart,
                         self.mark, [0, 0, 0], [0, 0, 0, 1]]]

        rospy.Subscriber(
            "/cam0/visualization_marker", Marker, self.publish_tf)

        self.motionProxy.setAngles("HeadYaw", -2, 0.5)

    def timer_callback(self, data):

        print "timer"
        old_trans, rot = self.listener.lookupTransform('/map',
                                                       self.namespace + "/" +
                                                       self.robotpart,
                                                       rospy.Time(0))
        old_euler = euler_from_quaternion(rot)

        if self.moy_it < 8:

            names = "HeadYaw"
            changes = 0.5
            fractionMaxSpeed = 0.1
            self.motionProxy.changeAngles(names, changes, fractionMaxSpeed)
            time.sleep(3)  # todo

            new_trans, rot = self.listener.lookupTransform('/map',
                                                           self.namespace + "/" +
                                                           self.robotpart,
                                                           rospy.Time(0))
            new_euler = euler_from_quaternion(rot)
            self.delta_angle = new_euler[2] - old_euler[2]

            d = math.sqrt(math.pow(old_trans[0] - new_trans[0], 2) +
                          math.pow(old_trans[1] - new_trans[1], 2))

            print "d", d / 2
            cx = d / (2. * math.tan(self.delta_angle / 2))

            print "aaaax", cx

            self.moy_vec.append((d / 2, cx))
            self.moy_it += 1

        else:
            print self.moy_vec

        # calcul de la dist entre centre de rotatation et la marque

    def publish_tf(self, data):
        """
        publish tf contained in self.vect_tf
        This is called while a mark is detected

        In this function we put things that need to be looped ( as
        publish tf or calcul speed )
        """

        for i in range(len(self.vect_tf)):
            self.broadcaster.sendTransform(
                self.vect_tf[i][2], self.vect_tf[i][3], rospy.Time.now(),
                self.vect_tf[i][0], self.vect_tf[i][1])

        if self.FLAG_init == False:
            self.init_plan()
        else:

            # print"************"
            # for k in range(len(self.vect_tf)):
            #     print [self.vect_tf[k][0], self.vect_tf[k][1]]
            # print"************"

            self.func_link_to_robot()
            self.MAJ_link_head_mark()
            self.MAJ_angle_head_mark()
            # TODOOOOOOO

        # ============================= ROS SERVICES ======================
    def init_plan(self):
        """
            add in vect_mark[0] the coordinates of the camera deduced from
            the position of the mark we choose to be the origin. This step
            enable the trackin in our plan.
        """
        try:
            trans, rot = self.listener.lookupTransform(
                self.mark_init, '/map', rospy.Time(0))
            self.vect_tf[0] = [CAMERA_NAME, MAP, trans, rot]
            print " plan intialized !"
            self.FLAG_init = True
        except Exception, exc:
            print "init_plan", exc

    def MAJ_link_head_mark(self):
        a = 1
        # try:
        #     (trans, rot) = self.listener.lookupTransform(
        #         MAP, self.namespace + "/base_footprint",
        #         rospy.Time(0))
        #     if self.FLAG_old == False:
        #         self.FLAG_old = True
        #         self.x_old = trans[0]
        #         self.y_old = trans[1]
        #     else:
        #         if self.iter == 100:

        #             delta_x = trans[0] - self.x_old
        #             delta_y = trans[1] - self.y_old

        #             self.x_old = trans[0]
        #             self.y_old = trans[1]
        #             print "iniii"

        #             self.iter = 0

        #             print "delta_x", delta_x
        #             print "delta_y", delta_y

        #             if abs(delta_x) > 0.005:
        #                 print "===========delta_x"
        #                 self.vect_tf[1] = [
        #                     self.vect_tf[1][0], self.vect_tf[1][1],
        #                     (self.vect_tf[1][2][0] + delta_x / 4 *
        #                      sign(delta_x), self.vect_tf[1][2][1], 0),
        #                     (0, 0, 0, 1)]
        #                 self.x_old = trans[0]

        # if abs(delta_y) > 0.001:
        # print "===========delta_y"
        # self.vect_tf[1] = [
        # self.vect_tf[1][0], self.vect_tf[1][1],
        # (self.x_old, self.y_old +
        # 0.001 * sign(delta_y), 0),
        # (0, 0, 0, 1)]
        # self.y_old = trans[1]

        #         self.iter += 1
        #         print self.iter

        # except Exception, exc:
        #     print "init_plan", exc

    def MAJ_angle_head_mark(self):

        a = 1
        (trans, rot) = self.listener.lookupTransform(
            MAP, self.namespace + "/base_footprint",
            rospy.Time(0))

        (trans_r, rot_r) = self.listener.lookupTransform(
            self.namespace + "/base_footprint",
            self.namespace + "/HeadTouchFront_frame",
            rospy.Time(0))

        euler_foot = euler_from_quaternion(rot)
        new_quat = quaternion_from_euler(
            0.0, -0.04, euler_foot[2])
        self.broadcaster.sendTransform(
            (trans[0], trans[1],
             0.0), new_quat, rospy.Time.now(),
            self.namespace + "/jojojojo", MAP)

        self.broadcaster.sendTransform(
            trans_r, rot_r, rospy.Time.now(),
            self.namespace + "/jojojojo/head",
            self.namespace + "/jojojojo")

        # if abs(euler_foot[0]) > 0.1 or abs(euler_foot[1]) > 0.1:  #
        # magiqu

        (trans_fin, rot_fin) = self.listener.lookupTransform(
            self.vect_tf[1][1],
            self.namespace + "/jojojojo/head",
            rospy.Time(0))

        self.vect_tf[1] = [
            self.vect_tf[1][0], self.vect_tf[1][1],
            (self.vect_tf[1][2][0], self.vect_tf[
                1][2][1], trans_fin[2]),
            rot_fin]

    def func_link_to_robot(self):
        """
            it publishes the link between the map and the robot, it
             permit to the robot to be in the same tf
             tree than map and marker.
             We know where the head should be (is is saved in self.vect_tf[1]),
             we look the relation between the head and /base_link (
             which is the origin of the tf tree of the robot ) and we link it

            (run >>rosrun tf view_frames for more info)
        """
        try:
            # get the relation to set the link between base_link and
            # our map

            # name = str(self.vect_tf[1][0]).split("/")
            name = self.namespace
            # we split because we need to know where
            # is the robot_part relative
            # to the baselink ( the true one )
            # and to mix it with our supposed
            # robot part (my_tf/robotpart, seen
            # with mark_tracker): we want to
            # remove the "my_tf/" but keep the namespace

            # if there is a namespace, len = 3 , else len = 2
            # namespace = ""
            # if len(name) == 3:
            # namespace = name[1] + "/"
            # maj du vect de namespace ( permet au pg de savoir cb1 de
            # robots tournent en meme temps, utile pour publish l'odom
            # relative a notre plan de chaque robot )
            # exist = False
            # for i in range(0, len(self.namespace)):
            #     if namespace == self.namespace[i]:
            #         exist = True
            # if exist == False:  # ie if not already existing
            #     self.namespace.append(namespace)

            # robot_part = namespace + name[len(name) - 1]
            # print self.vect_tf
            robot_split = self.vect_tf[1][0].split("/")
            robot_part = robot_split[1] + "/" + robot_split[2]
            # print "oooooooo", robot_part

            (trans, rot) = self.listener.lookupTransform(
                self.namespace + "/" + self.robotpart, self.namespace + "/base_link", rospy.Time(0))

            # ou devrait donc etre notre base_link par rapport a notre mark: on
            # cree un base_link "virtuel"

            self.broadcaster.sendTransform(
                trans, rot, rospy.Time.now(),
                self.namespace + "/base_link",
                self.vect_tf[1][0])

            # comment est notre base_link virtuel par rapport a notre map?
            # (trans_fin, rot_fin) = self.listener.lookupTransform(
            #     MAP, "mon_tf/" + self.namespace[i] + "/base_link",
            #     rospy.Time(0))

            # self.broadcaster.sendTransform(
            #     trans_fin, rot_fin, rospy.Time.now(), self.namespace[
            #         i] + "/base_link",
            #     MAP)
        except Exception, exc:
            print "aaaaa", exc
            a = 1


def write_message(vecteur):
    """
    To give the info into the form to write in the file
    """
    message = ""
    for k in range(0, len(vecteur)):

        message = message + str(vecteur[k][0]) + " " + str(
            vecteur[k][1]) + " " + str(
            vecteur[k][2][0]) + " " + str(
            vecteur[k][2][1]) + " " + str(
            vecteur[k][2][2]) + " " + str(
            vecteur[k][3][0]) + " " + str(
            vecteur[k][3][1]) + " " + str(
            vecteur[k][3][2]) + " " + str(
            vecteur[k][3][3])
        message = message + "\n"
    return message


def main(args):
    """
        main
    """

    rospy.init_node('tools_pepper', anonymous=True)

    value = args[1]
    namespace = args[2]
    mark = args[3]
    mark_init = args[4]
    robot_part = args[5]
    ip = args[6]
    port = args[7]

    CAMERA_NAME = "axis_camera"
    # ROBOT_REF = "base_link"
    ROBOT_FOOT = "base_footprint"
    MAP = "/map"
    FILE_SAVED_MARK = "saved_mark.txt"
    FILE_SAVED_INIT_PLAN = "saved_init_plan.txt"
    FILE_SAVED_MARK_TO_ROBOT = "saved_mark_to_robot.txt"
    TIMER = 0.2
    # MARKER_NAME = "4x4_"
    MARKER_NAME = "ar_marker_"
    NAMESPACE = ["robot1"]

    chdir(value)
    ToolsPepper(value, namespace, mark, mark_init, robot_part, ip, port)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
        # mon_fichier_speed.close()

        print "Finished."


if __name__ == '__main__':
    main(sys.argv)
