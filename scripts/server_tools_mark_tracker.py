#!/usr/bin/env python
import roslib
import rospy
import numpy
import time
import sys
from visualization_msgs.msg import Marker

import tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from mark_tracker_tools.srv import *
from os import chdir
# -- variables magiques
TIME_BROADCAST_LISTEN = 0.5
CAMERA_NAME = "axis_camera"
# ROBOT_REF = "base_link"
ROBOT_FOOT = "base_footprint"
MAP = "/map"
FILE_SAVED_MARK = "saved_mark.txt"
# -- variables magiques
chdir("/home/sfress/catkin_ws/src/mark_tracker/launch/")  # to put


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

    def __init__(self):
        # =================
        # self.motionProxy = ALProxy("ALMotion", IP, PORT)
       # self.postureProxy = ALProxy("ALRobotPosture", IP, PORT)
        self.mark_to_save = []

        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()
        # rospy.Subscriber("/result_position", Odometry,self.position_callback)
        # rospy.Timer(rospy.Duration(TIMER_ALMOTION), self.timer_callback)
        self.vect_tf = [[CAMERA_NAME, MAP, [0, 0, 0], [0, 0, 0, 1]]]
        rospy.Subscriber(
            "/cam0/visualization_marker", Marker, self.publish_tf)
        # self.vect_tf = [CAMERA_NAME, [0, 0, 0], [0, 0, 0, 1]]

        rospy.Service(
            'init_plan', InitPlan, self.init_plan)
        rospy.Service(
            'where_is', WhereIs, self.where_is)

        rospy.Service(
            'how_to_go', HowToGo, self.how_to_go)

        rospy.Service(
            'add_mark', AddMark, self.add_mark)

        rospy.Service(
            'how_to_go_to_mark', HowToGoToMark, self.how_to_go_to_mark)

        rospy.Service(
            'init_mark_to_robot', InitMarkToRobot, self.init_mark_to_robot)

        self.len = 0
        # rospy.Service(
        #   'how_to_GoToMark', HowToGoToMark, self.how_to_GoToMark)

    # mais doit marcer ac launch init et relaunch le track no pepper
    def publish_tf(self, data):
        """
        publish tf contained in self.vect_tf
        """
        for i in range(len(self.vect_tf)):
            self.broadcaster.sendTransform(
                self.vect_tf[i][2], self.vect_tf[i][3], rospy.Time.now(),
                self.vect_tf[i][0], self.vect_tf[i][1])
        if self.len != len(self.vect_tf):  # a chaque ajout
            print"************"
            for k in range(len(self.vect_tf)):
                print [self.vect_tf[k][0], self.vect_tf[k][1]]
            print"************"
            self.len = len(self.vect_tf)

    def init_plan(self, req):
        """
            add in vect_mark[0] the coordinates of the camera deduced from
            the position of the mark we choose to be the origin. This step
            enable the trackin in our plan.
        """
        try:
            marker = '/ar_marker_' + str(req.marknumber)
            trans, rot = self.listener.lookupTransform(
                marker, '/map', rospy.Time(0))
            self.vect_tf[0] = [CAMERA_NAME, MAP, trans, rot]
            print " plan intialized !"
            return InitPlanResponse(True)
        except Exception, exc:
            print exc

    def init_mark_to_robot(self, req):
        """
           link the mark to a part of the robot, then every part's coordinates
           would be accessible in our plan
        """

        past1sec = rospy.Time.now() - rospy.Duration(1.0)
        timeout = rospy.Duration(4.0)
        try:
            marker = '/ar_marker_' + str(req.marknumber)
            print "111"
            (trans_foot_to_body,
             rot_foot_to_body) = self.listener.lookupTransform(
                ROBOT_FOOT, req.robotpart, rospy.Time(0))
            # permet de prendre en compte l'inclinaison de la marque par
            # rapport a son positionnement sur la tete
            # jusqua present on ne sait pas ou se situe le robot dans notre
            # referentiel, pour cela on publie des donnees de la ou devrait
            # etre notre robot dans le referentiel de la map

            # les pieds du robot doivent etre ici par rapport a la map
            quaternion = quaternion_from_euler(0, 0, req.theta)
            print "222"
            self.broadcaster.sendTransform(
                (req.x, req.y, 0), quaternion, rospy.Time.now(),
                "/mon_tf/" + ROBOT_FOOT, MAP)
            print "333"

            self.listener.waitForTransform(
                "/mon_tf/" + ROBOT_FOOT, MAP, rospy.Time(), rospy.Duration(4.0))

            # donc notre bodypart serait ici par rapport a la map
            self.broadcaster.sendTransform(
                trans_foot_to_body, rot_foot_to_body, rospy.Time.now(),
                "/mon_tf/" + req.robotpart,
                "/mon_tf/" + ROBOT_FOOT)
            print "444"
            # on a donc maintenant notre body_part et notre mark qui
            # appartiennent au meme arbre : on peut connaitre la
            # transformation qui les separe
            self.listener.waitForTransform(
                marker, "/mon_tf/" + req.robotpart, rospy.Time(), rospy.Duration(4.0))
            # self.vect_tf.append(
            #     ["/mon_tf/" + req.robotpart,  "/mon_tf/" + ROBOT_FOOT,
            #      trans_foot_to_body, rot_foot_to_body])

            print "555"
            (trans_marker_to_body,
                rot_marker_to_body) = self.listener.lookupTransform(
                "/mon_tf/" + req.robotpart, "/mon_tf/" + ROBOT_FOOT,
                rospy.Time())

            print "666"
            self.vect_tf.append(
                [marker, req.robotpart, trans_marker_to_body,
                 rot_marker_to_body])
            return InitMarkToRobotResponse(True)
        except Exception, exc:
            print exc
        # except (tf.LookupException, tf.ConnectivityException,
        #         tf.ExtrapolationException):
        #     print " waiting for tf..."
        # self.init_mark_to_robot(req)

    def write_file(self, req):
        """
            write in a permanent way the tf that has been published
        """
        try:
           # chdir("/home/sfress/catkin_ws/src/mark_tracker/launch/")

            mon_fichier = open("launch_tf_cam_map.launch", "w")
            marker = '/ar_marker_' + str(req.marknumber)
            trans, rot = self.listener.lookupTransform(
                marker, '/map', rospy.Time.now())
            message = str("""
      <launch>
      <node pkg="tf" type="static_transform_publisher"
        name=""")
            message = message + '"' + req.cameraname + \
                '"' + str(""" args=" """)
            message = message + str(trans[0]) + " " + str(
                trans[1]) + " " + str(trans[2]) + " "
            message = message + str(rot[0]) + " " + str(rot[1]) + " " + str(
                rot[2]) + " " + str(rot[3]) + " /map /" + req.cameraname + str(
                """ 30"/>""")
            message = message + "</launch>"
            print "======message saved in launchfile : "
            print message
            print "======"
            mon_fichier.write(message)
            mon_fichier.close()
            # subprocess.call(
            #["roslaunch /home/sfress/catkin_ws/src/mark_tracker/
            # launch/launch_tf_cam_map.launch"])
            return InitPlanResponse(True)
        except Exception, exc:
            print exc

    def where_is(self, req):
        """
        arguments :robot_part,
        return: (x,y,theta) of the robot_part.
        The possible names are tf frames ( visible in rviz for example )
        """
        try:
            trans, rot = self.listener.lookupTransform(
                '/map', req.robotpart, rospy.Time.now())
            euler = euler_from_quaternion(rot)
            return WhereIsResponse(trans[0],
                                   trans[1], euler[2])
        except Exception, exc:
            print"where_is() error"
            print exc

    def how_to_go(self, req):
        """
        Arguments: absolute (x,y,theta) of where we want to go
        return: (x,y,theta) relative to the robot
        """
        try:
            quat = quaternion_from_euler(0, 0, req.theta)
            self.broadcaster.sendTransform(
                [req.x, req.y, 0], quat, rospy.Time(0),
                "/mon_tf/mygoal", "/map")
            time.sleep(TIME_BROADCAST_LISTEN)  # or bug BUG
            # equivaut a un changement de repere
            (trans, rot) = self.listener.lookupTransform("/base_footprint",
                                                         "/mon_tf/mygoal",
                                                         rospy.Time(0))
            euler = euler_from_quaternion(rot)
            return HowToGoResponse(trans[0], trans[1],
                                   euler[2])
        except Exception, e:
            print "how_to_go() error"
            print e

    def add_mark(self, req):
        """
        Arguments:
        marknumber: Number of the mark
        permanent: if we want to save the mark position in a file
        return: True if new or False if overwrited a mark
        """
        try:
            # To be sure to listen to the right thing
            time.sleep(TIME_BROADCAST_LISTEN)
            marker = '/ar_marker_' + str(req.marknumber)
            my_mark_publish = '/my_mark_publish' + str(req.marknumber)
            trans, rot = self.listener.lookupTransform(
                MAP, marker, rospy.Time(0))
            for i in range(0, len(self.vect_tf)):  # if not already
                if self.vect_tf[i][0] != my_mark_publish:
                    self.vect_tf.append([my_mark_publish, MAP, trans, rot])
                    new = True
                    print len(self.vect_tf), " mark learnt"
                else:
                    self.vect_tf[i] = [my_mark_publish, trans, rot]
                    new = False

            if req.permanent == True:
                self.save(my_mark_publish, MAP, trans, rot)

            return AddMarkResponse(new)
        except Exception, e:
            print"add_mark_temporary error"
            print e

    def save(self, my_mark_publish, MAP, trans, rot):
        if len(self.mark_to_save) > 0:
            for i in range(0, len(self.mark_to_save)):  # if not already
                if self.mark_to_save[i][0] != my_mark_publish:
                    self.mark_to_save.append(
                        [my_mark_publish, MAP, trans, rot])
                else:
                    self.mark_to_save[i] = [my_mark_publish, trans, rot]
        else:
            self.mark_to_save.append(
                [my_mark_publish, MAP, trans, rot])
        mon_fichier = open(FILE_SAVED_MARK, "w")
        message = ""
        for k in range(0, len(self.mark_to_save)):
            print "iiiii"
            message = message + str(self.mark_to_save[k][0]) + "&" + str(
                self.mark_to_save[k][1]) + "&" + str(
                self.mark_to_save[k][2]) + "&" + str(
                self.mark_to_save[k][3])

            message = message + "\n"

        print message
        mon_fichier.write(message)
        mon_fichier.close()

    def load_mark(self):

        fichier = numpy.genfromtxt(FILE_SAVED_MARK, skiprows=1, delimiter='&')
        frame_base = fichier[:, 0]
        frame_child = fichier[:, 1]
        trans = fichier[:, 2]
        ros = fichier[:, 3]
        # delete the current vector broadcasted
        size = len(self.vect_tf)
        for i in range(0, size - 2):
            self.vect_tf.pop()
            i = i + 1
        self.vect_tf.append()

    def how_to_go_to_mark(self, req):
        """
        Arguments: Number of the mark
        return: (x,y,theta) relative to the robot to go to the mark
        """
        try:
            my_mark_publish = '/my_mark_publish' + str(req.marknumber)
            (trans, rot) = self.listener.lookupTransform("/base_footprint",
                                                         my_mark_publish,
                                                         rospy.Time.now())
            euler = euler_from_quaternion(rot)
            return HowToGoToMarkResponse(trans[0], trans[1], euler[2])

        except Exception, e:
            print "howtgotomark() error, is ", my_mark_publish, " saved?"
            print e


def main(args):
    """
        main
    """

    rospy.init_node('tools_pepper', anonymous=True)
    ToolsPepper()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"

        print "Finished."


if __name__ == '__main__':
    main(sys.argv)
