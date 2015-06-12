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
FILE_SAVED_INIT_PLAN = "saved_init_plan.txt"
FILE_SAVED_MARK_TO_ROBOT = "saved_mark_to_robot.txt"
TIMER = 0.2
# MARKER_NAME = "4x4_"
MARKER_NAME = "ar_marker_"

mon_fichier_speed = open("test_vitesse.txt", "w")
# -- variables magiques


chdir("/home/sfress/catkin_ws/src/mark_tracker_tools/saved_tf/")


def sign(var):
    """
    signe function
    """
    if var > 0:
        return 1
    if var == 0:
        return 0
    return -1


class State:

    """
    frame
    timeold
    positionold
    rotationold
    timenew
    positionnew
    rotationnew
    """

    def __init__(self, frame):
        "constructeur avec specification du frame"

        self.compteur = 0
        self.time_init = rospy.Time.now()
        self.frame = frame
        self.time_old = rospy.Time.now()
        self.position_old = [0.0, 0.0, 0.0]
        self.rotation_old = [0.0, 0.0, 0.0, 1.0]

        self.time_new = rospy.Time.now()
        self.position_new = [0.0, 0.0, 0.0]
        self.rotation_new = [0.0, 0.0, 0.0, 1.0]
        self.vitesse = [0.0, 0.0, 0.0]

    def print_info(self):
        print self.frame
        print self.time_old
        print self.position_old
        print self.rotation_old

        print self.time_new
        print self.position_new
        print self.rotation_new
        print self.vitesse

    def maj_state(self, postion, rotation):
        """
            save newer data in "new" and save the old ones in "old"
        """
        if self.compteur == 3:
            self.time_old = self.time_new
            self.position_old = self.position_new
            self.rotation_old = self.rotation_new

            self.time_new = rospy.Time.now()
            self.position_new = postion
            self.rotation_new = rotation
            self.compteur = 0
            self.calcul_speed_lin()

        self.compteur += 1

    def calcul_speed_lin(self):
        """
        look the distance made during delta_t and calcul the speed of the robot
        """
        # in nanoseconds //* 10 ** -9

        delta_t = (self.time_new - self.time_old).nsecs
        delta_position = [
            self.position_new[0] - self.position_old[0],
            self.position_new[1] - self.position_old[1],
            self.position_new[2] - self.position_old[2]]

        if delta_position[0] != 0 and delta_position[
                1] != 0 and delta_position[2] != 0:

            self.vitesse = [
                delta_position[0] / delta_t,
                delta_position[1] / delta_t,
                delta_position[2] / delta_t]

            print "#################"

            message = str(
                self.time_new.secs + self.time_new.nsecs * 10 ** -9) + " "
            message = message + str(
                self.vitesse[0]) + " " + str(
                self.vitesse[1]) + " " + str(self.vitesse[2])
            message = message + "\n"
            mon_fichier_speed.write(message)

            print delta_position
            print self.vitesse[0]
            print self.vitesse[1]
            print self.vitesse[2]
            print "-----------------"


class ToolsPepper:

    """
    contains all services that can be called by the client
    """

    def __init__(self):
        # =================
        # self.motionProxy = ALProxy("ALMotion", IP, PORT)
       # self.postureProxy = ALProxy("ALRobotPosture", IP, PORT)
        self.mark_to_save = []
        # flag actived by service init_mark_to_robot to bring robot
        self.link_to_robot = False
        # flag activated by service init_track_speed to specify which mark we
        # want to track
        self.speed_tracker_activated = False
        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()
        self.state_speed_calcul = State(" ")
        # rospy.Subscriber("/result_position", Odometry,self.position_callback)
        # rospy.Timer(rospy.Duration(TIMER_ALMOTION), self.timer_callback)
        self.vect_tf = [[CAMERA_NAME, MAP, [0, 0, 0], [0, 0, 0, 1]], [
            "ini", "child", [0, 0, 0], [0, 0, 0, 1]]]
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

        rospy.Service(
            'load_mark', LoadMark, self.load_mark)

        rospy.Service(
            'load_init', LoadInit, self.load_init)

        rospy.Service(
            'init_track_speed', InitTrackSpeed, self.init_track_speed)

        rospy.Service(
            'ask_tf', AskTf, self.ask_tf)

        self.len = 0
        # rospy.Service(
        #   'how_to_GoToMark', HowToGoToMark, self.how_to_GoToMark)

    # mais doit marcer ac launch init et relaunch le track no pepper

    def publish_tf(self, data):
        """
        publish tf contained in self.vect_tf
        This is called while a mark is detected

        In this function we put things that need to be looped ( as
        publish tf or calcul speed )
        """
        # print self.vect_tf
        for i in range(len(self.vect_tf)):
            self.broadcaster.sendTransform(
                self.vect_tf[i][2], self.vect_tf[i][3], rospy.Time.now(),
                self.vect_tf[i][0], self.vect_tf[i][1])

        if self.len != len(self.vect_tf):  # PRINT a chaque ajout
            print"************"
            for k in range(len(self.vect_tf)):
                print [self.vect_tf[k][0], self.vect_tf[k][1]]
            print"************"
            self.len = len(self.vect_tf)

        ##### ici partie pour calculer la vitesse du frame choisit#####
        if self.speed_tracker_activated == True:
            (trans, rot) = self.listener.lookupTransform(
                self.state_speed_calcul.frame, MAP, rospy.Time(0))
            self.state_speed_calcul.maj_state(trans, rot)

        if self.link_to_robot == True:
            self.func_link_to_robot()

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

            # get the relation to set to make the link between base_link and
            # our map
            name = str(self.vect_tf[1][0]).split("/")
            robot_part = name[1]
            (trans, rot) = self.listener.lookupTransform(
                robot_part, "base_link", rospy.Time(0))
            # ou devrait donc etre notre base_link par rapport a notre mark: on
            # cree un base_link "virtuel"
            self.broadcaster.sendTransform(
                trans, rot, rospy.Time.now(),
                "/mon_tf/base_link", "/mon_tf/" + robot_part)
            # comment est notre base_link virtuel apr rapport a notre map?
            (trans_fin, rot_fin) = self.listener.lookupTransform(
                "mon_tf/base_link", MAP, rospy.Time(0))
            self.broadcaster.sendTransform(
                trans_fin, rot_fin, rospy.Time.now(), MAP, "/base_link")

            # creation d'un frame pour les data de l'odom dans notre plan
            (trans_odom, rot_odom) = self.listener.lookupTransform(
                "base_link", "odom", rospy.Time(0))
            self.broadcaster.sendTransform(
                trans_odom, rot_odom, rospy.Time.now(), "/odom_base_link", MAP)

        except Exception, exc:
            print " waiting for tf..."


# ============================= ROS SERVICES ======================

    def init_plan(self, req):
        """
            add in vect_mark[0] the coordinates of the camera deduced from
            the position of the mark we choose to be the origin. This step
            enable the trackin in our plan.
        """
        try:
            marker = MARKER_NAME + str(req.marknumber)
            trans, rot = self.listener.lookupTransform(
                marker, '/map', rospy.Time(0))
            self.vect_tf[0] = [CAMERA_NAME, MAP, trans, rot]
            print " plan intialized !"
            if req.permanent == True:
                mon_fichier = open(FILE_SAVED_INIT_PLAN, "w")
                message = write_message([self.vect_tf[0]])
                mon_fichier.write(message)
                mon_fichier.close()
            return InitPlanResponse(True)
        except Exception, exc:
            print exc

    def init_mark_to_robot(self, req):
        """
           link the mark to a part of the robot, then every frames'coordinates
           would be accessible in our plan
        """
        past1sec = rospy.Time.now() - rospy.Duration(1.0)
        timeout = rospy.Duration(4.0)
        ret = False
        while ret != True:
            ret = self.init_mark_to_robot_publish_temporary(req)
        self.link_to_robot = True

        if req.permanent == True:
            mon_fichier = open(FILE_SAVED_MARK_TO_ROBOT, "w")
            message = write_message([self.vect_tf[1]])
            mon_fichier.write(message)
            mon_fichier.close()
        return InitMarkToRobotResponse(True)

    def init_mark_to_robot_publish_temporary(self, req):
        """
            Just called by InitMarkToRobot:
            we need to publish the tf several times, make sure that you're
            repeatedly publishing the positions of the frames so that tf can
            correctly interpolate. If you only publish single datapoints for
            each frame_id at different times a lookup will never succeed
        """
        try:
            marker = MARKER_NAME + str(req.marknumber)
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
            self.broadcaster.sendTransform(
                (req.x, req.y, 0), quaternion, rospy.Time.now(),
                "mon_tf/" + ROBOT_FOOT, MAP)
            self.listener.waitForTransform(
                "mon_tf/" + ROBOT_FOOT, MAP, rospy.Time(0), rospy.Duration(1.0))
            # donc notre bodypart serait ici par rapport a la map
            self.broadcaster.sendTransform(
                trans_foot_to_body, rot_foot_to_body, rospy.Time.now(),
                "mon_tf/" + req.robotpart, "mon_tf/" + ROBOT_FOOT)
            self.listener.waitForTransform(
                marker, "mon_tf/" + req.robotpart,
                rospy.Time(0), rospy.Duration(1.0))
            (trans_marker_to_body,
                rot_marker_to_body) = self.listener.lookupTransform(
                marker, "mon_tf/" + req.robotpart, rospy.Time(0))
            self.vect_tf[1] = ["mon_tf/" + req.robotpart, marker,
                               trans_marker_to_body, rot_marker_to_body]
            return True
        except Exception, exc:
            print " waiting for tf..."

    def where_is(self, req):
        """
        arguments :robot_part,
        return: (x,y,theta) of the robot_part.
        The possible names are tf frames ( visible in rviz for example )
        """
        try:
            trans, rot = self.listener.lookupTransform(
                '/map', req.robotpart, rospy.Time(0))
            euler = euler_from_quaternion(rot)

            return WhereIsResponse(trans[0],
                                   trans[1], euler[2], True)
        except Exception, exc:
            print"where_is() error"
            print exc
            return WhereIsResponse(0,
                                   0, 0, False)

    def how_to_go(self, req):
        """
        Arguments: absolute (x,y,theta) of where we want to go
        return: (x,y,theta) relative to the robot
        """

        ret = False
        while ret != True:
            (trans, rot, ret) = self.how_to_go_publish_temp(req)
        euler = euler_from_quaternion(rot)

        return HowToGoResponse(trans[0], trans[1],
                               euler[2])

    def how_to_go_publish_temp(self, req):
        try:
            quat = quaternion_from_euler(0, 0, req.theta)
            self.broadcaster.sendTransform(
                [req.x, req.y, 0], quat, rospy.Time.now(),
                "/mon_tf/mygoal", MAP)
            print "1111"
            # equivaut a un changement de repere
            (trans, rot) = self.listener.lookupTransform("/base_footprint",
                                                         "/mon_tf/mygoal",
                                                         rospy.Time(0))

            print "222"
            return(trans, rot, True)
        except Exception, exc:
            print " waiting for tf..."
            return(0, 0, False)

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
            marker = MARKER_NAME + str(req.marknumber)
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
                self.save_mark(my_mark_publish, trans, rot)

            return AddMarkResponse(new)
        except Exception, e:
            print"add_mark_temporary error"
            print e

    def save_mark(self, my_mark_publish, trans, rot):
        """
        called by add mark if permanent==True. It checks if we already tried
        to save our mark before to write it in the file  then it overwrite it)
        """
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
        message = write_message(self.mark_to_save)
        mon_fichier.write(message)
        mon_fichier.close()

    def load_mark(self, req):
        """
        take in arguments the path of the file where the "saved_mark.txt" is
        and publish the corresponding tf
        """
        chdir(req.path)  # to put
        temp_vect = []
        temp_vect.append(self.vect_tf[0])
        temp_vect.append(self.vect_tf[1])
        fichier = numpy.genfromtxt(FILE_SAVED_MARK, delimiter=' ', dtype=None)
        if fichier.ndim == 0:
            fichier = fichier.ravel()
        for k in range(0, len(fichier)):
            toadd = [fichier[k][0], fichier[k][1],
                     [fichier[k][i] for i in range(2, 5)],
                     [fichier[k][i] for i in range(5, 9)]]
            temp_vect.append(toadd)
        print temp_vect
        self.vect_tf = temp_vect
        return LoadMarkResponse(True)

    def load_init(self, req):
        """
        if req.init=0, we initialize the plan with the file
        "saved_init_plan.txt" is. if req.init=1, we initialize the relation
         between the mark and the robot with "saved_mark_to_robot.txt"

        """
        if req.init == 0:
            indice = 0
            doc = FILE_SAVED_INIT_PLAN
        if req.init == 1:
            indice = 1
            doc = FILE_SAVED_MARK_TO_ROBOT
            self.link_to_robot = True

        chdir(req.path)
        fichier = numpy.genfromtxt(
            doc, delimiter=' ', dtype=None)
        fichier = fichier.ravel()
        toadd = [fichier[0][0], fichier[0][1],
                 [fichier[0][i] for i in range(2, 5)],
                 [fichier[0][i] for i in range(5, 9)]]
        self.vect_tf[indice] = toadd
        return LoadInitResponse(True)

    def how_to_go_to_mark(self, req):
        """
        Arguments: Number of the mark
        return: (x,y,theta) relative to the robot to go to the mark
        """
        try:
            my_mark_publish = '/my_mark_publish' + str(req.marknumber)
            (trans, rot) = self.listener.lookupTransform("/base_footprint",
                                                         my_mark_publish,
                                                         rospy.Time(0))
            euler = euler_from_quaternion(rot)

            if trans[2] > 0.3:  # si bug, on tourne juste un peu
                print "glitch iif "
                return HowToGoToMarkResponse(0, 0, 0.3, False)
            else:
                print'ok'
                return HowToGoToMarkResponse(trans[0], trans[1], euler[2], True)

        except Exception, e:
            print "howtgotomark() error, is ", my_mark_publish, " saved?"
            print e

    def init_track_speed(self, req):
        """
        Arguments: name of the frame to track( if it is the
        mark or the robot's feets)
        return: true if frame exists
        """
        try:
            (trans, rot) = self.listener.lookupTransform(req.frame,
                                                         MAP,
                                                         rospy.Time(0))
            # if listener OK :
            self.state_speed_calcul = State(req.frame)
            self.speed_tracker_activated = True
            return InitTrackSpeedResponse(True)

        except Exception, e:
            print "init_track_speed() error, is ", req.frame, " saved?"
            print e

    def ask_tf(self, req):
        try:
            (trans, rot) = self.listener.lookupTransform(req.frameA,
                                                         req.frameB,
                                                         rospy.Time(0))
            # if listener OK :
            euler = euler_from_quaternion(rot)
            return AskTfResponse(trans[0], trans[1], euler[2])

        except Exception, e:
            print "ask_tf() error, is ", req.frameA, "or ", req.frameB, "published"
            print e


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
    ToolsPepper()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
        mon_fichier_speed.close()

        print "Finished."


if __name__ == '__main__':
    main(sys.argv)

    # maybe usefull later
    # def write_file(self, req):
    #     """
    #         write in a permanent launchfile the tf that has been published
    #     """
    #     try:
    # chdir("/home/sfress/catkin_ws/src/mark_tracker/launch/")

    #         mon_fichier = open("launch_tf_cam_map.launch", "w")
    #         marker = '/ar_marker_' + str(req.marknumber)
    #         trans, rot = self.listener.lookupTransform(
    #             marker, '/map', rospy.Time.now())
    #         message = str("""
    #   <launch>
    #   <node pkg="tf" type="static_transform_publisher"
    #     name=""")
    #         message = message + '"' + req.cameraname + str(
    #             '"') + str(""" args=" """)
    #         message = message + str(trans[0]) + " " + str(
    #             trans[1]) + " " + str(trans[2]) + " "
    #         message = message + str(rot[0]) + " " + str(rot[1]) + " " + str(
    #             rot[2]) + " " + str(
    # rot[3]) + " /map /" + req.cameraname + str(
    #             """ 30"/>""")
    #         message = message + "</launch>"
    #         print "======message saved in launchfile : "
    #         print message
    #         print "======"
    #         mon_fichier.write(message)
    #         mon_fichier.close()
    # subprocess.call(
    # ["roslaunch /home/sfress/catkin_ws/src/mark_tracker/
    # launch/launch_tf_cam_map.launch"])
    #         return InitPlanResponse(True)
    #     except Exception, exc:
    #         print exc
