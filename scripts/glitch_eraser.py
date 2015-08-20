#!/usr/bin/env python
import roslib
import rospy
import numpy
import time
import sys
from gazebo_msgs.msg import ModelState
from visualization_msgs.msg import Marker
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty, EmptyResponse
import tf
from tf.msg import tfMessage
from tf.transformations import (quaternion_from_euler,
                                euler_from_quaternion)
from mark_tracker_tools.srv import *
from os import chdir
import functools
import math


def sign(var):
    """
    signe function
    """
    if var > 0:
        return 1
    if var == 0:
        return 0
    return -1


class GlitchEraser:

    """
    contains all services that can be called by the client
    """

    def __init__(self):

        self.old_vector = []

        # self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()
        self.pub_tf_track = rospy.Publisher(
            "/tf_relay_track", tfMessage, queue_size=1)

        rospy.Subscriber("tf_relay_track", tfMessage, self.callback)

    def callback(self, data):

        trans = [data.transforms[0].transform.translation.x,
                 data.transforms[0].transform.translation.y,
                 data.transforms[0].transform.translation.z]
        rot = [data.transforms[0].transform.rotation.x,
               data.transforms[0].transform.rotation.y,
               data.transforms[0].transform.rotation.z,
               data.transforms[0].transform.rotation.w
               ]

        transformer = tf.TransformerROS()
        mat_A = numpy.matrix(transformer.fromTranslationRotation(trans,
                                                                 rot))
        mat_B = numpy.matrix(
            transformer.fromTranslationRotation((0, 0, -1.5),
                                                (0, 0, 0, 1)))

        mat_C = mat_A * mat_B

        vect = numpy.matrix([[1, 1, 1, 1]])
        vectC = vect * mat_C
        vectA = vect * mat_A

        print "vectC", vectC, "vectA", vectA
        print "A", mat_A
        print "B", mat_B
        print "C", mat_C
        print "000", mat_C[0].item(3)
        print "111", mat_C[1].item(3)
        print "222", mat_C[2].item(3)

        # print (data.transforms[0].child_frame_id,
        #        data.transforms[0].header.frame_id,
        #        trans, euler_from_quaternion(rot))

        dist = math.sqrt(
            math.pow(mat_A[0].item(3),
                     2) + math.pow(mat_A[1].item(3),
                                   2) + math.pow(mat_A[2].item(3), 2))

        dist_bis = math.sqrt(
            math.pow(mat_C[0].item(3),
                     2) + math.pow(mat_C[1].item(3),
                                   2) + math.pow(mat_C[2].item(3), 2))

        print "dist=", dist, "dist_bis=", dist_bis
        # dist_bis = math.sqrt(
        #     math.pow(trans_bis[0] - trans[0],
        #              2) + math.pow(trans_bis[1] - trans[1],
        #                            2) + math.pow(trans_bis[2] - trans[2], 2))

        # print "dist", dist, "dist_bis", dist_bis

        print data.transforms[0].child_frame_id, euler_from_quaternion(rot)

        if vectC[0].item(3) < vectA[0].item(3):
            print "GLITCH"
        else:

            self.broadcaster.sendTransform(trans, rot, rospy.Time.now(),
                                           data.transforms[0].child_frame_id,
                                           data.transforms[0].header.frame_id)


def main(args):
    """
        main
    """

    rospy.init_node('glitch_eraser', anonymous=True)

    GlitchEraser()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
        # mon_fichier_speed.close()

        print "Finished."


if __name__ == '__main__':
    main(sys.argv)
