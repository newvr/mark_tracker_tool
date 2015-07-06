#!/usr/bin/env python
import roslib
import rospy
import tf
import geometry_msgs.msg
from tf.msg import tfMessage
from tf.transformations import quaternion_inverse, quaternion_conjugate

"""
when launching pepper, odom is the begining of the tf tree. We want
our frame "map" ( which is initalized by a mark) to be the origin of the frame.
Base_link would have the parent odom, and the parent mark. It is not possible
so we have to invert the parent and child between base_link and odom to
modify the t tree as we want
"""


def callback(data):
    broadcaster = tf.TransformBroadcaster()
    trans = [data.transforms[0].transform.translation.x,
             data.transforms[0].transform.translation.y,
             data.transforms[0].transform.translation.z]
    rot = [data.transforms[0].transform.rotation.x,
           data.transforms[0].transform.rotation.y,
           data.transforms[0].transform.rotation.z,
           data.transforms[0].transform.rotation.w
           ]

    # euler = euler_from_quaternion(rot)
    # rot_fin = quaternion_from_euler(-euler[0], -euler[1], -euler[2])

    rot_bis = quaternion_conjugate(rot)
    broadcaster.sendTransform(trans, rot, rospy.Time.now(),
                              data.transforms[0].header.frame_id,
                              data.transforms[0].child_frame_id)

if __name__ == '__main__':
    rospy.init_node('invert_tf_odom_parent')
    rospy.Subscriber("tf_relay", tfMessage, callback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
        # mon_fichier_speed.close()

        print "Finished."
