#! /usr/bin/env python
import rospy
# Brings in the SimpleActionClient
import actionlib
# Brings in the messages used by the GoTo action, including the
# goal message and the result message.
import mark_tracker_tools.msg


client = actionlib.SimpleActionClient(
    'GoToPlace', mark_tracker_tools.msg.GoToAction)


def GoTo_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (GoToAction) to the constructor.

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.

    goal = mark_tracker_tools.msg.GoToGoal(order=[0.4, 6, 0.017])
    print "goal =", goal
    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()
    print client.get_result()
    # Prints out the result of executing the action
    return client.get_result()  # A GoToResult


def myhook():
    print "shutdown time!"


if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('GoTo_client_py')
        result = GoTo_client()
        # print "Result:", ', '.join([str(n) for n in result.sequence])
        # client.cancel_goal()
        if rospy.is_shutdown() == True:
            myhook()
    except rospy.ROSInterruptException:

        print "program interrupted before completion"
