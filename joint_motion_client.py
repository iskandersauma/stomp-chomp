#! /usr/bin/env python

from __future__ import print_function
import rospy
import asp_tools.msg
from geometry_msgs.msg import Pose
import numpy as np

# Brings in the SimpleActionClient
import actionlib

def joint_motion_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('joint_motion', asp_tools.msg.JointMotionAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    x, y, b, z, a = 0.5, 0.05, 1.32, 0.10, np.pi/2

    # Creates a goal to send to the action server.
    goal = asp_tools.msg.JointMotionGoal(x=x, y=y, b=b, z=z, a=a)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('joint_motion_client_py')
        result = joint_motion_client()
        # print("Result:", ', '.join([str(n) for n in result.sequence]))
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
