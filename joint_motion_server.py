#! /usr/bin/env python
import rospy
import actionlib
import asp_tools.msg
from asp_tools.srv import MoveJoints
from abstract_motion_server import AbstractMotionServer

class JointMotionAction(AbstractMotionServer):

    def __init__(self, name):
        super(JointMotionAction, self).__init__(name)

    def _init_server(self):
        self._feedback = asp_tools.msg.JointMotionFeedback()
        self._result = asp_tools.msg.JointMotionResult()
        self._as = actionlib.SimpleActionServer(self._action_name, asp_tools.msg.JointMotionAction, execute_cb=self.execute_cb, auto_start = False)

    def call_service(self, goal):
        """
        Calls the motion service. Returns true if the call wass successfull,
        false otherwise.
        """
        # publish info to the console for the user
        rospy.loginfo('%s: Executing the joint motion action' % (self._action_name))
        rospy.wait_for_service('/asp/move_joints')
        try:
            self.plan_executed = False
            move_joints = rospy.ServiceProxy('/asp/move_joints', MoveJoints)
            resp = move_joints(x=goal.x, y=goal.y, b=goal.b, z=goal.z, a=goal.a, async=True)
            executed, planned = resp.executed, resp.planned
        except rospy.ServiceException, e:
            print "move_joints service call failed: %s"%e
            executed, planned = False, False

        return executed, planned

if __name__ == '__main__':
    rospy.init_node('joint_motion')
    server = JointMotionAction(rospy.get_name())
    rospy.spin()
