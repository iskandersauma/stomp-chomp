#!/usr/bin/env python
import rospy
from moveit_commander import RobotCommander, PlanningSceneInterface
import geometry_msgs.msg
import time
import sys

class CollisionSceneExample(object):

    def __init__(self):
        self._scene = PlanningSceneInterface()

        # clear the scene
        self._scene.remove_world_object()

        self.robot = RobotCommander()

        # pause to wait for rviz to load
        print "============ Waiting while RVIZ displays the scene with obstacles..."

        # TODO: need to replace this sleep by explicitly waiting for the scene to be updated.
        rospy.sleep(2)


    def add_pap1(self):
	floor_pose = [1.0, -1.0, -0.17, 0, 0, 0, 1]
	floor_dimensions = [5.0, 4.0, 0.02]

 	wall1_pose = [1.0, 1.0, 0.92, 0, 0, 0, 1]
        wall1_dimensions = [5.0,0.02, 2.18]


	wall2_pose = [3.5, -1.0, 0.92, 0, 0, 0, 1]
	wall2_dimensions = [0.02, 4.0, 2.18]

 	wall3_pose = [-1.5, -1.0, 1.0, 0, 0, 0, 1]
        wall3_dimensions = [0.02,4.0, 2.18]

 	obj1_pose = [0.3, 0.4, -0.08, 0, 0, 0, 1]
        obj1_dimensions = [0.27, 0.4, 0.12]

 	obj2_pose = [0.3, -0.01, -0.08, 0, 0, 0, 1]
        obj2_dimensions = [0.27, 0.4, 0.12]

 	obj3_pose = [0.6, 0.4, -0.08, 0, 0, 0, 1]
        obj3_dimensions = [0.27, 0.4, 0.12]

 	obj4_pose = [0.6, -0.01, -0.08, 0, 0, 0, 1]
        obj4_dimensions = [0.27, 0.4, 0.12]

 	obj5_pose = [0.45, 0.10, 0.05, 0, 0, 0, 1]
        obj5_dimensions = [0.27, 0.4, 0.12]

 	con_bottom_pose = [1.9, -0.5, 0.012, 0, 0, 0, 1]
        con_bottom_dimensions = [0.704, 0.80, 0.02]

 	con_left_pose = [1.9, -0.9, 0.705, 0, 0, 0, 1]
        con_left_dimensions = [0.744, 0.044, 1.386]

 	con_right_pose = [2.252, -0.5, 0.705, 0, 0, 0, 1]
        con_right_dimensions = [0.044, 0.8, 1.386]

 	con_back_pose = [1.548, -0.5, 0.705, 0, 0, 0, 1]
        con_back_dimensions = [0.044, 0.8, 1.386]


        self.add_box_object("wall1", wall1_dimensions, wall1_pose)
	self.add_box_object("floor", floor_dimensions, floor_pose)
        self.add_box_object("wall3", wall3_dimensions, wall3_pose)
        self.add_box_object("wall2", wall2_dimensions, wall2_pose)
        self.add_box_object("obj3", obj3_dimensions, obj3_pose)
        self.add_box_object("obj2", obj2_dimensions, obj2_pose)
        self.add_box_object("obj1", obj1_dimensions, obj1_pose)
        self.add_box_object("obj4", obj4_dimensions, obj4_pose)
        self.add_box_object("obj5", obj5_dimensions, obj5_pose)
        self.add_box_object("con_bottom", con_bottom_dimensions, con_bottom_pose)
        self.add_box_object("con_left", con_left_dimensions, con_left_pose)
        self.add_box_object("con_right", con_right_dimensions, con_right_pose)
        self.add_box_object("con_back", con_back_dimensions, con_back_pose)

        print "========== Added test 1 scene!!"

    def add_pap2(self):
	floor_pose = [1.0, -1.0, -0.17, 0, 0, 0, 1]
	floor_dimensions = [5.0, 4.0, 0.02]

 	wall1_pose = [1.0, 1.0, 0.92, 0, 0, 0, 1]
        wall1_dimensions = [5.0,0.02, 2.18]


	wall2_pose = [3.5, -1.0, 0.92, 0, 0, 0, 1]
	wall2_dimensions = [0.02, 4.0, 2.18]

 	wall3_pose = [-1.5, -1.0, 1.0, 0, 0, 0, 1]
        wall3_dimensions = [0.02,4.0, 2.18]

 	obj1_pose = [0.3, 0.4, -0.08, 0, 0, 0, 1]
        obj1_dimensions = [0.27, 0.4, 0.12]

 	obj2_pose = [0.3, -0.01, -0.08, 0, 0, 0, 1]
        obj2_dimensions = [0.27, 0.4, 0.12]

 	obj3_pose = [0.6, 0.4, -0.08, 0, 0, 0, 1]
        obj3_dimensions = [0.27, 0.4, 0.12]

 	obj4_pose = [0.6, -0.01, -0.08, 0, 0, 0, 1]
        obj4_dimensions = [0.27, 0.4, 0.12]

 	obj5_pose = [0.45, 0.10, 0.05, 0, 0, 0, 1]
        obj5_dimensions = [0.27, 0.4, 0.12]

 	obs_pose = [1.7, 0.20, 0.35, 0, 0, 0, 1]
        obs_dimensions = [0.1, 0.1, 0.9]


 	con_bottom_pose = [1.9, -0.5, 0.012, 0, 0, 0, 1]
        con_bottom_dimensions = [0.704, 0.80, 0.02]

 	con_left_pose = [1.9, -0.9, 0.705, 0, 0, 0, 1]
        con_left_dimensions = [0.744, 0.044, 1.386]

 	con_right_pose = [2.252, -0.5, 0.705, 0, 0, 0, 1]
        con_right_dimensions = [0.044, 0.8, 1.386]

 	con_back_pose = [1.548, -0.5, 0.705, 0, 0, 0, 1]
        con_back_dimensions = [0.044, 0.8, 1.386]


        self.add_box_object("wall1", wall1_dimensions, wall1_pose)
	self.add_box_object("floor", floor_dimensions, floor_pose)
        self.add_box_object("wall3", wall3_dimensions, wall3_pose)
        self.add_box_object("wall2", wall2_dimensions, wall2_pose)
        self.add_box_object("obj3", obj3_dimensions, obj3_pose)
        self.add_box_object("obj2", obj2_dimensions, obj2_pose)
        self.add_box_object("obj1", obj1_dimensions, obj1_pose)
        self.add_box_object("obj4", obj4_dimensions, obj4_pose)
        self.add_box_object("obj5", obj5_dimensions, obj5_pose)
        self.add_box_object("con_bottom", con_bottom_dimensions, con_bottom_pose)
        self.add_box_object("con_left", con_left_dimensions, con_left_pose)
        self.add_box_object("con_right", con_right_dimensions, con_right_pose)
        self.add_box_object("con_back", con_back_dimensions, con_back_pose)
        self.add_box_object("obs", obs_dimensions, obs_pose)

        print "========== Added test 2 scene!!"

    def add_pap3(self):
	floor_pose = [1.0, -1.0, -0.17, 0, 0, 0, 1]
	floor_dimensions = [5.0, 4.0, 0.02]

 	wall1_pose = [1.0, 1.0, 0.92, 0, 0, 0, 1]
        wall1_dimensions = [5.0,0.02, 2.18]


	wall2_pose = [3.5, -1.0, 0.92, 0, 0, 0, 1]
	wall2_dimensions = [0.02, 4.0, 2.18]

 	wall3_pose = [-1.5, -1.0, 0.92, 0, 0, 0, 1]
        wall3_dimensions = [0.02,4.0, 2.18]

 	obj1_pose = [0.3, 0.6, -0.08, 0, 0, 0, 1]
        obj1_dimensions = [0.27, 0.4, 0.12]


 	obj2_pose = [0.3, 0.21, -0.08, 0, 0, 0, 1]
        obj2_dimensions = [0.27, 0.4, 0.12]

 	obj3_pose = [0.6, 0.6, -0.08, 0, 0, 0, 1]
        obj3_dimensions = [0.27, 0.4, 0.12]

 	obj4_pose = [0.6, 0.21, -0.08, 0, 0, 0, 1]
        obj4_dimensions = [0.27, 0.4, 0.12]

 	obj5_pose = [0.45, 0.40, 0.05, 0, 0, 0, 1]
        obj5_dimensions = [0.27, 0.4, 0.12]


 	con_bottom_pose = [1.9, -0.5, 0.012, 0, 0, 0, 1]
        con_bottom_dimensions = [0.704, 0.80, 0.02]

 	con_left_pose = [1.9, -0.9, 0.705, 0, 0, 0, 1]
        con_left_dimensions = [0.744, 0.044, 1.386]

 	con_right_pose = [2.252, -0.5, 0.705, 0, 0, 0, 1]
        con_right_dimensions = [0.044, 0.8, 1.386]

 	con_back_pose = [1.548, -0.5, 0.705, 0, 0, 0, 1]
        con_back_dimensions = [0.044, 0.8, 1.386]
 	

	con2_bottom_pose = [0.3, -0.5, 0.012, 0, 0, 0, 1]
        con2_bottom_dimensions = [0.704, 0.80, 0.02]

 	con2_left_pose = [0.3, -0.9, 0.705, 0, 0, 0, 1]
        con2_left_dimensions = [0.744, 0.044, 1.386]

 	con2_right_pose = [0.652, -0.5, 0.705, 0, 0, 0, 1]
        con2_right_dimensions = [0.044, 0.8, 1.386]

 	con2_back_pose = [-0.052, -0.5, 0.705, 0, 0, 0, 1]
        con2_back_dimensions = [0.044, 0.8, 1.386]


	con3_bottom_pose = [1.1, -0.5, 0.012, 0, 0, 0, 1]
        con3_bottom_dimensions = [0.704, 0.80, 0.02]

 	con3_left_pose = [1.1, -0.9, 0.705, 0, 0, 0, 1]
        con3_left_dimensions = [0.744, 0.044, 1.386]

 	con3_right_pose = [1.452, -0.5, 0.705, 0, 0, 0, 1]
        con3_right_dimensions = [0.044, 0.8, 1.386]

 	con3_back_pose = [0.748, -0.5, 0.705, 0, 0, 0, 1]
        con3_back_dimensions = [0.044, 0.8, 1.386]



        self.add_box_object("wall1", wall1_dimensions, wall1_pose)
	self.add_box_object("floor", floor_dimensions, floor_pose)
        self.add_box_object("wall3", wall3_dimensions, wall3_pose)
        self.add_box_object("wall2", wall2_dimensions, wall2_pose)
        self.add_box_object("obj3", obj3_dimensions, obj3_pose)
        self.add_box_object("obj2", obj2_dimensions, obj2_pose)
        self.add_box_object("obj1", obj1_dimensions, obj1_pose)
        self.add_box_object("obj4", obj4_dimensions, obj4_pose)
        self.add_box_object("obj5", obj5_dimensions, obj5_pose)

        self.add_box_object("con_bottom", con_bottom_dimensions, con_bottom_pose)
        self.add_box_object("con_left", con_left_dimensions, con_left_pose)
        self.add_box_object("con_right", con_right_dimensions, con_right_pose)
        self.add_box_object("con_back", con_back_dimensions, con_back_pose)

        self.add_box_object("con2_bottom", con2_bottom_dimensions, con2_bottom_pose)
        self.add_box_object("con2_left", con2_left_dimensions, con2_left_pose)
        self.add_box_object("con2_right", con2_right_dimensions, con2_right_pose)
        self.add_box_object("con2_back", con2_back_dimensions, con2_back_pose)

        self.add_box_object("con3_bottom", con3_bottom_dimensions, con3_bottom_pose)
        self.add_box_object("con3_left", con3_left_dimensions, con3_left_pose)
        self.add_box_object("con3_right", con3_right_dimensions, con3_right_pose)
        self.add_box_object("con3_back", con3_back_dimensions, con3_back_pose)


        print "========== Added test 3 scene!!"

    def add_pap4(self):
	floor_pose = [1.0, -1.0, -0.17, 0, 0, 0, 1]
	floor_dimensions = [5.0, 4.0, 0.02]

 	wall1_pose = [1.0, 1.0, 0.92, 0, 0, 0, 1]
        wall1_dimensions = [5.0,0.02, 2.18]


	wall2_pose = [3.5, -1.0, 0.92, 0, 0, 0, 1]
	wall2_dimensions = [0.02, 4.0, 2.18]

 	wall3_pose = [-1.5, -1.0, 0.92, 0, 0, 0, 1]
        wall3_dimensions = [0.02,4.0, 2.18]

 	obj1_pose = [0.3, 0.6, -0.08, 0, 0, 0, 1]
        obj1_dimensions = [0.27, 0.4, 0.12]


 	obj2_pose = [0.3, 0.21, -0.08, 0, 0, 0, 1]
        obj2_dimensions = [0.27, 0.4, 0.12]

 	obj3_pose = [0.6, 0.6, -0.08, 0, 0, 0, 1]
        obj3_dimensions = [0.27, 0.4, 0.12]

 	obj4_pose = [0.6, 0.21, -0.08, 0, 0, 0, 1]
        obj4_dimensions = [0.27, 0.4, 0.12]

 	obj5_pose = [0.45, 0.40, 0.05, 0, 0, 0, 1]
        obj5_dimensions = [0.27, 0.4, 0.12]

 	obj10_pose = [2.08, -0.65, 0.1, 0, 0, 0, 1]
        obj10_dimensions = [0.27, 0.4, 0.12]

 	obj11_pose = [2.08, -0.65, 0.23, 0, 0, 0, 1]
        obj11_dimensions = [0.27, 0.4, 0.12]

 	obj12_pose = [2.08, -0.65, 0.36, 0, 0, 0, 1]
        obj12_dimensions = [0.27, 0.4, 0.12]

 	con_bottom_pose = [1.9, -0.5, 0.012, 0, 0, 0, 1]
        con_bottom_dimensions = [0.704, 0.80, 0.02]

 	con_left_pose = [1.9, -0.9, 0.705, 0, 0, 0, 1]
        con_left_dimensions = [0.744, 0.044, 1.386]

 	con_right_pose = [2.252, -0.5, 0.705, 0, 0, 0, 1]
        con_right_dimensions = [0.044, 0.8, 1.386]

 	con_back_pose = [1.548, -0.5, 0.705, 0, 0, 0, 1]
        con_back_dimensions = [0.044, 0.8, 1.386]
 	

	con2_bottom_pose = [0.3, -0.5, 0.012, 0, 0, 0, 1]
        con2_bottom_dimensions = [0.704, 0.80, 0.02]

 	con2_left_pose = [0.3, -0.9, 0.705, 0, 0, 0, 1]
        con2_left_dimensions = [0.744, 0.044, 1.386]

 	con2_right_pose = [0.652, -0.5, 0.705, 0, 0, 0, 1]
        con2_right_dimensions = [0.044, 0.8, 1.386]

 	con2_back_pose = [-0.052, -0.5, 0.705, 0, 0, 0, 1]
        con2_back_dimensions = [0.044, 0.8, 1.386]


	con3_bottom_pose = [1.1, -0.5, 0.012, 0, 0, 0, 1]
        con3_bottom_dimensions = [0.704, 0.80, 0.02]

 	con3_left_pose = [1.1, -0.9, 0.705, 0, 0, 0, 1]
        con3_left_dimensions = [0.744, 0.044, 1.386]

 	con3_right_pose = [1.452, -0.5, 0.705, 0, 0, 0, 1]
        con3_right_dimensions = [0.044, 0.8, 1.386]

 	con3_back_pose = [0.748, -0.5, 0.705, 0, 0, 0, 1]
        con3_back_dimensions = [0.044, 0.8, 1.386]



        self.add_box_object("wall1", wall1_dimensions, wall1_pose)
	self.add_box_object("floor", floor_dimensions, floor_pose)
        self.add_box_object("wall3", wall3_dimensions, wall3_pose)
        self.add_box_object("wall2", wall2_dimensions, wall2_pose)
        self.add_box_object("obj3", obj3_dimensions, obj3_pose)
        self.add_box_object("obj2", obj2_dimensions, obj2_pose)
        self.add_box_object("obj1", obj1_dimensions, obj1_pose)
        self.add_box_object("obj4", obj4_dimensions, obj4_pose)
        self.add_box_object("obj5", obj5_dimensions, obj5_pose)

        self.add_box_object("obj10", obj10_dimensions, obj10_pose)
        self.add_box_object("obj11", obj11_dimensions, obj11_pose)
        self.add_box_object("obj12", obj12_dimensions, obj12_pose)

        self.add_box_object("con_bottom", con_bottom_dimensions, con_bottom_pose)
        self.add_box_object("con_left", con_left_dimensions, con_left_pose)
        self.add_box_object("con_right", con_right_dimensions, con_right_pose)
        self.add_box_object("con_back", con_back_dimensions, con_back_pose)

        self.add_box_object("con2_bottom", con2_bottom_dimensions, con2_bottom_pose)
        self.add_box_object("con2_left", con2_left_dimensions, con2_left_pose)
        self.add_box_object("con2_right", con2_right_dimensions, con2_right_pose)
        self.add_box_object("con2_back", con2_back_dimensions, con2_back_pose)

        self.add_box_object("con3_bottom", con3_bottom_dimensions, con3_bottom_pose)
        self.add_box_object("con3_left", con3_left_dimensions, con3_left_pose)
        self.add_box_object("con3_right", con3_right_dimensions, con3_right_pose)
        self.add_box_object("con3_back", con3_back_dimensions, con3_back_pose)


        print "========== Added test 4 scene!!"

    def add_pap5(self):
	floor_pose = [1.0, -1.0, -0.17, 0, 0, 0, 1]
	floor_dimensions = [5.0, 4.0, 0.02]

 	wall1_pose = [1.0, 1.0, 0.92, 0, 0, 0, 1]
        wall1_dimensions = [5.0,0.02, 2.18]


	wall2_pose = [3.5, -1.0, 0.92, 0, 0, 0, 1]
	wall2_dimensions = [0.02, 4.0, 2.18]

 	wall3_pose = [-1.5, -1.0, 0.92, 0, 0, 0, 1]
        wall3_dimensions = [0.02,4.0, 2.18]

 	obj1_pose = [0.3, 0.6, -0.08, 0, 0, 0, 1]
        obj1_dimensions = [0.27, 0.4, 0.12]


 	obj2_pose = [0.3, 0.21, -0.08, 0, 0, 0, 1]
        obj2_dimensions = [0.27, 0.4, 0.12]

 	obj3_pose = [0.6, 0.6, -0.08, 0, 0, 0, 1]
        obj3_dimensions = [0.27, 0.4, 0.12]

 	obj4_pose = [0.6, 0.21, -0.08, 0, 0, 0, 1]
        obj4_dimensions = [0.27, 0.4, 0.12]

 	obj5_pose = [0.45, 0.40, 0.05, 0, 0, 0, 1]
        obj5_dimensions = [0.27, 0.4, 0.12]


 	obj10_pose = [2.08, -0.65, 0.1, 0, 0, 0, 1]
        obj10_dimensions = [0.27, 0.4, 0.12]

 	obj11_pose = [2.08, -0.65, 0.23, 0, 0, 0, 1]
        obj11_dimensions = [0.27, 0.4, 0.12]

 	obj12_pose = [2.08, -0.65, 0.36, 0, 0, 0, 1]
        obj12_dimensions = [0.27, 0.4, 0.12]


 	obs_pose = [2.1, -0.1, 0.35, 0, 0, 0, 1]
        obs_dimensions = [0.1, 0.1, 0.9]


 	con_bottom_pose = [1.9, -0.5, 0.012, 0, 0, 0, 1]
        con_bottom_dimensions = [0.704, 0.80, 0.02]

 	con_left_pose = [1.9, -0.9, 0.705, 0, 0, 0, 1]
        con_left_dimensions = [0.744, 0.044, 1.386]

 	con_right_pose = [2.252, -0.5, 0.705, 0, 0, 0, 1]
        con_right_dimensions = [0.044, 0.8, 1.386]

 	con_back_pose = [1.548, -0.5, 0.705, 0, 0, 0, 1]
        con_back_dimensions = [0.044, 0.8, 1.386]
 	

	con2_bottom_pose = [0.3, -0.5, 0.012, 0, 0, 0, 1]
        con2_bottom_dimensions = [0.704, 0.80, 0.02]

 	con2_left_pose = [0.3, -0.9, 0.705, 0, 0, 0, 1]
        con2_left_dimensions = [0.744, 0.044, 1.386]

 	con2_right_pose = [0.652, -0.5, 0.705, 0, 0, 0, 1]
        con2_right_dimensions = [0.044, 0.8, 1.386]

 	con2_back_pose = [-0.052, -0.5, 0.705, 0, 0, 0, 1]
        con2_back_dimensions = [0.044, 0.8, 1.386]


	con3_bottom_pose = [1.1, -0.5, 0.012, 0, 0, 0, 1]
        con3_bottom_dimensions = [0.704, 0.80, 0.02]

 	con3_left_pose = [1.1, -0.9, 0.705, 0, 0, 0, 1]
        con3_left_dimensions = [0.744, 0.044, 1.386]

 	con3_right_pose = [1.452, -0.5, 0.705, 0, 0, 0, 1]
        con3_right_dimensions = [0.044, 0.8, 1.386]

 	con3_back_pose = [0.748, -0.5, 0.705, 0, 0, 0, 1]
        con3_back_dimensions = [0.044, 0.8, 1.386]



        self.add_box_object("wall1", wall1_dimensions, wall1_pose)
	self.add_box_object("floor", floor_dimensions, floor_pose)
        self.add_box_object("wall3", wall3_dimensions, wall3_pose)
        self.add_box_object("wall2", wall2_dimensions, wall2_pose)
        self.add_box_object("obj3", obj3_dimensions, obj3_pose)
        self.add_box_object("obj2", obj2_dimensions, obj2_pose)
        self.add_box_object("obj1", obj1_dimensions, obj1_pose)
        self.add_box_object("obj4", obj4_dimensions, obj4_pose)
        self.add_box_object("obj5", obj5_dimensions, obj5_pose)

        self.add_box_object("obj10", obj10_dimensions, obj10_pose)
        self.add_box_object("obj11", obj11_dimensions, obj11_pose)
        self.add_box_object("obj12", obj12_dimensions, obj12_pose)

        self.add_box_object("obs", obs_dimensions, obs_pose)

        self.add_box_object("con_bottom", con_bottom_dimensions, con_bottom_pose)
        self.add_box_object("con_left", con_left_dimensions, con_left_pose)
        self.add_box_object("con_right", con_right_dimensions, con_right_pose)
        self.add_box_object("con_back", con_back_dimensions, con_back_pose)

        self.add_box_object("con2_bottom", con2_bottom_dimensions, con2_bottom_pose)
        self.add_box_object("con2_left", con2_left_dimensions, con2_left_pose)
        self.add_box_object("con2_right", con2_right_dimensions, con2_right_pose)
        self.add_box_object("con2_back", con2_back_dimensions, con2_back_pose)

        self.add_box_object("con3_bottom", con3_bottom_dimensions, con3_bottom_pose)
        self.add_box_object("con3_left", con3_left_dimensions, con3_left_pose)
        self.add_box_object("con3_right", con3_right_dimensions, con3_right_pose)
        self.add_box_object("con3_back", con3_back_dimensions, con3_back_pose)


        print "========== Added test 5 scene!!"



    def add_box_object(self, name, dimensions, pose):
        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = self.robot.get_planning_frame()
        p.header.stamp = rospy.Time.now()
        p.pose.position.x = pose[0]
        p.pose.position.y = pose[1]
        p.pose.position.z = pose[2]
        p.pose.orientation.x = pose[3]
        p.pose.orientation.y = pose[4]
        p.pose.orientation.z = pose[5]
        p.pose.orientation.w = pose[6]

        self._scene.add_box(name, p, (dimensions[0], dimensions[1], dimensions[2]))
	if name == "obj5":
	    print True
	    self._scene.attach_box("pressure_box", name)

if __name__ == "__main__":
    rospy.init_node("collision_scene_example_cluttered")
    while not rospy.search_param('robot_description_semantic') and not rospy.is_shutdown():
        time.sleep(0.5)
    load_scene = CollisionSceneExample()

    if (len(sys.argv) != 2):
        print "Correct usage:: \n\"rosrun moveit_tutorials collision_scene_example.py cluttered\" OR \n\"rosrun moveit_tutorials collision_scene_example.py sparse\""
        sys.exit()
    if sys.argv[1] == "pap1":
        load_scene.add_pap1();
    elif sys.argv[1] == "pap2":
        load_scene.add_pap2();
    elif sys.argv[1] == "pap3":
        load_scene.add_pap3();
    elif sys.argv[1] == "pap4":
        load_scene.add_pap4();
    elif sys.argv[1] == "pap5":
        load_scene.add_pap5();
    else:
        print "Please specify correct type of scene as cluttered or sparse"
        sys.exit()
