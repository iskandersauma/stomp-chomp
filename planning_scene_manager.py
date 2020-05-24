#!/usr/bin/env python
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Pose, PoseArray
from asp_tools.srv import AddRCs, RemoveRCs
import copy
import rospy

class PlanningSceneManager:
    def __init__(self, moveit_commander):
        self.scene = moveit_commander.PlanningSceneInterface()
        self.robot = moveit_commander.RobotCommander()

        # RC dimensions
        self.rc_wall_thickness = 0.020+0.150
        self.rc_wall_width = 0.800+0.200
        self.rc_wall_height = 1.580

        self.rc_base_width = 0.705-0.020-(self.rc_wall_thickness)
        self.rc_base_height = 0.190

    def init_safety_cushions(self):
        """
        Initializes the planning scene adding obstacles.
        """
        # SAFETY CUSHIONS
        e_cushion = geometry_msgs.msg.PoseStamped()
        e_cushion.header.frame_id = self.robot.get_planning_frame()
        e_cushion.pose.position.x = -(0.112+0.112/2)
        e_cushion.pose.position.y = 1.25
        e_cushion.pose.position.z = 1.0
        self.scene.add_box("e_cushion", e_cushion, size=(0.112, 2.5, 2.0))

        w_cushion = geometry_msgs.msg.PoseStamped()
        w_cushion.header.frame_id = self.robot.get_planning_frame()
        w_cushion.pose.position.x = (2.750-0.112-0.122/2+0.122)
        w_cushion.pose.position.y = 1.25
        w_cushion.pose.position.z = 1.0
        self.scene.add_box("w_cushion", w_cushion, size=(0.122, 2.5, 2.0))

        n_cushion = geometry_msgs.msg.PoseStamped()
        n_cushion.header.frame_id = self.robot.get_planning_frame()
        n_cushion.pose.position.x = (2.750-0.112-0.122)/2
        n_cushion.pose.position.y = -0.080/2-0.080+0.056+0.1
        n_cushion.pose.position.z = 0.75
        self.scene.add_box("n_cushion", n_cushion, size=(2.750-0.112-0.122, 0.080, 1.5))

        b_cushion = geometry_msgs.msg.PoseStamped()
        b_cushion.header.frame_id = self.robot.get_planning_frame()
        b_cushion.pose.position.x = (2.750-0.112-0.122)/2
        b_cushion.pose.position.y = 1.25
        b_cushion.pose.position.z = 0.10/2
        #self.scene.add_box("b_cushion", b_cushion, size=(2.750-0.112-0.122, 2.5, 0.10))

        s_cushion = geometry_msgs.msg.PoseStamped()
        s_cushion.header.frame_id = self.robot.get_planning_frame()
        s_cushion.pose.position.x = (2.750-0.112-0.122)/2
        s_cushion.pose.position.y = 1.17+0.3 + self.rc_wall_width/2 + 0.080/2
        s_cushion.pose.position.z = 0.75
        self.scene.add_box("s_cushion", s_cushion, size=(2.750-0.112-0.122, 0.080, 1.5))

    def init_RCs(self):
        self.current_roller_container_0 = Pose()
        self.current_roller_container_0.position.x = 0.5525
        self.current_roller_container_0.position.y = 1.4
        # self.current_roller_container_0.position.x = 0.4-0.112
        # self.current_roller_container_0.position.y = 1.4
        # self.current_roller_container_1 = copy.deepcopy(self.current_roller_container_0)
        # self.current_roller_container_1.position.x += self.rc_base_width + 0.05
        # self.current_roller_container_2 = copy.deepcopy(self.current_roller_container_1)
        # self.current_roller_container_2.position.x += self.rc_base_width + 0.05
        # self.current_roller_container_3 = copy.deepcopy(self.current_roller_container_2)
        # self.current_roller_container_3.position.x += self.rc_base_width + 0.05

        self.current_roller_containers = PoseArray()
        # self.current_roller_containers.poses = [self.current_roller_container_0, self.current_roller_container_1, self.current_roller_container_2, self.current_roller_container_3]
        self.current_roller_containers.poses = [self.current_roller_container_0]

        roller_containers = AddRCs()
        roller_containers.rcs = self.current_roller_containers

        added_rcs = self.add_rc(roller_containers)

    def add_rc(self, roller_containers):
        rc_poses = roller_containers.rcs.poses
        for id, roller_container in enumerate(rc_poses):
            rc = geometry_msgs.msg.PoseStamped()
            rc.header.frame_id = self.robot.get_planning_frame()
            rc.pose.position.x = roller_container.position.x
            rc.pose.position.y = roller_container.position.y
            rc.pose.position.z = self.rc_base_height/2
            self.scene.add_box("rc"+str(id), rc, size=(self.rc_base_width, self.rc_wall_width, self.rc_base_height))

            rc_e_wall = geometry_msgs.msg.PoseStamped()
            rc_e_wall.header.frame_id = self.robot.get_planning_frame()
            rc_e_wall.pose.position.x = rc.pose.position.x - self.rc_base_width/2 - self.rc_wall_thickness/2
            rc_e_wall.pose.position.y = rc.pose.position.y
            rc_e_wall.pose.position.z = self.rc_wall_height/2
            self.scene.add_box("rc"+str(id)+"_e_wall", rc_e_wall, size=(self.rc_wall_thickness, self.rc_wall_width, self.rc_wall_height))

            rc_w_wall = geometry_msgs.msg.PoseStamped()
            rc_w_wall.header.frame_id = self.robot.get_planning_frame()
            rc_w_wall.pose.position.x = rc.pose.position.x + self.rc_base_width/2 + self.rc_wall_thickness/2
            rc_w_wall.pose.position.y = rc.pose.position.y
            rc_w_wall.pose.position.z = self.rc_wall_height/2
            self.scene.add_box("rc"+str(id)+"_w_wall", rc_w_wall, size=(self.rc_wall_thickness, self.rc_wall_width, self.rc_wall_height))

        return True

    def remove_rc(self, roller_containers):
        rc_ids = roller_containers.rcs
        if len(rc_ids) == 0:
            for id in range(4):
                self.scene.remove_world_object("rc"+str(id)+"_e_wall")
                self.scene.remove_world_object("rc"+str(id))
                self.scene.remove_world_object("rc"+str(id)+"_w_wall")
        else:
            for id in rc_ids:
                self.scene.remove_world_object("rc"+str(id)+"_e_wall")
                self.scene.remove_world_object("rc"+str(id))
                self.scene.remove_world_object("rc"+str(id)+"_w_wall")
        return True

    def advertise_services(self):
        add_rc =  rospy.Service('/asp/add_rc', AddRCs, self.add_rc)
        remove_rc =  rospy.Service('/asp/remove_rc', RemoveRCs, self.remove_rc)
