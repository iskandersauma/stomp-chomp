#!/usr/bin/env python
import sys
import time
import copy
import rospy
import numpy as np
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import String, Header, Bool
from math import pi, sin, cos, sqrt, ceil, floor
from moveit_msgs.msg import RobotTrajectory
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from asp_tools.srv import CommandJoints, CommandPose, AddRCs, RemoveRCs, CheckPose, MoveVertical, MoveJoints, MovePose, CheckJointsValidity, GetJointsBounds, StopMotion
from trajectory_msgs.msg import JointTrajectoryPoint
from asp_IK import IKSolver
from planning_scene_manager import PlanningSceneManager
import yaml

EXECUTING = rospy.get_param('/EXECUTE')
SIMULATION = rospy.get_param('/SIMULATION')
SAFE_Z = 0.03
MERGE_Z = 0.10
SMOOTH = True
MOTION_FUSE_LAG = 1e-3
MOTION_START_DELAY = 1e-2

class MoveServer:
    """
    Class that implements the main services necessary to move the robot.
    """
    def __init__(self, moveit_commander, scene_manager):
        self.group = moveit_commander.move_group.MoveGroupCommander("asp_arm")
        self.robot = moveit_commander.RobotCommander()
        self.max_z_vel = rospy.get_param("joint_limits/Z/max_velocity")
        self.max_z_ac = rospy.get_param("joint_limits/Z/max_acceleration")

        self.__init_joint_bounds()
        
        init_jnts = CommandJoints()
        init_jnts.x, init_jnts.y, init_jnts.b,  init_jnts.z, init_jnts.a = 1.0, 0.01, -1.15, 1.30, 0.0

        # Initialize the scene
        rospy.sleep(5) # Do not remove, us
        if SIMULATION:
            self.command_joints(init_jnts)
            #scene_manager.init_safety_cushions()
            #scene_manager.init_RCs()
        else:
            #scene_manager.init_safety_cushions()
            #scene_manager.init_RCs()
            self.command_joints(init_jnts)
        rospy.sleep(5) # Do not remove, us

        self.__init_group()
        self.__advertise_services()
        self.ik_solver = IKSolver()


    def __init_group(self):
        """
        Initializes the move_group attributes.
        """
        tolerance = 0.001 if SIMULATION else 1e-4
        self.group.set_goal_position_tolerance(tolerance)
        self.group.set_goal_joint_tolerance(tolerance)
        self.group.set_planning_time(1.0)
        self.group.set_num_planning_attempts(100)

    def __init_joint_bounds(self):
        """
        Initializes joint limits.
        """
        self.min_bounds = []
        self.max_bounds = []
        for joint in ['X', 'Y', 'B', 'Z', 'A']:
            self.min_bounds.append(self.robot.get_joint(joint).bounds()[0])
            self.max_bounds.append(self.robot.get_joint(joint).bounds()[1])

    def __get_joint_bounds(self, req):
        return {"min_bounds": self.min_bounds,
                "max_bounds": self.max_bounds}

    def __move_joints(self, req):

        current_joints = self.group.get_current_joint_values()
        target_joints = [req.x, req.y, req.b, req.z, req.a]

        planned, executed = False, False
        # check if the robot is already at the target
        if self.ik_solver.at_target_joints(current_joints, target_joints):
            return {'executed':True, 'planned':False}
        # check if a vertical motion can get the robot to the target
        elif self.ik_solver.at_pillar_joints(current_joints, target_joints):
            planned, trajectory = self.__plan_vertical_motion(current_joints[3], target_joints[3])
        # we need to do the signature vertical-horizontal-vertical motion
        else:
            planned, trajectory = self.__plan_arch_motion(current_joints, target_joints)

        if planned:
            if req.async:
                self.group.execute(trajectory, wait=False)
                executed = False
            else:
                executed = self.group.execute(trajectory, wait=True)
                planned  = executed # In case we had a plan, but the execution failed -> we don't have a successfull plan

        return {'executed':executed, 'planned':planned}

    def __move_pose(self, req):

        current_joints = self.group.get_current_joint_values()
        current_pose = self.ik_solver.solve_fk(current_joints)
        target_pose = req.target_pose
        target_joints, _ = self.ik_solver.solve_ik(current_joints, target_pose)

        planned, executed = False, False
        # check if the robot is already at the target
        if self.ik_solver.at_target_pose(current_joints, target_pose):
            return {'executed':True, 'planned':False}
        # check if a vertical motion can get the robot to the target
        elif self.ik_solver.at_pillar_pose(current_pose, target_pose):
            planned, trajectory = self.__plan_vertical_motion(current_joints[3], target_joints[3])
        # we need to do the signature vertical-horizontal-vertical motion
        else:
            planned, trajectory = self.__plan_arch_motion(current_joints, target_joints)

        if planned:
            if req.async:
                self.group.execute(trajectory, wait=False)
                executed = False
            else:
                executed = self.group.execute(trajectory, wait=True)
                planned  = executed # In case we had a plan, but the execution failed -> we don't have a successfull plan

        return {'executed':executed, 'planned':planned}

    def __plan_arch_motion(self, current_joints, target_joints):
        """
        It implments execution of a plan with a initial stage of vertical movement,
        followed by a merged x,y,z movement and a final vertical movement down.
        This sequence is ignored if a pure vertical motion can get us to the target.
        """

        def return_failure():
            rt = RobotTrajectory()
            return False, rt

        req_current = CheckJointsValidity()
        req_target = CheckJointsValidity()
        req_current.joint_config = current_joints
        req_target.joint_config = target_joints
        if not self.__valid_joints(req_current) or not self.__valid_joints(req_target):
            rt = RobotTrajectory()
            return False, rt

        #If we start with end-effector higher than specified SAFE_Z, update the safe_z
        safe_z = min(SAFE_Z, current_joints[3], target_joints[3])
        MERGER = SMOOTH and not MERGE_Z == SAFE_Z # if SMOOTH is false OR SAFE_Z and MERGE_Z are the same do not merge
        ASCENT = current_joints[3] > safe_z # does the end-effector need to go up at all?
        DESCENT = target_joints[3] > safe_z # does the end-effector need to go down at all?
        if not MERGER:
            rospy.logwarn("MERGER = %s", MERGER)
        if not ASCENT:
            rospy.logwarn("ASCENT = %s", ASCENT)
        if not DESCENT:
            rospy.logwarn("DESCENT = %s", DESCENT)

        # Set the start state of the planning, to the current state but modify
        # z to be a "safe z"
        current_joints_safe_z, target_joints_safe_z = copy.deepcopy(current_joints), copy.deepcopy(target_joints)
        current_joints_safe_z[3] = safe_z
        target_joints_safe_z[3] = safe_z

        # Plan the xy motion at the safe_z
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = ['X', 'Y', 'B', 'Z', 'A']
        joint_state.position = current_joints_safe_z
        moveit_robot_state = RobotState()
        moveit_robot_state.joint_state = joint_state
        self.group.set_start_state(moveit_robot_state)
        xy_plan = self.group.plan(target_joints_safe_z)
        xy_plan_points = xy_plan.joint_trajectory.points

        if len(xy_plan_points) == 0:
            rt = RobotTrajectory()
            return False, rt

        # Convert tuples to list for the sake of future assignments
        for point in xy_plan_points:
            point.positions = list(point.positions)
            point.velocities = list(point.velocities)

        # Plan the vertical motion
        if ASCENT:
            success, z_plan_up = self.__plan_vertical_motion(current_joints[3], safe_z)  # Going up
            z_plan_up_points = z_plan_up.joint_trajectory.points
        if DESCENT:
            success, z_plan_dn = self.__plan_vertical_motion(safe_z, target_joints[3])  # Going down
            z_plan_dn_points = z_plan_dn.joint_trajectory.points

        if not MERGER:
            if ASCENT:
                t_f_vertical_up = z_plan_up_points[-1].time_from_start.to_sec() # get t_final of vertical_up motion
                for p in xy_plan_points: # offset the horizontal motion by t_final of vertical_up motion
                    p.time_from_start += rospy.Duration.from_sec(t_f_vertical_up+MOTION_FUSE_LAG)
                    p.positions[3] = safe_z

            if DESCENT:
                t_f_horizontal = xy_plan_points[-1].time_from_start.to_sec() # get t_final of horizontal motion
                for p in z_plan_dn_points: # offset the vertical_dn motion by t_final of horizontal motion
                    p.time_from_start += rospy.Duration.from_sec(t_f_horizontal+MOTION_FUSE_LAG)
                    p.positions[0] = xy_plan_points[-1].positions[0]
                    p.positions[1] = xy_plan_points[-1].positions[1]
                    p.positions[2] = xy_plan_points[-1].positions[2]
                    p.positions[4] = xy_plan_points[-1].positions[4]

            # append trajectory points to form the whole trajectory
            if ASCENT:
                trajectory_points = z_plan_up_points + xy_plan_points
            else:
                trajectory_points = xy_plan_points
            if DESCENT:
                trajectory_points += z_plan_dn_points

        else:
            # Merge the two plans : divide the vertical plan into (1) the initial
            # vertical movement (2) the merged movement and (3) the final down movement
            if ASCENT:
                vertical_up        = [p for p in z_plan_up_points if p.positions[3] >  MERGE_Z] # vertical_up motion
                merger_up_vertical = [p for p in z_plan_up_points if p.positions[3] <= MERGE_Z] # vertical_up to be combined with horizontal
                VERTICAL_UP = len(vertical_up) > 0
                if VERTICAL_UP:
                    dt_merger_up = merger_up_vertical[-1].time_from_start.to_sec() - vertical_up[-1].time_from_start.to_sec() # dutation of merger when going up
                else:
                    dt_merger_up = merger_up_vertical[-1].time_from_start.to_sec() # dutation of merger when going up
                    if not VERTICAL_UP:
                        rospy.logwarn("VERTICAL_UP = %s", VERTICAL_UP)

            if DESCENT:
                merger_dn_vertical = [p for p in z_plan_dn_points if p.positions[3] <= MERGE_Z] # vertical_dn to be combined with horizontal
                vertical_dn        = [p for p in z_plan_dn_points if p.positions[3] >  MERGE_Z] # vertical_dn motion
                dt_merger_dn = merger_dn_vertical[-1].time_from_start.to_sec() # duration of merger when going down
                VERTICAL_DN = len(vertical_dn) > 0
                if not VERTICAL_DN:
                    rospy.logwarn("VERTICAL_DN = %s", VERTICAL_DN)


            dt_traverse = xy_plan_points[-1].time_from_start.to_sec() # dutation of motion in x-y plane
            dt_sum_of_mergers = 0. # dutation of motion in x-y plane that is to be merged with z motion
            if ASCENT:
                dt_sum_of_mergers += dt_merger_up
            if DESCENT:
                dt_sum_of_mergers += dt_merger_dn

            HORIZONTAL = dt_sum_of_mergers < dt_traverse
            if not HORIZONTAL:
                rospy.logwarn("HORIZONTAL = %s", HORIZONTAL)

            if HORIZONTAL:
                # divide the motion in x-y plane into 3 chuknes: to be merged with z going up, pure x-y, to be merged with z going down
                if ASCENT:
                    merger_up_horizontal = [p for p in xy_plan_points if p.time_from_start.to_sec() <= dt_merger_up]
                if DESCENT:
                    merger_dn_horizontal = [p for p in xy_plan_points if (xy_plan_points[-1].time_from_start.to_sec() - p.time_from_start.to_sec()) <= dt_merger_dn]
                if ASCENT and DESCENT:
                    horizontal = [p for p in xy_plan_points if p.time_from_start.to_sec() > dt_merger_up and (xy_plan_points[-1].time_from_start.to_sec() - p.time_from_start.to_sec()) > dt_merger_dn]
                elif ASCENT:
                    horizontal = [p for p in xy_plan_points if p.time_from_start.to_sec() > dt_merger_up]
                elif DESCENT:
                    horizontal = [p for p in xy_plan_points if (xy_plan_points[-1].time_from_start.to_sec() - p.time_from_start.to_sec()) > dt_merger_dn]
                else:
                    horizontal = xy_plan_points
                for p in horizontal:
                    p.positions[3] = safe_z
                    p.velocities[3] = 0.
            else:
                # strech the merged motions of z across the whole x-y plane motion
                t_normalizer = dt_traverse/dt_sum_of_mergers
                if ASCENT:
                    merger_up_horizontal = [p for p in xy_plan_points if p.time_from_start.to_sec() <= dt_merger_up*t_normalizer]
                if DESCENT:
                    merger_dn_horizontal = [p for p in xy_plan_points if (xy_plan_points[-1].time_from_start.to_sec() - p.time_from_start.to_sec()) <=  dt_merger_dn*t_normalizer]


            # start merging z motion and x-y motion at the end of the vertical up motion and shift all future time steps
            if ASCENT and VERTICAL_UP:
                t_f_vertical_up = vertical_up[-1].time_from_start.to_sec()
                for p in merger_up_horizontal:
                    p.time_from_start += rospy.Duration.from_sec(t_f_vertical_up+MOTION_FUSE_LAG)

                if HORIZONTAL:
                    for p in horizontal:
                        p.time_from_start += rospy.Duration.from_sec(t_f_vertical_up+MOTION_FUSE_LAG)

                if DESCENT:
                    for p in merger_dn_horizontal:
                        p.time_from_start += rospy.Duration.from_sec(t_f_vertical_up+MOTION_FUSE_LAG)

            # start merging z motion and x-y motion at the end of the pure x-y motion motion and shift all future time steps
            if DESCENT:
                t_0_merger_down = merger_dn_horizontal[0].time_from_start.to_sec()
                for p in merger_dn_vertical:
                    p.time_from_start += rospy.Duration.from_sec(t_0_merger_down+MOTION_FUSE_LAG)

                if VERTICAL_DN:
                    for p in vertical_dn:
                        p.time_from_start += rospy.Duration.from_sec(t_0_merger_down+MOTION_FUSE_LAG)

            # interpolate z joint positions during merged motion
            if ASCENT:
                merger_up = merger_up_horizontal
                for p in merger_up:
                    # Update z by interpolating the plan in time
                    time = p.time_from_start.to_sec()
                    p.positions[3] = self.__interpolate_z(merger_up_vertical, time)

            # interpolate z joint positions during merged motion
            if DESCENT:
                merger_dn = merger_dn_horizontal
                for p in merger_dn:
                    # Update z by interpolating the plan in time
                    time = p.time_from_start.to_sec()
                    p.positions[3] = self.__interpolate_z(merger_dn_vertical, time)

                # set the joint values of the final vertical down motion to those at the end of the x-y motion
                if VERTICAL_DN:
                    for p in vertical_dn:
                        p.positions[0] = merger_dn[-1].positions[0]
                        p.positions[1] = merger_dn[-1].positions[1]
                        p.positions[2] = merger_dn[-1].positions[2]
                        p.positions[4] = merger_dn[-1].positions[4]

            # merge trajectory points
            if ASCENT:
                if VERTICAL_UP:
                    trajectory_points = vertical_up
                    for p in vertical_up:
                        rospy.logwarn("SAFE_MOTION: vertical_up: %s", p.positions[3])
                try:
                    trajectory_points += merger_up
                except NameError:
                    trajectory_points = merger_up
                for p in merger_up:
                    rospy.logwarn("SAFE_MOTION: merger_up: %s", p.positions[3])

            if HORIZONTAL:
                try:
                    trajectory_points += horizontal
                except NameError:
                    trajectory_points = horizontal
                for p in horizontal:
                    rospy.logwarn("SAFE_MOTION: horizontal: %s", p.positions[3])

            if DESCENT:
                try:
                    trajectory_points += merger_dn
                except NameError:
                    trajectory_points = merger_dn
                for p in merger_dn:
                    rospy.logwarn("SAFE_MOTION: merger_dn: %s", p.positions[3])

                if VERTICAL_DN:
                    trajectory_points += vertical_dn
                    for p in vertical_dn:
                        rospy.logwarn("SAFE_MOTION: vertical_dn: %s", p.positions[3])

        # shift all trajectory points in time to avoid dropping the first point due to its time being
        for p in trajectory_points:
            p.time_from_start += rospy.Duration.from_sec(MOTION_START_DELAY)
        rt = RobotTrajectory()
        rt.joint_trajectory.header.stamp = rospy.Time.now()
        rt.joint_trajectory.joint_names = ["X", "Y", "B", "Z", "A"]
        rt.joint_trajectory.points = trajectory_points

        return True, rt

    def __move_vertical(self, req):
        """
        Plans and execute a trajectory in order to reach the given target height
        in world space with the given (or default) maximum velocity and
        acceleration.
        """
        current_joints = self.group.get_current_joint_values()
        current_pose = self.ik_solver.solve_fk(current_joints)
        current_z = current_joints[3]
        target_z = self.ik_solver.z_to_joint(req.target_height)
        target_pose = copy.deepcopy(current_pose)
        target_pose.position.z = target_z

        if self.ik_solver.at_target_pose(current_joints, target_pose):
            return {'executed':True, 'planned':False} # Already there
        elif target_z is None: # Unreachable height
            return {'executed':False, 'planned':False}
        else:
            v_max = self.max_z_vel if req.max_velocity == 0 else min(abs(req.max_velocity), self.max_z_vel)
            a_max = self.max_z_ac  if req.max_acceleration == 0 else min(abs(req.max_acceleration), self.max_z_ac)# m/s^2

            planned, trajectory = self.__plan_vertical_motion(current_z, target_z, v_max, a_max)

        if planned:
            if req.async:
                self.group.execute(trajectory, wait=False)
                executed = False
            else:
                executed = self.group.execute(trajectory, wait=True)
                planned  = executed # In case we had a plan, but the execution failed -> we don't have a successfull plan

        return {'executed':executed, 'planned':planned}

    def __plan_vertical_motion(self, current_z, target_z, v_max = None, a_max = None):
        """
        Computes a robot trajectory in order to perform a vertical movement,
        given a maximum velocity and a constant acceleration.
        """
        v_max = self.max_z_vel if v_max is None else v_max
        a_max = self.max_z_ac if a_max is None else a_max

        dt = 0.01
        ds = target_z - current_z           # Space that has to be traversed
        v_max, a_max = (v_max, a_max) if ds > 0 else (-v_max, -a_max)

        #Determine if the velocity profile will be a triangle or a trapezoid
        T = v_max/a_max # Time to reach the maximum velocity from a velocity of 0

        # Round down T to a number which can be divided by dt
        T = int(T/dt) * dt
        ts = [dt * i for i in range(int(T/dt))]

        v_0 = [a_max * t for t in ts]  			#Aceleration phase
        v_2 = [v_max - a_max*t for t in ts]		#Deceleration phase
        v_t_half = sqrt(2*a_max*ds/2) if ds > 0 else -sqrt(2*a_max*ds/2)

        if abs(v_t_half) < abs(v_max):          # Velocity profile is triangular
        	v_reached = sqrt(2*a_max*ds/2)      # Maximum velocity reached
        	v_0 = [v for v in v_0 if abs(v) < abs(v_reached)]
        	v_1 = copy.deepcopy(v_0)
        	v_1.reverse()
        	vs = v_0 + v_1                       # To get the triangular shape
        else: # V profile  is a trapezoid, we can get all the way to the max v
            done_ds = v_max**2/(2*a_max)*2   # v1^2 = v0^2 + 2*a*ds
            remaining_ds = ds - done_ds
            no_ac_dt = remaining_ds/v_max
            v_1 = [v_max for i in range(int(ceil(no_ac_dt/dt)))] # Plateau
            vs = v_0 + v_1 + v_2

        t = [dt*i for i in range(len(vs))]

        # "Integrate" velocity to get thcurrent_high[3] = SAFE_Z
        s = [v*dt for v in vs]
        s = list(np.cumsum(np.array(s))+current_z) #+ [target_z]
        t = [dt*i for i in range(len(s))]

        #Force it to arrive exactly at the target pose
        dds = s[-1]-target_z
        s = s - dds
        vs = vs[1:] + [0]
        rt = self.__z_to_joint_traj(*self.__downsample(s, vs, t))

        return True, rt

    def __z_to_joint_traj(self, z, v, t):
        """
        Converts a list of z joint values to a suitable RobotTrajectory over all
        the joints. It keeps for the other joints the current joint value.
        """
        import genpy
        traj = RobotTrajectory()
        traj.joint_trajectory.header.stamp = rospy.Time.now()
        traj.joint_trajectory.joint_names = ["X", "Y", "B", "Z", "A"]
        x, y, b, _, a = self.group.get_current_joint_values()
        for zi, vi, ti in zip(z, v, t):
            pt = JointTrajectoryPoint()
            pt.positions = [x, y, b, zi, a]
            pt.velocities = [0, 0, 0, vi, 0]
            # pt.time_from_start = rospy.Time.from_sec(ti+MOTION_START_DELAY) # shift all
            pt.time_from_start = rospy.Duration.from_sec(ti+MOTION_START_DELAY) # shift all
            # trajectory points in time to avoid dropping the first point due to its time being
            traj.joint_trajectory.points.append(pt)
        return traj

    def __downsample(self, s, v, t, n_samples = None):
        """
        Downsamples the set of points so that they have in the end only
        n_samples elements (+ the initial and final one).
        """
        if n_samples is None:
            n_samples = max((max(s)-min(s))/0.10, 1)  # BOH
        #Downsample
        filter_coeff = max(int(len(t)/n_samples),1)
        s = [z for i, z in enumerate(s) if i%filter_coeff == 0 or i ==len(t)-1]
        v = [v for i, v in enumerate(v) if i%filter_coeff == 0 or i ==len(t)-1]
        t = [ts for i, ts in enumerate(t) if i%filter_coeff == 0 or i ==len(t)-1]
        return s, v, t

    def __interpolate_z(self, z_plan, time):
        """
        Given a plan for the z joint, interpolates it linearly around t = time.
        """
        # Take the couple of joint values around time
        idx_t = [(idx, point.time_from_start.to_sec()) for idx,point in enumerate(z_plan) if point.time_from_start.to_sec() > time]

        if not idx_t:
            return z_plan[-1].positions[3] # The trajectory has already been completed, return last value

        idx, t = zip(*idx_t)
        if idx[0] == 0:
            return z_plan[0].positions[3]

        # idx contains at least 2 elements, pick the first and the previous one
        p0, p1 = z_plan[idx[0]-1], z_plan[idx[0]]
        DT = p1.time_from_start.to_sec() - p0.time_from_start.to_sec()
        DZ = p1.positions[0]-p0.positions[0]
        dt = time - p0.time_from_start.to_sec()
        int_z = p0.positions[3] + DZ*dt/DT
        return int_z if int_z > 0 else 0

    def __valid_joints(self, req):
        for joint_nbr, joint_state in enumerate(req.joint_config):
            if joint_state < self.min_bounds[joint_nbr] or joint_state > self.max_bounds[joint_nbr]:
                return False
        return True

    def __check_if_at_target_pose(self, pose_req):
        current_joints = self.group.get_current_joint_values()
        target_pose = pose_req.target_pose
        tolerance = pose_req.tolerance
        return self.ik_solver.at_target_pose(current_joints, target_pose, tolerance)

    def __stop_motion(self, req):
        self.group.stop()
        return True

    def command_pose(self, pose_req):
        """
        It receives a target pose and tries to plan and go there.
        """
        #Get current joint position
        current = self.group.get_current_joint_values()
        joint_config, at_target = self.ik_solver.solve_ik(current, pose_req.target_pose)
        if joint_config is None:
            return False
        if at_target:
            return True
        command_joints = CommandJoints()
        command_joints.x, command_joints.y, command_joints.b, command_joints.z, command_joints.a  = joint_config
        return self.command_joints(command_joints)

    def command_joints(self, jnts):
        """
        It receives a desired joint configurations and compute and executes a
        path to reach it.
        """
        joints = [jnts.x, jnts.y, jnts.b, jnts.z, jnts.a]
        success = False
        
        req = CheckJointsValidity()
        req.joint_config = joints
        if self.__valid_joints(req):
            if EXECUTING :
                success = self.group.go(joints, wait = True)
            else:
                plan = self.group.plan(joints)
                success = len(plan.joint_trajectory.points) > 0

            self.group.stop()
        return success

    def __advertise_services(self):
        """
        Advertises the services offered by the MoveServer.
        """
        command_joints = rospy.Service('/asp/command_joints', CommandJoints, self.command_joints)
        command_pose   = rospy.Service('/asp/command_pose', CommandPose, self.command_pose)

        move_vertical = rospy.Service('/asp/move_vertical', MoveVertical, self.__move_vertical)
        move_joints   = rospy.Service('/asp/move_joints', MoveJoints, self.__move_joints)
        move_pose     = rospy.Service('/asp/move_pose', MovePose, self.__move_pose)

        stop_motion = rospy.Service('/asp/stop_motion', StopMotion, self.__stop_motion)
        check_if_at_target_pose = rospy.Service('/asp/at_target_pose', CheckPose, self.__check_if_at_target_pose)
        get_joint_bounds = rospy.Service('/asp/get_joint_bounds', GetJointsBounds, self.__get_joint_bounds)
        check_joints_validity = rospy.Service('/asp/check_joints_validity', CheckJointsValidity, self.__valid_joints)

def main():
    rospy.init_node('move_server', anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)
    scene_manager = PlanningSceneManager(moveit_commander)
    executor = MoveServer(moveit_commander, scene_manager)
    scene_manager.advertise_services()

    rospy.spin()

if __name__=="__main__":
    main()
