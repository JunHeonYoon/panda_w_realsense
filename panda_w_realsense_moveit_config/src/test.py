#!/usr/bin/env python

import rospy
import time
import _thread
import actionlib

from control_msgs.msg    import FollowJointTrajectoryGoal
from control_msgs.msg    import FollowJointTrajectoryResult
from control_msgs.msg    import FollowJointTrajectoryFeedback
from control_msgs.msg    import FollowJointTrajectoryAction
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg	 import JointState

joint_states = JointState()
lock_ob = _thread.allocate_lock()
conf_joint_names = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']
pub = rospy.Publisher('/joint_states',JointState, queue_size=10)

conf_blending = 0.02 # M

def joint_states_callback(msg):
    lock_ob.acquire()
    global joint_states
    joint_states = msg

    lock_ob.release()

    new_joint_states = msg
    new_joint_states.header = rospy.Header()
    new_joint_states.header.stamp = rospy.get_rostime()

    pub.publish(new_joint_states)


class MoveItAction(object):
    _feedback = FollowJointTrajectoryFeedback()
    _result = FollowJointTrajectoryResult()

    def rearrange(self, joint_trajectory:JointTrajectory):

        mapping = [joint_trajectory.joint_names.index(j) for j in conf_joint_names]

        for point in joint_trajectory.points:

            temp_positions = []
            temp_velocities = []
            temp_accelerations = []
            temp_effort = []

            for i in range(len(point.positions)):
                temp_positions.append(point.positions[mapping[i]])
            for i in range(len(point.velocities)):
                temp_velocities.append(point.velocities[mapping[i]])
            for i in range(len(point.accelerations)):
                temp_accelerations.append(point.accelerations[mapping[i]])
            for i in range(len(point.effort)):
                temp_effort.append(point.effort[mapping[i]])

            point.positions = temp_positions
            point.velocities = temp_velocities
            point.accelerations = temp_accelerations
            point.effort = temp_effort

        joint_trajectory.joint_names = conf_joint_names

    def __init__(self, name):
        self.publisher = rospy.Publisher('/joint_command',JointState, queue_size=10)
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, FollowJointTrajectoryAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
           
    def execute_cb(self, goal:FollowJointTrajectoryGoal):
        rospy.logerr("action callback====================================================================================")
        
		# It is required to rearrange the arrays because MoveIt doesn't guarantee orden preservation
        self.rearrange(goal.trajectory)

		# A trajectory needs at least 2 points		
        if len(goal.trajectory.points) < 2:
            return
        
        time_start = rospy.Time.from_sec(time.time())

		# ------------- Send command list

        # trajectory = CommandList()
        # trajectory.replace_previous_commands = True

        # last_point = goal.trajectory.points[0]
        # for point in goal.trajectory.points[1:]:
        #     command = Command()
        #     command.command_id = goal.trajectory.points.index(point) - 1
        #     command.command_type = 'LIN_TIMED'
        #     command.pose_type = 'JOINTS'
        #     command.pose = point.positions
        #     command.blending_type = 'M'
        #     if goal.trajectory.points.index(point) == len(goal.trajectory.points) - 1:
        #         command.blending = [0.0]
        #     else:
        #         command.blending = [conf_blending]
        #     command.additional_values = [point.time_from_start.to_sec() - last_point.time_from_start.to_sec()]

        #     trajectory.commands.append(command)	

        #     last_point = point

        # self.publisher.publish(trajectory)

		# ------------- Wait until the termination while providing feedback
        
        last_point = goal.trajectory.points[0]
        for point in goal.trajectory.points[1:]:
            
            # Publish Goal Joint
            goal_joint = JointState()
            goal_joint.header = rospy.Header()
            goal_joint.name = conf_joint_names
            goal_joint.position = point.positions
            goal_joint.velocity = point.velocities
            goal_joint.effort = point.effort
            self.publisher.publish(goal_joint)

			# Wait	
            rospy.sleep(point.time_from_start - last_point.time_from_start)

			# Trajectory abort!
			# To abort the current movement, it is possible to send an empty trajectory
            if self._as.is_preempt_requested():
                # trajectory_2 = CommandList()
                # trajectory_2.replace_previous_commands = True
                # self.publisher.publish(trajectory_2)
                self._as.set_preempted()
                return
			# ---------------------------------------
			# Feedback
            self._feedback.joint_names = goal.trajectory.joint_names
            self._feedback.desired = point
            lock_ob.acquire()
            self._feedback.actual.positions = joint_states.position
            self._feedback.actual.velocities = joint_states.velocity
            self._feedback.actual.time_from_start = rospy.Time.from_sec(time.time()) - time_start
            lock_ob.release()
            self._as.publish_feedback(self._feedback)
            # ---------------------------------------
            last_point = point

        # ---------------------------------------
		# Result
        self._result.error_code = 0
        self._as.set_succeeded(self._result)
		# ---------------------------------------
		


if __name__ == '__main__':
    rospy.init_node('test_py')
    rospy.Subscriber('/raw_joint_states', JointState, joint_states_callback)
    MoveItAction("/effort_joint_trajectory_controller/follow_joint_trajectory")
    rospy.spin()