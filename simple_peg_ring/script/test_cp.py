#!/usr/bin/env python3
import numpy

numpy.set_printoptions(suppress=True)

import rospy

import crtk
import dvrk
import math
import numpy
import PyKDL
import sys

import tf

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, PoseArray


class SimplePegRing():
    def __init__(self, ral):
        self.pose_name_ = rospy.get_param('~pose_name', 'pose_name_default')
        self.rings_topic_name_ = rospy.get_param('~ring_poses', 'rings/poses')
        self.pegs_topic_name_ = rospy.get_param('~pegs_poses', 'pegs/poses')

        self.pose_pub_ = rospy.Publisher(self.pose_name_, PoseStamped, queue_size=1)
        self.rings_sub_ = rospy.Subscriber(self.rings_topic_name_, PoseArray, self.rings_callback)
        self.pegs_sub_ = rospy.Subscriber(self.pegs_topic_name_, PoseArray, self.pegs_callback)

        self.rings_poses_ = []  # PoseArray()
        self.pegs_poses_ = []  # PoseArray()

        self.ral = ral

        self.psm1_ = dvrk.psm(ral=self.ral, arm_name='PSM1', expected_interval=0.01)
        self.psm2_ = dvrk.psm(ral=self.ral, arm_name='PSM2', expected_interval=0.01)

        self.psm1_base_to_world_ = self.getMatrix([-1.538, -0.138, 0.799, -0.000, 0.000, 0.574, 0.819])
        self.psm2_base_to_world_ = self.getMatrix([-0.657, -1.400, -0.799, -0.000, -0.000, -0.819, 0.574])

    def rings_callback(self, msg):
        self.rings_poses_ = msg.poses

    def pegs_callback(self, msg):
        self.pegs_poses_ = msg.poses

    def getMatrix(self, pose):
        trasl_matrix = tf.transformations.translation_matrix([pose[0], pose[1], pose[2]])
        rot_matrix = tf.transformations.quaternion_matrix([pose[3], pose[4], pose[5], pose[6]])
        T_matrix = numpy.dot(trasl_matrix, rot_matrix)
        return T_matrix

    def home(self, arm):
        self.ral.check_connections()

        print('starting enable')
        if not arm.enable(10):
            sys.exit('failed to enable within 10 seconds')
        print('starting home')
        if not arm.home(10):
            sys.exit('failed to home within 10 seconds')

        print('move to starting position')
        goal = numpy.copy(arm.setpoint_jp())
        goal.fill(0)
        goal[2] = 0.12
        arm.move_jp(goal).wait()

    def prepare_cartesian(self, arm):
        goal = numpy.copy(arm.setpoint_jp())
        if arm.name().endswith(('PSM1', 'PSM2', 'PSM3')):
            print('preparing for cartesian motion')
            goal[0] = 0.0
            goal[1] = 0.0
            goal[2] = 0.12
            goal[3] = 0.0
            arm.move_jp(goal).wait()

    def test_kinematics(self):
        self.home(self.psm1_)
        self.prepare_cartesian(self.psm1_)

        initial_cartesian_position = self.psm1_.setpoint_cp()
        initial_matrix = self.psm1_base_to_world_ @ numpy.array([initial_cartesian_position.p[0],
                                                                  initial_cartesian_position.p[1],
                                                                  initial_cartesian_position.p[2],
                                                                  1.0])
        amplitude = 0.05  # 5 cm

        for i in range(3):
            goal = PyKDL.Frame()
            new_position = initial_matrix.copy()
            new_position[i] += amplitude

            transformed_goal = numpy.linalg.inv(self.psm1_base_to_world_) @ new_position
            goal.p = PyKDL.Vector(transformed_goal[0], transformed_goal[1], transformed_goal[2])
            goal.M = self.psm1_.setpoint_cp().M

            self.psm1_.move_cp(goal).wait(is_busy=True)
            rospy.sleep(1)

        self.psm1_.jaw.close().wait()

    def update(self):
        print('UPDATE')


# ------------------------------------------------------------------

def main():
    ral = crtk.ral('dvrk_simple_peg_ring')
    simple_peg_ring = SimplePegRing(ral)
    ral.spin_and_execute(simple_peg_ring.test_kinematics)


if __name__ == '__main__':
    main()
