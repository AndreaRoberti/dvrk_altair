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
    def __init__(self,ral):

        self.pose_name_ = rospy.get_param('~pose_name','pose_name_default')
        self.rings_topic_name_ = rospy.get_param('~ring_poses','rings/poses')
        self.pegs_topic_name_ = rospy.get_param('~pegs_poses','pegs/poses')
        
        self.pose_pub_ = rospy.Publisher(self.pose_name_, PoseStamped, queue_size=1)

        self.rings_sub_ = rospy.Subscriber(self.rings_topic_name_, PoseArray, self.rings_callback)
        self.pegs_sub_ = rospy.Subscriber(self.pegs_topic_name_, PoseArray, self.pegs_callback)
        

        self.rings_poses_ = [] # PoseArray()
        self.pegs_poses_ = [] # PoseArray()

        self.ral = ral

        self.psm1_ = dvrk.psm(ral = self.ral,
                            arm_name = 'PSM1',
                            expected_interval = 0.01)

        self.psm2_ = dvrk.psm(ral = self.ral,
                            arm_name = 'PSM2',
                            expected_interval = 0.01)

        self.psm1_base_to_world_ = self.getMatrix([-1.538, -0.138, 0.799,-0.000, 0.000, 0.574, 0.819])
        self.psm2_base_to_world_ = self.getMatrix([-0.657, -1.400, -0.799, -0.000, -0.000, -0.819, 0.574])
        
#---------------------------------------------------------------------------------------------------------
    # [blue,cyan,red,yellow,purple,green]
    def rings_callback(self, msg):
        self.rings_poses_ = msg.poses
        # print(len(self.rings_poses_))

    # [left, right]
    def pegs_callback(self, msg):
        self.pegs_poses_ = msg.poses
        # print(len(self.pegs_poses_))

    # x,y,z,qx,qy,qz,qw
    def getMatrix(self,pose):
        trasl_matrix = tf.transformations.translation_matrix( [pose[0], pose[1], pose[2]])
        rot_matrix = tf.transformations.quaternion_matrix([pose[3],pose[4],pose[5],pose[6]])
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

        # get current joints just to set size
        print('move to starting position')
        goal = numpy.copy(arm.setpoint_jp())
        # go to zero position, make sure 3rd joint is past cannula
        goal.fill(0)
        goal[2] = 0.12
        arm.move_jp(goal).wait()

    def prepare_cartesian(self, arm):
        # make sure the tip is past the cannula and tool vertical
        goal = numpy.copy(arm.setpoint_jp())
        if ((arm.name().endswith('PSM1')) or (arm.name().endswith('PSM2'))
            or (arm.name().endswith('PSM3'))):
            print('preparing for cartesian motion')
            # set in position joint mode
            goal[0] = 0.0
            goal[1] = 0.0
            goal[2] = 0.12
            goal[3] = 0.0
            arm.move_jp(goal).wait()

    def convertToPSM_Base(self, object_matrix, psm_base_matrix):
        inv_object_matrix = numpy.linalg.inv(object_matrix)
        transform_object_to_psm = numpy.dot(inv_object_matrix, psm_base_matrix)
        return transform_object_to_psm
#---------------------------------------------------------------------------------------------------------
    def test_kinematics(self):
        self.home(self.psm1_)
        self.prepare_cartesian(self.psm1_)

        initial_cartesian_position = PyKDL.Frame()
        initial_cartesian_position.p = self.psm1_.setpoint_cp().p
        initial_cartesian_position.M = self.psm1_.setpoint_cp().M
        goal = PyKDL.Frame()
        goal.p = self.psm1_.setpoint_cp().p
        goal.M = self.psm1_.setpoint_cp().M
        
        amplitude = 0.05 # 5 cm

         # first motion
        goal.p[0] =  initial_cartesian_position.p[0]  + amplitude
        goal.p[1] =  initial_cartesian_position.p[1]
        goal.p[2] =  initial_cartesian_position.p[2]
        
        self.psm1_.move_cp(goal).wait(is_busy = True)
        self.psm1_.jaw.close().wait() 
        
        else :
            print('[ERROR] maybe run coppelia')

    def update(self):
        print ('UPDATE')
        # print(len(self.rings_poses_))



#------------------------------------------------------------------

def main():
    
    ral = crtk.ral('dvrk_simple_peg_ring')
    
    simple_peg_ring = SimplePegRing(ral)

    ral.spin_and_execute(simple_peg_ring.test_kinematics)

if __name__ == '__main__':
    main()
