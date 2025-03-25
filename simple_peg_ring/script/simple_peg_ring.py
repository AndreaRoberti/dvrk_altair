#!/usr/bin/env python3
import numpy

numpy.set_printoptions(suppress=True)

import rospy

import time
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
        
        self.expected_interval_ = 0.01

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
        
        amplitude = 0.1 # 5 cm

         # first motion
        goal.p[0] =  initial_cartesian_position.p[0]  + amplitude
        goal.p[1] =  initial_cartesian_position.p[1]
        goal.p[2] =  initial_cartesian_position.p[2]
        
        self.psm1_.move_cp(goal).wait(is_busy = True)
        self.psm1_.jaw.close().wait() 
        
        # else :
        #     print('[ERROR] maybe run coppelia')


    def run_spiral_trajectory(self):
        print('starting spiral trajectory')
        # get current position
        initial_joint_position = numpy.copy(self.psm1_.setpoint_jp())
        print('testing spiral trajectory with initial joint positions: %s' % str(initial_joint_position))
        
        amplitude = math.radians(10.0)  # +/- 25 degrees for joint movement
        duration = 30  # seconds, adjust as needed
        samples = duration / self.expected_interval_
        
        # create a new goal starting with the current position
        goal_p = numpy.copy(initial_joint_position)
        goal_v = numpy.zeros(goal_p.size)
        start = time.time()

        sleep_rate = self.ral.create_rate(1.0 / self.expected_interval_)
        
        for i in range(int(samples)):
            # Compute angle for circular motion (spiral)
            angle = i * math.radians(360.0) / samples
            
            # Change in x, y, z to create a spiral trajectory
            # Assuming joint[0] and joint[1] are the ones for spiral, and joint[5] is adjusted for z height
            
            # For a spiral, modify the first two joints (0 and 1) to move in a circle
            goal_p[0] = initial_joint_position[0] + amplitude * math.cos(angle)
            goal_p[1] = initial_joint_position[1] + amplitude * math.sin(angle)
            
            # Gradual increase in z-axis (or joint[5]) for the spiral effect
            goal_p[3] = initial_joint_position[5] + 0.05 * i  # Adjust the z-axis increment (0.01 can be changed)

            # Set the velocity for smooth movement
            goal_v[0] = amplitude * -math.sin(angle)
            goal_v[1] = amplitude * math.cos(angle)
            goal_v[3] = 0.01  # Smooth increment for the z-axis

            # goal_p[2] =  0.11
            # goal_v[2] = 0.01

            goal_p[4] = 0
            goal_v[4] = 0
            goal_p[5] = 0
            goal_v[5] = 0

            # Maintain the rest of the joints at their initial positions (if they are not moving)
            for j in range(2, 5):
                goal_p[j] = initial_joint_position[j]
                goal_v[j] = 0

            # Send the new joint positions and velocities to the robot
            self.psm1_.servo_jp(goal_p, goal_v)
            
            # Sleep to maintain the specified rate
            sleep_rate.sleep()

        actual_duration = time.time() - start
        print('Spiral trajectory complete in %2.2f seconds (expected %2.2f)' % (actual_duration, duration))


    def update(self):
        print ('UPDATE')
        # print(len(self.rings_poses_))



#------------------------------------------------------------------

def main():
    
    ral = crtk.ral('dvrk_simple_peg_ring')
    
    simple_peg_ring = SimplePegRing(ral)

    ral.spin_and_execute(simple_peg_ring.run_spiral_trajectory)

if __name__ == '__main__':
    main()
