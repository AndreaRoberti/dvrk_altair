#!/usr/bin/env python3
import numpy
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
        

        self.rings_poses_ = PoseArray()
        self.pegs_poses_ = PoseArray()

        self.ral = ral

        self.psm1_ = dvrk.psm(ral = self.ral,
                            arm_name = 'PSM1',
                            expected_interval = 0.01)

        self.psm2_ = dvrk.psm(ral = self.ral,
                            arm_name = 'PSM2',
                            expected_interval = 0.01)

        ## HOME MEGLIO DA INTERFACCIA
        #self.home(self.psm1_)
        #self.home(self.psm2_)
        
        ## FACCIO QUANDO VOGLIO DARE IL COMANDO 
        #self.prepare_cartesian(self.psm1_)
        #self.prepare_cartesian(self.psm1_)

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

    # [blue,cyan,red,yellow,purple,green]
    def rings_callback(self, msg):
        self.rings_poses_ = msg.poses
        # print(len(self.rings_poses_))

    # [left, right]
    def pegs_callback(self, msg):
        self.pegs_poses_ = msg.poses
        # print(len(self.pegs_poses_))


    def update(self):
        print ('UPDATE')



#------------------------------------------------------------------

def main():
    
    ral = crtk.ral('dvrk_simple_peg_ring')
    
    simple_peg_ring = SimplePegRing(ral)
    
    rate = 100 # Hz
    ros_rate = rospy.Rate(rate)
    while not rospy.is_shutdown():
        simple_peg_ring.update()
        ros_rate.sleep()

if __name__ == '__main__':
    main()
