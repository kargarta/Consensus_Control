#!/usr/bin/env python

'''
Copyright (c) 2015, Mark Silliman
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

# A very basic differential robot script that moves Myrobot in a circle indefinitely. Press CTRL + C to stop.

from geometry_msgs.msg import Twist
import	rospy
from sensor_msgs.msg import Range
import message_filters
from math import *
from nav_msgs.msg import Odometry
from numpy import *
from random import *
from robot import *
from ekf_functions import *
from matplotlib.pyplot import *
from geometry_msgs.msg import Point, Quaternion
import tf
import time
import geometry_msgs.msg
import ar_track_alvar_msgs.msg
from ar_track_alvar_msgs.msg import AlvarMarkers 
import xlsxwriter 
import pandas as pd 
import sensor_msgs.msg
from sensor_msgs.msg import BatteryState

global dist_sens_min_range, dist_sens_max_range, proximity_min_range, proximity_max_range,MOT_STEP_DIST,WHEEL_CIRCUMFERENCE,WHEEL_DISTANCE,WHEEL_SEPARATION,WHEEL_DIAMETER,  ROBOT_RADIUS


## e-puck2 dimensions
# Wheel Radio (cm)
WHEEL_DIAMETER = 4
# Separation between wheels (cm)
WHEEL_SEPARATION = 5.3

# Distance between wheels in meters (axis length); it's the same value as "WHEEL_SEPARATION" but expressed in meters.
WHEEL_DISTANCE = 0.053
# Wheel circumference (meters).
WHEEL_CIRCUMFERENCE = ((WHEEL_DIAMETER*math.pi)/100.0)
# Distance for each motor step (meters); a complete turn is 1000 steps.
MOT_STEP_DIST = (WHEEL_CIRCUMFERENCE/1000.0)    # 0.000125 meters per step (m/steps)


class CircleMode():
    def __init__(self,deltaTheta_epuck0=0,deltaTheta_epuck1=0,deltaSteps_epuck0=0,deltaSteps_epuck1=0,deltaTheta_epuck2=0,deltaTheta_epuck3=0,deltaSteps_epuck2=0,deltaSteps_epuck3=0, init_xpos_epuck0=-0.2,init_ypos_epuck0=0.551, init_theta_epuck0=0,init_xpos_epuck1=-0.1, init_ypos_epuck1=0.511, init_theta_epuck1=0, init_xpos_epuck2=0, init_ypos_epuck2=0.1098, init_theta_epuck2=3.14,init_xpos_epuck3=0.3, init_ypos_epuck3=0.154, init_theta_epuck3=3.14, yaw_epuck0=0, yaw_epuck1=0, yaw_epuck2=0, yaw_epuck3=0, camera_epuck0=matrix([0.378, -0.1293 ,0]), camera_epuck1=matrix([0.311, -0.145 ,0]), camera_epuck2=matrix([0.268, -0.010,0]), camera_epuck3=matrix([0.3400, -0.006 ,0]), flag_epuck0=0, flag_epuck1=0, flag_epuck2=0, flag_epuck3=0):
        # initiliaze
        rospy.init_node('CircleMode', anonymous=False)
        self.theta_epuck0 = init_theta_epuck0
        self.x_pos_epuck0 = init_xpos_epuck0
        self.y_pos_epuck0 = init_ypos_epuck0
        self.theta_epuck1 = init_theta_epuck1
        self.x_pos_epuck1 = init_xpos_epuck1
        self.y_pos_epuck1 = init_ypos_epuck1
        self.theta_epuck2 = init_theta_epuck2
        self.x_pos_epuck2 = init_xpos_epuck2
        self.y_pos_epuck2 = init_ypos_epuck2
        self.theta_epuck3 = init_theta_epuck3
        self.x_pos_epuck3 = init_xpos_epuck3
        self.y_pos_epuck3 = init_ypos_epuck3
	# tell user how to stop Myrobot
        r = rospy.Rate(80);

        self.theta_event_epuck0 = init_theta_epuck0
        self.x_pos_event_epuck0 = init_xpos_epuck0

        self.theta_event_epuck1 = init_theta_epuck1
        self.x_pos_event_epuck1 = init_xpos_epuck1

        self.theta_event_epuck2 = init_theta_epuck2
        self.x_pos_event_epuck2 = init_xpos_epuck2

        self.theta_event_epuck3 = init_theta_epuck3
        self.x_pos_event_epuck3 = init_xpos_epuck3

        self.SOC_epuck0=0
        self.SOC_epuck1=0
        self.SOC_epuck2=0
        self.SOC_epuck3=0

        self.deltaTheta_epuck1=deltaTheta_epuck1
        self.deltaTheta_epuck0=deltaTheta_epuck0
        self.deltaSteps_epuck0=deltaSteps_epuck0
        self.deltaSteps_epuck1=deltaSteps_epuck1

        self.deltaTheta_epuck2=deltaTheta_epuck2
        self.deltaTheta_epuck3=deltaTheta_epuck3
        self.deltaSteps_epuck2=deltaSteps_epuck2
        self.deltaSteps_epuck3=deltaSteps_epuck3

        self.leftStepsPrev_epuck0 = 0
        self.rightStepsPrev_epuck0 = 0
        self.leftStepsPrev_epuck1 = 0
        self.rightStepsPrev_epuck1 = 0
        self.leftStepsPrev_epuck2 = 0
        self.rightStepsPrev_epuck2 = 0
        self.leftStepsPrev_epuck3 = 0
        self.rightStepsPrev_epuck3 = 0

        self.total_distance_epuck0=0
        self.total_distance_epuck1=0
        self.total_distance_epuck2=0
        self.total_distance_epuck3=0

        self.previous_x_epuck0 = 0
        self.previous_y_epuck0 = 0
        self.previous_x_epuck1 = 0
        self.previous_y_epuck1 = 0
        self.previous_x_epuck2 = 0
        self.previous_y_epuck2 = 0
        self.previous_x_epuck3 = 0
        self.previous_y_epuck3 = 0

        #self.Gamma_x=0.3
        #self.Gamma_theta=0.2

        self.Gamma_x=0.1
        self.Gamma_theta=0.1
        self.content0=[]
        self.content1=[]
        self.content2=[]
        self.content3=[]
        self.content4=[]
        self.content5=[]
        self.content6=[]
        self.content7=[]

        self.flag_epuck0=flag_epuck0
        self.flag_epuck1=flag_epuck1
        self.flag_epuck2=flag_epuck2
        self.flag_epuck3=flag_epuck3

        self.first_run_epuck0 = True
        self.second_run_epuck0 = True
        self.second_run_epuck1 = True
        self.first_run_epuck1 = True
        self.first_run_epuck2 = True
        self.second_run_epuck2 = True
        self.second_run_epuck3 = True
        self.first_run_epuck3 = True

        self.camera_epuck0=camera_epuck0
        self.camera_epuck1=camera_epuck1
        self.camera_epuck2=camera_epuck2
        self.camera_epuck3=camera_epuck3

        self.yaw_epuck0=yaw_epuck0
        self.yaw_epuck1=yaw_epuck1
        self.yaw_epuck2=yaw_epuck2
        self.yaw_epuck3=yaw_epuck3


        
	# Create a publisher which can "talk" to Myrobot and tell it to move
        self.cmd_vel_0= rospy.Publisher('/epuck2_robot_0/mobile_base/cmd_vel', Twist, queue_size=10)
        self.cmd_vel_1= rospy.Publisher('/epuck2_robot_1/mobile_base/cmd_vel', Twist, queue_size=10)
        self.cmd_vel_2= rospy.Publisher('/epuck2_robot_2/mobile_base/cmd_vel', Twist, queue_size=10)
        self.cmd_vel_3= rospy.Publisher('/epuck2_robot_3/mobile_base/cmd_vel', Twist, queue_size=10)
        self.Odometry_epuck0 = message_filters.Subscriber('/epuck2_robot_0/odom',Odometry)
        self.Odometry_epuck1 = message_filters.Subscriber('/epuck2_robot_1/odom',Odometry)
        self.Odometry_epuck2 = message_filters.Subscriber('/epuck2_robot_2/odom',Odometry)
        self.Odometry_epuck3 = message_filters.Subscriber('/epuck2_robot_3/odom',Odometry)
        self.camera_epuck0_sub=rospy.Subscriber('/zed/ar_pose_marker',AlvarMarkers,self.Camera_epuck0,queue_size=100)
        self.camera_epuck1_sub=rospy.Subscriber('/zed/ar_pose_marker',AlvarMarkers,self.Camera_epuck1,queue_size=100)
        self.camera_epuck2_sub=rospy.Subscriber('/zed/ar_pose_marker',AlvarMarkers,self.Camera_epuck2,queue_size=100)
        self.camera_epuck3_sub=rospy.Subscriber('/zed/ar_pose_marker',AlvarMarkers,self.Camera_epuck3,queue_size=100)

        self.battery_epuck0_sub=rospy.Subscriber('/epuck2_robot_0/battery',BatteryState,self.Battery_epuck0,queue_size=100)
        self.battery_epuck1_sub=rospy.Subscriber('/epuck2_robot_1/battery',BatteryState,self.Battery_epuck1,queue_size=100)
        self.battery_epuck2_sub=rospy.Subscriber('/epuck2_robot_2/battery',BatteryState,self.Battery_epuck2,queue_size=100)
        self.battery_epuck3_sub=rospy.Subscriber('/epuck2_robot_3/battery',BatteryState,self.Battery_epuck3,queue_size=100)

        ts_0 = message_filters.ApproximateTimeSynchronizer([self.Odometry_epuck0,self.Odometry_epuck1,self.Odometry_epuck2,self.Odometry_epuck3], queue_size=10, slop=0.1)
        ts_0.registerCallback(self.callback_epuck)

	#Myrobot will stop if we don't keep telling it to move.  How often should we tell it to move? 10 HZ
        r = rospy.Rate(80);
   
        a1=(init_xpos_epuck0 + init_xpos_epuck1 + init_xpos_epuck2 + init_xpos_epuck3)/4
        print a1

        a2=(init_ypos_epuck0 + init_ypos_epuck1 + init_ypos_epuck2 + init_ypos_epuck3)/4
        print a2


 

    def  callback_epuck(self,odom_sens0,odom_sens1,odom_sens2,odom_sens3):

         if(self.first_run_epuck0):
            self.previous_x_epuck0 = odom_sens0.pose.pose.position.x
            self.previous_y_epuck0 = odom_sens0.pose.pose.position.y
         x_epuck0 = odom_sens0.pose.pose.position.x
         y_epuck0 = odom_sens0.pose.pose.position.y
         theta_epuck0=0
         d_increment_epuck0 = sqrt((x_epuck0 - self.previous_x_epuck0) * (x_epuck0 - self.previous_x_epuck0) +
                   (y_epuck0 - self.previous_y_epuck0)*(y_epuck0 - self.previous_y_epuck0))

         self.leftStepsDiff_epuck0 = odom_sens0.twist.twist.linear.x*MOT_STEP_DIST - self.leftStepsPrev_epuck0    # Expressed in meters.
         self.rightStepsDiff_epuck0 = odom_sens0.twist.twist.linear.y*MOT_STEP_DIST - self.rightStepsPrev_epuck0  # Expressed in meters.

         self.deltaTheta_epuck0 = (self.rightStepsDiff_epuck0 - self.leftStepsDiff_epuck0)/2*WHEEL_DISTANCE # Expressed in radiant.
         self.deltaSteps_epuck0 = (self.rightStepsDiff_epuck0 + self.leftStepsDiff_epuck0)/2  # Expressed in meters.

         self.x_pos_epuck0 += self.deltaSteps_epuck0*math.cos(self.theta_epuck0 + self.deltaTheta_epuck0)  # Expressed in meters.
         self.y_pos_epuck0 += self.deltaSteps_epuck0*math.sin(self.theta_epuck0 + self.deltaTheta_epuck0)  # Expressed in meters.
         self.theta_epuck0 += 2*self.deltaTheta_epuck0   # Expressed in radiant.

         self.leftStepsPrev_epuck0 = odom_sens0.twist.twist.linear.x*MOT_STEP_DIST  # Expressed in meters.
         self.rightStepsPrev_epuck0 = odom_sens0.twist.twist.linear.y*MOT_STEP_DIST    # Expressed in meters.
         self.flag_epuck0=d_increment_epuck0*1000 
         self.total_distance_epuck0 = self.total_distance_epuck0 + d_increment_epuck0
         #print "Total distance traveled e-puck0 is %0.2f"  %    self.total_distance_epuck0
         self.previous_x_epuck0=odom_sens0.pose.pose.position.x
         self.previous_y_epuck0=odom_sens0.pose.pose.position.y
         self.first_run_epuck0 = False
         if(self.second_run_epuck0):
               self.previous_x_pos_epuck0 = self.x_pos_epuck0
               self.previous_theta_epuck0 = self.theta_epuck0
         current_x_pos_epuck0 = self.x_pos_epuck0
         current_theta_epuck0 = self.theta_epuck0
 
         if sqrt((current_x_pos_epuck0 - self.previous_x_pos_epuck0) * (current_x_pos_epuck0 - self.previous_x_pos_epuck0)) >   self.Gamma_x*sqrt(current_x_pos_epuck0*current_x_pos_epuck0):
                   self.x_pos_event_epuck0 = current_x_pos_epuck0
         
         else:
                   self.x_pos_event_epuck0 = self.previous_x_pos_epuck0

         self.previous_x_pos_epuck0=self.x_pos_event_epuck0
         self.second_run_epuck0 = False


         if(self.first_run_epuck1):
            self.previous_x_epuck1 = odom_sens1.pose.pose.position.x
            self.previous_y_epuck1 = odom_sens1.pose.pose.position.y
         x_epuck1 = odom_sens1.pose.pose.position.x
         y_epuck1 = odom_sens1.pose.pose.position.y
         theta_epuck1=0
         d_increment_epuck1 = sqrt((x_epuck1 - self.previous_x_epuck1) * (x_epuck1 - self.previous_x_epuck1) +
                   (y_epuck1 - self.previous_y_epuck1)*(y_epuck1 - self.previous_y_epuck1))

         self.leftStepsDiff_epuck1 = odom_sens1.twist.twist.linear.x*MOT_STEP_DIST - self.leftStepsPrev_epuck1    # Expressed in meters.
         self.rightStepsDiff_epuck1 = odom_sens1.twist.twist.linear.y*MOT_STEP_DIST - self.rightStepsPrev_epuck1  # Expressed in meters.

         self.deltaTheta_epuck1 = (self.rightStepsDiff_epuck1 - self.leftStepsDiff_epuck1)/2*WHEEL_DISTANCE # Expressed in radiant.
         self.deltaSteps_epuck1 = (self.rightStepsDiff_epuck1 + self.leftStepsDiff_epuck1)/2  # Expressed in meters.

         self.x_pos_epuck1 += self.deltaSteps_epuck1*math.cos(self.theta_epuck1 + self.deltaTheta_epuck1)  # Expressed in meters.
         self.y_pos_epuck1 += self.deltaSteps_epuck1*math.sin(self.theta_epuck1 + self.deltaTheta_epuck1)  # Expressed in meters.
         self.theta_epuck1 += 2*self.deltaTheta_epuck1   # Expressed in radiant.

         self.leftStepsPrev_epuck1 = odom_sens1.twist.twist.linear.x*MOT_STEP_DIST  # Expressed in meters.
         self.rightStepsPrev_epuck1 = odom_sens1.twist.twist.linear.y*MOT_STEP_DIST    # Expressed in meters.
         self.flag_epuck1=d_increment_epuck1*1000 
         self.total_distance_epuck1 = self.total_distance_epuck1 + d_increment_epuck1
         self.previous_x_epuck1=odom_sens1.pose.pose.position.x
         self.previous_y_epuck1=odom_sens1.pose.pose.position.y
         self.first_run_epuck1 = False
         if(self.second_run_epuck1):
               self.previous_x_pos_epuck1 = self.x_pos_epuck1
               self.previous_theta_epuck1 = self.theta_epuck1
         current_x_pos_epuck1 = self.x_pos_epuck1
         current_theta_epuck1 = self.theta_epuck1
 
         if sqrt((current_x_pos_epuck1 - self.previous_x_pos_epuck1) * (current_x_pos_epuck1 - self.previous_x_pos_epuck1)) >   self.Gamma_x*sqrt(current_x_pos_epuck1*current_x_pos_epuck1):
                   self.x_pos_event_epuck1 = current_x_pos_epuck1
         
         else:
                   self.x_pos_event_epuck1 = self.previous_x_pos_epuck1
         self.previous_x_pos_epuck1=self.x_pos_event_epuck1
         self.second_run_epuck1 = False
   



         if(self.first_run_epuck2):
            self.previous_x_epuck2 = odom_sens2.pose.pose.position.x
            self.previous_y_epuck2 = odom_sens2.pose.pose.position.y
         x_epuck2 = odom_sens2.pose.pose.position.x
         y_epuck2 = odom_sens2.pose.pose.position.y
         theta_epuck2=0
         d_increment_epuck2 = sqrt((x_epuck2 - self.previous_x_epuck2) * (x_epuck2 - self.previous_x_epuck2) +
                   (y_epuck2 - self.previous_y_epuck2)*(y_epuck2 - self.previous_y_epuck2))

         self.leftStepsDiff_epuck2 = odom_sens2.twist.twist.linear.x*MOT_STEP_DIST - self.leftStepsPrev_epuck2    # Expressed in meters.
         self.rightStepsDiff_epuck2 = odom_sens2.twist.twist.linear.y*MOT_STEP_DIST - self.rightStepsPrev_epuck2  # Expressed in meters.

         self.deltaTheta_epuck2 = (self.rightStepsDiff_epuck2 - self.leftStepsDiff_epuck2)/2*WHEEL_DISTANCE # Expressed in radiant.
         self.deltaSteps_epuck2 = (self.rightStepsDiff_epuck2 + self.leftStepsDiff_epuck2)/2  # Expressed in meters.

         self.x_pos_epuck2 += self.deltaSteps_epuck2*math.cos(self.theta_epuck2 + self.deltaTheta_epuck2)  # Expressed in meters.
         self.y_pos_epuck2 += self.deltaSteps_epuck2*math.sin(self.theta_epuck2 + self.deltaTheta_epuck2)  # Expressed in meters.
         self.theta_epuck2 += 2*self.deltaTheta_epuck2   # Expressed in radiant.

         self.leftStepsPrev_epuck2 = odom_sens2.twist.twist.linear.x*MOT_STEP_DIST  # Expressed in meters.
         self.rightStepsPrev_epuck2 = odom_sens2.twist.twist.linear.y*MOT_STEP_DIST    # Expressed in meters.
         self.flag_epuck2=d_increment_epuck2*1000 

         self.total_distance_epuck2 = self.total_distance_epuck2 + d_increment_epuck2
         #print "Total distance traveled e-puck0 is %0.2f"  %    self.total_distance_epuck0
         self.previous_x_epuck2=odom_sens2.pose.pose.position.x
         self.previous_y_epuck2=odom_sens2.pose.pose.position.y
         self.first_run_epuck2 = False
         if(self.second_run_epuck2):
               self.previous_x_pos_epuck2 = self.x_pos_epuck2
               self.previous_theta_epuck2 = self.theta_epuck2
         current_x_pos_epuck2 = self.x_pos_epuck2
         current_theta_epuck2 = self.theta_epuck2
 
         if sqrt((current_x_pos_epuck2 - self.previous_x_pos_epuck2) * (current_x_pos_epuck2 - self.previous_x_pos_epuck2)) >   self.Gamma_x*sqrt(current_x_pos_epuck2*current_x_pos_epuck2):
                   self.x_pos_event_epuck2 = current_x_pos_epuck2
         
         else:
                   self.x_pos_event_epuck2 = self.previous_x_pos_epuck2
         self.previous_x_pos_epuck2=self.x_pos_event_epuck2
         self.second_run_epuck2 = False


         if(self.first_run_epuck3):
            self.previous_x_epuck3 = odom_sens3.pose.pose.position.x
            self.previous_y_epuck3 = odom_sens3.pose.pose.position.y
         x_epuck3 = odom_sens3.pose.pose.position.x
         y_epuck3 = odom_sens3.pose.pose.position.y
         theta_epuck3=0
         d_increment_epuck3 = sqrt((x_epuck3 - self.previous_x_epuck3) * (x_epuck3 - self.previous_x_epuck3) +
                   (y_epuck3 - self.previous_y_epuck3)*(y_epuck3 - self.previous_y_epuck3))

         self.leftStepsDiff_epuck3 = odom_sens3.twist.twist.linear.x*MOT_STEP_DIST - self.leftStepsPrev_epuck3    # Expressed in meters.
         self.rightStepsDiff_epuck3 = odom_sens3.twist.twist.linear.y*MOT_STEP_DIST - self.rightStepsPrev_epuck3  # Expressed in meters.

         self.deltaTheta_epuck3 = (self.rightStepsDiff_epuck3 - self.leftStepsDiff_epuck3)/2*WHEEL_DISTANCE # Expressed in radiant.
         self.deltaSteps_epuck3 = (self.rightStepsDiff_epuck3 + self.leftStepsDiff_epuck3)/2  # Expressed in meters.

         self.x_pos_epuck3 += self.deltaSteps_epuck3*math.cos(self.theta_epuck3 + self.deltaTheta_epuck3)  # Expressed in meters.
         self.y_pos_epuck3 += self.deltaSteps_epuck3*math.sin(self.theta_epuck3 + self.deltaTheta_epuck3)  # Expressed in meters.
         self.theta_epuck3 += 2*self.deltaTheta_epuck3   # Expressed in radiant.

         self.leftStepsPrev_epuck3 = odom_sens3.twist.twist.linear.x*MOT_STEP_DIST  # Expressed in meters.
         self.rightStepsPrev_epuck3 = odom_sens3.twist.twist.linear.y*MOT_STEP_DIST    # Expressed in meters.
         self.flag_epuck3=d_increment_epuck3*1000 

         self.total_distance_epuck3 = self.total_distance_epuck3 + d_increment_epuck3
         #print "Total distance traveled e-puck0 is %0.2f"  %    self.total_distance_epuck0
         self.previous_x_epuck3=odom_sens3.pose.pose.position.x
         self.previous_y_epuck3=odom_sens3.pose.pose.position.y
         self.first_run_epuck3 = False
         if(self.second_run_epuck3):
               self.previous_x_pos_epuck3 = self.x_pos_epuck3
               self.previous_theta_epuck3 = self.theta_epuck3
         current_x_pos_epuck3 = self.x_pos_epuck3
         current_theta_epuck3 = self.theta_epuck3
 
         if sqrt((current_x_pos_epuck3 - self.previous_x_pos_epuck3) * (current_x_pos_epuck3 - self.previous_x_pos_epuck3)) >   self.Gamma_x*sqrt(current_x_pos_epuck3*current_x_pos_epuck3):
                   self.x_pos_event_epuck3 = current_x_pos_epuck3
         
         else:
                   self.x_pos_event_epuck3 = self.previous_x_pos_epuck3
         self.previous_x_pos_epuck3=self.x_pos_event_epuck3
         self.second_run_epuck3 = False



         # Twist is a datatype for velocity
         move_cmd0 = Twist()
	# let's go forward at 0.2 m/s
         move_cmd0.linear.x = -5*(5*self.x_pos_event_epuck0 -self.x_pos_event_epuck1 -self.x_pos_event_epuck2)
	# let's turn at 0 radians/s
	 move_cmd0.angular.z = 0


 # Twist is a datatype for velocity
         move_cmd1 = Twist()
	# let's go forward at 0.2 m/s
         move_cmd1.linear.x = -3.7*(-self.x_pos_event_epuck0 + 4*self.x_pos_event_epuck1 -self.x_pos_event_epuck2)
	# let's turn at 0 radians/s
	 move_cmd1.angular.z = 0


       # Twist is a datatype for velocity
         move_cmd2 = Twist()
	# let's go forward at 0.2 m/s
         move_cmd2.linear.x = 6*(-self.x_pos_event_epuck0 + self.x_pos_event_epuck1 + 5*self.x_pos_event_epuck2 -3*self.x_pos_event_epuck3)
	# let's turn at 0 radians/s
	 move_cmd2.angular.z = 0


        # Twist is a datatype for velocity
         move_cmd3 = Twist()
	# let's go forward at 0.2 m/s
         move_cmd3.linear.x =  5*(-self.x_pos_event_epuck2 + 4.5*self.x_pos_event_epuck3)
	# let's turn at 0 radians/s
	 move_cmd3.angular.z = 0





         df = pd.DataFrame() 
         self.content0.append(str(self.x_pos_event_epuck0))

         df['robot0'] = self.content0 
         self.content1.append(str(self.x_pos_event_epuck1))

         df['robot1'] = self.content1
         self.content2.append(str(self.x_pos_event_epuck2))

         df['robot2'] = self.content2
         self.content3.append(str(self.x_pos_event_epuck3))

         df['robot3'] = self.content3
         self.content4.append(str(self.SOC_epuck0))
         df['SOC_robot0'] = self.content4
         self.content5.append(str(self.SOC_epuck1))
         df['SOC_robot1'] = self.content5
         self.content6.append(str(self.SOC_epuck2))
         df['SOC_robot2'] = self.content6
         self.content7.append(str(self.SOC_epuck3))
         df['SOC_robot3'] = self.content7
         df.to_excel('Tabuda_Type.xlsx', index = False) 

	    # publish the velocity
         self.cmd_vel_0.publish(move_cmd0)
         self.cmd_vel_1.publish(move_cmd1)
         self.cmd_vel_2.publish(move_cmd2)
         self.cmd_vel_3.publish(move_cmd3)
	    # wait for 0.1 seconds (10 HZ) and publish again

  





    def  Battery_epuck0(self,batt_epuck0):
          self.SOC_epuck0=batt_epuck0.percentage
          print self.SOC_epuck0

    def  Battery_epuck1(self,batt_epuck1):
          self.SOC_epuck1=batt_epuck1.percentage
          print self.SOC_epuck1

    def  Battery_epuck2(self,batt_epuck2):
          self.SOC_epuck2=batt_epuck2.percentage
          print self.SOC_epuck2

    def  Battery_epuck3(self,batt_epuck3):
          self.SOC_epuck3=batt_epuck3.percentage
          print self.SOC_epuck3    


   

    def  Camera_epuck0(self,AlvarMarkers_epuck0):
           self.camera_x_epuck0=1.7*AlvarMarkers_epuck0.markers[0].pose.pose.position.x - 0.3 +0.127 + 0.063 + 0.02 +0.01
           self.camera_y_epuck0=AlvarMarkers_epuck0.markers[0].pose.pose.position.y -0.109 -0.01
           self.camera_z_epuck0=AlvarMarkers_epuck0.markers[0].pose.pose.position.z
           x_epuck0=AlvarMarkers_epuck0.markers[0].pose.pose.orientation.x
           y_epuck0=AlvarMarkers_epuck0.markers[0].pose.pose.orientation.y
           z_epuck0=AlvarMarkers_epuck0.markers[0].pose.pose.orientation.z
           w_epuck0=AlvarMarkers_epuck0.markers[0].pose.pose.orientation.w
           self.camera_epuck0=matrix([self.camera_x_epuck0,self.camera_y_epuck0,self.camera_z_epuck0])          
           siny_cosp_epuck0 = 2 * (w_epuck0 * z_epuck0 + x_epuck0 * y_epuck0)
           cosy_cosp_epuck0 = w_epuck0**2 + x_epuck0**2 - y_epuck0**2 -z_epuck0**2
           self.yaw_epuck1 = atan2(siny_cosp_epuck0, cosy_cosp_epuck0)
           #print self.yaw_epuck1*(180/pi)



    def  Camera_epuck1(self,AlvarMarkers_epuck1):
           self.camera_x_epuck1=1.775*AlvarMarkers_epuck1.markers[1].pose.pose.position.x -0.36 + 0.15 +0.045 -0.09 + 0.02
           self.camera_y_epuck1=AlvarMarkers_epuck1.markers[1].pose.pose.position.y + 0.023 -0.017
           self.camera_z_epuck1=AlvarMarkers_epuck1.markers[1].pose.pose.position.z
           x_epuck1=AlvarMarkers_epuck1.markers[1].pose.pose.orientation.x
           y_epuck1=AlvarMarkers_epuck1.markers[1].pose.pose.orientation.y
           z_epuck1=AlvarMarkers_epuck1.markers[1].pose.pose.orientation.z
           w_epuck1=AlvarMarkers_epuck1.markers[1].pose.pose.orientation.w
           self.camera_epuck1=matrix([self.camera_x_epuck1,self.camera_y_epuck1,self.camera_z_epuck1])          
           siny_cosp_epuck1 = 2 * (w_epuck1 * z_epuck1 + x_epuck1 * y_epuck1)
           cosy_cosp_epuck1 = w_epuck1**2 + x_epuck1**2 - y_epuck1**2 -z_epuck1**2
           self.yaw_epuck0 = atan2(siny_cosp_epuck1, cosy_cosp_epuck1)


    def  Camera_epuck2(self,AlvarMarkers_epuck2):
           self.camera_x_epuck2=1.85*AlvarMarkers_epuck2.markers[2].pose.pose.position.x -0.36 + 0.15 -0.0542 -0.08 -0.01
           self.camera_y_epuck2=1.05*AlvarMarkers_epuck2.markers[2].pose.pose.position.y + 0.0115 -0.04 +0.03
           self.camera_z_epuck2=AlvarMarkers_epuck2.markers[2].pose.pose.position.z
           x_epuck2=AlvarMarkers_epuck2.markers[2].pose.pose.orientation.x
           y_epuck2=AlvarMarkers_epuck2.markers[2].pose.pose.orientation.y
           z_epuck2=AlvarMarkers_epuck2.markers[2].pose.pose.orientation.z
           w_epuck2=AlvarMarkers_epuck2.markers[2].pose.pose.orientation.w
           self.camera_epuck2=matrix([self.camera_x_epuck2,self.camera_y_epuck2,self.camera_z_epuck2])          
           siny_cosp_epuck2 = 2 * (w_epuck2 * z_epuck2 + x_epuck2 * y_epuck2)
           cosy_cosp_epuck2 = w_epuck2**2 + x_epuck2**2 - y_epuck2**2 -z_epuck2**2
           self.yaw_epuck2 = atan2(siny_cosp_epuck2, cosy_cosp_epuck2)


    def  Camera_epuck3(self,AlvarMarkers_epuck3):
           self.camera_x_epuck3=1.6*AlvarMarkers_epuck3.markers[-1].pose.pose.position.x -0.36 + 0.15 -0.0377
           self.camera_y_epuck3=AlvarMarkers_epuck3.markers[-1].pose.pose.position.y + 0.13
           self.camera_z_epuck3=AlvarMarkers_epuck3.markers[-1].pose.pose.position.z
           x_epuck3=AlvarMarkers_epuck3.markers[-1].pose.pose.orientation.x
           y_epuck3=AlvarMarkers_epuck3.markers[-1].pose.pose.orientation.y
           z_epuck3=AlvarMarkers_epuck3.markers[-1].pose.pose.orientation.z
           w_epuck3=AlvarMarkers_epuck3.markers[-1].pose.pose.orientation.w
           self.camera_epuck3=matrix([self.camera_x_epuck3,self.camera_y_epuck3,self.camera_z_epuck3])          
           siny_cosp_epuck3 = 2 * (w_epuck3 * z_epuck3 + x_epuck3 * y_epuck3)
           cosy_cosp_epuck3 = w_epuck3**2 + x_epuck3**2 - y_epuck3**2 -z_epuck3**2
           self.yaw_epuck3 = atan2(siny_cosp_epuck3, cosy_cosp_epuck3)

        
    def shutdown(self):
        # stop Myrobot
        rospy.loginfo("Stop Myrobot")
	# a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop Myrobot
        self.cmd_vel_0.publish(Twist())
        self.cmd_vel_1.publish(Twist())
        self.cmd_vel_2.publish(Twist())
        self.cmd_vel_3.publish(Twist())
	# sleep just makes sure Myrobot receives the stop command prior to shutting down the script

 
if __name__ == '__main__':
    ic = CircleMode()
    rospy.init_node('CircleMode', anonymous=False)
    r = rospy.Rate(80);
    try:
        rospy.spin()
    except:
        rospy.loginfo("CircleMode node terminated.")

