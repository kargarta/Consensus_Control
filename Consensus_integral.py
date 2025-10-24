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
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
import math 

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
    def __init__(self,deltaTheta_epuck0=0,deltaTheta_epuck1=0,deltaSteps_epuck0=0,deltaSteps_epuck1=0,deltaTheta_epuck2=0,deltaTheta_epuck3=0,deltaSteps_epuck2=0,deltaSteps_epuck3=0, init_xpos_epuck0=-0.2,init_ypos_epuck0=0.1, init_theta_epuck0=0,init_xpos_epuck1=-0.2, init_ypos_epuck1=-0.1, init_theta_epuck1=0, init_xpos_epuck2=0.2, init_ypos_epuck2=-0.1, init_theta_epuck2=3.14,init_xpos_epuck3=0.2, init_ypos_epuck3=0.1, init_theta_epuck3=3.14, yaw_epuck0=0, yaw_epuck1=0, yaw_epuck2=0, yaw_epuck3=0, camera_epuck0=matrix([0.378, -0.1293 ,0]), camera_epuck1=matrix([0.311, -0.145 ,0]), camera_epuck2=matrix([0.268, -0.010,0]), camera_epuck3=matrix([0.3400, -0.006 ,0]), flag_epuck0=0, flag_epuck1=0, flag_epuck2=0, flag_epuck3=0):
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

        self.J_epuck0=0
        self.J_epuck1=0
        self.J_epuck2=0
        self.J_epuck3=0

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
        self.Delta_x=0.00
        self.Delta_theta=0.00

        self.content0=[]
        self.content1=[]
        self.content2=[]
        self.content3=[]
        self.content4=[]
        self.content5=[]
        self.content6=[]
        self.content7=[]
        self.content8=[]
        self.content9=[]
        self.content10=[]
        self.content11=[]
        self.content12=[]
        self.content13=[]
        self.content14=[]
        self.content15=[]
        self.content16=[]
        self.content17=[]
        self.content18=[]
        self.content19=[]
        self.content20=[]
        self.content21=[]
        self.content22=[]
        self.content23=[]
        self.content24=[]
        self.content25=[]
        self.content26=[]
        self.content27=[]

        self.X0=0
        self.Y0=0
        self.X1=0
        self.Y1=0
        self.X2=0
        self.Y2=0
        self.X3=0
        self.Y3=0

        self.Z0=0
        self.W0=0
        self.Z1=0
        self.W1=0
        self.Z2=0
        self.W2=0
        self.Z3=0
        self.W3=0


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

        self.EKF_epuck0 = message_filters.Subscriber('/epuck2_robot_0/robot1_pose_ekf/odom_combined',PoseWithCovarianceStamped)
        self.EKF_epuck1 = message_filters.Subscriber('/epuck2_robot_1/robot2_pose_ekf/odom_combined',PoseWithCovarianceStamped)
        self.EKF_epuck2 = message_filters.Subscriber('/epuck2_robot_2/robot3_pose_ekf/odom_combined',PoseWithCovarianceStamped)
        self.EKF_epuck3 = message_filters.Subscriber('/epuck2_robot_3/robot4_pose_ekf/odom_combined',PoseWithCovarianceStamped)

        ts_0 = message_filters.ApproximateTimeSynchronizer([self.Odometry_epuck0,self.Odometry_epuck1,self.Odometry_epuck2,self.Odometry_epuck3, self.EKF_epuck0, self.EKF_epuck1, self.EKF_epuck2, self.EKF_epuck3], queue_size=100, slop=0.1)
        ts_0.registerCallback(self.callback_epuck)

	#Myrobot will stop if we don't keep telling it to move.  How often should we tell it to move? 10 HZ
        r = rospy.Rate(80);
   
        a1=(init_xpos_epuck0 + init_xpos_epuck1 + init_xpos_epuck2 + init_xpos_epuck3)/4
        print a1

        a2=(init_ypos_epuck0 + init_ypos_epuck1 + init_ypos_epuck2 + init_ypos_epuck3)/4
        print a2


 

    def  callback_epuck(self,odom_sens0,odom_sens1,odom_sens2,odom_sens3,EKF0,EKF1,EKF2,EKF3):

         self.x_epuck0 = odom_sens0.pose.pose.position.x
         self.y_epuck0 = odom_sens0.pose.pose.position.y

         self.x_epuck1 = odom_sens1.pose.pose.position.x
         self.y_epuck1 = odom_sens1.pose.pose.position.y

         self.x_epuck2 = odom_sens2.pose.pose.position.x
         self.y_epuck2 = odom_sens2.pose.pose.position.y


         self.x_epuck3 = odom_sens3.pose.pose.position.x
         self.y_epuck3 = odom_sens3.pose.pose.position.y

         
         self.filtered_x_epuck0 = EKF0.pose.pose.position.x
         self.filtered_y_epuck0 = EKF0.pose.pose.position.y
         self.filtered_theta_x_epuck0 = EKF0.pose.pose.orientation.x
         self.filtered_theta_y_epuck0 = EKF0.pose.pose.orientation.y
         self.filtered_theta_z_epuck0 = EKF0.pose.pose.orientation.z
         self.filtered_theta_w_epuck0 = EKF0.pose.pose.orientation.w
         siny_cosp_epuck0 = 2 * (self.filtered_theta_w_epuck0 * self.filtered_theta_z_epuck0 + self.filtered_theta_x_epuck0 * self.filtered_theta_y_epuck0)
         cosy_cosp_epuck0 = self.filtered_theta_w_epuck0**2 + self.filtered_theta_x_epuck0**2 - self.filtered_theta_y_epuck0**2 -self.filtered_theta_z_epuck0**2
         self.filtered_theta_epuck0 = abs(atan2(siny_cosp_epuck0, cosy_cosp_epuck0))

         if(self.second_run_epuck0):
               self.previous_x_pos_epuck0 = self.filtered_x_epuck0
               self.previous_theta_epuck0 = self.filtered_theta_epuck0
         current_x_pos_epuck0 = self.filtered_x_epuck0
         current_theta_epuck0 = self.filtered_theta_epuck0
 
         self.X0 +=abs(current_x_pos_epuck0 - self.previous_x_pos_epuck0)*abs(current_x_pos_epuck0)
         self.Y0 +=abs(current_x_pos_epuck0*current_x_pos_epuck0)

         self.Z0 +=abs(current_theta_epuck0 - self.previous_theta_epuck0)*abs(current_theta_epuck0)
         self.W0 +=abs(current_theta_epuck0*current_theta_epuck0)

         if abs(self.X0) > self.Gamma_x*abs(self.Y0) :
                   self.x_pos_event_epuck0 = current_x_pos_epuck0
                   self.X0=0
                   self.Y0=0        
         else:
                   self.x_pos_event_epuck0 = self.previous_x_pos_epuck0


         if abs(self.Z0) > self.Gamma_theta*abs(self.W0) :
                   self.theta_event_epuck0 = current_theta_epuck0
                   self.Z0=0
                   self.W0=0        
         else:
                   self.theta_event_epuck0 = self.previous_theta_epuck0

         self.previous_x_pos_epuck0=self.x_pos_event_epuck0
         self.previous_theta_epuck0=self.theta_event_epuck0
         self.second_run_epuck0 = False


         self.filtered_x_epuck1 = EKF1.pose.pose.position.x
         self.filtered_y_epuck1 = EKF1.pose.pose.position.y
         self.filtered_theta_x_epuck1 = EKF1.pose.pose.orientation.x
         self.filtered_theta_y_epuck1 = EKF1.pose.pose.orientation.y
         self.filtered_theta_z_epuck1 = EKF1.pose.pose.orientation.z
         self.filtered_theta_w_epuck1 = EKF1.pose.pose.orientation.w
         siny_cosp_epuck1 = 2 * (self.filtered_theta_w_epuck1 * self.filtered_theta_z_epuck1 + self.filtered_theta_x_epuck1 * self.filtered_theta_y_epuck1)
         cosy_cosp_epuck1 = self.filtered_theta_w_epuck1**2 + self.filtered_theta_x_epuck1**2 - self.filtered_theta_y_epuck1**2 -self.filtered_theta_z_epuck1**2
         self.filtered_theta_epuck1 = abs(atan2(siny_cosp_epuck1, cosy_cosp_epuck1))

         if(self.second_run_epuck1):
               self.previous_x_pos_epuck1 = self.filtered_x_epuck1
               self.previous_theta_epuck1= self.filtered_theta_epuck1
         current_x_pos_epuck1 = self.filtered_x_epuck1
         current_theta_epuck1 = self.filtered_theta_epuck1
 
         self.X1 +=abs(current_x_pos_epuck1 - self.previous_x_pos_epuck1)*abs(current_x_pos_epuck1)
         self.Y1 +=abs(current_x_pos_epuck1*current_x_pos_epuck1)

         self.Z1 +=abs(current_theta_epuck1 - self.previous_theta_epuck1)*abs(current_theta_epuck1)
         self.W1 +=abs(current_theta_epuck1*current_theta_epuck1)

         if abs(self.X1) > self.Gamma_x*abs(self.Y1) :
                   self.x_pos_event_epuck1 = current_x_pos_epuck1
                   self.X1=0
                   self.Y1=0        
         else:
                   self.x_pos_event_epuck1 = self.previous_x_pos_epuck1


         if abs(self.Z1) > self.Gamma_theta*abs(self.W1) :
                   self.theta_event_epuck1 = current_theta_epuck1
                   self.Z1=0
                   self.W1=0        
         else:
                   self.theta_event_epuck1 = self.previous_theta_epuck1

         self.previous_x_pos_epuck1=self.x_pos_event_epuck1
         self.previous_theta_epuck1=self.theta_event_epuck1
         self.second_run_epuck1 = False



         self.filtered_x_epuck2 = EKF2.pose.pose.position.x
         self.filtered_y_epuck2 = EKF2.pose.pose.position.y
         self.filtered_theta_x_epuck2 = EKF2.pose.pose.orientation.x
         self.filtered_theta_y_epuck2 = EKF2.pose.pose.orientation.y
         self.filtered_theta_z_epuck2 = EKF2.pose.pose.orientation.z
         self.filtered_theta_w_epuck2 = EKF2.pose.pose.orientation.w
         siny_cosp_epuck2 = 2 * (self.filtered_theta_w_epuck2 * self.filtered_theta_z_epuck2 + self.filtered_theta_x_epuck2 * self.filtered_theta_y_epuck2)
         cosy_cosp_epuck2 = self.filtered_theta_w_epuck2**2 + self.filtered_theta_x_epuck2**2 - self.filtered_theta_y_epuck2**2 -self.filtered_theta_z_epuck2**2
         self.filtered_theta_epuck2 = abs(atan2(siny_cosp_epuck2, cosy_cosp_epuck2))

         if(self.second_run_epuck2):
               self.previous_x_pos_epuck2 = self.filtered_x_epuck2
               self.previous_theta_epuck2 = self.filtered_theta_epuck2
         current_x_pos_epuck2 = self.filtered_x_epuck2
         current_theta_epuck2 = self.filtered_theta_epuck2
 
         self.X2 +=abs(current_x_pos_epuck2 - self.previous_x_pos_epuck2)*abs(current_x_pos_epuck2)
         self.Y2 +=abs(current_x_pos_epuck2*current_x_pos_epuck2)

         self.Z2 +=abs(current_theta_epuck2 - self.previous_theta_epuck2)*abs(current_theta_epuck2)
         self.W2 +=abs(current_theta_epuck2*current_theta_epuck2)

         if abs(self.X2) > self.Gamma_x*abs(self.Y2) :
                   self.x_pos_event_epuck2 = current_x_pos_epuck2
                   self.X2=0
                   self.Y2=0        
         else:
                   self.x_pos_event_epuck2 = self.previous_x_pos_epuck2


         if abs(self.Z2) > self.Gamma_theta*abs(self.W2) :
                   self.theta_event_epuck2 = current_theta_epuck2
                   self.Z2=0
                   self.W2=0        
         else:
                   self.theta_event_epuck2 = self.previous_theta_epuck2

         self.previous_x_pos_epuck2=self.x_pos_event_epuck2
         self.previous_theta_epuck2=self.theta_event_epuck2
         self.second_run_epuck2 = False



         self.filtered_x_epuck3 = EKF3.pose.pose.position.x
         self.filtered_y_epuck3 = EKF3.pose.pose.position.y
         self.filtered_theta_x_epuck3 = EKF3.pose.pose.orientation.x
         self.filtered_theta_y_epuck3 = EKF3.pose.pose.orientation.y
         self.filtered_theta_z_epuck3 = EKF3.pose.pose.orientation.z
         self.filtered_theta_w_epuck3 = EKF3.pose.pose.orientation.w
         siny_cosp_epuck3 = 2 * (self.filtered_theta_w_epuck3 * self.filtered_theta_z_epuck3 + self.filtered_theta_x_epuck3 * self.filtered_theta_y_epuck3)
         cosy_cosp_epuck3 = self.filtered_theta_w_epuck3**2 + self.filtered_theta_x_epuck3**2 - self.filtered_theta_y_epuck3**2 -self.filtered_theta_z_epuck3**2
         self.filtered_theta_epuck3 = abs(atan2(siny_cosp_epuck3, cosy_cosp_epuck3))

         if(self.second_run_epuck3):
               self.previous_x_pos_epuck3 = self.filtered_x_epuck3
               self.previous_theta_epuck3 = self.filtered_theta_epuck3
         current_x_pos_epuck3 = self.filtered_x_epuck3
         current_theta_epuck3 = self.filtered_theta_epuck3
 
         self.X3 +=abs(current_x_pos_epuck3 - self.previous_x_pos_epuck3)*abs(current_x_pos_epuck3)
         self.Y3 +=abs(current_x_pos_epuck3*current_x_pos_epuck3)

         self.Z3 +=abs(current_theta_epuck3 - self.previous_theta_epuck3)*abs(current_theta_epuck3)
         self.W3 +=abs(current_theta_epuck3*current_theta_epuck3)

         if abs(self.X3) > self.Gamma_x*abs(self.Y3) :
                   self.x_pos_event_epuck3 = current_x_pos_epuck3
                   self.X3=0
                   self.Y3=0        
         else:
                   self.x_pos_event_epuck3 = self.previous_x_pos_epuck3


         if abs(self.Z3) > self.Gamma_theta*abs(self.W3) :
                   self.theta_event_epuck3 = current_theta_epuck3
                   self.Z3=0
                   self.W3=0        
         else:
                   self.theta_event_epuck3 = self.previous_theta_epuck3

         self.previous_x_pos_epuck3=self.x_pos_event_epuck3
         self.previous_theta_epuck3=self.theta_event_epuck3
         self.second_run_epuck3 = False


        
   # Twist is a datatype for velocity
         move_cmd0 = Twist()
	# let's go forward at 0.2 m/s
         move_cmd0.linear.x = -5*0.8*(5*self.x_pos_event_epuck0 -self.x_pos_event_epuck1 -self.x_pos_event_epuck2)
	# let's turn at 0 radians/s
	 move_cmd0.angular.z = -0.025*(2*self.theta_event_epuck0 + self.theta_event_epuck1 -self.theta_event_epuck2)


 # Twist is a datatype for velocity
         move_cmd1 = Twist()
	# let's go forward at 0.2 m/s
         move_cmd1.linear.x = -3.7*0.8*(-self.x_pos_event_epuck0 + 4*self.x_pos_event_epuck1 -self.x_pos_event_epuck2)
	# let's turn at 0 radians/s
	 move_cmd1.angular.z = 0.02*(-self.theta_event_epuck0 + 2*self.theta_event_epuck1 -self.theta_event_epuck2)


       # Twist is a datatype for velocity
         move_cmd2 = Twist()
	# let's go forward at 0.2 m/s
         move_cmd2.linear.x = 6*0.8*(-self.x_pos_event_epuck0 + self.x_pos_event_epuck1 + 5*self.x_pos_event_epuck2 -3*self.x_pos_event_epuck3)
	# let's turn at 0 radians/s
	 move_cmd2.angular.z = 0.01*(-self.theta_event_epuck0 - self.theta_event_epuck1 + 2.5*self.theta_event_epuck2  - self.theta_event_epuck3)
        


        # Twist is a datatype for velocity
         move_cmd3 = Twist()
	# let's go forward at 0.2 m/s
         move_cmd3.linear.x =  5*0.8*(-self.x_pos_event_epuck2 + 4.5*self.x_pos_event_epuck3)
	# let's turn at 0 radians/s
	 move_cmd3.angular.z = -0.005*(self.theta_event_epuck2  + 2*self.theta_event_epuck3)

         Q=1
         R=1

         self.J_epuck0 +=self.x_pos_event_epuck0*self.x_pos_event_epuck0*Q + 0.01*0.01*abs(move_cmd0.linear.x*math.cos(self.theta_event_epuck0)*move_cmd0.linear.x*math.cos(self.theta_event_epuck0)*R) + self.filtered_y_epuck0*self.filtered_y_epuck0*Q +0.01*0.01*abs(move_cmd0.linear.x*math.sin(self.theta_event_epuck0)*move_cmd0.linear.x*math.sin(self.theta_event_epuck0)*R)

         self.J_epuck1 +=self.x_pos_event_epuck1*self.x_pos_event_epuck1*Q + 0.01*0.01*abs(move_cmd1.linear.x*math.cos(self.theta_event_epuck1)*move_cmd1.linear.x*math.cos(self.theta_event_epuck1)*R) + self.filtered_y_epuck1*self.filtered_y_epuck1*Q +0.01*0.01*abs(move_cmd1.linear.x*math.sin(self.theta_event_epuck1)*move_cmd1.linear.x*math.sin(self.theta_event_epuck1)*R)

         self.J_epuck2 +=self.x_pos_event_epuck2*self.x_pos_event_epuck2*Q + 0.01*0.01*abs(move_cmd2.linear.x*math.cos(self.theta_event_epuck2)*move_cmd2.linear.x*math.cos(self.theta_event_epuck2)*R) + self.filtered_y_epuck2*self.filtered_y_epuck2*Q +0.01*0.01*abs(move_cmd2.linear.x*math.sin(self.theta_event_epuck2)*move_cmd2.linear.x*math.sin(self.theta_event_epuck2)*R)

         self.J_epuck3 +=self.x_pos_event_epuck3*self.x_pos_event_epuck3*Q + 0.01*0.01*abs(move_cmd3.linear.x*math.cos(self.theta_event_epuck3)*move_cmd3.linear.x*math.cos(self.theta_event_epuck3)*R) + self.filtered_y_epuck3*self.filtered_y_epuck3*Q +0.01*0.01*abs(move_cmd3.linear.x*math.sin(self.theta_event_epuck3)*move_cmd3.linear.x*math.sin(self.theta_event_epuck3)*R)

         df = pd.DataFrame() 
         self.content0.append(str(self.x_pos_event_epuck0))
         df['filtered_x_epuck0'] = self.content0 
         self.content1.append(str(self.x_pos_event_epuck1))
         df['filtered_x_epuck1'] = self.content1
         self.content2.append(str(self.x_pos_event_epuck2))
         df['filtered_x_epuck2'] = self.content2
         self.content3.append(str(self.x_pos_event_epuck3))
         df['filtered_x_epuck3'] = self.content3
         self.content4.append(str(self.filtered_y_epuck0))
         df['filtered_y_epuck0'] = self.content4
         self.content5.append(str(self.filtered_y_epuck1))
         df['filtered_y_epuck1'] = self.content5
         self.content6.append(str(self.filtered_y_epuck2))
         df['filtered_y_epuck2'] = self.content6
         self.content7.append(str(self.filtered_y_epuck3))
         df['filtered_y_epuck3'] = self.content7
         self.content8.append(str(move_cmd0.linear.x))
         df['linear_speed_robot0'] = self.content8
         self.content9.append(str(move_cmd1.linear.x))
         df['linear_speed_robot1'] = self.content9
         self.content10.append(str(move_cmd2.linear.x))
         df['linear_speed_robot2'] = self.content10
         self.content11.append(str(move_cmd3.linear.x))
         df['linear_speed_robot3'] = self.content11
         self.content12.append(str(move_cmd0.angular.z))
         df['angular_speed_robot0'] = self.content12
         self.content13.append(str(move_cmd1.angular.z))
         df['angular_speed_robot1'] = self.content13
         self.content14.append(str(move_cmd2.angular.z))
         df['angular_speed_robot2'] = self.content14
         self.content15.append(str(move_cmd3.angular.z))
         df['angular_speed_robot3'] = self.content15
 
         self.content16.append(str(self.x_epuck0))
         df['x_epuck0'] = self.content16
         self.content17.append(str(self.x_epuck1))
         df['x_epuck1'] = self.content17
         self.content18.append(str(self.x_epuck2))
         df['x_epuck2'] = self.content18
         self.content19.append(str(self.x_epuck3))
         df['x_epuck3'] = self.content19

         self.content20.append(str(self.y_epuck0))
         df['y_epuck0'] = self.content20
         self.content21.append(str(self.y_epuck1))
         df['y_epuck1'] = self.content21
         self.content22.append(str(self.y_epuck2))
         df['y_epuck2'] = self.content22
         self.content23.append(str(self.y_epuck3))
         df['y_epuck3'] = self.content23

         self.content24.append(str(self.J_epuck0))
         df['cost_epuck0'] = self.content24
         self.content25.append(str(self.J_epuck1))
         df['cost_epuck1'] = self.content25
         self.content26.append(str(self.J_epuck2))
         df['cost_epuck2'] = self.content26
         self.content27.append(str(self.J_epuck3))
         df['cost_epuck3'] = self.content27

         df.to_excel('LQG_integral.xlsx', index = False) 

	    # publish the velocity
         self.cmd_vel_0.publish(move_cmd0)
         self.cmd_vel_1.publish(move_cmd1)
         self.cmd_vel_2.publish(move_cmd2)
         self.cmd_vel_3.publish(move_cmd3)
	    # wait for 0.1 seconds (10 HZ) and publish again

  
        
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

