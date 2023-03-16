#!/usr/bin/env python

'''

*Team Id: 2436
*Author list: Amith Prakash S P, Dushyantha Kumar E, Pruthvi M G, Tharun V K
*Filename: task_3_pos_hold
*Theme: Pollinator bee 
*Functions: _init_, PlutoMsg, arm, disarm, position_hold, calc_pid, pid_roll, pid_pitch, pid_throt, pid_yaw,limit, get_pose, set_drone_yaw
*Global Variables: none

'''
#The required packages are imported here
from plutodrone.msg import *
from pid_tune.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int32
from std_msgs.msg import Float64
from plutodrone.srv import *
import time
import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import	Image
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int32

freq=0
class DroneFly():
    """docstring for DroneFly"""
    points=[]
    points.append([0.0,1.5,18.0])
    points.append([-4.2,1.4,18.1])
    points.append([0,0,0])
    points.append([0,0,0])
    points.append([0,0,0])
    points.append([0.0,0.0,25.0])
    def __init__(self):   
        '''
        *Function Name: _init_
        *Input: self
        *output: self
        *Logic: creating a node
        *Example call: 
        '''
        rospy.init_node('pluto_fly','ros_bridge',disable_signals = True)

        # Create a ROS Bridge
        self.ros_bridge = cv_bridge.CvBridge()
        
        self.pluto_cmd = rospy.Publisher('/drone_command', PlutoMsg, queue_size=10)
        self.pluto_cmd = rospy.Publisher('/drone_command', PlutoMsg, queue_size=10)
        self.alt_error = rospy.Publisher('/alt_error', Float64 , queue_size=10)
        self.pitch_error = rospy.Publisher('/pitch_error', Float64, queue_size=10)
        self.roll_error = rospy.Publisher('/roll_error', Float64, queue_size=10)
        self.yaw_error = rospy.Publisher('/yaw_error', Float64, queue_size=10)
        
        self.image_sub = rospy.Subscriber('/usb_cam/image_rect_color', Image, self.image_callback)
        self.image_pub = rospy.Publisher('/whycon/image_out', Image, queue_size=10)
        self.red_pub = rospy.Publisher('/red_pub',Int32,queue_size=10)
        self.blue_pub = rospy.Publisher('/blue_pub',Int32,queue_size=10)
        self.green_pub = rospy.Publisher('/green_pub',Int32,queue_size=10)

        #rospy.Subscriber('/drone_yaw', Float64, self.set_drone_yaw)
        rospy.Subscriber('whycon/poses', PoseArray, self.get_pose)
        rospy.Subscriber('PlutoService', Float64, self.set_drone_yaw)


        self.cmd = PlutoMsg()

        # Position to hold.
        self.move(0)
        self.wp_yaw = 0.0
        
        #initial values
        self.cmd.rcRoll = 1500
        self.cmd.rcPitch = 1500
        self.cmd.rcYaw = 1500
        self.cmd.rcThrottle = 1500
        self.cmd.rcAUX1 = 1500
        self.cmd.rcAUX2 = 1500
        self.cmd.rcAUX3 = 1500
        self.cmd.rcAUX4 = 1000
        self.cmd.plutoIndex = 0

        self.drone_x = 0.0
        self.drone_y = 0.0
        self.drone_z = 0.0
        self.angle = 0.0
        self.error={1:0.0,2:0.0,3:0.0,4:0.0}
        self.error_sum={1:0.0,2:0.0,3:0.0,4:0.0}
        self.error_last={1:0.0,2:0.0,3:0.0,4:0.0}
        self.d_error={1:0.0,2:0.0,3:0.0,4:0.0}
        

        #PID constants for Roll
        self.kp_roll = 15
        self.ki_roll = 0.085
        self.kd_roll = 725
        #PID constants for Pitch
        self.kp_pitch = 15
        self.ki_pitch = 0.045
        self.kd_pitch = 400
        
        #PID constants for Yaw
        self.kp_yaw = 0.0
        self.ki_yaw = 0.0
        self.kd_yaw = 0.0

        #PID constants for Throttle
        self.kp_throt = 50
        self.ki_throt = 0.0
        self.kd_throt = 17

        # Correction values after PID is computed
        self.correct_roll = 0.0
        self.correct_pitch = 0.0
        self.correct_yaw = 0.0
        self.correct_throt = 0.0
        self.rflag = 0.0
        self.r2flag = 0.0
        self.bflag = 0.0
        self.gflag = 0.0

        # Loop time for PID computation. You are free to experiment with this
        self.last_time = 0.0
        self.loop_time = 0.032

        rospy.sleep(.1)

		
    def blue(self):
		area1 = [cv2.contourArea(contour) for contour in self.contours1]
		max_index = np.argmax(area1)
		cnt = self.contours1[max_index]
   		# get the bounding rect
		x, y, w, h = cv2.boundingRect(cnt)
       	        # draw a blue rectangle to visualize the bounding rect
		cv2.rectangle(self.img, (x, y), (x+w, y+h), (255,0, 0), 1)
		cv2.putText(self.img, "BLUE", (x,y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 2)
	
    def green(self):
		area2 = [cv2.contourArea(contour) for contour in self.contours2]
		max_index = np.argmax(area2)
		cnt = self.contours2[max_index]
   		# get the bounding rect
		x, y, w, h = cv2.boundingRect(cnt)
     		# draw a green rectangle to visualize the bounding rect
		cv2.rectangle(self.img, (x, y), (x+w, y+h), (0,255, 0), 1)
		cv2.putText(self.img, "GREEN", (x,y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

    def red(self):
		area3 = [cv2.contourArea(contour) for contour in self.contours3]
		max_index = np.argmax(area3)
		cnt = self.contours3[max_index]
   		# get the bounding rect
		x, y, w, h = cv2.boundingRect(cnt)
       	        # draw a blue rectangle to visualize the bounding rect
		cv2.rectangle(self.img, (x, y), (x+w, y+h), (0,0, 255), 1)
		cv2.putText(self.img, "RED", (x,y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)


    def image_callback(self,msg):
		# 'image' is now an opencv frame
		# You can run opencv operations on 'image'
        global freq
        self.img = self.ros_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(self.img,cv2.COLOR_BGR2HSV)
        
        lower_range1 = np.array([80,100,150])
        upper_range1 = np.array([130,200,255])
        mask1 = cv2.inRange(hsv,lower_range1,upper_range1)
        _,self.m1 = cv2.connectedComponents(mask1)
        self.var1 = np.amax(self.m1)
        _, self.contours1,_ = cv2.findContours(mask1, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        if len(self.contours1) > 1:
            self.blue()
            self.bflag=1
        self.blue_pub.publish(self.var1)

            
        lower_range2 = np.array([40,150,40])
        upper_range2 = np.array([70,255,255])
        mask2 = cv2.inRange(hsv,lower_range2,upper_range2)
        _,self.m2 = cv2.connectedComponents(mask2)
        self.var2 = np.amax(self.m2)
        _, self.contours2,_ = cv2.findContours(mask2, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        if len(self.contours2) > 1:
            self.green()
            self.gflag=1
        self.green_pub.publish(self.var2)

        
        lower_range3 = np.array([0,100,100])
        upper_range3 = np.array([20,255,255])
        mask3 = cv2.inRange(hsv,lower_range3,upper_range3)
        _,self.m3 = cv2.connectedComponents(mask3)
        self.var3 = np.amax(self.m3)
        _, self.contours3,_ = cv2.findContours(mask3, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        if len(self.contours3) > 1:
            self.red()
            if self.rflag ==1:
                self.r2flag=1
            self.rflag=1
            
            #print "Pollination Done! Pollinated 1 Red Daylily"
        self.red_pub.publish(self.var3)
        if freq==4:
            print "Pollination Done! Pollinated 2 Red Daylily\n"
            print "Pollination Done! Pollinated 1 Blue Daylily\n"
            print "Pollination Done! Pollinated 1 Green Daylily\n"
        
        self.image_pub.publish(self.ros_bridge.cv2_to_imgmsg(self.img,"bgr8"))

        cv2.waitKey(3)
   
				
    def arm(self):
	'''
	*Function Name: arm
	*Input: self
	*output: self
	*Logic: to arm the drone initially
	*Example call: self.arm()
	'''
        self.cmd.rcAUX4 = 1500
        self.cmd.rcThrottle = 1000
        self.pluto_cmd.publish(self.cmd)
        rospy.sleep(.1)

    def disarm(self):
	'''
	*Function Name: disarm
	*Input: self
	*output: self
	*Logic: to disarm the drone initially
	*Example call: self.disarm()
	'''
        self.cmd.rcAUX4 = 1100
        self.pluto_cmd.publish(self.cmd)
        rospy.sleep(.1)

    
    def move(self,a):
	self.wp_x=point[a][0]
	self.wp_y=points[a][1]
	self.wp_z=points[a][2]

    def check(self):
        self.calc_pid()

        # To check X,Y,Z and YAW parameters.
	    # These return the corresponding PWM values for controlling drone parameters like pitch,roll,throttle,yaw
        pitch_value = int(1500 + self.correct_pitch)
        self.cmd.rcPitch = self.limit (pitch_value, 1600, 1250)
                                                        
        roll_value = int(1500 + self.correct_roll)
        self.cmd.rcRoll = self.limit(roll_value, 1600,1250)
                                                        
        throt_value = int(1500 - self.correct_throt)
        self.cmd.rcThrottle = self.limit(throt_value, 1750,1350)

        yaw_value = int(1500 + self.correct_yaw)
        self.cmd.rcYaw = self.limit(yaw_value, 2000, 1000)

	    self.pluto_cmd.publish(self.cmd)
            self.roll_error.publish(self.error[1])
            self.pitch_error.publish(self.error[2])
            self.alt_error.publish(self.error[3])
            self.yaw_error.publish(self.error[4])
	
    def position_hold(self):
	'''
	*Function Name: position_hold
	*Input: self
	*output: self
	*Logic: make the drone to hold the position 
	*Example call: temp.position_hold()
	'''
	global freq
        rospy.sleep(2)

        print "disarm"
        self.disarm()
        rospy.sleep(.2)
        print "arm"
        self.arm()
        rospy.sleep(.1)

        while True:
	    self.check()			
	    if -0.5<self.error[3]<0.5 :
		    #Towards plant
		    self.move(1)
		    break
	time.sleep(0.05)
	
	while True:
            self.check()		
	    if -0.5<self.error[3]<0.5 and self.rflag==1 :
		    #Towards plant
		    freq=freq+1
		    self.move(2)
		    break
	time.sleep(0.05)

    while True:
    self.check()			
    if -0.5<self.error[3]<0.5 and self.r2flag==1 :
        #Towards plant
        freq=freq+1
        self.move(3)
        break
	time.sleep(0.05)

    while True:
    self.check()			
    if -0.5<self.error[3]<0.5 and self.bflag==1 :
        #Towards plant
        freq=freq+1
        self.move(4)
        break
	time.sleep(0.05)

    while True:
    self.check()			
    if -0.5<self.error[3]<0.5 and self.gflag==1 :
        #Towards plant
        freq=freq+1
        self.move(5)
        break
	time.sleep(0.05)

	while True:
        self.check()
            
	
	
    def calc_pid(self):
	'''
	    *Function Name: calc_pid
	    *Input: self
            *output: self
	    *Logic: to calculate overall pid error between final position to the current drone position
	    *Example call: self.calc_pid()
	'''
        self.seconds = time.time()
        current_time = self.seconds - self.last_time
        if(current_time >= self.loop_time):
            self.pid_throt()
            #self.pid_yaw()
	    self.pid_roll()
	    self.pid_pitch()
            
            self.last_time = self.seconds


    def pid_roll(self):
        '''
	    *Function Name: pid_roll
	    *Input: self
            *output: self
	    *Logic: to calculate pid error in Y-axis of drone position with respect to final position 
	    *Example call: self.pid_roll()
        '''
        #Function to calculate PID controller for roll parameter		
        self.error[1] = self.wp_y - self.drone_y
        #error element stores the difference between final position and present drone y-co-ordinate
        self.error_sum[1] = self.error_sum[1] + self.error[1]
        #error_sum element keeps track of all the error by adding present error with sum of previous errors
        self.d_error[1] = self.error[1] - self.error_last[1]
        #d_error element maintaines the rate of change of error in y co-ordinate by subtracting present error from previous error
        self.correct_roll = self.kp_roll * self.error[1] + self.ki_roll * self.error_sum[1] + self.kd_roll * self.d_error[1]
        #correct_roll is calculated by PID formula to tune roll in turn the y co-ordinate
        self.error_last[1] = self.error[1]
        #error_last holds the present error for next iteration
		
        

    def pid_pitch(self):
        '''
            *Function Name: pid_pitch
            *Input: self
                *output: self
            *Logic: to calculate pid error in X-axis of drone position with respect to final position 
            *Example call: self.pid_pitch()
        '''

            #Function to calculate PID controller for pitch parameter
            self.error[2] = self.wp_x - self.drone_x
        #error element stores the difference between final position and present drone x-co-ordinate
            self.error_sum[2]=self.error_sum[2] + self.error[2]
        #error_sum element keeps track of all the error by adding present error with sum of previous errors
            self.d_error[2]=self.error[2]-self.error_last[2]
        #d_error element maintaines the rate of change of error in x co-ordinate by subtracting present error from previous error
            self.correct_pitch = self.kp_pitch * self.error[2] + self.ki_pitch * self.error_sum[2] + self.kd_pitch * self.d_error[2]
        #correct_pitch is calculated by PID formula to tune pitch in turn the x co-ordinate
            self.error_last[2]=self.error[2]
        #error_last holds the present error for next iteration
        

    def pid_throt(self):
        '''
            *Function Name: pid_throt
            *Input: self
                *output: self
            *Logic: to calculate pid error in Z-axis of drone position with respect to final position 
            *Example call: self.pid_throt()
        '''

        #Function to calculate PID controller for throt parameter
            self.error[3] = self.wp_z - self.drone_z
        #error element stores the difference between final position and present drone z-co-ordinate
            self.error_sum[3]=self.error_sum[3] + self.error[3]
        #error_sum element keeps track of all the error by adding present error with sum of previous errors
            self.d_error[3]=self.error[3]-self.error_last[3]
        #d_error element maintaines the rate of change of error in y co-ordinate by subtracting present error from previous error
            self.correct_throt = self.kp_throt * self.error[3] + self.ki_throt * self.error_sum[3] + self.kd_throt * self.d_error[3]
        #correct_throt is calculated by PID formula to tune throttle in turn the z co-ordinate
            self.error_last[3]=self.error[3]
        #error_last holds the present error for next iteration
        

    def pid_yaw(self):
        '''
            *Function Name: pid_yaw
            *Input: self
                *output: self
            *Logic: to calculate pid error in orientation of drone position with respect to final position 
            *Example call: self.pid_yaw()
        '''
        #self.set_drone_yaw(self)
        #Function to calculate PID controller for yaw parameter        
        self.error[4] = self.wp_yaw - self.angle
        #error element stores the difference between final position and present drone final
        self.error_sum[4]=self.error_sum[4] + self.error[4]
        #error_sum element keeps track of all the error by adding present error with sum of previous errors
        self.d_error[4]=self.error[4]-self.error_last[4]
        #d_error element maintaines the rate of change of error in drone angle by subtracting present error from previous error
        self.correct_yaw = self.kp_yaw * self.error[4] + self.ki_yaw * self.error_sum[4] + self.kd_yaw * self.d_error[4]
        #correct_yaw is calculated by PID formula to tune yaw in turn the drone angle
        self.error_last[4]=self.error[4]
        #print self.error[4]
        #error_last holds the present error for next iteration
        
   

    def limit(self, input_value, max_value, min_value):
        '''
            *Function Name: limit
            *Input: self, input_value, max_value, min_value
                *output: self, max_value, min_value
            *Logic: to limit the speed of the motors to desired level 
            *Example call: self.limit(throt_value, 1750,1350)
        '''

        # Function to limit the maximum and minimum values you send to your drone

        if input_value > max_value:
            return max_value
        if input_value < min_value:
            return min_value
        else:
            return input_value

    def get_pose(self,pose):
        '''
            *Function Name: get_pose
            *Input: self
                *output: pose
            *Logic: to get the dynamic position of the drone using whycon
            *Example call: self.get_pose
        '''
        #This is the subscriber function to get the whycon poses
            #The x, y and z values are stored within the drone_x, drone_y and the drone_z variables
        
            self.drone_x = pose.poses[0].position.x
        #drone_x gets drone's current x co-ordinate position from whycon/poses
            self.drone_y = pose.poses[0].position.y
        #drone_x gets drone's current y co-ordinate position from whycon/poses
            self.drone_z = pose.poses[0].position.z
        #drone_x gets drone's current z co-ordinate position from whycon/poses
    
    def set_drone_yaw(self,req):
	'''
	    *Function Name: set_drone_yaw
	    *Input: self
            *output: yaw_value
	    *Logic: to get the dynamic dynamic orientation of the drone
	    *Example call: self.set_drone_yaw
	'''
	print "aaaa"
        #self.angle = req.yaw
	#angle element gets drone's current angle

if __name__ == '__main__':
    while not rospy.is_shutdown():
        temp = DroneFly()
        temp.position_hold()
        rospy.spin()
