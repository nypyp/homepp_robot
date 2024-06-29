#!/usr/bin/env python3

import rclpy
import time
import numpy as np
from geometry_msgs.msg import Twist, Vector3
from turn_on_wheeltec_robot.msg import Position as PositionMsg
from std_msgs.msg import String as StringMsg
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rcl_interfaces.msg import ParameterEvent


class YOLOFollower(Node):
    def __init__(self):
        super().__init__('yolofollower')
        qos = QoSProfile(depth=10)

        self.max_speed = 0.9
        self.max_ang_speed = 1.5
        self.active=False
        self.i=0
        self.cmdVelPublisher = self.create_publisher( Twist,'/cmd_vel', qos)

        self.positionSubscriber = self.create_subscription(PositionMsg, '/yolo_tracker/tracked_person_pos', self.positionUpdateCallback, qos)
        # PID parameters first is angular, dist
        #PID_param = rospy.get_param('~PID_controller')
        # the first parameter is the angular target (0 degrees always) the second is the target distance (say 1 meter)
        # self.PID_controller = simplePID([0, targetDist], [1.2 ,0.2 ], [0 ,0.00], [0.005 ,0.00])
        self.declare_parameter('targetDist', 2500)
        self.declare_parameter('PID_angular_p', 0.0)
        self.declare_parameter('PID_angular_i', 0.000)
        self.declare_parameter('PID_angular_d', 0.000)
        self.declare_parameter('PID_distance_p', 0.0008)
        self.declare_parameter('PID_distance_i', 0.00)
        self.declare_parameter('PID_distance_d', 0.0000)
        
        targetDist = self.get_parameter("targetDist").value
        PID_angular_p = self.get_parameter("PID_angular_p").value
        PID_angular_i = self.get_parameter("PID_angular_i").value
        PID_angular_d = self.get_parameter("PID_angular_d").value
        PID_distance_p = self.get_parameter("PID_distance_p").value
        PID_distance_i = self.get_parameter("PID_distance_i").value
        PID_distance_d = self.get_parameter("PID_distance_d").value
        self.PID_controller = simplePID([0, targetDist], [PID_angular_p ,PID_distance_p ], [PID_angular_i ,PID_distance_i], [PID_angular_d ,PID_distance_d])
        self.subscription = self.create_subscription(
            ParameterEvent,
            '/parameter_events',
            self.pidUpdateCallback,
            10)
        
    def pidUpdateCallback(self, msg):
        pid_param_list = ["PID_angular_p","PID_angular_i","PID_angular_d",
                          "PID_distance_p","PID_distance_i","PID_distance_d",
                          "targetDist"]
        for changed_parameter in msg.changed_parameters:
            if changed_parameter.name in pid_param_list:
                targetDist = self.get_parameter("targetDist").value
                PID_angular_p = self.get_parameter("PID_angular_p").value
                PID_angular_i = self.get_parameter("PID_angular_i").value
                PID_angular_d = self.get_parameter("PID_angular_d").value
                PID_distance_p = self.get_parameter("PID_distance_p").value
                PID_distance_i = self.get_parameter("PID_distance_i").value
                PID_distance_d = self.get_parameter("PID_distance_d").value
                self.PID_controller = simplePID([0, targetDist], [PID_angular_p ,PID_distance_p ], [PID_angular_i ,PID_distance_i], [PID_angular_d ,PID_distance_d])
                self.get_logger().warn(f"PID update:   ([0, {targetDist}],[{PID_angular_p}, {PID_distance_p}],[{PID_angular_i}, {PID_distance_i}],[{PID_angular_d}, {PID_distance_d}])")

    def trackerInfoCallback(self, info):
        # we do not handle any info from the object tracker specifically at the moment. just ignore that we lost the object for example
        self.get_logger().warn(info.data)

    def positionUpdateCallback(self, position: PositionMsg):

        # gets called whenever we receive a new position. It will then update the motorcomand

        #if(not(self.active)):
            #return #if we are not active we will return imediatly without doing anything

        angleX= -position.angle_x
        distance = position.distance


        # call the PID controller to update it and get new speeds
        [uncliped_ang_speed, uncliped_lin_speed] = self.PID_controller.update([angleX, distance])
        # clip these speeds to be less then the maximal speed specified above
        angularSpeed = np.clip(-uncliped_ang_speed, -self.max_ang_speed, self.max_ang_speed)
        linearSpeed  = np.clip(-uncliped_lin_speed, -self.max_speed, self.max_speed)
        
        # create the Twist message to send to the cmd_vel topic
        velocity = Twist()    

        velocity.linear.x = float(linearSpeed)
        velocity.linear.y = 0.0
        velocity.linear.z = 0.0

        velocity.angular.x = 0.0
        velocity.angular.y = 0.0
        velocity.angular.z = angularSpeed
        if((distance>4000)or((distance==0))):
            self.stopMoving()
            self.get_logger().info(f"out of tracking, distance: {distance}")
        else:
            self.get_logger().info(f"Update pos: angle - x: {angleX}, distance: {distance}")
            self.cmdVelPublisher.publish(velocity)
            self.get_logger().info(f"Publish linear - x: {velocity.linear.x}, angular - z: {velocity.angular.z}")
        #self.get_logger().info('linearSpeed: {}, angularSpeed: {}'.format(linearSpeed, angularSpeed))

    def stopMoving(self):
        velocity = Twist()

        velocity.linear.x = 0.0
        velocity.linear.y = 0.0
        velocity.linear.z = 0.0

        velocity.angular.x = 0.0
        velocity.angular.y = 0.0
        velocity.angular.z = 0.0
        
        self.cmdVelPublisher.publish(velocity)
        
class simplePID:
	'''very simple discrete PID controller'''
	def __init__(self, target, P, I, D):
		'''Create a discrete PID controller
		each of the parameters may be a vector if they have the same length
		
		Args:
		target (double) -- the target value(s)
		P, I, D (double)-- the PID parameter

		'''

		# check if parameter shapes are compatabile. 
		if(not(np.size(P)==np.size(I)==np.size(D)) or ((np.size(target)==1) and np.size(P)!=1) or (np.size(target )!=1 and (np.size(P) != np.size(target) and (np.size(P) != 1)))):
			raise TypeError('input parameters shape is not compatable')

		self.Kp		=np.array(P)
		self.Ki		=np.array(I)
		self.Kd		=np.array(D)
		self.setPoint   =np.array(target)
		
		self.last_error=0
		self.integrator = 0
		self.integrator_max = float('inf')
		self.timeOfLastCall = None 
		
		
	def update(self, current_value):
		'''Updates the PID controller. 

		Args:
			current_value (double): vector/number of same legth as the target given in the constructor

		Returns:
			controll signal (double): vector of same length as the target

		'''
		current_value=np.array(current_value)
		if(np.size(current_value) != np.size(self.setPoint)):
			raise TypeError('current_value and target do not have the same shape')
		if(self.timeOfLastCall is None):
			# the PID was called for the first time. we don't know the deltaT yet
			# no controll signal is applied
			self.timeOfLastCall = time.perf_counter()
			return np.zeros(np.size(current_value))
		
		error = self.setPoint - current_value

                #when bias is little, stop moving. errpr[0]=angle(rad),         error[1]=distance(mm)
		#                                  self.setPoint[0]=angle(rad), self.setPoint[1]=distance(mm)
		if error[0]<0.1 and error[0]>-0.1:
			error[0]=0
		if error[1]<150 and error[1]>-150:
			error[1]=0
		
		#when target is little, amplify velocity by amplify error.
		if (error[1]>0 and self.setPoint[1]<1200):
			error[1]=error[1]*(1200/self.setPoint[1])*0.5
			error[0]=error[0]*0.8
		P =  error
		
		currentTime = time.perf_counter()
		deltaT      = (currentTime-self.timeOfLastCall)

		# integral of the error is current error * time since last update
		self.integrator = self.integrator + (error*deltaT)
		I = self.integrator
		
		# derivative is difference in error / time since last update
		D = (error-self.last_error)/deltaT
		
		self.last_error = error
		self.timeOfLastCall = currentTime
		
		# return controll signal
		return self.Kp*P + self.Ki*I + self.Kd*D


def main(args=None):
    print('yoloFollower')
    rclpy.init(args=args)
    yoloFollower = YOLOFollower()
    print('yoloFollower init done')
    try:
        rclpy.spin(yoloFollower)
    #except KeyboardInterrupt:
    #	self.stopMoving()
    finally:
        yoloFollower.destroy_node()
        controllerLoss=yoloFollower.controllerLoss()
        rclpy.shutdown()
        
        
