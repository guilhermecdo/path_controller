#!/usr/bin/env python3

import rospy
import numpy as np
import math
from math import pi
from geometry_msgs.msg import Twist, Point, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from respawnGoal import Respawn
import rrt
import pid

class Env():
    def __init__(self):
        self.goal_x = 0
        self.goal_y = 0
        self.heading = 0
        self.path = []
        self.goal_numbers = 10
        self.collision_numbers = 0
        self.initGoal = True
        self.get_goalbox = False
        self.position = Pose()
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.sub_odom = rospy.Subscriber('odom', Odometry, self.getOdometry)
        self.reset_proxy = rospy.ServiceProxy('gazebo/reset_simulation', Empty)
        self.unpause_proxy = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
        self.pause_proxy = rospy.ServiceProxy('gazebo/pause_physics', Empty)
        self.respawn_goal = Respawn()
        self.yaw=0
        self.state_scan=[]
        self.linear_velocity=0
        self.angular_velocity=0
        self.control=pid.PID()
        self.lidar={
        'frontRight':self.state_scan[315:360],
        'frontleft':self.state_scan[:45],
        'leftFront':self.state_scan[45:90],
        'leftBack':self.state_scan[90:135],
        'backLeft':self.state_scan[135:180],
        'backRight':self.state_scan[180:225],
        'rightBack':self.state_scan[225:270],
        'rightFront':self.state_scan[270:315],
        }

    def getGoalDistace(self):
        goal_distance = round(np.linalg.norm(self.goal_x - self.position.x)-(self.goal_y - self.position.y), 2)
        return goal_distance
    
    def creatPath(self):
        print('Criando caminho')
        self.path=rrt.rrt([self.position.x,self.position.y],[self.goal_x,self.goal_y])
        for _ in range(10):
            if self.path == None:
                self.path=rrt.rrt([self.position.x,self.position.y],[self.goal_x,self.goal_y])
            else: return
        self.path=(self.goal_x,self.goal_y)

    def distance2point(self,point:list):
        return round(np.linalg.norm((point[0] - self.position.x)-(point[1] - self.position.y)), 2)
    
    def collision(self):
        angle=np.argmin(self.state_scan)
        print(angle)

        #while min(self.state_scan)<0.15:
            #self.step([-0.5,0])

        if min(self.state_scan)>0.5:return False
        elif 315<=angle<360:return'frontRight'
        elif 0<=angle<45:return'frontleft'
        elif 45<=angle<90:return'leftFront'
        elif 90<=angle<135:return'leftBack'
        elif 135<=angle<180:return'backLeft'
        elif 180<=angle<225:return'backRight'
        elif 225<=angle<270:return'rightBack'
        elif 270<=angle<315:return'rightFront'
        
    def obstacleAvoidance(self):

        while self.collision()!=False:
            #print(self.collision())

            if self.collision()=='frontRight' or self.collision()=='rightFront':
                #while self.yaw<=np.pi/2:
                    self.step([0,0.2])
                #break
            elif self.collision()=='frontleft' or self.collision()=='leftFront':
                #while self.yaw>=-np.pi/2:
                    self.step([0,-0.2])
                #break
            elif self.collision()=='backLeft' or self.collision()=='leftBack':
                #while self.yaw>=-np.pi/2:
                    self.step([0,0.2])
                #break
            elif self.collision()=='backRight' or self.collision()=='rightBack':
                #while self.yaw>=np.pi/2:
                    self.step([0,-0.2])
                #break
            self.step([0.05,0])
            '''
            elif self.collision()=='leftFront':self.step([0.1,-0.1])
            elif self.collision()=='leftBack':self.step([0.1,0.1])
            elif self.collision()=='backLeft':self.step([0.1,-0.1])
            elif self.collision()=='backRight':self.step([0.1,-0.1])
            elif self.collision()=='rightBack':self.step([0.1,0.1])
            elif self.collision()=='rightFront':self.step([0.1,-0.1])
            '''
        '''
        while min(state_scan)<0.5:
            left=max(state_scan[60:120])
            right=max(state_scan[240:300])
            front=min([min(state_scan[0:60]),min(state_scan[300:])])
            print(min(state_scan[60:120]))
            if front>0.5:
                state_scan=self.step([0.3,0])
            elif left<right:
                state_scan=self.step([0,0.1])
            elif left>right:
                state_scan=self.step([0,-0.1])
        '''
        self.step([0,0])
        #self.step([0.1,0])
        print('End Obstacle Avoidance')
    
    def wayPointHeading(self,waypoint:list):
        goal_angle=math.atan2(waypoint[1] - self.position.y, waypoint[0] - self.position.x)
    
        heading = goal_angle - self.yaw
        if heading > math.pi:
            heading -= 2 * math.pi
        elif heading < -math.pi:
            heading += 2 * math.pi
        return round(heading, 2)

    def goThroughPath(self):
        self.step([0,0])
        
        for waypoint in self.path:
            while self.distance2point(waypoint)>0.03:
                self.linear_velocity, self.angular_velocity=self.control.computeOutput(self.getGoalDistace(),self.wayPointHeading(waypoint))
                
                if min(self.state_scan)<0.4:
                    self.step([0,0])            
                    return False
                
                self.step([self.linear_velocity*0.2, self.angular_velocity*0.2])
     
        self.step([0,0])
        return True

    def getOdometry(self, odom):
        self.position = odom.pose.pose.position
        orientation = odom.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        self.yaw=round(yaw,2)

        goal_angle=math.atan2(self.goal_y - self.position.y, self.goal_x - self.position.x)

        heading = goal_angle - yaw
        if heading > pi:
            heading -= 2 * pi

        elif heading < -pi:
            heading += 2 * pi

        self.heading = round(heading, 2)

    def getState(self, scan):
        scan_range = []
        heading = self.heading
        min_range = 0.13 # bateu
        collision = False
        goal = False

        for i in range(len(scan.ranges)):
            if scan.ranges[i] == float('Inf'):
                scan_range.append(3.5)
            elif np.isnan(scan.ranges[i]):
                scan_range.append(0)
            else:
                scan_range.append(scan.ranges[i])

        if min_range > min(scan_range) > 0.07:
            collision = True

        current_distance=np.linalg.norm((self.goal_x - self.position.x)-(self.goal_y - self.position.y))
        current_distance = round(current_distance,2)
        if current_distance < 0.09:
            goal = True

        return scan_range, current_distance, collision, goal
    
    def shutdown(self):
        rospy.loginfo("Terminado atividade do TurtleBot")
        self.pub_cmd_vel.publish(Twist())
        rospy.sleep(1)
        rospy.signal_shutdown("The End")
    
    def report_goal_distance(self, distance, collision, goal):
        if collision:
            rospy.loginfo("Collision!!")
            self.collision_numbers += 1
        if goal:
            rospy.loginfo("Goal!!")
            self.pub_cmd_vel.publish(Twist())
            self.goal_x, self.goal_y = self.respawn_goal.getPosition(True, delete=True)
            self.goal_distance = self.getGoalDistace()
            distance = self.goal_distance
            rospy.loginfo("Target position: x-> %s, y-> %s!!", self.goal_x, self.goal_y)
            self.goal_numbers -= 1
        
        rospy.loginfo("Number of targets %s / distance to curent goal %s / collission number %s", self.goal_numbers, distance, self.collision_numbers)
        
    def step(self, action):
        liner_vel = action[0]
        ang_vel = action[1]
        
        if self.goal_numbers == 0:
            self.shutdown()

        vel_cmd = Twist()
        vel_cmd.linear.x = liner_vel
        vel_cmd.angular.z = ang_vel
        self.pub_cmd_vel.publish(vel_cmd)

        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('scan', LaserScan, timeout=5)
            except:
                pass

        state, distance, collision, goal = self.getState(data)
        
        self.report_goal_distance(distance, collision, goal)

        self.state_scan=np.asarray(state)
        return np.asarray(state)
    
    def reset(self):
        rospy.wait_for_service('gazebo/reset_simulation')
        try:
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print("gazebo/reset_simulation service call failed")

        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('scan', LaserScan, timeout=5)
            except:
                pass

        if self.initGoal:
            self.goal_x, self.goal_y = self.respawn_goal.getPosition()
            self.initGoal = False
            
        rospy.loginfo("Target position: x-> %s, y-> %s!!", self.goal_x, self.goal_y)

        self.goal_distance = self.getGoalDistace()
        state, distance, collision, goal = self.getState(data)
        
        self.report_goal_distance(distance, collision, goal)

        return np.asarray(state)