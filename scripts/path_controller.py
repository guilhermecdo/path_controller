#! /usr/bin/env python3
import rospy 
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *
from gazebo_msgs.msg import *
import numpy as np
from environment import Env

if __name__ == "__main__": 
    rospy.init_node("path_controller_node", anonymous=False)
    
    env = Env()
    state_scan = env.reset()
    action = np.zeros(2)
    env.step([0,0])
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)    
    r = rospy.Rate(5) # 10hz
    velocity = Twist()
    
    env.creatPath()
    print(env.path)
    while not rospy.is_shutdown():
        '''
        state_scan=env.step([0,0])
        left=min(state_scan[60:120])
        right=min(state_scan[240:300])
        front=min([min(state_scan[0:60]),min(state_scan[300:])])
        print('front,left,right')       
        print(front,left,right)
        
        while True:
            env.step([0,-0.3])
            print(env.yaw)
        '''
        if env.goal_numbers>0:
            if env.goThroughPath():
                env.creatPath()
            else:
                env.obstacleAvoidance()
                env.creatPath()
        else: 
            rospy.signal_shutdown()
        
        r.sleep()