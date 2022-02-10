#!/usr/bin/env python

import roslib; roslib.load_manifest('champ_teleop')
import rospy

from sensor_msgs.msg import Joy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from champ_msgs.msg import Pose as PoseLite
from geometry_msgs.msg import Pose as Pose
import tf


class ObstacleAvoidance:
    def __init__(self):
        self.rate = rospy.Rate(60)

        self.velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        self.pose_lite_publisher = rospy.Publisher('body_pose/raw', PoseLite, queue_size = 1)
        self.pose_publisher = rospy.Publisher('body_pose', Pose, queue_size = 1)
        self.joy_subscriber = rospy.Subscriber('joy', Joy, self.joy_callback)

        self.front_laser_sub = rospy.Subscriber('front/scan/laserscan', LaserScan, self.front_depth_callback)
        self.rear_laser_sub = rospy.Subscriber('rear/scan/laserscan', LaserScan, self.rear_depth_callback)
        self.left_laser_sub = rospy.Subscriber('left/scan/laserscan', LaserScan, self.left_depth_callback)
        self.right_laser_sub = rospy.Subscriber('right/scan/laserscan', LaserScan, self.right_depth_callback)

        # Speed parameters
        self.speed = 0.5
        self.turn = 0.5
        
        # Distance thresholds
        self.dist_threshold = 0.5
        self.corner_threshold = 0.4
        self.side_threshold = 0.45
        self.rear_dist_threshold = 0.6

        self.front_depth = None
        self.left_front_depth = None
        self.right_front_depth = None
        self.front_closest_obstacle = None

        self.rear_depth = None
        self.left_rear_depth = None
        self.right_rear_depth = None
        self.rear_closest_obstacle = None

        self.left_depth = None
        self.left_front_depth = None
        self.left_rear_depth = None
        self.left_closest_obstacle = None

        self.right_depth = None
        self.right_front_depth = None
        self.right_rear_depth = None
        self.right_closest_obstacle = None

        self.obst_close = False

    def front_depth_callback (self, msg):
        print(len(msg.ranges))
        '''self.front_depth = min(msg.ranges[200:599])
        self.left_front_depth = min(msg.ranges[600:799])
        self.right_front_depth = min(msg.ranges[0:199])

        self.front_closest_obstacle = min(self.front_depth,self.left_front_depth,self.right_front_depth)'''
        
    def rear_depth_callback (self, msg):
        ''''self.rear_depth = min(msg.ranges[200:599])
        self.left_rear_depth = min(msg.ranges[0:199])
        self.right_rear_depth = min(msg.ranges[600:799])

        self.rear_closest_obstacle = min(self.rear_depth,self.left_rear_depth,self.right_rear_depth)'''


    def left_depth_callback (self, msg):
        '''self.left_depth = min(msg.ranges[200:599])
        self.left_front_depth = min(msg.ranges[0:199])
        self.left_rear_depth = min(msg.ranges[600:799])

        self.left_closest_obstacle = min(self.left_depth,self.left_front_depth,self.left_rear_depth)'''

    def right_depth_callback (self, msg):
        '''self.right_depth = min(msg.ranges[200:599])
        self.right_front_depth = min(msg.ranges[600:799])
        self.right_rear_depth = min(msg.ranges[0:199])

        self.right_closest_obstacle = min(self.right_depth,self.right_front_depth,self.right_rear_depth)'''

    '''def front_depth_callback (self, msg):
        self.front_depth = min(msg.ranges[213:426])
        self.left_front_depth = min(msg.ranges[427:639])
        self.right_front_depth = min(msg.ranges[0:212])

        self.front_closest_obstacle = min(self.front_depth,self.left_front_depth,self.right_front_depth)
        
    def rear_depth_callback (self, msg):
        self.rear_depth = min(msg.ranges[213:426])
        self.left_rear_depth = min(msg.ranges[0:212])
        self.right_rear_depth = min(msg.ranges[427:639])

        self.rear_closest_obstacle = min(self.rear_depth,self.left_rear_depth,self.right_rear_depth)


    def left_depth_callback (self, msg):
        self.left_depth = min(msg.ranges[213:426])
        self.left_front_depth = min(msg.ranges[0:212])
        self.left_rear_depth = min(msg.ranges[427:639])

        self.left_closest_obstacle = min(self.left_depth,self.left_front_depth,self.left_rear_depth)

    def right_depth_callback (self, msg):
        self.right_depth = min(msg.ranges[213:426])
        self.right_front_depth = min(msg.ranges[427:639])
        self.right_rear_depth = min(msg.ranges[0:212])

        self.right_closest_obstacle = min(self.right_depth,self.right_front_depth,self.right_rear_depth) '''

    # Joystick Controller is called after obstacle avoidance check
    def joy_callback(self, data):

        twist = Twist()

        if self.front_closest_obstacle <= self.dist_threshold or self.rear_closest_obstacle <= self.dist_threshold \
            or self.left_closest_obstacle <= self.dist_threshold or self.right_closest_obstacle <= self.dist_threshold:

            self.obst_close = True

        else:
            self.obst_close = False

        if self.obst_close == True:
            rospy.loginfo("Obstacle: Speed reduced to 0.2 m/s")
            # If there is an obstacle anywhere within threshold, then lower speed 0.2 m/s
            self.obstacle_avoidance()
        
        elif 0.7 <= self.obst_close <= 1.0:
            rospy.loginfo("Obstacle in close range: Speed reduced to 0.3 m/s")
            # If obstacle within close range slightly lower speed 0.3 m/s
            self.speed = 0.3
            self.turn = 0.3

        else:
            rospy.loginfo("Area Clear: Speed set to 0.5 m/s")
            # Else no obstacle return to normal speed 0.5 m/s
            self.speed = 0.5
            self.turn = 0.5

        twist.linear.x = data.axes[1] * self.speed
        twist.linear.y = data.buttons[4] * data.axes[0] * self.speed
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = (not data.buttons[4]) * data.axes[0] * self.turn
        self.velocity_publisher.publish(twist)

        body_pose_lite = PoseLite()
        body_pose_lite.x = 0
        body_pose_lite.y = 0
        body_pose_lite.roll = (not data.buttons[5]) *-data.axes[3] * 0.349066
        body_pose_lite.pitch = data.axes[4] * 0.174533
        body_pose_lite.yaw = data.buttons[5] * data.axes[3] * 0.436332

        if data.axes[5] < 0:
            body_pose_lite.z = data.axes[5] * 0.5

        self.pose_lite_publisher.publish(body_pose_lite)

        body_pose = Pose()
        body_pose.position.z = body_pose_lite.z

        quaternion = tf.transformations.quaternion_from_euler(body_pose_lite.roll, body_pose_lite.pitch, body_pose_lite.yaw)
        body_pose.orientation.x = quaternion[0]
        body_pose.orientation.y = quaternion[1]
        body_pose.orientation.z = quaternion[2]
        body_pose.orientation.w = quaternion[3]

        self.pose_publisher.publish(body_pose)
       
    # Obstacle Avoidance functions   
    def obstacle_avoidance(self):
        speed = 0.2
        turn = 0.2
        twist = Twist()
        # Front obstacle
        while self.front_depth <= self.dist_threshold:
            rospy.loginfo("Front obstacle: Move back")
            twist.linear.x = -speed
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            self.velocity_publisher.publish(twist)

        # Front left obstacle
        while self.left_front_depth <= self.corner_threshold:
            rospy.loginfo("Front left obstacle: Rotate right")
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = -turn
            self.velocity_publisher.publish(twist)

        # Front right obstacle
        while self.right_front_depth <= self.corner_threshold:
            rospy.loginfo("Front right obstacle: Rotate left")
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = turn
            self.velocity_publisher.publish(twist)

        # Rear obstacle
        while self.rear_closest_obstacle <= self.rear_dist_threshold:
            rospy.loginfo("Rear obstacle:  Move forward")
            twist.linear.x = speed
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            self.velocity_publisher.publish(twist)

        # Left side obstacle
        while self.left_closest_obstacle <= self.side_threshold or self.left_front_depth <= self.corner_threshold and self.left_rear_depth <= self.corner_threshold:
            rospy.loginfo("Left side obstacle: Move right")
            twist.linear.x = 0.0
            twist.linear.y = -speed
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            self.velocity_publisher.publish(twist)

        # Right side obstacle
        while self.right_closest_obstacle <= self.side_threshold or self.right_front_depth <= self.corner_threshold and self.right_rear_depth <= self.corner_threshold: 
            rospy.loginfo("Right side obstacle: Move left")
            twist.linear.x = 0.0
            twist.linear.y = speed
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            self.velocity_publisher.publish(twist)

        while self.front_depth <= self.dist_threshold and self.rear_depth <= self.dist_threshold:
            rospy.loginfo("Front and Rear obstacle: Rotate")
            twist.linear.x = 0.0
            twist.linear.y = speed
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            self.velocity_publisher.publish(twist)

if __name__ == "__main__":
    
    rospy.init_node('spot_obst_avoidance')
    obstacle_avoidance = ObstacleAvoidance()
    rospy.spin()