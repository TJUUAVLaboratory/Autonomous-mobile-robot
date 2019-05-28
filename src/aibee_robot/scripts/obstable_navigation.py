#!/usr/bin/env python 
# -*- coding: utf-8 -*-
 
import roslib;
import rospy  
import actionlib  
from actionlib_msgs.msg import *  
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, PointStamped, Quaternion, Twist  
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal  
from random import sample  
from math import pow, sqrt, pi

class NavTest():  
    def __init__(self):  
        rospy.init_node('obstable_navigation', anonymous=True)  
        rospy.on_shutdown(self.shutdown)  
 
        # 在每个目标位置暂停的时间  
        self.rest_time = rospy.get_param("~rest_time", 3)  

        # 到达目标的状态  
        goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED',   
                       'SUCCEEDED', 'ABORTED', 'REJECTED',  
                       'PREEMPTING', 'RECALLING', 'RECALLED',  
                       'LOST']  
 
        # 设置 next_waypoint的位置  
        # 如果想要获得某一点的坐标，在rviz中点击 2D Nav Goal 按键，然后单机地图中一点  
        # 在终端中就会看到坐标信息  

        # locations = dict()  ## locations list
        # locations['p1'] = Pose(Point(1.150, 5.461, 0.000), Quaternion(0.000, 0.000, -0.013, 1.000))  
        # locations['p2'] = Pose(Point(6.388, 2.66, 0.000), Quaternion(0.000, 0.000, 0.063, 0.998))  
        # locations['p3'] = Pose(Point(8.089, -1.657, 0.000), Quaternion(0.000, 0.000, 0.946, -0.324))  
        # locations['p4'] = Pose(Point(9.767, 5.171, 0.000), Quaternion(0.000, 0.000, 0.139, 0.990))  
        # locations['p5'] = Pose(Point(0.502, 1.270, 0.000), Quaternion(0.000, 0.000, 0.919, -0.392)) 
        # locations['p6'] = Pose(Point(4.557, 1.234, 0.000), Quaternion(0.000, 0.000, 0.627, 0.779)) 

        next_waypoint = Point(1.5, 1.5, 0.0)

        # 发布控制机器人的消息  
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)  


        # 发布next_waypoint point
        self.next_waypoint_pub = rospy.Publisher("next_waypoint", Point, queue_size=10)
 
        # 订阅move_base服务器的消息  
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)  

        rospy.loginfo("Waiting for move_base action server...")  

        # 60s等待时间限制  
        self.move_base.wait_for_server(rospy.Duration(60))  
        rospy.loginfo("Connected to move base server")  

        # 保存机器人的在rviz中的初始位置  
        initial_pose = PoseWithCovarianceStamped()  

        # 运行时间、和距离的变量   
        distance_traveled = 0  
        start_time = rospy.Time.now()  
        running_time = 0  
        location = ""  
        last_location = ""  

        # 确保有初始位置  
        while initial_pose.header.stamp == "":  
            rospy.sleep(1)  

        rospy.loginfo("Starting navigation test")  

        # 开始主循环，随机导航  
        while not rospy.is_shutdown():  
            last_waypoint = next_waypoint  
            next_waypoint = PointStamped().point
            
            self.next_waypoint_pub.publish(Point())
            # self.cmd_vel_pub.publish(Twist())  

            # 跟踪行驶距离  
            # 使用更新的初始位置  
            if initial_pose.header.stamp == "":
                distance = sqrt(pow(last_waypoint.x -   
                                    next_waypoint.x, 2) +  
                                pow(last_waypoint.y -   
                                    next_waypoint.y, 2))  
            else:  
                rospy.loginfo("Updating current pose.")  
                distance = sqrt(pow(last_waypoint.x -   
                                    next_waypoint.x, 2) +  
                                pow(last_waypoint.y -   
                                    next_waypoint.y, 2))  
                initial_pose.header.stamp = ""  
 
 
            # 设定下一个目标点  
            self.goal = MoveBaseGoal()  
            self.goal.target_pose.pose = Pose(next_waypoint, Quaternion(0.000, 0.000, 0.00, 1.000))  
            self.goal.target_pose.header.frame_id = 'map'  
            self.goal.target_pose.header.stamp = rospy.Time.now()  

            # 让用户知道下一个位置  
            rospy.loginfo("Going to: " + str(next_waypoint))  
 
            # 向下一个位置进发  
            self.move_base.send_goal(self.goal)  

            # 五分钟时间限制  
            finished_within_time = self.move_base.wait_for_result(rospy.Duration(60))   

            # 查看是否成功到达  
            if not finished_within_time:  
                self.move_base.cancel_goal()  
                rospy.loginfo("Timed out achieving goal")  
            else:  
                state = self.move_base.get_state()  
                if state == GoalStatus.SUCCEEDED:  
                    rospy.loginfo("Goal succeeded!")  
                    distance_traveled += distance  
                    rospy.loginfo("State:" + str(state))  
                else:  
                  rospy.loginfo("Goal failed with error code: " + str(goal_states[state]))  

            # 运行所用时间  
            running_time = rospy.Time.now() - start_time  
            running_time = running_time.secs / 60.0  

            rospy.loginfo("Running time: " + str(trunc(running_time, 1)) +   
                          " min Distance: " + str(trunc(distance_traveled, 1)) + " m")  

            rospy.sleep(self.rest_time)  

    def update_initial_pose(self, initial_pose):  
        self.initial_pose = initial_pose  

    def shutdown(self):  
        rospy.loginfo("Stopping the robot...")  
        self.move_base.cancel_goal()  
        rospy.sleep(2)  
        self.cmd_vel_pub.publish(Twist())  
        rospy.sleep(1)  

def trunc(f, n):   
    slen = len('%.*f' % (n, f))  

    return float(str(f)[:slen])  

if __name__ == '__main__':  
    try:  
        NavTest()  
        rospy.spin()  

    except rospy.ROSInterruptException:  
        rospy.loginfo("Random navigation finished.")
