#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
import math

# 현재 로봇 위치와 방향
current_pose = None

def odom_callback(msg):
    global current_pose
    current_pose = msg

def publish_goal():
    global current_pose
    
    rospy.init_node('goal_publisher')
    
    # 오도메트리 구독
    rospy.Subscriber('/odom', PoseStamped, odom_callback)
    pub = rospy.Publisher('/goal_pose', PoseStamped, queue_size=1)
    
    rospy.loginfo("Goal publisher started. Waiting for robot pose...")
    rospy.sleep(1)  # 노드가 시작될 때까지 대기
    
    rate = rospy.Rate(10)  # 10 Hz 퍼블리시
    
    while not rospy.is_shutdown():
        if current_pose is None:
            rospy.logwarn("Waiting for robot pose from /odom...")
            rate.sleep()
            continue
        
        # 현재 위치와 방향 추출
        robot_x = current_pose.pose.position.x
        robot_y = current_pose.pose.position.y
        
        # Quaternion을 Euler 각도로 변환 (Yaw만 필요)
        qx = current_pose.pose.orientation.x
        qy = current_pose.pose.orientation.y
        qz = current_pose.pose.orientation.z
        qw = current_pose.pose.orientation.w
        
        # Roll, Pitch, Yaw 계산
        yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        
        # 전방 10m 지점 계산
        distance = 10.0
        goal_x = robot_x + distance * math.cos(yaw)
        goal_y = robot_y + distance * math.sin(yaw)
        
        # Goal 메시지 생성
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = goal_x
        goal.pose.position.y = goal_y
        goal.pose.position.z = 0.0
        goal.pose.orientation.x = qx
        goal.pose.orientation.y = qy
        goal.pose.orientation.z = qz
        goal.pose.orientation.w = qw
        
        pub.publish(goal)
        
        rospy.logdebug("Robot: (%.2f, %.2f), Yaw: %.2f rad -> Goal: (%.2f, %.2f)", 
                      robot_x, robot_y, yaw, goal_x, goal_y)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_goal()
    except rospy.ROSInterruptException:
        rospy.loginfo("Goal publisher shutdown")
