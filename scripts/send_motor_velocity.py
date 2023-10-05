#!/usr/bin/env python3

#import keyboard
import rospy
from geometry_msgs.msg import Twist


if __name__ == '__main__':
    twist_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('lookup_table_node', anonymous=True)
    rate= rospy.Rate(10)


    while not rospy.is_shutdown():

        vel_input = float(input())

        if(vel_input > 0.8):
            vel_input = 0.8
        elif(vel_input < 0.0):
            vel_input = 0.0

        cmd_msg = Twist()
        cmd_msg.linear.x = vel_input

        start_time = rospy.Time.now().secs 
        while(rospy.Time.now().secs - start_time < 10):
            twist_publisher.publish(cmd_msg)
            rospy.sleep(0.5)