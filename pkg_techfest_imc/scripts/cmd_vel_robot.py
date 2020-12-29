#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations

x_dist = 0
y_dist = 0
position
def clbk_odom(msg):
    global position

    # position
    position = msg.linear.x
    # gives x and y distance of the bot



def main():
    global position
    rospy.init_node('cmd_robot', anonymous=True)

    sub_odom = rospy.Subscriber('/cmd_vel', Twist, clbk_odom)

    rate = rospy.Rate(50) # 40hz

    while not rospy.is_shutdown():
        print(position)



if __name__ == '__main__':
    main()
