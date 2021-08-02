#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import random
import math

def main():
    rate = rospy.Rate(10) # 10hz
    # lpos = 0
    # rpos = 0
    # cent_x = rospy.Subscriber("/car_centroid_x", Float64)
    # cent_y = rospy.Subscriber("/car_centroid_y", Float64)
    cmd_vel =  rospy.Publisher("/cmd_vel", Twist, queue_size = 10)


    # left_pub = rospy.Publisher("/bot/left_wheel_controller/command",
    #                                         Float64, queue_size=10)
    # right_pub = rospy.Publisher("/bot/right_wheel_controller/command",
    #                                         Float64, queue_size=10)
    while not rospy.is_shutdown():

        # lpos += 2.0
        # rpos += 2.0
        # # if random.randint(0, 4) != 0:
        # left_pub.publish(lpos)
        # # if random.randint(0, 4) != 0:
        # right_pub.publish(rpos)

        vel = Twist()
        vel.angular.x = 1;
        # vel_linaer.y = 0
        # vel_lin

        cmd_vel.publish(vel)

        rate.sleep()

if __name__ == '__main__':
    rospy.init_node("move_node", anonymous=True)
    try:
        main()
    except rospy.ROSInterruptException:
        pass
