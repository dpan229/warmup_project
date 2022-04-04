#!/usr/bin/env python3
""" This script makes the robot perpetually spin in circles """
import rospy
from geometry_msgs.msg import Twist

class DriveSquare(object):
    """ Publishes cmd_vel commands that make the robot drive in a square """

    def __init__(self):
        rospy.init_node('drive_square')
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def run(self):
        # straight_cmd: drive forward in a straight line
        straight_cmd = Twist()
        straight_cmd.linear.x = 0.5

        # rotate_cmd: rotate in place
        rotate_cmd = Twist()
        rotate_cmd.angular.z = 0.5

        rate = 10
        timer = 0
        r = rospy.Rate(rate)
        while not rospy.is_shutdown():
            # send turn commands for about 3 seconds,
            # then send forward commands for about 3 seconds
            if timer < rate * 3:
                self.publisher.publish(rotate_cmd)
            else:
                self.publisher.publish(straight_cmd)
            timer = (timer + 1) % (rate * 6)
            r.sleep()

if __name__ == '__main__':
    node = DriveSquare()
    node.run()