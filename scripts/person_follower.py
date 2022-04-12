#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

def cap_magnitude(x, max_magnitude):
    '''
    If the magnitude of `x` is greater than `max_magnitude`,
    returns either `max_magnitude` or `-max_magnitude` matching
    the sign of `x`. Otherwise, returns `x`.
    '''
    return max(-1 * max_magnitude, min(max_magnitude, x))

class PersonFollower(object):

    def __init__(self, target_distance=0.25):
        rospy.init_node('person_follower')

        rospy.Subscriber('/scan', LaserScan, self.process_scan)
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.twist = Twist()
        self.target_distance = target_distance

    def process_scan(self, data):
        ranges = data.ranges
        # set 0.0 readings to match the maximum reading
        ranges_clipped = [r if r != 0.0 else max(ranges) for r in ranges]
        # average each reading with surrounding readings to reduce noise
        ranges_blurred = [sum(ranges_clipped[j % 360] for j in range(i - 5, i + 6)) for i in range(360)]
        min_index = np.argmin(ranges_blurred)

        # only move forward if closest object is within 45 degrees of straight ahead
        if min_index < 45 or min_index > 315:
            # proportional control to approach target distance
            self.twist.linear.x = cap_magnitude((ranges[min_index] - self.target_distance) * 0.2, 0.2)

        # proportional control for turn speed: target is having the closest object staight ahead
        if min_index <= 180:
            self.twist.angular.z = cap_magnitude(min_index * 0.02, 0.5)
        else:
            self.twist.angular.z = cap_magnitude((min_index - 360) * 0.02, 0.5)

        self.publisher.publish(self.twist)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = PersonFollower()
    node.run()
