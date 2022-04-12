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

class WallFollower(object):
    ''' Node that makes the robot follow walls '''
    def __init__(self):
        rospy.init_node('wall_follower')

        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.twist = Twist()
        self.prev_distance = 0.0
        # keep a list of 10 previous derivatives and use the average to
        # reduce noise
        self.derivs = [0.0 for _ in range(10)]
        rospy.Subscriber('/scan', LaserScan, self.process_scan)

    def process_scan(self, data):
        ranges = data.ranges
        # set 0.0 readings to match the maximum reading
        ranges_clipped = [r if r != 0.0 else max(ranges) for r in ranges]
        # average each reading with surrounding readings to reduce noise
        ranges_blurred = [sum(ranges_clipped[j % 360] for j in range(i - 5, i + 6))/11 for i in range(360)]
        min_index = np.argmin(ranges_blurred)
        # get distance derivative and add it to the list
        min_distance = ranges_blurred[min_index]
        self.derivs = self.derivs[1:] + [min_distance - self.prev_distance]
        self.prev_distance = min_distance

        # if there is no object sufficiently close (i.e. middle of a room),
        # move forward.
        if ranges_blurred[min_index] > 3.0:
            self.twist.linear.x = 0.1
        # if the closest object (wall) is to the left, move forward and
        # adjust angle (measured using derivative of distance) by turning
        elif 75 < min_index < 105:
            self.twist.linear.x = 0.1

            # proportional control: turn slighty right to increase distance,
            # slightly left to decrease distance
            self.twist.angular.z = cap_magnitude(np.mean(self.derivs) * 50.0, 0.5)
        # if the closest object is not on the left, turn in place in the direction
        # that will take it to the left faster
        else:
            self.twist.linear.x = 0.0

            self.twist.angular.z = 0.3 if 90 < min_index < 270 else -0.3

        self.publisher.publish(self.twist)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = WallFollower()
    node.run()

