#!/usr/bin/env python3

import threading
import time
import unittest

import rospy
import rostest
from nav_msgs.msg import Path


class TrajectoryGenerationTest(unittest.TestCase):
    def setUp(self):
        self._received_sizes = []
        self._non_empty_path = None
        self._path_event = threading.Event()
        topic = rospy.get_param("~trajectory_topic", "/esv_planner/trajectory")
        self._subscriber = rospy.Subscriber(topic, Path, self._path_callback, queue_size=1)
        # Give the ROS connection graph a brief moment to establish the subscription.
        time.sleep(0.5)

    def _path_callback(self, msg):
        size = len(msg.poses)
        self._received_sizes.append(size)
        if size > 0 and self._non_empty_path is None:
            self._non_empty_path = msg
            self._path_event.set()

    def test_publishes_non_empty_trajectory(self):
        timeout = rospy.get_param("~timeout", 20.0)
        success = self._path_event.wait(timeout)
        self.assertTrue(
            success,
            "expected a non-empty trajectory on /esv_planner/trajectory within "
            f"{timeout:.1f}s, received pose counts: {self._received_sizes}",
        )
        self.assertIsNotNone(self._non_empty_path)
        self.assertGreater(len(self._non_empty_path.poses), 0)


if __name__ == "__main__":
    rospy.init_node("trajectory_generation_test")
    rostest.rosrun("esv_planner", "trajectory_generation_test", TrajectoryGenerationTest)
