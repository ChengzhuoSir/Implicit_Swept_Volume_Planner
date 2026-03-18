#!/usr/bin/env python3

import threading
import time
import unittest

import rospy
import rostest
from nav_msgs.msg import Path
from std_msgs.msg import Float64MultiArray


class PaperPipelineSmokeTest(unittest.TestCase):
    def setUp(self):
        self._path_event = threading.Event()
        self._stats_event = threading.Event()
        self._non_empty_path = None
        self._stats_msg = None
        self._received_path_sizes = []
        self._received_stats_lengths = []

        traj_topic = rospy.get_param("~trajectory_topic", "/esv_planner/trajectory")
        stats_topic = rospy.get_param("~stats_topic", "/esv_planner/planning_stats")
        self._traj_sub = rospy.Subscriber(traj_topic, Path, self._path_callback, queue_size=1)
        self._stats_sub = rospy.Subscriber(
            stats_topic, Float64MultiArray, self._stats_callback, queue_size=1
        )
        time.sleep(0.5)

    def _path_callback(self, msg):
        size = len(msg.poses)
        self._received_path_sizes.append(size)
        if size > 0 and self._non_empty_path is None:
            self._non_empty_path = msg
            self._path_event.set()

    def _stats_callback(self, msg):
        self._received_stats_lengths.append(len(msg.data))
        if len(msg.data) >= 7:
            self._stats_msg = msg
            self._stats_event.set()

    def test_paper_pipeline_emits_trajectory_and_stats(self):
        timeout = rospy.get_param("~timeout", 20.0)
        self.assertTrue(
            self._path_event.wait(timeout),
            "expected a non-empty trajectory, received pose counts: {}".format(
                self._received_path_sizes
            ),
        )
        self.assertTrue(
            self._stats_event.wait(timeout),
            "expected planning stats with at least 7 fields, received lengths: {}".format(
                self._received_stats_lengths
            ),
        )
        self.assertIsNotNone(self._non_empty_path)
        self.assertIsNotNone(self._stats_msg)
        self.assertGreater(len(self._non_empty_path.poses), 0)
        self.assertGreaterEqual(len(self._stats_msg.data), 7)
        self.assertGreater(self._stats_msg.data[0], 0.0)
        self.assertLess(
            self._stats_msg.data[0],
            5.0,
            "expected the paper pipeline to finish within 5 seconds",
        )
        self.assertGreater(
            self._stats_msg.data[3],
            0.0,
            "expected positive clearance from the paper-aligned backend",
        )
        self.assertGreater(
            len(self._non_empty_path.poses),
            10,
            "expected optimized trajectory with >10 poses, not a raw coarse path",
        )


if __name__ == "__main__":
    rospy.init_node("paper_pipeline_smoke")
    rostest.rosrun("esv_planner", "paper_pipeline_smoke", PaperPipelineSmokeTest)
