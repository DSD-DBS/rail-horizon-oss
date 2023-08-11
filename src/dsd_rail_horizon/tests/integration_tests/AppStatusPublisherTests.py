# SPDX-FileCopyrightText: Copyright DB Netz AG
# SPDX-License-Identifier: Apache-2.0

import os, sys

sys.path.append(os.path.dirname(os.path.realpath(__file__)))

import unittest
import pytest
from time import time
from array import array

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_testing.actions import ReadyToTest
import rclpy

from dsd_ros_messages.msg import ApplicationStatusStamped
from helper.TestSubscriber import TestSubscriber

MISSION_CONTROL_NODE_ID = "test_control_id"
MISSION_CONTROL_APPLICATION_ID = "test_application_id"


@pytest.mark.rostest
def generate_test_description():
    rail_horizon_node = Node(
        package='dsd_rail_horizon',
        executable='dsd_rail_horizon',
        output="screen",
        emulate_tty=True,  # Unbuffered input for test
        parameters=[
            {
                "mission_control_supervision": True
            },
            {
                "application_status_topic": "/test_app_status_topic"
            },
            {
                "mission_control_node_id": MISSION_CONTROL_NODE_ID
            },
            {
                "mission_control_app_id": MISSION_CONTROL_APPLICATION_ID
            },
        ])

    launch_description = LaunchDescription([rail_horizon_node, ReadyToTest()])

    context = {
        'rail_horizon_node': rail_horizon_node,
    }

    return (launch_description, context)


class TestRunningAppStatusPublisher(unittest.TestCase):

    def setUp(self):
        rclpy.init()
        self.subscriber = TestSubscriber(ApplicationStatusStamped,
                                         "/test_app_status_topic")

    def tearDown(self):
        self.subscriber.tearDown()
        rclpy.shutdown()

    def test_message_content(self):
        for i in range(3):
            message = self.subscriber.spin(timeoutSec=10)
            self.assertMessage(message)

    def assertMessage(self, message):
        self.assertIsNotNone(message)

        self.assertEqual(0, message.seq)  # Not used currently
        self.assertAlmostEqual(time(), message.stamp.sec, delta=2)

        self.assertEqual([MISSION_CONTROL_NODE_ID],
                         message.application_status.compute_node_id)
        self.assertEqual([MISSION_CONTROL_APPLICATION_ID],
                         message.application_status.application_id)

        self.assertEqual(array('B', [0]), message.application_status.status)
        self.assertEqual(array('B', [0]),
                         message.application_status.sub_status)
        self.assertEqual([], message.application_status.comment)
