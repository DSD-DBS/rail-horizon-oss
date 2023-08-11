# SPDX-FileCopyrightText: Copyright DB Netz AG
# SPDX-License-Identifier: Apache-2.0

import rclpy


class TestSubscriber():

    def __init__(self, messageType, topic):
        self.message = None
        self.node = rclpy.create_node("test_subscription_node")
        self.subscriber = self.node.create_subscription(
            msg_type=messageType,
            topic=topic,
            callback=self.messageRecievedCallback,
            qos_profile=3)

    def spin(self, timeoutSec):
        self.message = None
        rclpy.spin_once(self.node, timeout_sec=timeoutSec)
        return self.message

    def tearDown(self):
        self.node.destroy_subscription(self.subscriber)
        self.node.destroy_node()

    def messageRecievedCallback(self, msg):
        self.message = msg
