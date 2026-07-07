"""The demo story, asserted end to end against the real launch file.
1. Command ACQUIRE
2. Assert that the terminal eventually reaches LOCK by itself
3. Assert that the lock is a stable state, not a transient
4. Script a blockage, assert that the terminal transitions to COAST
5. Clear the blockage, assert that the terminal re-locks by itself
6. Assert that the re-lock also holds
"""
import time
import unittest

import launch
import launch_testing.actions
import pytest
import rclpy
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from pat_interfaces.msg import ModeState
from pat_interfaces.srv import SetMode
from std_msgs.msg import Bool


@pytest.mark.launch_test
def generate_test_description():
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('plant_sim'), 'launch', 'simulation_launch.py',
            ])
        )
    )
    return launch.LaunchDescription([
        simulation,
        launch_testing.actions.ReadyToTest(),
    ])


class PATIntegrationTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('integration_checker')
        cls.mode = None
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        cls.node.create_subscription(
            ModeState, 'mode', cls._on_mode, latched)
        cls.blockage_pub = cls.node.create_publisher(Bool, 'blockage', 10)
        cls.set_mode_client = cls.node.create_client(SetMode, 'set_mode')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_mode(cls, msg):
        cls.mode = msg.mode

    def spin_until_mode(self, target, timeout_s):
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if self.mode == target:
                return True
        return False

    def spin_for(self, duration_s):
        deadline = time.time() + duration_s
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def set_blockage(self, blocked):
        self.blockage_pub.publish(Bool(data=blocked))

    def test_acquire_lock_blockage_coast_relock(self):
        self.assertTrue(
            self.set_mode_client.wait_for_service(timeout_sec=10.0),
            'set_mode service never appeared')

        # 1. Command ACQUIRE
        future = self.set_mode_client.call_async(SetMode.Request(mode=ModeState.ACQUIRE))
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        self.assertTrue(future.done(), 'set_mode call did not return')
        self.assertTrue(future.result().accepted, 'ACQUIRE was rejected from IDLE')

        # 2. Assert that the terminal eventually reaches LOCK by itself
        self.assertTrue(
            self.spin_until_mode(ModeState.LOCK, 15.0),
            'never reached LOCK after ACQUIRE')

        # 3. Assert that the lock is a stable state, not a transient
        self.spin_for(2.0)
        self.assertEqual(self.mode, ModeState.LOCK, 'lock did not hold for 2 s')

        # 4. Script a blockage, assert that the terminal transitions to COAST
        self.set_blockage(True)
        self.assertTrue(
            self.spin_until_mode(ModeState.COAST, 2.0),
            'blockage did not force COAST')

        # 5. Clear the blockage, assert that the terminal re-locks by itself
        self.set_blockage(False)
        self.assertTrue(
            self.spin_until_mode(ModeState.LOCK, 15.0),
            'never re-locked after the blockage cleared')

        # 6. Assert that the re-lock also holds
        self.spin_for(2.0)
        self.assertEqual(self.mode, ModeState.LOCK, 're-lock did not hold for 2 s')
