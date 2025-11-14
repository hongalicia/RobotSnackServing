import rclpy
from rclpy.executors import SingleThreadedExecutor
from ROS.robot_state_collector import SingleRobotStateCollector
from ROS.try_block import TMRobotController

class ROSCommunication:
    def __init__(self):
        rclpy.init()

        self.collector = SingleRobotStateCollector(fps=30)
        self.ctrl = TMRobotController(self.collector)
        self.ctrl.setup_services()
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.collector)
        self.executor.add_node(self.ctrl)

    def append_joints(self, joints, block=True):
        self.ctrl.append_joint(joints, block=block, executor=self.executor, )

    def open_gripper(self):
        self.ctrl.append_gripper_open()

    def close_gripper(self):
        self.ctrl.append_gripper_close()

    def half_open_gripper(self):
        self.ctrl.append_gripper_half_open()

    def close_tight_gripper(self):
        self.ctrl.append_gripper_close_tight()

    def quit(self):
        self.executor.shutdown()
        self.ctrl.destroy_node()
        self.collector.destroy_node()
        rclpy.shutdown()

