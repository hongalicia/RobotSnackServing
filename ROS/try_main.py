#!/usr/bin/env python3
import rclpy
from rclpy.executors import SingleThreadedExecutor
from robot_state_collector import SingleRobotStateCollector
from try_block import TMRobotController

def main():
    rclpy.init()

    collector = SingleRobotStateCollector(fps=30)
    ctrl = TMRobotController(collector)
    ctrl.setup_services()
    executor = SingleThreadedExecutor()
    executor.add_node(collector)
    executor.add_node(ctrl)

    j1 = [2.01, -10.63, 120.00, 70.83, -87.79, 179.26]
    j2 = [2.01, -10.63, 140.18, 70.83, -87.79, 179.26]
    ctrl.append_joint(j1)
    ctrl.append_gripper_close()
    ctrl.append_joint(j2,block=True, executor=executor,  )

    print("這行會等到上一個 joint 完成才印")


    #----------------結束整個程式才需要--------------
    executor.shutdown()
    ctrl.destroy_node()
    collector.destroy_node()
    rclpy.shutdown()
    #----------------結束整個程式才需要--------------
if __name__ == "__main__":
    main()
