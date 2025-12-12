#!/usr/bin/env python3
# coding: utf-8

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from qulido_robot_msgs.action import ExecuteMotion
from qulido_robot_msgs.msg import MotionPrimitive, MotionSequence


class MotionRequestServer(Node):

    def __init__(self):
        super().__init__("motion_request_server")

        # Connect to Action Server on /execute_motion
        self.client = ActionClient(self, ExecuteMotion, "/execute_motion")

        self.get_logger().info("MotionRequestServer ready. Waiting for motion requests...")

        # 테스트용 motion 자동 전송
        self.timer = self.create_timer(1.0, self.test_send_once)
        self.sent = False

    # ------------------------------------------------------------------
    # 테스트로 딱 1번만 전송
    # ------------------------------------------------------------------
    def test_send_once(self):
        if self.sent:
            return
        self.sent = True

        # Example Motion Dict
        test_motion = {
            'sequence': [
                {'primitive': 'move_pose', 'pose': [367.39,7.39,194.84,27.92,179.77,27.78]},
                # {'primitive': 'operate_gripper', 'width': 0},
                # {'primitive': 'move_pose', 'pose': [367.39,7.39,36.52,27.92,179.77,27.78]},
                # {'primitive': 'move_pose', 'pose': [367.39,7.39,36.52,27.92,179.77,117.78]},
                # {'primitive': 'force_control'},
                # {'primitive': 'operate_gripper', 'width': 200},
            ]
        }

        self.send_motion(test_motion)

    # ------------------------------------------------------------------
    # dict → ExecuteMotion goal 변환 후 전송
    # ------------------------------------------------------------------
    def send_motion(self, motion_dict):
        goal_msg = ExecuteMotion.Goal()
        seq_msg = MotionSequence()

        for step in motion_dict['sequence']:
            mp = MotionPrimitive()
            mp.primitive = step['primitive']

            if step['primitive'] == "move_pose":
                mp.target_pose = step['pose']
                mp.gripper_width = 0

            elif step['primitive'] == "operate_gripper":
                mp.target_pose = [0.0]*6
                mp.gripper_width = step['width']

            elif step['primitive'] == "force_control":
                mp.target_pose = [0.0]*6
                mp.gripper_width = 0

            else:
                self.get_logger().error(f"Unknown primitive: {step['primitive']}")
                continue

            seq_msg.sequence.append(mp)

        goal_msg.sequence = seq_msg

        self.get_logger().info("Waiting for /execute_motion action server...")
        self.client.wait_for_server()

        self.get_logger().info("Sending motion sequence to /execute_motion...")
        send_future = self.client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_cb
        )
        send_future.add_done_callback(self.goal_response_cb)

    # ------------------------------------------------------------------
    # Goal handle callback
    # ------------------------------------------------------------------
    def goal_response_cb(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by server.")
            return

        self.get_logger().info("Goal accepted. Executing motion...")

        # Register for result callback
        goal_handle.get_result_async().add_done_callback(self.result_cb)

    # ------------------------------------------------------------------
    # Feedback callback
    # ------------------------------------------------------------------
    def feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(f"[Feedback] Current step: {fb.current_step}")

    # ------------------------------------------------------------------
    # Result callback
    # ------------------------------------------------------------------
    def result_cb(self, future):
        result = future.result().result

        if result.success:
            self.get_logger().info("[Result] SUCCESS: Motion executed.")
        else:
            self.get_logger().info("[Result] FAILED: Execution error.")


def main(args=None):
    rclpy.init(args=args)
    node = MotionRequestServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
