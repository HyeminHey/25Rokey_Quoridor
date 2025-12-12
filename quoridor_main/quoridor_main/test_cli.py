import rclpy
from rclpy.node import Node

from qulido_robot_msgs.srv import AiCompute


class AiTestClient(Node):
    def __init__(self):
        super().__init__('ai_test_client')
        self.cli = self.create_client(AiCompute, '/ai_agent/get_robot_move')

        # 서비스 서버가 준비될 때까지 기다림
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /ai_agent/get_robot_move not available, waiting...')

        self.req = AiCompute.Request()

    def send_request(self, values):
        """ values: [int, int, int] """
        self.req.added = values
        self.get_logger().info(f"[CLIENT] Sending request: {values}")
        future = self.cli.call_async(self.req)
        return future


def main():
    rclpy.init()

    node = AiTestClient()

    try:
        while rclpy.ok():
            user_input = input("\nEnter 3 integers (e.g., 1 5 4), or 'q' to quit: ")

            if user_input.lower() == 'q':
                print("Bye!")
                break
            
            try:
                vals = list(map(int, user_input.split()))
                if len(vals) != 3:
                    print("❗ 반드시 3개 정수를 입력하세요.")
                    continue
            except:
                print("❗ 숫자만 입력하세요.")
                continue

            future = node.send_request(vals)

            # 응답 올 때까지 스핀
            rclpy.spin_until_future_complete(node, future)
            if future.result() is not None:
                res = future.result().ai_cmd
                print(f"[CLIENT] Received response ai_cmd: {res}") ##항상 리스트로 묶어서 사용하기! 오케스트레이터에서도 받으면 리스트로 처리하기!
            else:
                print("❌ 서비스 호출 실패!")

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
