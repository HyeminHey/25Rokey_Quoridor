import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from quoridor_main.detect_board.realsense import ImgNode
from scipy.spatial.transform import Rotation
import DR_init
import sys, os

# 로봇 초기화 (예시)
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

rclpy.init()
dsr_node = rclpy.create_node("get_points", namespace=ROBOT_ID)
DR_init.__dsr__node = dsr_node

try:
    from DSR_ROBOT2 import get_current_posx
except ImportError as e:
    print(f"Error importing DSR_ROBOT2: {e}")
    sys.exit()

resource_path = "/home/rokey/quoridor_ws/src/quoridor_main/resource"
gripper2cam = np.load(os.path.join(resource_path, "T_gripper2camera.npy"))

class CornerPicker:
    def __init__(self):
        self.img_node = ImgNode()
        self.intrinsics = self._wait_for_valid_data(
            self.img_node.get_camera_intrinsic, "camera intrinsics"
        )
        self.window_name = "Click Corners"
        self.corners = []

        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            depth = self._wait_for_valid_data(self.img_node.get_depth_frame, "depth frame")
            z = depth[y, x]
            if z <= 0:
                print("Depth invalid at this pixel!")
                return

            cam_xyz = self._pixel_to_camera_coords(x, y, z)
            base_xyz = self._camera_to_base(cam_xyz)
            print(f"Clicked pixel: ({x}, {y}) -> Base XYZ: {base_xyz}")
            self.corners.append(base_xyz)

    def _camera_to_base(self, camera_coords):
        robot_pos = get_current_posx()[0]
        x, y, z, rx, ry, rz = robot_pos
        base2gripper = self.get_robot_pose_matrix(x, y, z, rx, ry, rz)
        cam = np.append(np.array(camera_coords), 1)
        base = base2gripper @ gripper2cam @ cam
        return tuple(base[:3])

    def _pixel_to_camera_coords(self, u, v, z):
        fx, fy = self.intrinsics["fx"], self.intrinsics["fy"]
        ppx, ppy = self.intrinsics["ppx"], self.intrinsics["ppy"]
        x = (u - ppx) * z / fx
        y = (v - ppy) * z / fy
        return (x, y, z)

    def _wait_for_valid_data(self, getter, description):
        data = getter()
        while data is None or (isinstance(data, np.ndarray) and not data.any()):
            rclpy.spin_once(self.img_node)
            print(f"Retry getting {description}...")
            data = getter()
        return data

    def get_robot_pose_matrix(self, x, y, z, rx, ry, rz):
        R = Rotation.from_euler("ZYZ", [rx, ry, rz], degrees=True).as_matrix()
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [x, y, z]
        return T

    def run(self):
        while True:
            rclpy.spin_once(self.img_node)
            frame = self.img_node.get_color_frame()
            if frame is None:
                continue
            for i, corner in enumerate(self.corners):
                # 화면에 표시
                cv2.circle(frame, (int(corner[0]), int(corner[1])), 5, (0,0,255), -1)
                cv2.putText(frame, f"{i}", (int(corner[0])+5, int(corner[1])-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1)

            cv2.imshow(self.window_name, frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break

        cv2.destroyAllWindows()

if __name__ == "__main__":
    picker = CornerPicker()
    picker.run()
    print("Selected corners (Base XYZ):")
    for c in picker.corners:
        print(c)
