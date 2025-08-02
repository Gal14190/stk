import pyrealsense2 as rs
import numpy as np
import cv2

from SettingsGUI import SettingsGUI
from ControlWindow import ControlWindow


class DepthCameraProcessor:
    # FRAME_WIDTH = 640
    # FRAME_HEIGHT = 480
    # MAX_SERVO_ANGLE = 5

    # TOP_STOP = 70
    # BOTTOM_STOP = 70
    # LEFT_STOP = 100
    # RIGHT_STOP = 100

    # BOTTOM_DISTANCE_STOP = 0.2
    # TOP_DISTANCE_STOP = 0.2
    # FRONT_DISTANCE_STOP = 0.2

    def __init__(self, FRAME_WIDTH, FRAME_HEIGHT, MAX_SERVO_ANGLE,
                 TOP_STOP, BOTTOM_STOP, LEFT_STOP, RIGHT_STOP,
                 BOTTOM_DISTANCE_STOP, TOP_DISTANCE_STOP, FRONT_DISTANCE_STOP):
        self.FRAME_WIDTH = FRAME_WIDTH
        self.FRAME_HEIGHT = FRAME_HEIGHT
        self.MAX_SERVO_ANGLE = MAX_SERVO_ANGLE

        self.TOP_STOP = TOP_STOP
        self.BOTTOM_STOP = BOTTOM_STOP
        self.LEFT_STOP = LEFT_STOP
        self.RIGHT_STOP = RIGHT_STOP

        self.BOTTOM_DISTANCE_STOP = BOTTOM_DISTANCE_STOP
        self.TOP_DISTANCE_STOP = TOP_DISTANCE_STOP
        self.FRONT_DISTANCE_STOP = FRONT_DISTANCE_STOP
        
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, self.FRAME_WIDTH, self.FRAME_HEIGHT, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, self.FRAME_WIDTH, self.FRAME_HEIGHT, rs.format.rgb8, 30)
        config.enable_stream(rs.stream.accel)

        self.profile = self.pipeline.start(config)
        self.sensor = self.profile.get_device().first_motion_sensor()

        self.cross_position = self.FRAME_WIDTH // 2
        self.flag_stop = False
        self.left_flag = False
        self.right_flag = False
        self.delta = 0
        self.notification = ""

    def map_from_to(self, x, a, b, c, d):
        return (x - a) / (b - a) * (d - c) + c

    def process_frame(self):
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame:
            return None, None, None

        depth_scale = depth_frame.get_units()
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        depth_meters = depth_image * depth_scale

        color_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)
        vis = cv2.cvtColor((254 - depth_meters * 255).astype(np.uint8), cv2.COLOR_GRAY2BGR)

        return depth_frame, vis, color_image

    def process_bottom_area(self, depth_frame, vis):
        arr_dist = []
        for i in range(self.FRAME_WIDTH - self.LEFT_STOP - self.RIGHT_STOP):
            for j in range(self.BOTTOM_STOP):
                ptX = self.LEFT_STOP + i
                ptY = self.FRAME_HEIGHT - self.BOTTOM_STOP + j
                dist = depth_frame.get_distance(ptX, ptY)
                if dist != 0.0:
                    arr_dist.append(dist)
                cv2.circle(vis, (ptX, ptY), 1, (0, 0, int(10 * dist + 150) % 255), -1)

        if arr_dist:
            min_dist = min(arr_dist)
            if min_dist <= self.BOTTOM_DISTANCE_STOP:
                self.flag_stop = True
                self.notification = f'Object on bottom {min_dist:.2f}'

    def process_top_area(self, depth_frame, vis):
        arr_dist = []
        for i in range(self.FRAME_WIDTH - self.LEFT_STOP - self.RIGHT_STOP):
            for j in range(self.TOP_STOP):
                ptX = self.LEFT_STOP + i
                ptY = self.TOP_STOP - j
                dist = depth_frame.get_distance(ptX, ptY)
                if dist != 0.0:
                    arr_dist.append(dist)
                cv2.circle(vis, (ptX, ptY), 1, (0, 0, int(10 * dist + 150) % 255), -1)

        if arr_dist:
            min_dist = min(arr_dist)
            if min_dist <= self.TOP_DISTANCE_STOP:
                self.flag_stop = True
                self.notification = f'Object on top {min_dist:.2f}'

    def process_front_area(self, depth_frame):
        front_dist = depth_frame.get_distance(self.FRAME_WIDTH // 2, self.FRAME_HEIGHT // 2)
        if front_dist <= self.FRONT_DISTANCE_STOP and front_dist != 0.0:
            self.flag_stop = True
            self.notification = f'Object on front {front_dist:.2f}'

    def process_direction(self, depth_frame):
        arr_dist_left, arr_pos_left = [], []
        arr_dist_right, arr_pos_right = [], []

        for i in range(self.FRAME_WIDTH // 2 - self.LEFT_STOP):
            ptY = self.FRAME_HEIGHT // 2

            ptX = self.LEFT_STOP + i
            dist = depth_frame.get_distance(ptX, ptY)
            if dist != 0.0:
                arr_dist_left.append(dist)
                arr_pos_left.append(ptX)

            ptX = self.FRAME_WIDTH - self.RIGHT_STOP - i
            dist = depth_frame.get_distance(ptX, ptY)
            if dist != 0.0:
                arr_dist_right.append(dist)
                arr_pos_right.append(ptX)

        if arr_dist_left and arr_dist_right:
            avg_left = np.mean(arr_dist_left)
            avg_right = np.mean(arr_dist_right)

            if avg_left > avg_right:
                self.set_direction_flag(arr_dist_left, arr_pos_left, is_left=True)
            else:
                self.set_direction_flag(arr_dist_right, arr_pos_right, is_left=False)

    def set_direction_flag(self, distances, positions, is_left=True):
        max_dist = max(distances)
        index = distances.index(max_dist)
        ptX = positions[index]
        sum_points = np.mean(distances[max(0, index - 4):index + 5])

        if abs(sum_points - max_dist) <= 0.05 and abs(ptX - self.FRAME_WIDTH // 2) >= 50:
            self.cross_position = ptX
            self.delta = abs(ptX - self.FRAME_WIDTH // 2)

            if is_left:
                self.left_flag = True
                self.right_flag = False
            else:
                self.right_flag = True
                self.left_flag = False
        else:
            self.cross_position = self.FRAME_WIDTH // 2
            self.left_flag = self.right_flag = False

    def update_control_window(self):
        if self.flag_stop:
            ControlWindow((255, 255, 255), (0, 0), self.notification)
        elif self.right_flag:
            angle = int(self.map_from_to(self.delta, 0, self.FRAME_WIDTH // 2 - self.RIGHT_STOP, 0, self.MAX_SERVO_ANGLE))
            ControlWindow((0, 255, 255), (0, angle), "")
        elif self.left_flag:
            angle = int(self.map_from_to(self.delta, 0, self.FRAME_WIDTH // 2 - self.LEFT_STOP, 0, self.MAX_SERVO_ANGLE))
            ControlWindow((255, 255, 0), (angle, 0), "")
        else:
            ControlWindow((0, 0, 0), (0, 0), "")

    def draw_overlay(self, vis):
        cv2.circle(vis, (self.cross_position, self.FRAME_HEIGHT // 2), 4, (0, 255, 0), 2)
        cv2.circle(vis, (self.FRAME_WIDTH // 2, self.FRAME_HEIGHT // 2), 8, (0, 255, 0), 1)

        cv2.line(vis, (0, self.TOP_STOP), (self.FRAME_WIDTH, self.TOP_STOP), (0, 0, 200), 1)
        cv2.line(vis, (0, self.FRAME_HEIGHT - self.BOTTOM_STOP), (self.FRAME_WIDTH, self.FRAME_HEIGHT - self.BOTTOM_STOP), (0, 0, 200), 1)
        cv2.line(vis, (self.LEFT_STOP, 0), (self.LEFT_STOP, self.FRAME_HEIGHT), (0, 0, 200), 1)
        cv2.line(vis, (self.FRAME_WIDTH - self.RIGHT_STOP, 0), (self.FRAME_WIDTH - self.RIGHT_STOP, self.FRAME_HEIGHT), (0, 0, 200), 1)
        
        cv2.circle(vis, (self.LEFT_STOP, self.FRAME_HEIGHT - self.BOTTOM_STOP), 6, (255, 0, 0), -1)
        cv2.circle(vis, (self.FRAME_WIDTH - self.RIGHT_STOP, self.FRAME_HEIGHT - self.BOTTOM_STOP), 6, (255, 0, 0), -1)
        cv2.line(vis, (self.LEFT_STOP, self.FRAME_HEIGHT - self.BOTTOM_STOP), (self.FRAME_WIDTH//2, self.FRAME_HEIGHT//2), (255, 0, 0), 1)
        cv2.line(vis, (self.FRAME_WIDTH - self.RIGHT_STOP, self.FRAME_HEIGHT - self.BOTTOM_STOP), (self.FRAME_WIDTH//2, self.FRAME_HEIGHT//2), (255, 0, 0), 1)

    def run(self):
        try:
            while True:
                self.flag_stop = self.left_flag = self.right_flag = False
                self.notification = ""

                depth_frame, vis, color_image = self.process_frame()
                if depth_frame is None:
                    continue

                self.draw_overlay(vis)
                self.process_bottom_area(depth_frame, vis)
                self.process_top_area(depth_frame, vis)
                self.process_front_area(depth_frame)
                self.process_direction(depth_frame)
                self.update_control_window()

                cv2.imshow('Depth Cam', vis)
                cv2.imshow('Color Cam', color_image)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        finally:
            self.pipeline.stop()
            cv2.destroyAllWindows()


if __name__ == "__main__":
    gui = SettingsGUI()
    user_settings = gui.run()
    if user_settings:
        processor = DepthCameraProcessor(**user_settings)
        processor.run()
