import pyrealsense2 as rs
import numpy as np
import cv2
import heapq
import time

# === הגדרות ===
FRAME_WIDTH, FRAME_HEIGHT = 640, 480
START = (400, 100)  # (y, x)
GOAL = (100, 500)
reaction_time = 0.5  # שניות
safety_margin = 0.3  # מטרים
center_weight = 0.05  # משקל העדפה למרכז

# === משתני מהירות ===
velocity = 0.0
prev_time = None

# === אתחול RealSense עם עומק ו-IMU ===
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, FRAME_WIDTH, FRAME_HEIGHT, rs.format.z16, 30)
config.enable_stream(rs.stream.accel)  # חיישן תאוצה

profile = pipeline.start(config)
sensor = profile.get_device().first_motion_sensor()

# === פונקציות ===
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def astar_center(grid, start, goal):
    rows, cols = grid.shape
    center_line = cols // 2
    open_set = [(0 + heuristic(start, goal), 0, start, [])]
    visited = set()

    while open_set:
        est_total_cost, cost, current, path = heapq.heappop(open_set)
        if current in visited:
            continue
        visited.add(current)
        path = path + [current]

        if current == goal:
            return path

        for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
            ny, nx = current[0] + dy, current[1] + dx
            neighbor = (ny, nx)
            if (0 <= ny < rows and 0 <= nx < cols and
                grid[neighbor] == 0 and
                neighbor not in visited):

                distance_from_center = abs(nx - center_line)
                penalty = distance_from_center * center_weight
                new_cost = cost + 1 + penalty
                est = new_cost + heuristic(neighbor, goal)
                heapq.heappush(open_set, (est, new_cost, neighbor, path))
    return []

def get_velocity_from_imu():
    global velocity, prev_time
    frames = pipeline.wait_for_frames()
    accel_frame = frames.first_or_default(rs.stream.accel)

    if not accel_frame:
        return velocity

    accel_data = accel_frame.as_motion_frame().get_motion_data()
    accel_x = accel_data.x  # נניח תנועה בציר X בלבד

    current_time = time.time()
    if prev_time is not None:
        dt = current_time - prev_time
        velocity += accel_x * dt
    prev_time = current_time
    return abs(velocity)

# === לולאת ראשית ===
try:
    while True:
        # קבלת פריימים
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        if not depth_frame:
            continue

        # === מהירות מרגע ל־רגע ===
        velocity = get_velocity_from_imu()
        depth_scale = depth_frame.get_units()

        # סף מכשול דינמי לפי מהירות
        DEPTH_THRESHOLD = reaction_time * velocity + safety_margin

        # יצירת מפת מכשולים
        depth_image = np.asanyarray(depth_frame.get_data())
        depth_meters = depth_image * depth_scale
        obstacle_mask = (depth_meters < DEPTH_THRESHOLD).astype(np.uint8)

        # תכנון מסלול
        grid = obstacle_mask.copy()
        path = astar_center(grid, START, GOAL)

        # הדמיה
        vis = cv2.cvtColor((255 - grid * 255).astype(np.uint8), cv2.COLOR_GRAY2BGR)
        for point in path:
            cv2.circle(vis, (point[1], point[0]), 1, (0, 0, 255), -1)
        cv2.circle(vis, (START[1], START[0]), 5, (0, 255, 0), -1)
        cv2.circle(vis, (GOAL[1], GOAL[0]), 5, (255, 0, 0), -1)
        cv2.line(vis, (FRAME_WIDTH // 2, 0), (FRAME_WIDTH // 2, FRAME_HEIGHT), (200, 200, 200), 1)

        # הצגת מהירות
        cv2.putText(vis, f"Velocity: {velocity:.2f} m/s", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
        cv2.putText(vis, f"Depth Threshold: {DEPTH_THRESHOLD:.2f} m", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)

        cv2.imshow('Path Planning with IMU Velocity', vis)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
