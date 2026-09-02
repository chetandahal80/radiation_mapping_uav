#!/usr/bin/env python
import rospy, tf, math, numpy as np
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
from collections import deque
import threading, time
from scipy.ndimage import label as nd_label

# move_base action client
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class FrontierExplorer:
    def __init__(self):
        self.map_sub  = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        self.scan_sub = rospy.Subscriber("/laser/scan", LaserScan, self.scan_callback)
        self.listener = tf.TransformListener()

        self.map = None
        self.scan = None
        self.map_resolution = 0.05
        self.map_origin = (0.0, 0.0)

        self.height = 1.8
        self.cluster_threshold = 30
        self.prev_goal = None

        # Recompute plan only when map updates
        self._map_dirty = False

        # Submap window radius (meters) for fast frontier detection
        self.frontier_window_radius = rospy.get_param("~frontier_window_radius", 10.0)

        self.rate = rospy.Rate(20)
        self.current_goal = self.create_pose(0.0, 0.0, 0.0)
        self.blended_goal = self.create_pose(0.0, 0.0, 0.0)  # for API compatibility

        # move_base action client
        self.mb = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        rospy.loginfo("Waiting for /move_base action server...")
        self.mb.wait_for_server()
        rospy.loginfo("Connected to /move_base.")

        rospy.loginfo("FrontierExplorer initialized.")

        # Start planner thread
        threading.Thread(target=self.planner_loop, daemon=True).start()

    def planner_loop(self):
        while not rospy.is_shutdown():
            if self.map is not None and self._map_dirty:
                goal = self.explore()
                if goal is not None:
                    self.current_goal = goal
                    self.send_move_base_goal(self.current_goal)
                self._map_dirty = False
            self.rate.sleep()

    def map_callback(self, msg: OccupancyGrid):
        self.map_resolution = msg.info.resolution
        self.map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        self.map = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
        self._map_dirty = True  # trigger replan

    def scan_callback(self, msg: LaserScan):
        self.scan = msg  

    def get_robot_position(self):
        try:
            self.listener.waitForTransform("map", "base_link", rospy.Time(0), rospy.Duration(0.5))
            (trans, rot) = self.listener.lookupTransform("map", "base_link", rospy.Time(0))
            gx = int((trans[0] - self.map_origin[0]) / self.map_resolution)
            gy = int((trans[1] - self.map_origin[1]) / self.map_resolution)
            yaw = tf.transformations.euler_from_quaternion(rot)[2]
            return gx, gy, trans[0], trans[1], yaw
        except Exception:
            return None

    def _extract_submap_window(self, rx_cell: int, ry_cell: int):
        """Return (submap, x0, y0) for a rolling window around the robot."""
        H, W = self.map.shape
        R_px = max(3, int(self.frontier_window_radius / self.map_resolution))
        x0 = max(0, rx_cell - R_px)
        x1 = min(W - 1, rx_cell + R_px)
        y0 = max(0, ry_cell - R_px)
        y1 = min(H - 1, ry_cell + R_px)
        return self.map[y0:y1+1, x0:x1+1], x0, y0

    def _box_sum5(self, mask: np.ndarray) -> np.ndarray:
        """Fast 5x5 sum over a binary mask using stride-tricks."""
        k = 5
        pad = k // 2
        A = np.pad(mask.astype(np.uint8), ((pad, pad), (pad, pad)), mode='constant')
        H, W = mask.shape
        s0, s1 = A.strides
        view = np.lib.stride_tricks.as_strided(
            A, shape=(H, W, k, k), strides=(s0, s1, s0, s1), writeable=False
        )
        return view.sum(axis=(2, 3)).astype(np.int32)

    def _find_frontiers_fast(self, submap: np.ndarray) -> np.ndarray:
        """
        Frontier mask (True=CANDIDATE) in submap
        free & (>=5 unknowns in 5x5) & (no obstacle within 5x5)
        """
        free     = (submap == 0)
        unknown  = (submap == -1)
        obstacle = (submap == 100)

        unk_count = self._box_sum5(unknown)
        near_obs  = self._box_sum5(obstacle) > 0

        cand = free & (~near_obs) & (unk_count >= 5)

        # Trim 2-cell border
        if cand.shape[0] > 4 and cand.shape[1] > 4:
            cand[:2, :] = False; cand[-2:, :] = False
            cand[:, :2] = False; cand[:, -2:] = False
        return cand

    def _label_components(self, mask: np.ndarray):
        """Label 4-connected components; returns (labels, num_labels)."""
        from scipy.ndimage import label as nd_label
        structure = np.array([[0,1,0],
                              [1,1,1],
                              [0,1,0]], dtype=np.uint8)
        labels, num = nd_label(mask.astype(np.uint8), structure=structure)
        return labels, int(num)

    def explore(self):
        robot_pos = self.get_robot_position()
        if robot_pos is None or self.map is None:
            return None

        rx_cell, ry_cell, rx_m, ry_m, yaw = robot_pos

        # Submap around robot
        sub, x0, y0 = self._extract_submap_window(rx_cell, ry_cell)

        # Frontier candidates
        cand = self._find_frontiers_fast(sub)
        if not np.any(cand):
            rospy.loginfo("No frontiers found in window.")
            return None

        # Connected components
        labels, num = self._label_components(cand)
        if num == 0:
            rospy.loginfo("No frontier components.")
            return None

        # Sizes per label (skip background 0)
        sizes = np.bincount(labels.ravel())[1:]
        valid_ids = np.nonzero(sizes >= self.cluster_threshold)[0] + 1
        if valid_ids.size == 0:
            rospy.loginfo("No frontier clusters above threshold.")
            return None

        # Selection cost (travel + turn + small goal-change penalty)
        v = rospy.get_param("~cruise_speed", 2.0)     # m/s
        yaw_rate = rospy.get_param("~yaw_rate", 1.2)  # rad/s

        best = None
        best_cost = float('inf')

        for lab in valid_ids:
            ys, xs = np.where(labels == lab)
            if xs.size == 0:
                continue
            cx_cell = int(np.median(xs)) + x0
            cy_cell = int(np.median(ys)) + y0

            gx = cx_cell * self.map_resolution + self.map_origin[0]
            gy = cy_cell * self.map_resolution + self.map_origin[1]

            dx_m = gx - rx_m
            dy_m = gy - ry_m
            d_m  = float(math.hypot(dx_m, dy_m))

            if self.prev_goal is not None:
                goal_diff = float(math.hypot(self.prev_goal[0] - gx, self.prev_goal[1] - gy))
                goal_change_t = goal_diff / max(v, 1e-6)
            else:
                goal_change_t = 0.0

            travel_t = d_m / max(v, 1e-6)
            ang_to_goal = math.atan2(dy_m, dx_m)
            ang_diff = math.atan2(math.sin(ang_to_goal - yaw), math.cos(ang_to_goal - yaw))
            turn_t = abs(ang_diff) / max(yaw_rate, 1e-6)

            cost = travel_t + 0.5 * turn_t + 0.2 * goal_change_t

            if cost < best_cost:
                best_cost = cost
                best = (gx, gy)

        if best is None:
            return None

        wx, wy = best
        self.prev_goal = [wx, wy]
        yaw_goal = math.atan2(wy - ry_m, wx - rx_m)
        rospy.loginfo("Best goal = %.3f, %.3f  (cost=%.2f)  yaw=%.1f deg", wx, wy, best_cost, math.degrees(yaw_goal))
        return self.create_pose(wx, wy, yaw_goal)

    # move_base glue
    def send_move_base_goal(self, pose_stamped: PoseStamped):
        goal = MoveBaseGoal()
        goal.target_pose = pose_stamped  # header.frame_id="map" already set
        self.mb.send_goal(goal)
        rospy.loginfo("Sent goal to move_base: (%.2f, %.2f)",
                      goal.target_pose.pose.position.x, goal.target_pose.pose.position.y)

    def create_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = self.height 
        q = quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        return pose


def run_planner():
    FrontierExplorer()

if __name__ == '__main__':
    try:
        run_planner()
    except rospy.ROSInterruptException:
        pass
