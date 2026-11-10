#!/usr/bin/env python3
"""
占据栅格建图（Occupancy Grid Mapping）

输入：
    /odom           nav_msgs/Odometry      机器人位姿
    /range/front    sensor_msgs/Range      前超声波
    /range/left     sensor_msgs/Range      左超声波
    /range/right    sensor_msgs/Range      右超声波
输出：
    /map            nav_msgs/OccupancyGrid 栅格地图（frame_id = odom）

算法：log-odds 更新 + Bresenham 射线投射
    L(m) <- L(m) + log(p/(1-p)) - L0
    命中格   p=0.70 -> +0.85
    经过格   p=0.40 -> -0.40
    限幅     [-6, +6]
"""

import math

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Range
from geometry_msgs.msg import Point, Pose
from std_msgs.msg import ColorRGBA as Color
from visualization_msgs.msg import Marker

# 超声波在 base_link 下的安装位姿 (dx, dy, dyaw)，必须和 URDF 一致
#   front : x = body_x/2                  y = 0        yaw = 0
#   left  : x = body_x/2 - 0.05           y = +body_y/2 yaw = +0.35
#   right : x = body_x/2 - 0.05           y = -body_y/2 yaw = -0.35
SONAR_POSE = {
    'front': (0.125, 0.000, 0.00),
    'left':  (0.075, 0.090, 0.35),
    'right': (0.075, -0.090, -0.35),
}

L_OCC = 0.85
L_FREE = -0.40
L_CLAMP = 6.0
HIT_PROB_LO = 0.40
HIT_PROB_HI = 0.70


class OccupancyMapper(Node):

    def __init__(self):
        super().__init__('occupancy_mapper')

        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('width', 200)     # 200 * 0.05 = 10 m
        self.declare_parameter('height', 200)
        self.declare_parameter('origin_x', -5.0)
        self.declare_parameter('origin_y', -5.0)

        self.res = self.get_parameter('resolution').value
        self.w = self.get_parameter('width').value
        self.h = self.get_parameter('height').value
        self.ox = self.get_parameter('origin_x').value
        self.oy = self.get_parameter('origin_y').value

        self.log_odds = np.zeros((self.h, self.w), dtype=np.float32)

        # 机器人位姿（odom 系）
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.have_pose = False
        self.ray_segments = []      # [(start_xy, end_xy, is_hit), ...]

        # /map 用 Transient Local（和 map_server 一致），RViz 后连也能立刻拿到最后一帧
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', map_qos)
        self.ray_pub = self.create_publisher(Marker, '/sonar_rays', 10)

        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        for name in SONAR_POSE:
            self.create_subscription(
                Range, f'/range/{name}',
                lambda msg, n=name: self.on_range(n, msg), 10)

        self.create_timer(0.5, self.publish_map)
        self.get_logger().info(
            f'建图节点启动：{self.w}x{self.h} 格，分辨率 {self.res}m，'
            f'范围 10x10m，坐标系 odom')

    # ---------------- 位姿 ----------------
    def on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.x, self.y = p.x, p.y
        # 只取平面偏航角
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.th = math.atan2(siny, cosy)
        self.have_pose = True

    def world_to_cell(self, wx, wy):
        i = int((wx - self.ox) / self.res)
        j = int((wy - self.oy) / self.res)
        return i, j

    def in_grid(self, i, j):
        return 0 <= i < self.w and 0 <= j < self.h

    # ---------------- 单帧更新 ----------------
    def on_range(self, name, msg: Range):
        if not self.have_pose:
            return

        dx, dy, dyaw = SONAR_POSE[name]
        cos_t, sin_t = math.cos(self.th), math.sin(self.th)

        # 传感器在世界系下的位姿
        sx = self.x + cos_t * dx - sin_t * dy
        sy = self.y + sin_t * dx + cos_t * dy
        s_ang = self.th + dyaw

        r = float(msg.range)
        # 读数接近量程上限，说明这一束没打到东西：只清理沿途，不标记占据
        is_hit = r < (msg.max_range - 0.02)

        ex = sx + r * math.cos(s_ang)
        ey = sy + r * math.sin(s_ang)

        si, sj = self.world_to_cell(sx, sy)
        ei, ej = self.world_to_cell(ex, ey)

        # 记录射线端点，供 RViz 显示
        self.ray_segments.append(((sx, sy), (ex, ey), is_hit))
        if len(self.ray_segments) > 120:          # 只保留最近 120 条
            self.ray_segments = self.ray_segments[-120:]

        cells = self.bresenham(si, sj, ei, ej)
        if len(cells) < 2:
            return

        for k, (i, j) in enumerate(cells):
            if not self.in_grid(i, j):
                continue
            if k == len(cells) - 1 and is_hit:
                self.log_odds[j, i] += L_OCC
            else:
                self.log_odds[j, i] += L_FREE
            self.log_odds[j, i] = max(-L_CLAMP, min(L_CLAMP, self.log_odds[j, i]))

    @staticmethod
    def bresenham(x0, y0, x1, y1):
        """标准 Bresenham 直线，返回经过的格子列表。"""
        cells = []
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy
        x, y = x0, y0
        while True:
            cells.append((x, y))
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x += sx
            if e2 <= dx:
                err += dx
                y += sy
        return cells

    # ---------------- 发布 ----------------
    def publish_map(self):
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = 'odom'

        grid.info.resolution = self.res
        grid.info.width = self.w
        grid.info.height = self.h
        grid.info.origin.position.x = self.ox
        grid.info.origin.position.y = self.oy
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.w = 1.0

        # -1 未知 / 0 空闲 / 100 占据
        data = np.full((self.h, self.w), -1, dtype=np.int8)
        data[self.log_odds > HIT_PROB_LO] = 100
        data[self.log_odds < -0.2] = 0

        grid.data = data.reshape(-1).tolist()
        self.map_pub.publish(grid)
        self.publish_rays()

    def publish_rays(self):
        """把最近一次的超声波射线画成线段，方便在 RViz 里看"""
        if not self.ray_segments:
            return
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'odom'
        marker.ns = 'sonar'
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.01
        marker.pose.orientation.w = 1.0

        for (sx, sy), (ex, ey), is_hit in self.ray_segments:
            p0 = Point(x=sx, y=sy, z=0.02)
            p1 = Point(x=ex, y=ey, z=0.02)
            marker.points.append(p0)
            marker.points.append(p1)
            if is_hit:
                c = (1.0, 0.35, 0.0, 1.0)     # 命中 -> 橙色
            else:
                c = (0.2, 0.7, 1.0, 1.0)      # 未命中 -> 蓝色
            marker.colors.append(Color(r=c[0], g=c[1], b=c[2], a=c[3]))
            marker.colors.append(Color(r=c[0], g=c[1], b=c[2], a=c[3]))

        self.ray_pub.publish(marker)


def main():
    rclpy.init()
    node = OccupancyMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
