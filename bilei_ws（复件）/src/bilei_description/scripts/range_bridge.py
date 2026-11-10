#!/usr/bin/env python3
"""
把 Gazebo 的 LaserScan（单波束超声波模拟）转成 sensor_msgs/Range。

背景：本机 gazebo_ros_ray_sensor 的 output_type=sensor_msgs/Range 存在
消息序列化 bug（订阅端报 Unable to convert call argument to Python object），
所以仿真里改用 LaserScan 输出，再由本节点统一转成 Range。

这样做的价值：真机上的 HC-SR04 驱动本来就发 sensor_msgs/Range，
上层（建图、避障）订阅的话题和消息类型完全一致，切真机不用改代码。
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range

# name, 输入话题, 输出话题, frame_id
SONARS = [
    ('front', '/range_raw/front', '/range/front', 'sonar_front_link'),
    ('left',  '/range_raw/left',  '/range/left',  'sonar_left_link'),
    ('right', '/range_raw/right', '/range/right', 'sonar_right_link'),
]

MIN_RANGE = 0.02
MAX_RANGE = 4.0
FIELD_OF_VIEW = 0.26   # rad，约 15°，和 HC-SR04 的波束角接近


class RangeBridge(Node):

    def __init__(self):
        super().__init__('range_bridge')
        self.pubs = {}
        for name, in_topic, out_topic, frame in SONARS:
            self.pubs[name] = self.create_publisher(Range, out_topic, 10)
            self.create_subscription(
                LaserScan, in_topic,
                lambda msg, n=name, f=frame: self.on_scan(n, f, msg), 10)
            self.get_logger().info(f'{in_topic} -> {out_topic}')

    def on_scan(self, name, frame, msg: LaserScan):
        # Gazebo 里没有障碍时是 inf，统一钳到 max_range
        finite = [r for r in msg.ranges if math.isfinite(r) and r > 0.0]
        value = min(finite) if finite else MAX_RANGE
        value = max(min(value, MAX_RANGE), MIN_RANGE)

        out = Range()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = frame
        out.radiation_type = Range.ULTRASOUND
        out.field_of_view = FIELD_OF_VIEW
        out.min_range = MIN_RANGE
        out.max_range = MAX_RANGE
        out.range = float(value)
        self.pubs[name].publish(out)


def main():
    rclpy.init()
    node = RangeBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
