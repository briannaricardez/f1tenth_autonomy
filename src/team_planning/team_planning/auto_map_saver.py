#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from slam_toolbox.srv import SaveMap


class AutoMapSaver(Node):
    def __init__(self):
        super().__init__('auto_map_saver')

        self.declare_parameter('map_path', '/home/team2/f1tenth_autonomy/src/team_planning/maps/my_track_map')
        self.declare_parameter('save_interval', 30.0)

        self.map_path = self.get_parameter('map_path').value
        self.save_interval = float(self.get_parameter('save_interval').value)

        self.client = self.create_client(SaveMap, '/slam_toolbox/save_map')
        self.timer = self.create_timer(self.save_interval, self.save_map)

        self.get_logger().info(
            f'AutoMapSaver: saving map to {self.map_path} every {self.save_interval}s'
        )

    def save_map(self):
        if not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Map save service not available')
            return

        req = SaveMap.Request()
        req.name.data = self.map_path
        self.client.call_async(req)
        self.get_logger().info(f'Map saved to {self.map_path}')


def main():
    rclpy.init()
    node = AutoMapSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_map()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
