#!/usr/bin/env python3
import os
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_share_directory

class MissionNode(Node):
    def __init__(self):
        super().__init__('mission_node')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.waypoints = self.load_waypoints()
        self.current = 0
        self.active = False
        self.srv = self.create_service(Trigger, 'start_mission', self.start_cb)
        self.get_logger().info('Nodo listo. Llama a /start_mission')

    def load_waypoints(self):
        pkg_share = get_package_share_directory('mission_pkg')
        yaml_path = os.path.join(pkg_share, 'config', 'waypoints.yaml')
        self.get_logger().info(f'Cargando waypoints desde {yaml_path}')
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)
        wps = []
        for wp in data['waypoints']:
            p = PoseStamped()
            p.header.frame_id = wp['frame_id']
            p.pose.position.x = wp['pose']['position']['x']
            p.pose.position.y = wp['pose']['position']['y']
            p.pose.position.z = wp['pose']['position']['z']
            p.pose.orientation.x = wp['pose']['orientation']['x']
            p.pose.orientation.y = wp['pose']['orientation']['y']
            p.pose.orientation.z = wp['pose']['orientation']['z']
            p.pose.orientation.w = wp['pose']['orientation']['w']
            wps.append(p)
        return wps

    def start_cb(self, req, res):
        if self.active:
            res.success = False
            res.message = 'Misión ya en curso'
            return res
        if not self.waypoints:
            res.success = False
            res.message = 'No hay waypoints'
            return res
        self.active = True
        self.current = 0
        self.get_logger().info('Misión iniciada')
        self.send_next()
        res.success = True
        res.message = 'Misión iniciada'
        return res

    def send_next(self):
        if self.current >= len(self.waypoints):
            self.get_logger().info('Misión completada')
            self.active = False
            return
        goal = NavigateToPose.Goal()
        goal.pose = self.waypoints[self.current]
        self.get_logger().info(f'Enviando waypoint {self.current+1}')
        self.client.wait_for_server()
        self.future = self.client.send_goal_async(goal, feedback_callback=self.feedback)
        self.future.add_done_callback(self.goal_cb)

    def goal_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rechazado')
            self.active = False
            return
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.result_cb)

    def result_cb(self, future):
        future.result()
        self.get_logger().info('Waypoint alcanzado')
        self.current += 1
        self.send_next()

    def feedback(self, msg):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()