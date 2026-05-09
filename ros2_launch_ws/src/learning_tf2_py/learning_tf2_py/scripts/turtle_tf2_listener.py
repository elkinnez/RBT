import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import TransformException, Buffer, TransformListener
from turtlesim.srv import Spawn

class FrameListener(Node):
    def __init__(self):
        super().__init__('turtle_tf2_frame_listener')
        self.target_frame = self.declare_parameter('target_frame', 'turtle1').get_parameter_value().string_value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.spawner = self.create_client(Spawn, 'spawn')
        self.turtle_spawning_service_ready = False
        self.turtle_spawned = False

        self.publisher = self.create_publisher(Twist, 'turtle2/cmd_vel', 1)
        self.timer = self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        from_frame_rel = self.target_frame
        to_frame_rel = 'turtle2'

        if not self.turtle_spawning_service_ready:
            if self.spawner.service_is_ready():
                request = Spawn.Request()
                request.name = 'turtle2'
                request.x = 4.0
                request.y = 2.0
                request.theta = 0.0
                self.result = self.spawner.call_async(request)
                self.turtle_spawning_service_ready = True
                self.get_logger().info('Spawn request sent')
            else:
                self.get_logger().info('Service not ready')
            return

        if not self.turtle_spawned:
            if self.result.done():
                self.get_logger().info(f'Successfully spawned {self.result.result().name}')
                self.turtle_spawned = True
            else:
                self.get_logger().info('Spawn not finished')
                return

        try:
            t = self.tf_buffer.lookup_transform(to_frame_rel, from_frame_rel, rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

        msg = Twist()
        scale_rotation_rate = 1.0
        msg.angular.z = scale_rotation_rate * math.atan2(t.transform.translation.y, t.transform.translation.x)
        scale_forward_speed = 0.5
        msg.linear.x = scale_forward_speed * math.sqrt(t.transform.translation.x**2 + t.transform.translation.y**2)
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
