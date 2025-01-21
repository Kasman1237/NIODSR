#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Float32

class PointControlledMover(Node):
    def __init__(self):
        super().__init__('point_controlled_mover')

        # Parametry domyślne
        self.declare_parameter('window_height', 512)
        self.window_height = self.get_parameter('window_height').value

        # Publikator dla prędkości robota
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subskrybent tematu /point
        self.point_subscriber = self.create_subscription(Point, '/point', self.point_callback, 10)

        # Timer do cyklicznego publikowania prędkości
        self.timer = self.create_timer(0.1, self.publish_velocity)

        # Stan robota
        self.move_forward = False

        self.get_logger().info("PointControlledMover node has started.")

    def point_callback(self, point_msg):
        # Sprawdzenie, czy kliknięty punkt jest powyżej lub poniżej środka ekranu
        screen_middle = self.window_height // 2
        if point_msg.y < screen_middle:
            self.get_logger().info(f"Point above middle: {point_msg.y} < {screen_middle}. Moving forward.")
            self.move_forward = True
        else:
            self.get_logger().info(f"Point below middle: {point_msg.y} >= {screen_middle}. Stopping.")
            self.move_forward = False

    def publish_velocity(self):
        twist_msg = Twist()

        if self.move_forward:
            twist_msg.linear.x = 0.2  # Prędkość liniowa do przodu
            twist_msg.angular.z = 0.0  # Brak obrotu
        else:
            twist_msg.linear.x = 0.0  # Zatrzymanie ruchu liniowego
            twist_msg.angular.z = 0.5  # Robot obraca się w miejscu

        # Publikacja wiadomości Twist
        self.cmd_vel_publisher.publish(twist_msg)
        self.get_logger().info(f"Published velocity: linear.x={twist_msg.linear.x}, angular.z={twist_msg.angular.z}")

def main(args=None):
    rclpy.init(args=args)
    node = PointControlledMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
