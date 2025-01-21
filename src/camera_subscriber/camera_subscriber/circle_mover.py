#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist  # Wiadomość używana do sterowania robotem

class CircleMover(Node):
    def __init__(self):
        super().__init__('circle_mover')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.publish_velocity)  # Cyklicznie co 0.1 sekundy
        self.get_logger().info('CircleMover node has started.')

    def publish_velocity(self):
        # Tworzymy wiadomość Twist
        twist_msg = Twist()
        
        # Prędkość liniowa (do przodu)
        twist_msg.linear.x = 0.2  # np. 0.2 m/s
        
        # Prędkość kątowa (obrót)
        twist_msg.angular.z = 0.5  # np. 0.5 rad/s
        
        # Publikujemy wiadomość na temacie /cmd_vel
        self.publisher.publish(twist_msg)
        self.get_logger().info(f'Published: linear.x={twist_msg.linear.x}, angular.z={twist_msg.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    node = CircleMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

