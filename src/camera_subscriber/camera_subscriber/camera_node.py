#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point  
import cv2  # OpenCV library
import numpy as np  


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # Parametr dla długości kwadratu
        self.declare_parameter('square_size', 200)
        self.square_size = self.get_parameter('square_size').value

        # Publikator punktów kliknięcia
        self.point_publisher = self.create_publisher(Point, '/point', 10)

        # Narzędzia do obsługi obrazu
        self.window_name = "Camera Viewer"
        self.clicked_point = None  
        self.timer = self.create_timer(0.1, self.update_image)
        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)

        self.get_logger().info("Camera node initialized and running.")

    def update_image(self):
        # Tworzenie czarnego obrazu
        frame = np.zeros((512, 700, 3), dtype=np.uint8)
        if self.clicked_point is not None:
            x, y = self.clicked_point
            cv2.rectangle(frame, (x, y),
                          (x + self.square_size, y + self.square_size),
                          (0, 255, 0), 3)
        cv2.imshow(self.window_name, frame)
        cv2.waitKey(1)  # Odświeżenie okna

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:  
            self.clicked_point = (x, y)
            self.get_logger().info(f"Clicked point: x={x}, y={y}")
            point_msg = Point()
            point_msg.x = float(x)
            point_msg.y = float(y)
            point_msg.z = 0.0
            self.point_publisher.publish(point_msg)
            self.get_logger().info(f"Published point: {point_msg}")


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows() 
        rclpy.shutdown()


if __name__ == '__main__':
    main()
