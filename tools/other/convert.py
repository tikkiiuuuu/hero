#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSubscriber(Node):
    def __init__(self, topic_name):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            topic_name,
            self.listener_callback,
            10) # 注意：队列大小 (queue size) 可能需要调整以避免延迟
        self.subscription
        self.bridge = CvBridge()
        self.window_name = f"Video from {topic_name}"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

    def listener_callback(self, msg):
        try:
            # 将 ROS Image 消息转换为 OpenCV 图像
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        # 在窗口中显示图像
        cv2.imshow(self.window_name, cv_image)
        cv2.waitKey(1) # 1ms 延迟，允许 OpenCV 刷新窗口

def main(args=None):
    rclpy.init(args=args)
    topic_name = '/camera/image_raw' # 替换为你的实际话题名
    image_subscriber = ImageSubscriber(topic_name)
    
    print("Starting subscriber. Press 'q' in the OpenCV window to exit.")
    
    try:
        rclpy.spin(image_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        # 清理资源
        image_subscriber.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()