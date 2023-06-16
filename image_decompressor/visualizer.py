import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
# import cv2

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/frontpi/oak/rgb/image_raw/compressed',  # Replace 'image_topic' with your actual topic name
            self.image_callback,
            10
        )
        self.publisher = self.create_publisher(Image, '/frontpi/oak/rgb/image_raw',5)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # Decompress the image
        image = self.bridge.compressed_imgmsg_to_cv2(msg)
        # Publish decompressed image
        imgmsg = self.bridge.cv2_to_imgmsg(image,encoding='bgr8')
        self.publisher.publish(imgmsg)

        # Display the image
        # cv2.imshow("Image", image)
        # cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    i_node = ImageSubscriber()
    rclpy.spin(i_node)

    i_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()