import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')

        # Declare ROS 2 parameter for image topics
        self.declare_parameter('image_topics')

        # Retrieve image topics from ROS 2 parameter
        image_topics = self.get_parameter('image_topics').get_parameter_value().string_array_value

        # Iterate over each image topic
        for topic in image_topics:
            # Extract publisher topic and camera name from the image topic
            pub_topic = topic.strip().split('/compressed')[0]
            cam_name = topic.strip().split('/')[1]

            # Create subscription and publisher based on the camera name
            if cam_name == 'frontpi':
                # Create subscription to the compressed image topic and associate it with the front callback
                self.subscription = self.create_subscription(CompressedImage, topic, self.front_callback, 5)
                # Create publisher for the corresponding image topic
                self.front_publisher = self.create_publisher(Image, pub_topic, 5)
            if cam_name == 'pspi':
                self.subscription = self.create_subscription(CompressedImage, topic, self.portside_callback, 5)
                self.portside_publisher = self.create_publisher(Image, pub_topic, 5)
            if cam_name == 'aftpi':
                self.subscription = self.create_subscription(CompressedImage, topic, self.stern_callback, 5)
                self.stern_publisher = self.create_publisher(Image, pub_topic, 5)
            if cam_name == 'sbpi':
                self.subscription = self.create_subscription(CompressedImage, topic, self.starboard_callback, 5)
                self.starboard_publisher = self.create_publisher(Image, pub_topic, 5)
        
        # Initialize CvBridge for image conversion
        self.bridge = CvBridge()


    def front_callback(self, msg: CompressedImage) -> None:
        """
        Handles the front processing of the received message.
        
        Arguments:
            msg (CompressedImage): The received image message.
        """
        # Decompress the image
        image = self.bridge.compressed_imgmsg_to_cv2(msg)
        # Convert CV image to ROS sensor/image
        imgmsg = self.bridge.cv2_to_imgmsg(image,encoding='bgr8')
        imgmsg.header = msg.header
        # Publish decompressed image
        self.front_publisher.publish(imgmsg)


    def portside_callback(self, msg: CompressedImage) -> None:
        """
        Handles the portside processing of the received message.
        
        Arguments:
            msg (CompressedImage): The received image message.
        """
        # Decompress the image
        image = self.bridge.compressed_imgmsg_to_cv2(msg)
        # Convert CV image to ROS sensor/image
        imgmsg = self.bridge.cv2_to_imgmsg(image,encoding='bgr8')
        imgmsg.header = msg.header
        # Publish decompressed image
        self.portside_publisher.publish(imgmsg)

 
    def stern_callback(self, msg: CompressedImage) -> None:
        """
        Handles the stern processing of the received message.
        
        Arguments:
            msg (CompressedImage): The received image message.
        """
        # Decompress the image
        image = self.bridge.compressed_imgmsg_to_cv2(msg)
        # Convert CV image to ROS sensor/image
        imgmsg = self.bridge.cv2_to_imgmsg(image,encoding='bgr8')
        imgmsg.header = msg.header
        # Publish decompressed image
        self.stern_publisher.publish(imgmsg)


    def starboard_callback(self, msg: CompressedImage) -> None:
        """
        Handles the starboard processing of the received message.
        
        Arguments:
            msg (CompressedImage): The received image message.
        """
        # Decompress the image
        image = self.bridge.compressed_imgmsg_to_cv2(msg)
        # Convert CV image to ROS sensor/image
        imgmsg = self.bridge.cv2_to_imgmsg(image,encoding='bgr8')
        imgmsg.header = msg.header
        # Publish decompressed image
        self.starboard_publisher.publish(imgmsg)
        
        
def main(args=None) -> None:
    # Initialize the ROS2 Python client library
    rclpy.init(args=args)
    
    # Create an instance of the ImageSubscriber class
    i_node = ImageSubscriber()
    
    # Enter the ROS2 event loop
    rclpy.spin(i_node)
    
    # Print a shutdown message
    print("shutdown")
    
    # Clean up and destroy the YoloDetectorNode
    i_node.destroy_node()
    
    # Shutdown the ROS2 Python client library
    rclpy.shutdown()


if __name__ == '__main__':
    main()