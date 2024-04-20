import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class PublisherNode(Node):

    def __init__(self):
        super().__init__('publisher_node')
        self.pub_lat = self.create_publisher(Float64, 'latitude', 10)
        self.pub_long = self.create_publisher(Float64, 'longitude', 10)
        self.timer = self.create_timer(1, self.publish_data)
        self.publish_lat = 19.0183016
        self.publish_long = -98.2402208
    def publish_data(self):
        
        self.pub_lat.publish(Float64(data=self.publish_lat))
        print(self.publish_lat)
        self.pub_long.publish(Float64(data=self.publish_long))
        self.get_logger().info('Longitude is being sent')
        self.publish_lat = self.publish_lat +0.0000


def main(args=None):
    rclpy.init(args=args)

    publisher_node = PublisherNode()

    rclpy.spin(publisher_node)

    publisher_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
