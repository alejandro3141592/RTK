import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class PublisherNode(Node):

    def __init__(self):
        super().__init__('inject_angle')
        self.publisher = self.create_publisher(Float64, 'angulo_magnetometro', 10)
        self.timer = self.create_timer(1, self.publish_data)
        self.value = 0.0

    def publish_data(self):
        self.get_logger().info('Orientation is being sent: %f' % self.value)
        self.publisher.publish(Float64(data=self.value))
        self.value += 1
        if self.value >= 360:
            self.value = 0.0


def main(args=None):
    rclpy.init(args=args)

    publisher_node = PublisherNode()

    rclpy.spin(publisher_node)

    publisher_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
