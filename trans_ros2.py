import rclpy
from rclpy.node import Node
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg
from builtin_interfaces.msg import Duration

class TransformListenerNode(Node):

    def __init__(self):
        super().__init__('transform_listener_node')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            trans = transform.transform.translation
            rot = transform.transform.rotation
            print("trans",trans)
            print("rot",rot)
            point = geometry_msgs.msg.PointStamped()
            point.header.frame_id = 'base_link'
            point.point.x = 1.0
            point.point.y = 2.0
            point.point.z = 0.0
            transformed_point = tf2_geometry_msgs.do_transform_point(point, transform)
            self.get_logger().info('Transformed point in map frame: x={}, y={}, z={}'.format(
                transformed_point.point.x, transformed_point.point.y, transformed_point.point.z))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error('Failed to transform point: {}'.format(str(e)))

def main(args=None):
    rclpy.init(args=args)
    node = TransformListenerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()