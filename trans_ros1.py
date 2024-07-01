# 导入ROS1相关的库
import rospy
import tf
import tf2_geometry_msgs
import geometry_msgs.msg

class TransformListenerNode:

    def __init__(self):
        # 初始化节点
        rospy.init_node('transform_listener_node', anonymous=True)
        self.tf_listener = tf.TransformListener()

        # 设置定时器，每秒调用一次timer_callback
        rospy.Timer(rospy.Duration(1.0), self.timer_callback)

    def timer_callback(self, event):
        try:
            # 等待变换关系可用
            self.tf_listener.waitForTransform('map', 'base_link', rospy.Time(0), rospy.Duration(4.0))
            # 获取变换关系
            (trans, rot) = self.tf_listener.lookupTransform('map', 'base_link', rospy.Time(0))
            rospy.loginfo("trans: %s", str(trans))
            rospy.loginfo("rot: %s", str(rot))
            # 创建一个PointStamped消息，并转换到map坐标系下
            point = geometry_msgs.msg.PointStamped()
            point.header.frame_id = 'base_link'
            point.point.x = 1.0
            point.point.y = 2.0
            point.point.z = 0.0
            transformed_point = tf2_geometry_msgs.do_transform_point(point, self.tf_listener)
            rospy.loginfo('Transformed point in map frame: x={}, y={}, z={}'.format(
                transformed_point.point.x, transformed_point.point.y, transformed_point.point.z))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr('Failed to transform point: %s', str(e))

if __name__ == '__main__':
    node = TransformListenerNode()
    rospy.spin()