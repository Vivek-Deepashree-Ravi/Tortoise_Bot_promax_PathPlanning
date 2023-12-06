import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Vector3
from sensor_msgs.msg import Imu
import tf2_ros

class ImuToTf(Node):
    def __init__(self):
        super().__init__('imu_to_tf')
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10)
        self.lp_filter = None
        self.subscription  # prevent unused variable warning

    def imu_callback(self, msg):
        if self.lp_filter is None:
            self.lp_filter = LowPassFilter(msg.linear_acceleration)
        else:
            self.lp_filter.update(msg.linear_acceleration)

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'imu'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = self.lp_filter.get()[0]
        transform.transform.translation.y = self.lp_filter.get()[1]
        transform.transform.translation.z = self.lp_filter.get()[2]
        transform.transform.rotation = msg.orientation
        self.tf_broadcaster.sendTransform(transform)

class LowPassFilter:
    def __init__(self, initial_value, alpha=0.05):
        self.value = Vector3()
        self.value.x = initial_value.x
        self.value.y = initial_value.y
        self.value.z = initial_value.z
        self.alpha = alpha

    def update(self, new_value):
        self.value.x = self.alpha * new_value.x + (1 - self.alpha) * self.value.x
        self.value.y = self.alpha * new_value.y + (1 - self.alpha) * self.value.y
        self.value.z = self.alpha * new_value.z + (1 - self.alpha) * self.value.z

    def get(self):
        return [self.value.x, self.value.y, self.value.z]


def main(args=None):
    rclpy.init(args=args)
    imu_to_tf = ImuToTf()
    rclpy.spin(imu_to_tf)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
