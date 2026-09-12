import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from message_filters import Subscriber, ApproximateTimeSynchronizer


class StereoStampChecker(Node):
    def __init__(self):
        super().__init__('umi_dataset_tools')

        self.left_sub = Subscriber(
            self,
            Image,
            '/camera/camera/infra1/image_rect_raw'
        )

        self.right_sub = Subscriber(
            self,
            Image,
            '/camera/camera/infra2/image_rect_raw'
        )

        self.sync = ApproximateTimeSynchronizer(
            [self.left_sub, self.right_sub],
            queue_size=20,
            slop=0.02
        )
        self.sync.registerCallback(self.callback)

        self.count = 0

    def callback(self, left: Image, right: Image):
        left_time = (
            left.header.stamp.sec
            + left.header.stamp.nanosec * 1e-9
        )
        right_time = (
            right.header.stamp.sec
            + right.header.stamp.nanosec * 1e-9
        )

        delta_ms = abs(left_time - right_time) * 1000.0

        self.count += 1
        self.get_logger().info(
            f'pair={self.count:05d}, '
            f'left={left_time:.9f}, '
            f'right={right_time:.9f}, '
            f'delta={delta_ms:.6f} ms'
        )


def main(args=None):
    rclpy.init(args=args)
    node = StereoStampChecker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
