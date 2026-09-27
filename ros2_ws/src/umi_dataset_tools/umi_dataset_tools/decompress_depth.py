#!/usr/bin/env python3
"""Republish PNG compressedDepth as a raw sensor_msgs/Image."""

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image

from umi_dataset_tools.compressed_depth import decode_compressed_depth
from umi_dataset_tools.qos import reliable_image_qos


class DepthDecompressor(Node):
    def __init__(self):
        super().__init__('umi_depth_decompressor')
        input_topic = self.declare_parameter(
            'input_topic',
            '/camera/camera/depth/image_rect_raw/compressedDepth',
        ).value
        output_topic = self.declare_parameter(
            'output_topic',
            '/umi/depth/image_rect_raw',
        ).value
        input_qos = reliable_image_qos()
        output_qos = reliable_image_qos()
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, output_topic, output_qos)
        self.subscription = self.create_subscription(
            CompressedImage,
            input_topic,
            self.callback,
            input_qos,
        )
        self.get_logger().info(f'Decompressing {input_topic} -> {output_topic}')

    def callback(self, message):
        try:
            depth = decode_compressed_depth(message)
            if depth.dtype.name == 'uint16':
                encoding = '16UC1'
            elif depth.dtype.name == 'float32':
                encoding = '32FC1'
            else:
                raise RuntimeError(f'Unsupported decoded depth dtype: {depth.dtype}')
            output = self.bridge.cv2_to_imgmsg(depth, encoding=encoding)
            output.header = message.header
            self.publisher.publish(output)
        except RuntimeError as error:
            self.get_logger().error(str(error))


def main(args=None):
    rclpy.init(args=args)
    node = DepthDecompressor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
