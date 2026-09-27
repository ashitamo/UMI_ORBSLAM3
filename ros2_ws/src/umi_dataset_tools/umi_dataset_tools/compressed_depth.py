"""Utilities for ROS compressedDepth PNG messages."""

import cv2
import numpy as np


PNG_SIGNATURE = b'\x89PNG\r\n\x1a\n'


def decode_compressed_depth(message):
    format_text = str(message.format)
    if 'compressedDepth' not in format_text:
        raise RuntimeError(
            f'Expected compressedDepth message, received format {format_text!r}'
        )
    if 'rvl' in format_text.lower():
        raise RuntimeError('RVL compressedDepth is not supported; use PNG transport')
    if format_text.strip().startswith('32FC1'):
        raise RuntimeError(
            '32FC1 compressedDepth requires inverse-depth reconstruction and is '
            'not supported; use the D405 16UC1 PNG stream'
        )

    payload = bytes(message.data)
    png_offset = payload.find(PNG_SIGNATURE)
    if png_offset < 0:
        raise RuntimeError('compressedDepth message does not contain a PNG payload')
    image = cv2.imdecode(
        np.frombuffer(payload[png_offset:], dtype=np.uint8),
        cv2.IMREAD_UNCHANGED,
    )
    if image is None or image.ndim != 2:
        raise RuntimeError('Unable to decode compressedDepth PNG image')
    return image
