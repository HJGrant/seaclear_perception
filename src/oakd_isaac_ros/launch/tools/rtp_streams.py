"""Shared helper for building composable RTP video_output nodes.

Uses ros_deep_learning's composable jetson_ros::VideoOutputNode (not the
standalone `video_output` executable) so it can be loaded into the same
ComposableNodeContainer as its image source and benefit from rclcpp
intra-process zero-copy -- the node's subscription callback is written to
take a SharedPtr specifically for this. That only actually avoids a copy
when the publisher on the other end of the topic is *also* an
intra-process-enabled composable node in the same container; for anything
else it just behaves like a normal subscriber.
"""
from launch_ros.descriptions import ComposableNode


def rtp_composable_nodes(name_prefix, input_topic, port, destination_ips,
                          headless, bitrate=4000000, width=0, height=0):
    """One VideoOutputNode per non-empty destination IP.

    Args:
        name_prefix: unique-per-stream node name prefix (e.g. 'rgb', 'mask').
        input_topic: topic to remap this instance's 'image_in' to.
        port: RTP destination port (same port used for every destination IP).
        destination_ips: list of IP strings; empty entries are skipped.
        headless: passed straight through to the node's 'headless' param.
        bitrate: H264 encode bitrate.
        width, height: GPU-downscale frames to this size before encoding
            (0 = stream at the input topic's size).

    Returns:
        list[ComposableNode], one per active destination IP.
    """
    nodes = []
    for i, ip in enumerate(ip for ip in destination_ips if ip):
        nodes.append(ComposableNode(
            package='ros_deep_learning',
            plugin='jetson_ros::VideoOutputNode',
            name=f'{name_prefix}_video_output_{i}',
            remappings=[('image_in', input_topic)],
            parameters=[{
                'resource': f'rtp://{ip}:{port}',
                'codec': 'h264',
                'bitrate': int(bitrate),
                'headless': headless,
                'output_width': int(width),
                'output_height': int(height),
            }],
            extra_arguments=[{'use_intra_process_comms': True}],
        ))
    return nodes
