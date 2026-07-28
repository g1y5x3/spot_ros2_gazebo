import rclpy

from spot_state_estimator.state_adapter_node import StateAdapter


def test_state_adapter_has_safe_timing_defaults():
    rclpy.init()
    node = None
    try:
        try:
            node = StateAdapter()
        except TypeError as error:
            raise AssertionError(
                'timing parameters must work without a YAML override') from error
        assert node.get_parameter('publish_rate').value == 100.0
        assert node.get_parameter('max_age').value == 0.20
        assert node.get_parameter('max_skew').value == 0.11
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_state_adapter_publishes_typed_centroidal_state():
    rclpy.init()
    node = StateAdapter()
    try:
        publishers = node.get_publishers_info_by_topic('/spot/ocs2_state')
        assert [item.topic_type for item in publishers] == [
            'spot_state_interface/msg/CentroidalState']
    finally:
        node.destroy_node()
        rclpy.shutdown()
