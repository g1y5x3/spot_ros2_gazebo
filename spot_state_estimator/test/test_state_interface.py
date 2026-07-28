from spot_state_interface.msg import CentroidalState


def test_centroidal_state_has_fixed_semantic_fields():
    message = CentroidalState()

    assert len(message.normalized_momentum) == 6
    assert len(message.base_position) == 3
    assert len(message.base_ypr) == 3
    assert len(message.joint_position) == 12
