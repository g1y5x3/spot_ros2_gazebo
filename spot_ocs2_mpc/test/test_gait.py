from spot_ocs2_mpc.gait import (
    CONSERVATIVE_CRAWL_TEMPLATE, CONSERVATIVE_TROT_TEMPLATE, STANCE_TEMPLATE,
    command_requests_motion, gait_template, select_gait)


def test_conservative_trot_is_diagonal_with_full_stance_transitions():
    times, modes = CONSERVATIVE_TROT_TEMPLATE
    assert times == (0.0, 0.50, 2.50, 3.00, 5.00)
    assert modes == (9, 15, 6, 15)
    assert len(times) == len(modes) + 1


def test_command_motion_selection_and_stance():
    assert command_requests_motion((0.01, 0.0, 0.0))
    assert command_requests_motion((0.0, -0.01, 0.0))
    assert command_requests_motion((0.0, 0.0, 0.01))
    assert not command_requests_motion((0.0, 0.0, 0.0))
    assert gait_template('stance') == STANCE_TEMPLATE
    times, modes = CONSERVATIVE_CRAWL_TEMPLATE
    assert times == (
        0.0, 0.40, 2.00, 2.40, 4.00, 4.40, 6.00, 6.40, 8.00)
    assert modes == (7, 15, 14, 15, 11, 15, 13, 15)


def test_trot_in_place_can_be_selected_at_zero_command():
    assert select_gait(
        (0.0, 0.0, 0.0), float('inf'), 0.5,
        'conservative_crawl', 'conservative_trot') == 'conservative_trot'
    assert select_gait(
        (0.02, 0.0, 0.0), 0.1, 0.5,
        'conservative_crawl', 'conservative_trot') == 'conservative_crawl'
    assert select_gait(
        (0.02, 0.0, 0.0), 0.6, 0.5,
        'conservative_crawl', 'stance') == 'stance'
