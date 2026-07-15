import pytest

from geometry_msgs.msg import TransformStamped

from frontier_explorer.robot_pose_publisher import (
    pose_from_transform,
    validate_configuration,
)


def test_pose_from_transform_preserves_stamp_and_transform() -> None:
    transform = TransformStamped()
    transform.header.stamp.sec = 42
    transform.transform.translation.x = 1.5
    transform.transform.translation.y = -2.0
    transform.transform.translation.z = 0.25
    transform.transform.rotation.z = 0.5
    transform.transform.rotation.w = 0.866

    pose = pose_from_transform(transform, 'map')

    assert pose.header.frame_id == 'map'
    assert pose.header.stamp.sec == 42
    assert pose.pose.position.x == 1.5
    assert pose.pose.position.y == -2.0
    assert pose.pose.position.z == 0.25
    assert pose.pose.orientation.z == 0.5
    assert pose.pose.orientation.w == 0.866


@pytest.mark.parametrize(
    'arguments',
    [
        ('', 'base_link', '/robot_pose', 30.0, 1.0),
        ('map', '', '/robot_pose', 30.0, 1.0),
        ('map', 'base_link', '', 30.0, 1.0),
        ('map', 'base_link', '/robot_pose', 0.0, 1.0),
        ('map', 'base_link', '/robot_pose', 30.0, 0.0),
    ],
)
def test_invalid_configuration_is_rejected(arguments) -> None:
    with pytest.raises(ValueError):
        validate_configuration(*arguments)
