from types import SimpleNamespace
from unittest.mock import MagicMock, Mock, patch
import pytest

from frontier_explorer.coverage_manager import CoverageManager


@pytest.fixture
def manager():
    """Create a CoverageManager instance without running full Node __init__."""
    mgr = CoverageManager.__new__(CoverageManager)
    mgr._publish_status = Mock()
    mgr._on_clear_zone = Mock()
    mgr._cancel_active_goals = Mock()
    mgr._clear_cached_state = Mock()
    mgr.get_logger = Mock(return_value=Mock())
    mgr._slam_reset_client = Mock()
    mgr._slam_deserialize_client = Mock()
    return mgr


def test_slam_reset_success(manager):
    future = Mock()
    manager._slam_reset_client.wait_for_service.return_value = True
    manager._slam_reset_client.call_async.return_value = future

    manager._call_slam_reset()

    manager._slam_reset_client.wait_for_service.assert_called_once_with(timeout_sec=2.0)
    manager._slam_reset_client.call_async.assert_called_once()
    future.add_done_callback.assert_called_once()

    # Simulate callback with success (result == 0)
    cb = future.add_done_callback.call_args[0][0]
    cb_fut = Mock()
    cb_fut.result.return_value = SimpleNamespace(result=0)
    cb(cb_fut)

    manager._on_clear_zone.assert_called_once()
    manager._publish_status.assert_called_with('idle', 'Map and SLAM state reset successfully.')


def test_slam_reset_error_code(manager):
    future = Mock()
    manager._slam_reset_client.wait_for_service.return_value = True
    manager._slam_reset_client.call_async.return_value = future

    manager._call_slam_reset()

    cb = future.add_done_callback.call_args[0][0]
    cb_fut = Mock()
    cb_fut.result.return_value = SimpleNamespace(result=1)  # Non-zero error
    cb(cb_fut)

    manager._on_clear_zone.assert_not_called()
    manager._publish_status.assert_called_with('error', 'SLAM reset failed with code 1.')


def test_slam_reset_unavailable(manager):
    manager._slam_reset_client.wait_for_service.return_value = False

    manager._call_slam_reset()

    manager._slam_reset_client.call_async.assert_not_called()
    manager._publish_status.assert_called_with('error', 'SLAM reset service unavailable.')


def test_slam_deserialize_success(manager):
    future = Mock()
    manager._slam_deserialize_client.wait_for_service.return_value = True
    manager._slam_deserialize_client.call_async.return_value = future

    manager._call_slam_deserialize('/ros2_ws/map_serialized')

    manager._slam_deserialize_client.call_async.assert_called_once()
    cb = future.add_done_callback.call_args[0][0]
    cb_fut = Mock()
    cb_fut.result.return_value = SimpleNamespace(result=0)
    cb(cb_fut)

    manager._on_clear_zone.assert_called_once()
    manager._publish_status.assert_called_with('idle', 'Saved map reloaded successfully.')


def test_slam_deserialize_failure_falls_back_to_reset(manager):
    future = Mock()
    manager._slam_deserialize_client.wait_for_service.return_value = True
    manager._slam_deserialize_client.call_async.return_value = future
    manager._call_slam_reset = Mock()

    manager._call_slam_deserialize('/ros2_ws/map_serialized')

    cb = future.add_done_callback.call_args[0][0]
    cb_fut = Mock()
    cb_fut.result.return_value = SimpleNamespace(result=3)  # RESULT_FAILED_TO_READ_FILE
    cb(cb_fut)

    manager._call_slam_reset.assert_called_once()


def test_slam_deserialize_unavailable_falls_back_to_reset(manager):
    manager._slam_deserialize_client.wait_for_service.return_value = False
    manager._call_slam_reset = Mock()

    manager._call_slam_deserialize('/ros2_ws/map_serialized')

    manager._slam_deserialize_client.call_async.assert_not_called()
    manager._call_slam_reset.assert_called_once()


@patch('os.path.isfile')
def test_restart_slam_cancels_goals_and_clears_state(mock_isfile, manager):
    mock_isfile.return_value = False
    manager._call_slam_reset = Mock()

    manager._restart_slam()

    manager._cancel_active_goals.assert_called_once()
    manager._clear_cached_state.assert_called_once()
    manager._call_slam_reset.assert_called_once()


@patch('os.path.isfile')
def test_restart_slam_deserializes_when_map_exists(mock_isfile, manager):
    mock_isfile.side_effect = lambda path: path.endswith('.posegraph')
    manager._call_slam_deserialize = Mock()

    manager._restart_slam()

    manager._cancel_active_goals.assert_called_once()
    manager._clear_cached_state.assert_called_once()
    manager._call_slam_deserialize.assert_called_once_with('/ros2_ws/map_serialized')


def test_reset_map_cancels_goals_and_clears_state(manager):
    manager._call_slam_reset = Mock()

    with patch('os.path.isfile', return_value=False):
        manager._reset_map()

    manager._cancel_active_goals.assert_called_once()
    manager._clear_cached_state.assert_called_once()
    manager._call_slam_reset.assert_called_once()
