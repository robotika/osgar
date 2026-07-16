import unittest
from unittest.mock import MagicMock, patch
import numpy as np
from datetime import timedelta
from pygame.locals import QUIT, KEYDOWN, K_q

# Import target function
from osgar.tools.lidarview import lidarview, Frame

class LidarviewTest(unittest.TestCase):

    @patch('osgar.tools.lidarview.pygame')
    @patch('osgar.tools.lidarview.cv2')
    def test_lidarview_resolution_assertion(self, mock_cv2, mock_pygame):
        # Setup mock video writer
        mock_writer = MagicMock()
        mock_cv2.VideoWriter.return_value = mock_writer
        mock_cv2.VideoWriter_fourcc.return_value = 1234
        
        # Setup mock screen and surfaces
        mock_screen = MagicMock()
        mock_screen.get_size.return_value = (1600, 1000)
        mock_pygame.display.set_mode.return_value = mock_screen
        
        mock_surface = MagicMock()
        mock_surface.get_size.return_value = (1600, 1000)
        mock_pygame.Surface.return_value = mock_surface
        
        # Mock surfarray.array3d to return a frame with mismatched size
        # e.g., if pygame surface has size (800, 600), transpose makes it (600, 800)
        # but VideoWriter expects (1600, 1000)
        mismatched_view = np.zeros((800, 600, 3), dtype=np.uint8)
        mock_pygame.surfarray.array3d.return_value = mismatched_view
        mock_cv2.cvtColor.return_value = mismatched_view.transpose([1, 0, 2]) # (600, 800, 3)

        # Mock generator (history)
        mock_gen = MagicMock()
        dummy_frame = Frame()
        dummy_frame.lidar_up = None
        dummy_frame.lidar_down = None
        
        # history.next() returns: timestamp, frame, pose, pose3d, scan, scan2, image, image2, bbox, joint, keyframe, title, eof
        mock_gen.next.return_value = (
            timedelta(seconds=1), dummy_frame, (0, 0, 0), None, [], [], None, None, [], None, False, ["Test"], False
        )
        
        # Run lidarview and expect AssertionError
        with self.assertRaises(AssertionError) as ctx:
            lidarview(mock_gen, "dummy_caption", callback=None, out_video="dummy.mp4")
            
        self.assertIn("does not match video writer size", str(ctx.exception))
        
        # On crash, VideoWriter release is not expected to be called explicitly
        mock_writer.release.assert_not_called()

    @patch('osgar.tools.lidarview.pygame')
    @patch('osgar.tools.lidarview.cv2')
    def test_lidarview_writer_released_on_quit(self, mock_cv2, mock_pygame):
        # Setup mock video writer
        mock_writer = MagicMock()
        mock_cv2.VideoWriter.return_value = mock_writer
        mock_cv2.VideoWriter_fourcc.return_value = 1234
        
        # Setup mock screen and surfaces
        mock_screen = MagicMock()
        mock_screen.get_size.return_value = (1600, 1000)
        mock_pygame.display.set_mode.return_value = mock_screen
        
        mock_surface = MagicMock()
        mock_surface.get_size.return_value = (1600, 1000)
        mock_pygame.Surface.return_value = mock_surface
        
        # Mock surfarray.array3d to return a matching frame size
        matching_view = np.zeros((1600, 1000, 3), dtype=np.uint8)
        mock_pygame.surfarray.array3d.return_value = matching_view
        mock_cv2.cvtColor.return_value = matching_view.transpose([1, 0, 2]) # (1000, 1600, 3)

        # Mock generator (history)
        mock_gen = MagicMock()
        dummy_frame = Frame()
        dummy_frame.lidar_up = None
        dummy_frame.lidar_down = None
        
        mock_gen.next.return_value = (
            timedelta(seconds=1), dummy_frame, (0, 0, 0), None, [], [], None, None, [], None, False, ["Test"], False
        )
        
        # Mock event poll to return QUIT event on first call
        mock_event = MagicMock()
        mock_event.type = QUIT
        mock_pygame.event.poll.return_value = mock_event
        
        # Run lidarview - should return normally
        lidarview(mock_gen, "dummy_caption", callback=None, out_video="dummy.mp4")
        
        # Ensure that writer.release() was called!
        mock_writer.release.assert_called_once()
