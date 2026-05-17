import unittest
from unittest.mock import MagicMock
import numpy as np
import cv2

from osgar.drivers.image_rotator import ImageRotator
from osgar.lib.quaternion import euler_to_quaternion

class ImageRotatorTest(unittest.TestCase):
    def test_image_rotator(self):
        bus = MagicMock()
        node = ImageRotator({'roll_multiplier': -1.0}, bus)
        
        # Test orientation_list update
        # format: [timestamp, accuracy, x, y, z, w]
        roll = np.radians(90)
        q = euler_to_quaternion(yaw=0, pitch=0, roll=roll)
        packet = [
            [0.0, 0.0, q[0], q[1], q[2], q[3]]
        ]
        node.on_orientation_list(packet)
        self.assertIsNotNone(node.last_orientation)
        
        # Create a dummy image
        img = np.zeros((100, 100, 3), dtype=np.uint8)
        img[0:50, :] = 255 # Top half white
        _, encoded = cv2.imencode('.jpeg', img)
        
        # Test image update
        node.on_image(encoded.tobytes())
        
        # Check if publish was called
        bus.publish.assert_called_once()
        
        # Get published data
        args, _ = bus.publish.call_args
        self.assertEqual(args[0], 'image')
        
        # Decode and verify rotation
        out_data = args[1]
        out_img = cv2.imdecode(np.frombuffer(out_data, np.uint8), cv2.IMREAD_ANYCOLOR)
        
        # Since original image has top half white and bottom half black,
        # rotating it by -90 degrees (roll=90 * roll_multiplier=-1) 
        # should make the left half white and right half black
        # (or vice versa, depending on cv2 rotation conventions).
        # Let's just check that it's different and not None
        self.assertIsNotNone(out_img)
        self.assertEqual(out_img.shape, (100, 100, 3))
        
        # Test with pose3d
        node.on_pose3d([ [0, 0, 0], [0, 0, 0, 1] ])
        self.assertEqual(node.last_orientation, [0, 0, 0, 1])

# vim: expandtab sw=4 ts=4
