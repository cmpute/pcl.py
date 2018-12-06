from ._test_types import test_PointXYZI_size
import unittest

class TestWrapper(unittest.TestCase):
    def test_types(self):
        assert test_PointXYZI_size()
