import sys
import unittest
from pathlib import Path


PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

import config
from chassis_link import ChassisLink, encode_target_packet
from vision import letterbox_param, map_target_center, nms_detections, select_target


class FakeUart:
    def __init__(self):
        self.writes = []
        self.closed = False

    def write(self, payload):
        self.writes.append(payload)

    def deinit(self):
        self.closed = True


class RuntimeLogicTest(unittest.TestCase):
    def test_motion_gates_are_disabled(self):
        self.assertFalse(config.CHASSIS_LINK_ENABLED)
        self.assertFalse(config.CAN_ENABLED)
        self.assertFalse(config.GIMBAL_MOTION_ENABLED)

    def test_target_packet_clamps_to_contract(self):
        self.assertEqual(encode_target_packet(True, -2, 999), "@1,000,239#")
        self.assertEqual(encode_target_packet(False, 200, 120), "@0,200,120#")

    def test_link_keeps_last_coordinates_on_target_loss(self):
        uart = FakeUart()
        link = ChassisLink(uart)
        self.assertEqual(link.publish(True, 203, 117), (203, 117, True))
        self.assertEqual(link.publish(False), (203, 117, False))
        self.assertEqual(uart.writes, ["@1,203,117#", "@0,203,117#"])
        link.close()
        self.assertTrue(uart.closed)

    def test_letterbox_matches_accepted_pipeline(self):
        self.assertEqual(
            letterbox_param((640, 384), (320, 320)),
            (64, 64, 0, 0, 0.5),
        )

    def test_coordinate_mapping_preserves_center_and_bounds(self):
        self.assertEqual(map_target_center(320, 192), (200, 120))
        self.assertEqual(map_target_center(0, 0), (0, 0))
        self.assertEqual(map_target_center(640, 384), (399, 239))

    def test_highest_confidence_detection_is_published(self):
        detections = [
            [0, 0, 100, 100, 0.50, 0],
            [270, 142, 370, 242, 0.90, 0],
        ]
        self.assertEqual(select_target(detections), (200, 120))

    def test_nms_suppresses_only_same_class_overlap(self):
        detections = [
            [0, 0, 100, 100, 0.90, 0],
            [5, 5, 95, 95, 0.80, 0],
            [5, 5, 95, 95, 0.70, 1],
        ]
        kept = nms_detections(detections, 0.45)
        self.assertEqual(len(kept), 2)
        self.assertEqual([item[5] for item in kept], [0, 1])


if __name__ == "__main__":
    unittest.main()
