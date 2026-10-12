from __future__ import annotations

import socket
import unittest

from protocol import HEADER, MAGIC, VERSION, newest_jpeg_offset, receive_frame


class ProtocolTest(unittest.TestCase):
    def test_receives_a_fragmented_frame(self) -> None:
        sender, receiver = socket.socketpair()
        self.addCleanup(sender.close)
        self.addCleanup(receiver.close)

        jpeg = b"\xff\xd8pretend-jpeg\xff\xd9"
        packet = HEADER.pack(
            MAGIC, VERSION, HEADER.size, 42, 123456, len(jpeg), 640, 480, 7
        ) + jpeg

        # Tiny writes reproduce the packet fragmentation TCP is allowed to do.
        for start in range(0, len(packet), 3):
            sender.sendall(packet[start : start + 3])

        frame = receive_frame(receiver)
        self.assertEqual(frame.sequence, 42)
        self.assertEqual((frame.width, frame.height), (640, 480))
        self.assertEqual(frame.dropped_on_camera, 7)
        self.assertEqual(frame.jpeg, jpeg)


class MergedPayloadTest(unittest.TestCase):
    def test_a_single_jpeg_is_decoded_from_the_start(self) -> None:
        self.assertEqual(newest_jpeg_offset(b"\xff\xd8\xff\xe0oldest\xff\xd9"), 0)

    def test_the_newest_of_several_glued_jpegs_is_chosen(self) -> None:
        """The camera driver can deliver several frames glued end to end; the
        decoder must be pointed at the last one, not the first."""
        oldest = b"\xff\xd8\xff\xe0oldest\xff\xd9"
        middle = b"\xff\xd8\xff\xe0middle\xff\xd9"
        newest = b"\xff\xd8\xff\xe0newest\xff\xd9"
        payload = oldest + middle + newest
        self.assertEqual(payload[newest_jpeg_offset(payload):], newest)

    def test_stuffed_ff_bytes_are_not_mistaken_for_a_frame_start(self) -> None:
        jpeg = b"\xff\xd8\xff\xe0" + b"\xff\x00\xd8\xff\x00" * 4 + b"\xff\xd9"
        self.assertEqual(newest_jpeg_offset(jpeg), 0)


if __name__ == "__main__":
    unittest.main()
