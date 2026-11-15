import json
import socket
import unittest

from agrisarthi_gps_nav.esp32_link import ESP32Link


class ESP32LinkTests(unittest.TestCase):
    def setUp(self):
        self.peer = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.peer.bind(("127.0.0.1", 0))
        self.peer.settimeout(1.0)
        self.link = ESP32Link("127.0.0.1", self.peer.getsockname()[1],
                              ack_timeout_s=0.6, session_id="test-session-1")

    def tearDown(self):
        self.link.sock.close()
        self.peer.close()

    def recv_packet(self):
        raw, address = self.peer.recvfrom(2048)
        return json.loads(raw), address

    def test_handshake_command_ack_and_timeout(self):
        self.assertTrue(self.link.send_hello(now=1.0))
        hello, client = self.recv_packet()
        self.assertEqual(hello["type"], "hello")
        self.peer.sendto(json.dumps({"type": "hello_ack", "session_id": hello["session_id"]}).encode(), client)
        self.link.poll(now=1.01)
        self.assertTrue(self.link.ready(now=1.02))

        seq = self.link.send_command({"enable": True, "left": 0.2, "right": 0.1}, now=1.03)
        command, client = self.recv_packet()
        self.assertEqual(command["seq"], seq)
        self.assertEqual(command["session_id"], "test-session-1")
        previous_ack = self.link.last_ack
        self.peer.sendto(json.dumps({"type": "ack", "session_id": command["session_id"],
                                     "seq": seq + 1, "accepted": True}).encode(), client)
        self.link.poll(now=1.035)
        self.assertEqual(self.link.last_ack, previous_ack)
        self.peer.sendto(json.dumps({"type": "ack", "session_id": command["session_id"],
                                     "seq": seq, "accepted": True}).encode(), client)
        self.link.poll(now=1.04)
        self.assertEqual(self.link.last_ack_seq, seq)
        self.assertTrue(self.link.ready(now=1.05))
        self.assertFalse(self.link.ready(now=1.7))

    def test_new_pi_process_gets_new_session_and_starts_sequence_at_one(self):
        replacement = ESP32Link("127.0.0.1", self.peer.getsockname()[1], session_id="test-session-2")
        try:
            self.assertNotEqual(replacement.session_id, self.link.session_id)
            replacement.send_hello(now=1.0)
            hello, client = self.recv_packet()
            self.peer.sendto(json.dumps({"type": "hello_ack", "session_id": hello["session_id"]}).encode(), client)
            replacement.poll(now=1.01)
            self.assertEqual(replacement.send_command({"enable": False}, now=1.02), 1)
        finally:
            replacement.sock.close()

    def test_probe_ack_is_matched_even_after_a_later_motor_sequence_was_sent(self):
        self.link.send_hello(now=1.0)
        hello, client = self.recv_packet()
        self.peer.sendto(json.dumps({"type": "hello_ack", "session_id": hello["session_id"]}).encode(), client)
        self.link.poll(now=1.01)
        probe_seq = self.link.send_probe_command("PROBE_INSERT", "unique-probe-cmd", now=1.02)
        probe, client = self.recv_packet()
        self.assertEqual(probe["type"], "probe_cmd")
        self.assertEqual(probe["command_id"], "unique-probe-cmd")
        self.link.send_command({"enable": False}, now=1.03)
        self.recv_packet()  # later motor seq must not invalidate the probe ack
        self.peer.sendto(json.dumps({"type": "probe_ack", "session_id": probe["session_id"],
                                     "seq": probe_seq, "accepted": True,
                                     "command_id": "unique-probe-cmd", "probe_state": "INSERTING"}).encode(), client)
        received = self.link.poll(now=1.04)
        self.assertEqual(received[0]["command_id"], "unique-probe-cmd")
        self.assertTrue(self.link.ready(now=1.05))


if __name__ == "__main__":
    unittest.main()
