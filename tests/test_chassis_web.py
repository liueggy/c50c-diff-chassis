"""Host-only safety tests for the chassis web dashboard."""

import json
from pathlib import Path
import sys
import threading
import unittest
from urllib.error import HTTPError
from urllib.request import Request, urlopen


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import chassis_web  # noqa: E402


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


class FakeSerial:
    def __init__(self):
        self.writes = []

    def write(self, data):
        self.writes.append(bytes(data))
        return len(data)


class FailingReadSerial:
    def __init__(self, stop_event):
        self.stop_event = stop_event

    def read(self, _size):
        self.stop_event.set()
        raise OSError("serial disconnected")


def telemetry(status=chassis_web.STATUS_ENABLED | 1):
    frame = bytearray(chassis_web.TELEM_SIZE)
    frame[0] = chassis_web.TELEM_HEADER
    frame[1] = status
    frame[21] = chassis_web.TELEM_TAIL
    checksum = 0
    for value in frame[:20]:
        checksum ^= value
    frame[20] = checksum
    return bytes(frame)


class DashboardServer:
    def __init__(self, state):
        self.server = chassis_web.ThreadingHTTPServer(
            ("127.0.0.1", 0), chassis_web.make_handler(state)
        )
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.base = "http://127.0.0.1:%d" % self.server.server_address[1]

    def close(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=1.0)

    def get(self, path):
        with urlopen(self.base + path, timeout=1.0) as response:
            return response.status, response.read()

    def post(self, path, payload=None):
        body = b"" if payload is None else json.dumps(payload).encode("utf-8")
        request = Request(
            self.base + path,
            data=body,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        try:
            with urlopen(request, timeout=1.0) as response:
                return response.status, json.loads(response.read())
        except HTTPError as error:
            try:
                return error.code, json.loads(error.read())
            finally:
                error.close()


class ChassisWebSafetyTests(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.state = chassis_web.State(clock=self.clock)
        self.state.apply_telemetry(telemetry())
        self.http = DashboardServer(self.state)

    def tearDown(self):
        self.http.close()

    def test_initial_state_poll_does_not_block_first_operation(self):
        clock = FakeClock()
        state = chassis_web.State(clock=clock)
        self.assertTrue(state.snapshot()["telemetry_stale"])
        state.apply_telemetry(telemetry(
            chassis_web.STATUS_ARMED | chassis_web.STATUS_STOP |
            chassis_web.STATUS_ENABLED
        ))
        self.assertTrue(state.set_command(100, 0))
        self.assertEqual(state.command_for_write(), (100, 0))

    def test_operation_lease_expires_and_state_polling_does_not_renew_it(self):
        status, result = self.http.post("/cmd", {"vx": 200, "wz": -300})
        self.assertEqual((status, result["ok"]), (200, True))

        for _ in range(4):
            self.clock.advance(0.1)
            self.state.apply_telemetry(telemetry())
            _, body = self.http.get("/state")
            snapshot = json.loads(body)
            self.assertEqual((snapshot["vx"], snapshot["wz"]), (200, -300))

        self.clock.advance(0.11)
        self.state.apply_telemetry(telemetry())
        _, body = self.http.get("/state")
        snapshot = json.loads(body)
        self.assertEqual((snapshot["vx"], snapshot["wz"]), (0, 0))

    def test_late_renewal_requires_release_even_before_writer_checks_timeout(self):
        self.http.post("/cmd", {"vx": 200, "wz": 0})
        self.clock.advance(0.51)
        self.state.apply_telemetry(telemetry())
        status, _ = self.http.post("/cmd", {"vx": 200, "wz": 0})
        self.assertEqual(status, 409)
        self.assertEqual(self.state.command_for_write(), (0, 0))
        self.http.post("/stop")
        status, _ = self.http.post("/cmd", {"vx": 200, "wz": 0})
        self.assertEqual(status, 200)

    def test_invalid_commands_are_rejected_and_clear_the_old_command(self):
        self.http.post("/cmd", {"vx": 100, "wz": 100})
        status, result = self.http.post("/cmd", {"vx": float("nan"), "wz": 0})
        self.assertEqual(status, 400)
        self.assertFalse(result["ok"])
        self.assertEqual(self.state.command_for_write(), (0.0, 0.0))

        self.http.post("/cmd", {"vx": 100, "wz": 100})
        status, result = self.http.post("/cmd", {"vx": 701, "wz": 0})
        self.assertEqual(status, 400)
        self.assertFalse(result["ok"])
        self.assertEqual(self.state.command_for_write(), (0.0, 0.0))

    def test_page_stop_endpoint_and_lifecycle_handlers_clear_motion(self):
        _, page = self.http.get("/")
        html = page.decode("utf-8")
        self.assertIn("setInterval(send,100)", html)
        self.assertIn("visibilitychange", html)
        self.assertIn("pagehide", html)
        self.assertIn("beforeunload", html)
        self.assertIn("[\"ARMED\",\"已解锁\",0,0]", html)

        self.http.post("/cmd", {"vx": -200, "wz": 500})
        status, result = self.http.post("/stop")
        self.assertEqual((status, result["ok"]), (200, True))
        self.assertEqual(self.state.command_for_write(), (0.0, 0.0))

    def test_serial_fault_and_telemetry_timeout_do_not_replay_old_command(self):
        self.http.post("/cmd", {"vx": 300, "wz": 0})
        self.state.serial_failed()
        self.assertEqual(self.state.command_for_write(), (0.0, 0.0))

        self.state.apply_telemetry(telemetry())
        self.assertEqual(self.state.command_for_write(), (0.0, 0.0))
        status, _ = self.http.post("/cmd", {"vx": 300, "wz": 0})
        self.assertEqual(status, 409)
        self.http.post("/stop")

        self.http.post("/cmd", {"vx": 300, "wz": 0})
        self.clock.advance(chassis_web.TELEMETRY_STALE_SECONDS)
        self.assertEqual(self.state.command_for_write(), (0.0, 0.0))
        self.state.apply_telemetry(telemetry())
        self.assertEqual(self.state.command_for_write(), (0.0, 0.0))
        status, _ = self.http.post("/cmd", {"vx": 300, "wz": 0})
        self.assertEqual(status, 409)
        self.http.post("/stop")
        status, result = self.http.post("/cmd", {"vx": 300, "wz": 0})
        self.assertEqual((status, result["ok"]), (200, True))

    def test_reader_serial_exception_clears_motion(self):
        self.state.set_command(250, 10)
        stop_event = threading.Event()
        chassis_web.reader_loop(
            FailingReadSerial(stop_event), self.state, False, stop_event
        )
        self.assertTrue(self.state.serial_fault)
        self.assertEqual(self.state.command_for_write(), (0.0, 0.0))

    def test_armed_enabled_stop_state_allows_a_new_motion_operation(self):
        stopped_ready = (chassis_web.STATUS_ARMED | chassis_web.STATUS_STOP |
                         chassis_web.STATUS_ENABLED)
        self.state.apply_telemetry(telemetry(stopped_ready))
        status, result = self.http.post("/cmd", {"vx": 150, "wz": 20})
        self.assertEqual((status, result["ok"]), (200, True))
        self.assertEqual(self.state.command_for_write(), (150.0, 20.0))

    def test_fault_or_disabled_feedback_clears_motion_and_requires_release(self):
        faults = [
            chassis_web.STATUS_TIMEOUT,
            chassis_web.STATUS_RX_ERROR,
            chassis_web.STATUS_WAIT,
            chassis_web.STATUS_LOW_VOLTAGE,
            chassis_web.STATUS_SELF_ERROR,
        ]
        for fault in faults:
            self.http.post("/stop")
            self.state.apply_telemetry(telemetry())
            self.http.post("/cmd", {"vx": 150, "wz": 20})
            self.state.apply_telemetry(telemetry(
                chassis_web.STATUS_ARMED | chassis_web.STATUS_ENABLED | fault
            ))
            self.assertEqual(self.state.command_for_write(), (0.0, 0.0))
            self.state.apply_telemetry(telemetry())
            status, _ = self.http.post("/cmd", {"vx": 150, "wz": 20})
            self.assertEqual(status, 409)

        self.http.post("/stop")
        self.state.apply_telemetry(telemetry())
        self.http.post("/cmd", {"vx": 150, "wz": 20})
        self.state.apply_telemetry(telemetry(chassis_web.STATUS_ARMED))
        self.assertEqual(self.state.command_for_write(), (0.0, 0.0))
        self.state.apply_telemetry(telemetry())
        status, _ = self.http.post("/cmd", {"vx": 150, "wz": 20})
        self.assertEqual(status, 409)
        self.http.post("/stop")
        status, result = self.http.post("/cmd", {"vx": 150, "wz": 20})
        self.assertEqual((status, result["ok"]), (200, True))

    def test_shutdown_writer_sends_only_a_finite_zero_burst(self):
        serial = FakeSerial()
        stop_event = threading.Event()
        stop_event.set()
        chassis_web.writer_loop(serial, self.state, False, stop_event, period=0.0)
        self.assertEqual(
            serial.writes,
            [chassis_web.build_command(0, 0)] * chassis_web.SHUTDOWN_ZERO_FRAMES,
        )


def owner_frame(owner=4, status=1):
    body = bytes((0x5B, 1, owner, status))
    checksum = 0
    for value in body:
        checksum ^= value
    return body + bytes((checksum, 0xA6))


class BluetoothOwnerTests(unittest.TestCase):
    def test_mixed_fragmented_stream_and_bad_checksum_resync(self):
        state = chassis_web.State(expected_source=4)
        corrupt = bytearray(owner_frame())
        corrupt[4] ^= 1
        stream = b"junk" + corrupt + telemetry() + owner_frame()
        buffer = bytearray()
        for value in stream:
            buffer.append(value)
            chassis_web.consume_feedback(buffer, state)
        self.assertEqual(buffer, bytearray())
        self.assertEqual(state.rx_ok, 1)
        self.assertEqual(state.owner, 4)
        self.assertTrue(state.control_ready)
        self.assertTrue(state.set_command(100, 0))
        # The next telemetry arrives before its owner frame: preserve fresh owner.
        buffer.extend(telemetry())
        chassis_web.consume_feedback(buffer, state)
        self.assertEqual(state.command_for_write(), (100, 0))

    def test_wrong_owner_and_expiry_lock_current_operation(self):
        clock = FakeClock()
        state = chassis_web.State(clock=clock, expected_source=4)
        state.apply_telemetry(telemetry())
        self.assertFalse(state.control_ready)
        state.apply_owner(owner_frame())
        self.assertTrue(state.set_command(100, 0))
        state.apply_owner(owner_frame(3))
        self.assertEqual(state.command_for_write(), (0, 0))
        state.apply_owner(owner_frame())
        self.assertFalse(state.set_command(100, 0))
        state.clear_command(release=True)
        self.assertTrue(state.set_command(100, 0))
        clock.advance(0.5)
        state.apply_telemetry(telemetry())
        self.assertEqual(state.command_for_write(), (0, 0))
        state.apply_owner(owner_frame())
        self.assertFalse(state.set_command(100, 0))
        state.clear_command(release=True)
        self.assertTrue(state.set_command(100, 0))

    def test_stopped_armed_owner_allows_new_operation(self):
        state = chassis_web.State(expected_source=4)
        state.apply_telemetry(telemetry(chassis_web.STATUS_ENABLED |
                                      chassis_web.STATUS_ARMED | chassis_web.STATUS_STOP))
        state.apply_owner(owner_frame())
        self.assertTrue(state.set_command(100, 0))
        self.assertEqual(state.snapshot()["owner"], 4)

    def test_startup_wait_does_not_latch_an_operation_that_never_started(self):
        state = chassis_web.State(expected_source=4)
        state.apply_telemetry(telemetry(chassis_web.STATUS_ENABLED | chassis_web.STATUS_WAIT))
        state.apply_owner(owner_frame(0, 0))
        state.apply_telemetry(telemetry())
        state.apply_owner(owner_frame())
        self.assertTrue(state.set_command(100, 0))
        state.apply_owner(owner_frame(4, 4))
        self.assertEqual(state.command_for_write(), (0, 0))

    def test_sim_sets_expected_owner(self):
        state = chassis_web.State(sim=True, expected_source=4)
        class OneIteration:
            stopped = False
            def is_set(self):
                return self.stopped
            def wait(self, _seconds):
                self.stopped = True
        chassis_web.reader_loop(None, state, True, OneIteration())
        self.assertEqual(state.snapshot()["owner"], 4)
        self.assertTrue(state.set_command(100, 0))


if __name__ == "__main__":
    unittest.main()
