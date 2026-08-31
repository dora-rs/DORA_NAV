"""Tests for ourrosbag.player — Player timing, loop, range, and pause/resume."""
import unittest
from unittest.mock import MagicMock, patch
import threading
import time
from dataclasses import dataclass, field


class MockBagConfig:
    """Minimal config stand-in for Player.__init__."""
    def __init__(self, speed=1.0, loop=False, start_time=0.0, end_time=None):
        self.playback = MagicMock()
        self.playback.speed = speed
        self.playback.loop = loop
        self.playback.start_time = start_time
        self.playback.end_time = end_time


def _fake_reader(messages):
    """Returns a mock BagReader whose .messages() yields the given list."""
    reader = MagicMock()
    reader.messages.return_value = iter(messages)
    return reader


def _fake_parser():
    """Returns a mock MessageParser that passes data through."""
    parser = MagicMock()
    parser.parse.side_effect = lambda topic, msgtype, ts, msg: {
        "topic": topic, "msgtype": msgtype, "timestamp": ts, "data": msg,
    }
    return parser


# ── tests ─────────────────────────────────────────────────────────────────────

class TestTimeRange(unittest.TestCase):
    def test_start_time_skips_early_messages(self):
        from ourrosbag.player import Player
        config = MockBagConfig(speed=1000, start_time=2.0)
        player = Player(config)

        messages = [
            ("/t", "m", 1_000_000_000, "a"),   # 1.0s — should be skipped
            ("/t", "m", 2_000_000_000, "b"),   # 2.0s — first yielded
            ("/t", "m", 3_000_000_000, "c"),   # 3.0s
        ]
        reader = _fake_reader(messages)
        results = list(player.run(reader, _fake_parser()))
        self.assertEqual(len(results), 2)
        self.assertEqual(results[0]["data"], "b")

    def test_end_time_stops_playback(self):
        from ourrosbag.player import Player
        config = MockBagConfig(speed=1000, end_time=2.5)
        player = Player(config)

        messages = [
            ("/t", "m", 1_000_000_000, "a"),
            ("/t", "m", 2_000_000_000, "b"),
            ("/t", "m", 3_000_000_000, "c"),   # > 2.5s — should not appear
        ]
        reader = _fake_reader(messages)
        results = list(player.run(reader, _fake_parser()))
        self.assertEqual(len(results), 2)


class TestLoop(unittest.TestCase):
    def test_loop_replays(self):
        from ourrosbag.player import Player
        config = MockBagConfig(speed=1000, loop=True)
        player = Player(config)

        call_count = [0]
        messages = [("/t", "m", 1_000_000_000, "a")]

        def make_reader():
            r = MagicMock()
            def gen():
                call_count[0] += 1
                if call_count[0] <= 3:
                    yield from messages
                # stop yielding after 3 iterations to prevent infinite loop
            r.messages.return_value = gen()
            return r

        # collect up to 10 to be safe
        results = []
        reader_factory_calls = [0]
        reader = MagicMock()
        def messages_side_effect():
            reader_factory_calls[0] += 1
            if reader_factory_calls[0] <= 3:
                return iter(messages)
            return iter([])
        reader.messages.side_effect = messages_side_effect

        for msg in player.run(reader, _fake_parser()):
            results.append(msg)
            if len(results) >= 3:
                break

        self.assertEqual(len(results), 3)

    def test_no_loop_plays_once(self):
        from ourrosbag.player import Player
        config = MockBagConfig(speed=1000, loop=False)
        player = Player(config)

        messages = [("/t", "m", 1_000_000_000, "a")]
        reader = _fake_reader(messages)
        results = list(player.run(reader, _fake_parser()))
        self.assertEqual(len(results), 1)


class TestSpeed(unittest.TestCase):
    @patch("ourrosbag.player.time")
    def test_high_speed_reduces_sleep(self, mock_time):
        from ourrosbag.player import Player
        mock_time.monotonic.side_effect = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        mock_time.sleep = MagicMock()

        config = MockBagConfig(speed=10.0)
        player = Player(config)

        messages = [
            ("/t", "m", 0, "a"),
            ("/t", "m", 10_000_000_000, "b"),  # 10s in bag time
        ]
        reader = _fake_reader(messages)
        list(player.run(reader, _fake_parser()))

        # at 10x speed, 10s of bag time = 1s wall time
        if mock_time.sleep.called:
            sleep_arg = mock_time.sleep.call_args[0][0]
            self.assertLessEqual(sleep_arg, 1.1)


class TestPauseResume(unittest.TestCase):
    def test_pause_sets_flag(self):
        from ourrosbag.player import Player
        config = MockBagConfig(speed=1.0)
        player = Player(config)

        self.assertFalse(player.is_paused)
        player.pause()
        self.assertTrue(player.is_paused)
        player.resume()
        self.assertFalse(player.is_paused)

    def test_toggle(self):
        from ourrosbag.player import Player
        config = MockBagConfig(speed=1.0)
        player = Player(config)

        player.toggle_pause()
        self.assertTrue(player.is_paused)
        player.toggle_pause()
        self.assertFalse(player.is_paused)

    def test_pause_blocks_generator(self):
        from ourrosbag.player import Player
        config = MockBagConfig(speed=1000)
        player = Player(config)

        messages = [
            ("/t", "m", 1_000_000_000, "a"),
            ("/t", "m", 2_000_000_000, "b"),
        ]
        reader = _fake_reader(messages)
        gen = player.run(reader, _fake_parser())

        # get first message
        first = next(gen)
        self.assertEqual(first["data"], "a")

        # pause, then schedule resume after 200ms
        player.pause()
        resumed = threading.Event()

        def resume_later():
            time.sleep(0.2)
            player.resume()
            resumed.set()

        threading.Thread(target=resume_later, daemon=True).start()

        # this should block until resume
        second = next(gen)
        self.assertEqual(second["data"], "b")
        self.assertTrue(resumed.is_set())

    def test_double_pause_is_safe(self):
        from ourrosbag.player import Player
        config = MockBagConfig(speed=1.0)
        player = Player(config)

        player.pause()
        player.pause()  # should not crash
        self.assertTrue(player.is_paused)

    def test_double_resume_is_safe(self):
        from ourrosbag.player import Player
        config = MockBagConfig(speed=1.0)
        player = Player(config)

        player.resume()
        player.resume()  # should not crash
        self.assertFalse(player.is_paused)


if __name__ == "__main__":
    unittest.main()
