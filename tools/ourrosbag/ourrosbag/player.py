import time
import threading
from .config import BagConfig
from .reader import BagReader
from .parser import MessageParser


class Player:
    def __init__(self, config: BagConfig):
        self.speed = config.playback.speed
        self.loop = config.playback.loop
        self.start_time = config.playback.start_time
        self.end_time = config.playback.end_time

        # pause/resume gate — set = running, cleared = paused
        self._resume_event = threading.Event()
        self._resume_event.set()
        self._pause_start = None    # monotonic time when pause began
        self._pause_total = 0.0     # total seconds spent paused

    @property
    def is_paused(self) -> bool:
        return not self._resume_event.is_set()

    def pause(self):
        """Pause playback. The generator will block until resume() is called."""
        if not self.is_paused:
            self._pause_start = time.monotonic()
            self._resume_event.clear()
            print("[Player] Paused. Press 'p' to resume.")

    def resume(self):
        """Resume playback. Wall-clock offset is adjusted so messages
        don't burst-fire to catch up."""
        if self.is_paused and self._pause_start is not None:
            self._pause_total += time.monotonic() - self._pause_start
            self._pause_start = None
        self._resume_event.set()
        print("[Player] Resumed.")

    def toggle_pause(self):
        """Toggle between paused and running states."""
        if self.is_paused:
            self.resume()
        else:
            self.pause()

    def run(self, reader: BagReader, parser: MessageParser):
        while True:
            yield from self._play_once(reader, parser)
            if not self.loop:
                break
            print("[Player] Looping...")

    def _play_once(self, reader: BagReader, parser: MessageParser):
        wall_start = None
        bag_start = None

        for topic, msgtype, timestamp, msg in reader.messages():
            # block while paused
            self._resume_event.wait()

            # timestamp is in nanoseconds
            bag_ts_sec = timestamp / 1e9

            # skip messages before start_time
            if bag_ts_sec < self.start_time:
                continue

            # stop at end_time if set
            if self.end_time and bag_ts_sec > self.end_time:
                print("[Player] Reached end_time, stopping.")
                break

            # sync wall clock to bag clock on first message
            if wall_start is None:
                wall_start = time.monotonic()
                bag_start = bag_ts_sec
                self._pause_total = 0.0

            # compute how long we should wait, subtracting time spent paused
            bag_elapsed = bag_ts_sec - bag_start
            wall_elapsed = (time.monotonic() - wall_start) - self._pause_total
            sleep_time = (bag_elapsed / self.speed) - wall_elapsed

            if sleep_time > 0:
                time.sleep(sleep_time)

            parsed = parser.parse(topic, msgtype, timestamp, msg)
            yield parsed