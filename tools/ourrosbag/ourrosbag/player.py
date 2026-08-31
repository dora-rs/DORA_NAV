import time
from .config import BagConfig
from .reader import BagReader
from .parser import MessageParser


class Player:
    def __init__(self, config: BagConfig):
        self.speed = config.playback.speed
        self.loop = config.playback.loop
        self.start_time = config.playback.start_time
        self.end_time = config.playback.end_time

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

            # compute how long we should wait
            bag_elapsed = bag_ts_sec - bag_start
            wall_elapsed = time.monotonic() - wall_start
            sleep_time = (bag_elapsed / self.speed) - wall_elapsed

            if sleep_time > 0:
                time.sleep(sleep_time)

            parsed = parser.parse(topic, msgtype, timestamp, msg)
            yield parsed