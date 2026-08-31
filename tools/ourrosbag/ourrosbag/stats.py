import time
from collections import defaultdict


class StatsTracker:
    def __init__(self):
        self.start_time = time.monotonic()
        self.msg_count = defaultdict(int)
        self.byte_count = defaultdict(int)
        self.parse_times = defaultdict(list)
        self.total = 0

    def record(self, msgtype: str, byte_size: int, parse_time_ns: float):
        self.msg_count[msgtype] += 1
        self.byte_count[msgtype] += byte_size
        self.parse_times[msgtype].append(parse_time_ns)
        self.total += 1

    def summary(self):
        elapsed = time.monotonic() - self.start_time
        print("\n" + "="*60)
        print(f"  ourrosbag session summary")
        print("="*60)
        print(f"  total messages : {self.total}")
        print(f"  elapsed time   : {elapsed:.2f}s")
        print(f"  throughput     : {self.total / elapsed:.1f} msg/sec")
        print()
        for msgtype, count in self.msg_count.items():
            short = msgtype.split("/")[-1]
            times = self.parse_times[msgtype]
            avg_us = (sum(times) / len(times)) / 1000
            mb = self.byte_count[msgtype] / 1e6
            print(f"  [{short}]")
            print(f"    messages     : {count}")
            print(f"    data         : {mb:.2f} MB")
            print(f"    avg parse    : {avg_us:.2f} µs")
        print("="*60)