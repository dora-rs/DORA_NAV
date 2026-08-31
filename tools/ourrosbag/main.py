import sys
import threading
from ourrosbag import load_config, BagReader, MessageParser, Player, Publisher
from ourrosbag.recorder import Recorder


def _keyboard_listener(player: Player):
    """Daemon thread: listens for 'p' key to toggle pause/resume."""
    try:
        # Windows
        import msvcrt
        while True:
            ch = msvcrt.getwch()
            if ch.lower() == "p":
                player.toggle_pause()
    except ImportError:
        # Unix / macOS
        import tty
        import termios
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while True:
                ch = sys.stdin.read(1)
                if ch.lower() == "p":
                    player.toggle_pause()
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main():
    config = load_config("configs/default.yml")
    reader = BagReader(config)

    if "--discover" in sys.argv:
        topics = reader.get_available_topics()
        print("\n[ourrosbag] Available topics:")
        for topic, msgtype in topics.items():
            print(f"  {topic}  →  {msgtype}")
        return

    record_mode = "--record" in sys.argv
    recorder = Recorder("output/parquet_session") if record_mode else None

    parser = MessageParser(config)
    player = Player(config)
    publisher = Publisher(config)

    print(f"[ourrosbag] Loading: {config.bag_path}")
    print(f"[ourrosbag] Topics: {config.topics}")
    if record_mode:
        print(f"[ourrosbag] Record mode ON")
    print(f"[ourrosbag] Press 'p' to pause/resume playback.")

    # start keyboard listener as daemon thread
    kb_thread = threading.Thread(target=_keyboard_listener, args=(player,), daemon=True)
    kb_thread.start()

    try:
        for msg in player.run(reader, parser):
            publisher.send(msg)
            if recorder:
                recorder.record(msg)
    except KeyboardInterrupt:
        print("\n[ourrosbag] Stopped by user.")
    finally:
        parser.stats.summary()
        if recorder:
            recorder.close()


if __name__ == "__main__":
    main()