import sys
from ourrosbag import load_config, BagReader, MessageParser, Player, Publisher
from ourrosbag.recorder import Recorder


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