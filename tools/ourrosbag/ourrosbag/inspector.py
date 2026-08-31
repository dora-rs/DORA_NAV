import sys
import argparse
import numpy as np
import pyarrow.parquet as pq
from pathlib import Path


def load_parquet_session(session_dir: str) -> dict:
    d = Path(session_dir)
    tables = {}
    for f in d.glob("*.parquet"):
        import pandas as pd
        tables[f.stem] = pd.read_parquet(f)
        print(f"[Inspector] Loaded {f.name}: {len(tables[f.stem])} rows")
    return tables


def plot_imu(df):
    import matplotlib.pyplot as plt
    import matplotlib.gridspec as gridspec

    ts = (df["timestamp"] - df["timestamp"].iloc[0]) / 1e9

    av = np.array(df["angular_velocity"].tolist())
    la = np.array(df["linear_acceleration"].tolist())

    fig = plt.figure(figsize=(14, 8), facecolor="#0a0e14")
    fig.suptitle("IMU Data", color="#c8d8e8", fontsize=13, fontfamily="monospace")
    gs = gridspec.GridSpec(2, 1, hspace=0.4)

    ax1 = fig.add_subplot(gs[0])
    ax1.set_facecolor("#111720")
    ax1.plot(ts, av[:, 0], color="#00d4ff", linewidth=0.8, label="ω.x")
    ax1.plot(ts, av[:, 1], color="#39ff7a", linewidth=0.8, label="ω.y")
    ax1.plot(ts, av[:, 2], color="#ff6b35", linewidth=0.8, label="ω.z")
    ax1.set_title("Angular Velocity (rad/s)", color="#c8d8e8", fontsize=10)
    ax1.legend(facecolor="#111720", labelcolor="#c8d8e8", fontsize=9)
    ax1.tick_params(colors="#4a6070")
    for spine in ax1.spines.values():
        spine.set_edgecolor("#1e2a38")

    ax2 = fig.add_subplot(gs[1])
    ax2.set_facecolor("#111720")
    ax2.plot(ts, la[:, 0], color="#00d4ff", linewidth=0.8, label="a.x")
    ax2.plot(ts, la[:, 1], color="#39ff7a", linewidth=0.8, label="a.y")
    ax2.plot(ts, la[:, 2], color="#ff6b35", linewidth=0.8, label="a.z")
    ax2.set_title("Linear Acceleration (m/s²)", color="#c8d8e8", fontsize=10)
    ax2.set_xlabel("time (s)", color="#4a6070", fontsize=9)
    ax2.legend(facecolor="#111720", labelcolor="#c8d8e8", fontsize=9)
    ax2.tick_params(colors="#4a6070")
    for spine in ax2.spines.values():
        spine.set_edgecolor("#1e2a38")

    plt.show()


def plot_path(df):
    import matplotlib.pyplot as plt

    pos = np.array(df["position"].tolist())
    x, y = pos[:, 0], pos[:, 1]

    fig, ax = plt.subplots(figsize=(8, 8), facecolor="#0a0e14")
    ax.set_facecolor("#111720")
    fig.suptitle("Odometry Path", color="#c8d8e8", fontsize=13, fontfamily="monospace")

    # color path by time progression
    points = np.array([x, y]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)

    from matplotlib.collections import LineCollection
    from matplotlib.colors import LinearSegmentedColormap
    cmap = LinearSegmentedColormap.from_list("dora", ["#ff6b35", "#00d4ff"])
    lc = LineCollection(segments, cmap=cmap, linewidth=1.5, alpha=0.85)
    lc.set_array(np.linspace(0, 1, len(segments)))
    ax.add_collection(lc)

    ax.scatter(x[0],  y[0],  color="#39ff7a", s=60, zorder=5, label="start")
    ax.scatter(x[-1], y[-1], color="#00d4ff", s=60, zorder=5, label="end")

    ax.set_xlim(x.min() - 0.5, x.max() + 0.5)
    ax.set_ylim(y.min() - 0.5, y.max() + 0.5)
    ax.set_xlabel("x (m)", color="#4a6070", fontsize=9)
    ax.set_ylabel("y (m)", color="#4a6070", fontsize=9)
    ax.tick_params(colors="#4a6070")
    ax.legend(facecolor="#111720", labelcolor="#c8d8e8", fontsize=9)
    for spine in ax.spines.values():
        spine.set_edgecolor("#1e2a38")

    ax.set_aspect("equal")
    plt.tight_layout()
    plt.show()


def show_image(df, idx: int = 0):
    import matplotlib.pyplot as plt

    row = df.iloc[idx]
    w, h = int(row["width"]), int(row["height"])
    encoding = row["encoding"]
    pixels = row["pixels"]

    if isinstance(pixels, (bytes, bytearray)):
        arr = np.frombuffer(pixels, dtype=np.uint8)
    else:
        arr = np.array(pixels, dtype=np.uint8)

    channels = 1 if "mono" in encoding.lower() else 3
    try:
        img = arr.reshape((h, w, channels))
    except ValueError:
        print(f"[Inspector] Cannot reshape {len(arr)} bytes into ({h},{w},{channels})")
        return

    fig, ax = plt.subplots(figsize=(10, 6), facecolor="#0a0e14")
    ax.set_facecolor("#0a0e14")
    ts = row["timestamp"]
    fig.suptitle(
        f"Image frame {idx}  |  {w}x{h}  {encoding}  ts={ts}",
        color="#c8d8e8", fontsize=10, fontfamily="monospace"
    )

    if channels == 1:
        ax.imshow(img[:, :, 0], cmap="gray")
    else:
        ax.imshow(img)

    ax.axis("off")
    plt.tight_layout()
    plt.show()


def scrub_images(df):
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation
    from matplotlib.widgets import Slider, Button

    fig, ax = plt.subplots(figsize=(10, 7), facecolor="#0a0e14")
    plt.subplots_adjust(bottom=0.18)
    ax.set_facecolor("#0a0e14")
    ax.axis("off")
    fig.suptitle("Image scrubber  ←/→ to step  |  Space to play/pause",
                 color="#c8d8e8", fontsize=10, fontfamily="monospace")

    state = {"idx": 0, "playing": False, "anim": None, "updating": False}

    def get_frame(idx):
        row = df.iloc[idx]
        w, h = int(row["width"]), int(row["height"])
        encoding = row["encoding"]
        pixels = row["pixels"]
        arr = np.frombuffer(
            pixels if isinstance(pixels, (bytes, bytearray)) else bytes(pixels),
            dtype=np.uint8
        )
        channels = 1 if "mono" in encoding.lower() else 3
        try:
            img = arr.reshape((h, w, channels))
            return img, w, h, encoding
        except ValueError:
            return None, w, h, encoding

    def render(idx):
        if state["updating"]:
            return
        state["updating"] = True
        img, w, h, encoding = get_frame(idx)
        if img is None:
            state["updating"] = False
            return
        ax.clear()
        ax.axis("off")
        if img.shape[2] == 1:
            ax.imshow(img[:, :, 0], cmap="gray", vmin=0, vmax=255)
        else:
            ax.imshow(img)
        ax.set_title(
            f"frame {idx}/{len(df)-1}  |  {w}x{h}  {encoding}  |  ts={df.iloc[idx]['timestamp']}",
            color="#c8d8e8", fontsize=8, fontfamily="monospace"
        )
        slider.set_val(idx)
        fig.canvas.draw_idle()
        state["updating"] = False

    # slider — defined before first render call
    ax_slider = plt.axes([0.12, 0.09, 0.76, 0.025], facecolor="#111720")
    slider = Slider(ax_slider, "", 0, len(df) - 1, valinit=0,
                    valstep=1, color="#00d4ff")

    def on_slider(val):
        if not state["playing"] and not state["updating"]:
            state["idx"] = int(slider.val)
            render(state["idx"])

    slider.on_changed(on_slider)

    # play/pause button
    ax_btn = plt.axes([0.45, 0.03, 0.10, 0.04], facecolor="#111720")
    btn = Button(ax_btn, "▶ Play", color="#111720", hovercolor="#1e2a38")
    btn.label.set_color("#00d4ff")
    btn.label.set_fontfamily("monospace")
    btn.label.set_fontsize(10)

    def animate(frame):
        if state["playing"]:
            state["idx"] = (state["idx"] + 1) % len(df)
            render(state["idx"])

    def toggle_play(event):
        state["playing"] = not state["playing"]
        if state["playing"]:
            btn.label.set_text("⏸ Pause")
            state["anim"] = animation.FuncAnimation(
                fig, animate, interval=33, cache_frame_data=False
            )
        else:
            btn.label.set_text("▶ Play")
            if state["anim"]:
                state["anim"].event_source.stop()
                state["anim"] = None
        fig.canvas.draw_idle()

    btn.on_clicked(toggle_play)

    def on_key(event):
        if event.key == "right":
            state["idx"] = min(len(df) - 1, state["idx"] + 1)
            render(state["idx"])
        elif event.key == "left":
            state["idx"] = max(0, state["idx"] - 1)
            render(state["idx"])
        elif event.key == " ":
            toggle_play(None)

    fig.canvas.mpl_connect("key_press_event", on_key)
    render(0)
    plt.show()
    
def main():
    parser = argparse.ArgumentParser(description="ourrosbag inspector")
    parser.add_argument("--session", default="output/parquet_session",
                        help="Path to parquet session directory")
    parser.add_argument("--imu",    action="store_true", help="Plot IMU data")
    parser.add_argument("--path",   action="store_true", help="Plot odometry path")
    parser.add_argument("--image",  action="store_true", help="Show single image frame")
    parser.add_argument("--scrub",  action="store_true", help="Interactive image scrubber")
    parser.add_argument("--frame",  type=int, default=0,  help="Frame index for --image")
    parser.add_argument("--all",    action="store_true", help="Show all plots")
    args = parser.parse_args()

    tables = load_parquet_session(args.session)

    if args.imu or args.all:
        if "imu" in tables:
            plot_imu(tables["imu"])
        else:
            print("[Inspector] No IMU data found")

    if args.path or args.all:
        if "odometry" in tables:
            plot_path(tables["odometry"])
        else:
            print("[Inspector] No odometry data found")

    if args.image or args.all:
        if "image" in tables:
            show_image(tables["image"], args.frame)
        else:
            print("[Inspector] No image data found")

    if args.scrub:
        if "image" in tables:
            scrub_images(tables["image"])
        else:
            print("[Inspector] No image data found")

    if not any([args.imu, args.path, args.image, args.scrub, args.all]):
        print("\nUsage:")
        print("  py -3.12 -m ourrosbag.inspector --imu        # IMU charts")
        print("  py -3.12 -m ourrosbag.inspector --path       # odometry path")
        print("  py -3.12 -m ourrosbag.inspector --image      # single frame")
        print("  py -3.12 -m ourrosbag.inspector --scrub      # interactive scrubber")
        print("  py -3.12 -m ourrosbag.inspector --all        # everything")
        print(f"\nTopics found: {list(tables.keys())}")


if __name__ == "__main__":
    main()