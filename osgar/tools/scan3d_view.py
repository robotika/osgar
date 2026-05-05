"""Simple viewer for OSGAR 3D scans"""
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from osgar.logger import LogReader, lookup_stream_id
from osgar.lib.serialize import deserialize


def load_scans_from_log(log_filename, stream_name):
    """
    Loads a 2D array of scans from an OSGAR log.
    """
    try:
        # Get the internal stream ID based on its name
        stream_id = lookup_stream_id(log_filename, stream_name)
    except Exception as e:
        print(f"Error: Stream '{stream_name}' not found in the log. ({e})")
        return []

    scans = []
    # Read the log filtering only the requested stream
    with LogReader(log_filename, only_stream_id=stream_id) as log:
        for dt, channel, data in log:
            # Deserialize binary data back to the original Python/NumPy object
            scan = deserialize(data)
            scans.append([dt,scan])

    print(f"Successfully loaded {len(scans)} scans.")
    return scans


def animate_scans(scans, stream, vmax, interval_ms = 20):
    """
    Renders a sequence of 2D arrays as an animation.
    """
    if not scans:
        print("The scan list is empty, nothing to animate.")
        return

    fig, ax = plt.subplots(figsize=(12, 5))

    # Initialize the first frame.
    # Note: vmin and vmax set the colormap range. If the data is in millimeters,
    if "reflectivity" in stream:
        vmax = 255
    im = ax.imshow(scans[0][1], cmap='viridis', aspect='auto', vmin=0, vmax=vmax)

    fig.colorbar(im, ax=ax, label='Measured value (e.g., mm)')
    ax.set_title("OSGAR Log Playback: 3D Scan")
    ax.set_xlabel("Columns (Azimuth)")
    ax.set_ylabel("Rows (Lasers)")

    def update(frame_index):
        im.set_array(scans[frame_index][1])
        ax.set_title(f"OSGAR Log Playback: 3D Scan ({scans[frame_index][0]})")
        return [im]

    # Create the animation loop
    ani = animation.FuncAnimation(
        fig, update, frames=len(scans), interval=interval_ms, blit=False
    )

    plt.show()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('logfile', help='recorded log file')
    parser.add_argument('--stream', help='stream name', default='app.scan3d')
    parser.add_argument('--vmax', help='Range limit in mm', default=10_000)
    args = parser.parse_args()

    loaded_scans = load_scans_from_log(args.logfile, args.stream)
    animate_scans(loaded_scans, args.stream, args.vmax, interval_ms=50)
