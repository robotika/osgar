import cv2
from osgar.logger import LogReader, lookup_stream_id, lookup_stream_names
from osgar.lib.serialize import deserialize
import argparse


def visualize_depth_video(logfile, stream):
    """
    Reads a logfile, visualizes the depth stream in a continuous loop,
    and allows for interactive control over playback and colormap.
    """
    print(f"Opening logfile: {logfile}")
    print("--- Controls ---")
    print("  'p': Pause / Resume")
    print("  'c': Change Colormap")
    print("  'q': Quit")
    print("----------------")

    # --- Stream Selection ---
    try:
        names = lookup_stream_names(logfile)
        print(f"Available streams: {names}")
        only_stream = lookup_stream_id(logfile, stream)
    except Exception as e:
        print(f"Error looking up stream: {e}")
        return

    # --- Colormap and Playback State ---
    colormaps = [
        cv2.COLORMAP_JET, cv2.COLORMAP_BONE, cv2.COLORMAP_RAINBOW,
        cv2.COLORMAP_HOT, cv2.COLORMAP_SPRING, cv2.COLORMAP_COOL, cv2.COLORMAP_INFERNO
    ]
    colormap_index = 0
    is_paused = False

    # --- Main Application Loop ---
    keep_running = True
    while keep_running:
        # The 'with' statement handles opening and closing the log file.
        # It will be re-opened each time the outer loop iterates.
        with LogReader(logfile, only_stream_id=only_stream) as log:
            # --- Video Playback Loop ---
            for timestamp, stream_id, data in log:
                # Deserialize the raw data to get the depth frame
                depth_frame = deserialize(data)

                # Normalize the depth frame to an 8-bit scale (0-255) for visualization
                normalized_frame = cv2.normalize(depth_frame, None, 255, 0, cv2.NORM_MINMAX, cv2.CV_8U)

                # Apply the currently selected colormap
                colored_frame = cv2.applyColorMap(normalized_frame, colormaps[colormap_index])

                # Display the resulting frame
                cv2.imshow('Depth Video (Looping)', colored_frame)

                # --- User Input Handling ---
                # Wait for a key press. If paused, wait indefinitely (0). Otherwise, wait for 30ms.
                key = cv2.waitKey(30 if not is_paused else 0) & 0xFF

                if key == ord('q'):  # Press 'q' to quit the application
                    print("Exiting...")
                    keep_running = False  # Set flag to false to exit the outer while loop
                    break  # Exit the inner for loop

                elif key == ord('p'):  # Press 'p' to pause/unpause the video
                    is_paused = not is_paused
                    print("Paused" if is_paused else "Resumed")

                elif key == ord('c'):  # Press 'c' to cycle through colormaps
                    colormap_index = (colormap_index + 1) % len(colormaps)
                    print(f"Changed colormap.")

        if keep_running:
            print("Video finished. Restarting...")

    # --- Cleanup ---
    cv2.destroyAllWindows()
    print("Application closed.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Visualize depth video from a logfile with looping.')
    parser.add_argument('--logfile', required=True, help='Path to the recorded log file')
    parser.add_argument('--stream', help='The stream ID or name to visualize', default='oak.depth')
    args = parser.parse_args()
    visualize_depth_video(args.logfile, args.stream)
