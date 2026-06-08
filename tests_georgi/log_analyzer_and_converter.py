import cv2
from osgar.logger import LogReader, lookup_stream_id, lookup_stream_names
from osgar.lib.serialize import deserialize
import argparse
import av


def save_video_to_mp4(logfile, stream, output_file, fps=30):
    """
    Reads an H.265 stream from a logfile, decodes it, and saves it as an MP4 file.
    """
    print(f"Reading stream '{stream}' from '{logfile}'...")
    print(f"Saving to '{output_file}' at {fps} FPS...")

    try:
        only_stream = lookup_stream_id(logfile, stream)
    except Exception as e:
        print(f"Error looking up stream: {e}")
        return

    decoder = av.codec.CodecContext.create('hevc', 'r')
    output_container = None
    out_stream = None  # <-- 1. INITIALIZE out_stream TO None
    frame_count = 0

    with LogReader(logfile, only_stream_id=only_stream) as log:
        for timestamp, stream_id, data in log:
            raw_data = deserialize(data)
            packets = decoder.parse(raw_data)
            for packet in packets:
                frames = decoder.decode(packet)
                for frame in frames:
                    if output_container is None:
                        print("First frame decoded. Initializing output MP4 file...")
                        output_container = av.open(output_file, mode='w')
                        out_stream = output_container.add_stream('libx264', rate=fps)
                        out_stream.width = frame.width
                        out_stream.height = frame.height
                        out_stream.pix_fmt = 'yuv420p'

                    for out_packet in out_stream.encode(frame):
                        output_container.mux(out_packet)

                    frame_count += 1
                    if frame_count % 100 == 0:
                        print(f"  {frame_count} frames processed...", end='\r')

    # --- 2. CHANGE THE FINAL CHECK ---
    # After the loop, flush the encoder only if the stream was actually created.
    if out_stream:
        print(f"\nFlushing encoder...")
        for out_packet in out_stream.encode():
            output_container.mux(out_packet)
        output_container.close()
        print(f"Done. Saved {frame_count} frames to '{output_file}'.")
    else:
        print("\nNo valid video frames were found. Output file not created.")


def visualize_depth_video(logfile, stream):
    """
    Reads a logfile, visualizes the depth stream in a continuous loop,
    and allows for interactive control over playback and colormap.
    """
    # This function remains unchanged...
    print(f"Opening logfile: {logfile}")
    print("--- Controls ---")
    print("  'p': Pause / Resume")
    print("  'c': Change Colormap")
    print("  'q': Quit")
    print("----------------")
    try:
        names = lookup_stream_names(logfile)
        print(f"Available streams: {names}")
        only_stream = lookup_stream_id(logfile, stream)
    except Exception as e:
        print(f"Error looking up stream: {e}")
        return
    colormaps = [
        cv2.COLORMAP_JET, cv2.COLORMAP_BONE, cv2.COLORMAP_RAINBOW,
        cv2.COLORMAP_HOT, cv2.COLORMAP_SPRING, cv2.COLORMAP_COOL, cv2.COLORMAP_INFERNO
    ]
    colormap_index = 0
    is_paused = False
    keep_running = True
    while keep_running:
        with LogReader(logfile, only_stream_id=only_stream) as log:
            for timestamp, stream_id, data in log:
                depth_frame = deserialize(data)
                normalized_frame = cv2.normalize(depth_frame, None, 255, 0, cv2.NORM_MINMAX, cv2.CV_8U)
                colored_frame = cv2.applyColorMap(normalized_frame, colormaps[colormap_index])
                cv2.imshow('Depth Video (Looping)', colored_frame)
                key = cv2.waitKey(30 if not is_paused else 0) & 0xFF
                if key == ord('q'):
                    keep_running = False
                    break
                elif key == ord('p'):
                    is_paused = not is_paused
                    print("Paused" if is_paused else "Resumed")
                elif key == ord('c'):
                    colormap_index = (colormap_index + 1) % len(colormaps)
                    print(f"Changed colormap.")
        if keep_running:
            print("Video finished. Restarting...")
    cv2.destroyAllWindows()
    print("Application closed.")


def visualize_video(logfile, stream):
    """
    Reads a logfile, decodes an H.265 video stream, and visualizes it.
    """
    # This function remains unchanged...
    print(f"Opening logfile: {logfile}")
    print("--- Controls ---")
    print("  'p': Pause / Resume")
    print("  'q': Quit")
    print("----------------")
    try:
        names = lookup_stream_names(logfile)
        print(f"Available streams: {names}")
        only_stream = lookup_stream_id(logfile, stream)
    except Exception as e:
        print(f"Error looking up stream: {e}")
        return
    is_paused = False
    codec = av.codec.CodecContext.create('hevc', 'r')
    with LogReader(logfile, only_stream_id=only_stream) as log:
        for timestamp, stream_id, data in log:
            raw_data = deserialize(data)
            try:
                packets = codec.parse(raw_data)
                for packet in packets:
                    frames = codec.decode(packet)
                    for frame in frames:
                        video_frame = frame.to_ndarray(format='bgr24')
                        cv2.imshow('H.265 Video', video_frame)
                        key = cv2.waitKey(30 if not is_paused else 0) & 0xFF
                        if key == ord('q'):
                            cv2.destroyAllWindows()
                            return
                        elif key == ord('p'):
                            is_paused = not is_paused
                            print("Paused" if is_paused else "Resumed")
            except av.AVError as e:
                print(f"Skipping problematic packet: {e}")
                continue
    cv2.destroyAllWindows()
    print("Application closed.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Analyze and process video from an OSGAR logfile.')
    parser.add_argument('logfile', help='Path to the recorded log file')
    parser.add_argument('--stream', help='The stream ID or name to process (e.g., oak.depth, oak.color)', required=True)
    parser.add_argument('--output', help='Path to save the output MP4 file. If not set, the script visualizes the stream.')
    parser.add_argument('--fps', type=int, default=5, help='Frames per second for the output video.')
    args = parser.parse_args()

    if 'depth' in args.stream:
        if args.output:
            print("Error: Saving depth streams to MP4 is not supported. Only visualization is available.")
        else:
            visualize_depth_video(args.logfile, args.stream)
    elif 'color' in args.stream or 'video' in args.stream or 't_im' in args.stream:
        if args.output:
            save_video_to_mp4(args.logfile, args.stream, args.output, args.fps)
        else:
            visualize_video(args.logfile, args.stream)
    else:
        print(f"Error: Unknown stream type for '{args.stream}'. Please specify a stream containing 'depth' or 'color'.")