import numpy as np
import cv2
import matplotlib.pyplot as plt
from tqdm import tqdm
from ultralytics import YOLO
from collections import deque

def run_pose_inference_image(pose_model, image_path):
    results = pose_model(image_path)  # predict on an image

    # Get the first result object
    result = results[0]

    # Get the original image and convert it from BGR to RGB for matplotlib
    img_rgb = cv2.cvtColor(result.orig_img, cv2.COLOR_BGR2RGB)

    # Get the keypoints data for all detected individuals
    # The format is a tensor of shape (num_persons, num_keypoints, 2) for x, y coordinates
    keypoints = result.keypoints.xy.cpu().numpy()

    # Define the skeleton connections for the COCO 17-keypoint format
    # Each tuple represents a connection between two keypoint indices
    skeleton = [
        (0, 1), (0, 2), (1, 3), (2, 4),  # Head
        (5, 6), (5, 7), (7, 9), (6, 8), (8, 10),  # Arms
        (11, 12), (5, 11), (6, 12),  # Torso
        (11, 13), (13, 15), (12, 14), (14, 16)  # Legs
    ]

    # Create a new plot
    plt.figure(figsize=(10, 10))
    ax = plt.gca()  # Get current axes

    # Display the image
    ax.imshow(img_rgb)

    # --- Loop through each detected person and draw their pose ---
    for person_kpts in keypoints:
        # Draw the keypoints (joints) 🎯
        # We filter out points with coordinates (0,0) as they are not detected
        visible_kpts = person_kpts[person_kpts[:, 0] > 0]
        ax.scatter(visible_kpts[:, 0], visible_kpts[:, 1], s=40, color='red', marker='o', zorder=3)

        # Draw the skeleton (limbs) 🦴
        for start_idx, end_idx in skeleton:
            # Check if both connected keypoints are detected
            if person_kpts[start_idx, 0] > 0 and person_kpts[end_idx, 0] > 0:
                # Get coordinates for the start and end points of the limb
                x_limb = [person_kpts[start_idx, 0], person_kpts[end_idx, 0]]
                y_limb = [person_kpts[start_idx, 1], person_kpts[end_idx, 1]]
                ax.plot(x_limb, y_limb, 'c-', linewidth=2, zorder=2)  # 'c-' for cyan line

    # Final settings for a clean plot
    ax.axis('off')  # Hide the axes
    plt.title("YOLO Pose Estimation with Matplotlib")
    plt.show()


def run_pose_inference_mp4(pose_model, video_path, output_path, movement_threshold: int = 4,
                           smoothing_window_size: int = 7):
    """
    Processes a video to detect and track human poses, visualizing movement status.

    This function uses a moving average of keypoint displacements to stabilize the
    "Moving" vs. "Still" prediction, preventing label flickering.

    Args:
        pose_model: The pre-loaded YOLO pose estimation model.
        video_path (str): Path to the input video file.
        output_path (str): Path to save the processed output video.
        movement_threshold (int): The average displacement (in pixels) above which a
                                  person is considered "Moving". Defaults to 4.
        smoothing_window_size (int): The number of recent frames to average for
                                     stabilizing the movement prediction. Defaults to 7.
    """
    # Open the input video file 📹
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"Error: Could not open video file {video_path}")
        return

    # Get video properties for writer
    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    # Setup video writer
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(output_path, fourcc, fps, (frame_width, frame_height))

    # Dictionary to store tracking history, now including a deque for smoothing
    track_history = {}

    # --- Processing Loop ---
    for _ in tqdm(range(total_frames), desc="Detecting Movement"):
        ret, frame = cap.read()
        if not ret:
            break

        # Use pose_model.track() for persistent tracking
        results = pose_model.track(frame, persist=True, verbose=False)

        # Get the annotated frame, but disable the default labels to draw our own
        annotated_frame = results[0].plot(conf=False, labels=False)

        # Check if tracking IDs are available
        if results[0].boxes.id is not None:
            track_ids = results[0].boxes.id.int().cpu().tolist()
            keypoints = results[0].keypoints.xy.cpu().numpy()

            # Iterate over each tracked person
            for box, kpts, track_id in zip(results[0].boxes.data, keypoints, track_ids):
                # --- START: Movement Prediction with Smoothing ---
                status = "Still"  # Default status

                # If track is new, initialize its history
                if track_id not in track_history:
                    track_history[track_id] = {
                        "kpts": kpts,
                        "displacements": deque(maxlen=smoothing_window_size)
                    }
                # If track exists, calculate smoothed movement
                else:
                    prev_kpts = track_history[track_id]["kpts"]
                    visible = (kpts[:, 0] > 0) & (prev_kpts[:, 0] > 0)

                    # Calculate displacement for the current frame
                    current_displacement = np.mean(
                        np.linalg.norm(kpts[visible] - prev_kpts[visible], axis=1)) if np.any(visible) else 0

                    # Add current displacement to the history
                    track_history[track_id]["displacements"].append(current_displacement)

                    # Calculate the smoothed displacement
                    smoothed_displacement = np.mean(track_history[track_id]["displacements"])

                    # Determine status based on the smoothed value
                    status = "Moving" if smoothed_displacement > movement_threshold else "Still"

                # Update the keypoint history for the next frame
                track_history[track_id]["kpts"] = kpts
                # --- END: Movement Prediction with Smoothing ---

                # Visualization Logic (remains the same)
                color = (0, 0, 255) if status == "Moving" else (0, 255, 0)
                x1, y1, _, _ = box[:4].int().cpu().tolist()
                conf = box[4]
                class_name = pose_model.names[int(box[5])]
                label_line1 = f"{class_name.capitalize()} {conf:.2f}"
                label_line2 = f"ID {track_id}: {status}"
                font_scale = 0.7
                font_thickness = 2
                text_color = (255, 255, 255)
                (w1, h1), _ = cv2.getTextSize(label_line1, cv2.FONT_HERSHEY_SIMPLEX, font_scale, font_thickness)
                (w2, h2), _ = cv2.getTextSize(label_line2, cv2.FONT_HERSHEY_SIMPLEX, font_scale, font_thickness)
                panel_width = max(w1, w2) + 10
                panel_height = h1 + h2 + 10
                rect_x1 = x1
                rect_y1 = y1 - panel_height - 5
                cv2.rectangle(annotated_frame, (rect_x1, rect_y1), (rect_x1 + panel_width, y1 - 5), color, -1)
                cv2.putText(annotated_frame, label_line1, (x1 + 5, rect_y1 + h1 + 5), cv2.FONT_HERSHEY_SIMPLEX,
                            font_scale, text_color, font_thickness)
                cv2.putText(annotated_frame, label_line2, (x1 + 5, rect_y1 + h1 + h2 + 10), cv2.FONT_HERSHEY_SIMPLEX,
                            font_scale, text_color, font_thickness)
        out.write(annotated_frame)

    # Release resources
    cap.release()
    out.release()
    print(f"✅ Processed video saved to {output_path}")

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description='Extract data from logfile')
    parser.add_argument('-i', '--input_file', help='recorded log file')
    parser.add_argument('-o', '--output_file', help='recorded log file')
    parser.add_argument('-mt', '--movement_threshold', default=6., type=int, help='recorded log file')
    parser.add_argument('-w', '--smoothing_window_size', type=int, default=7, help='Number of frames for smoothing the movement prediction.')

    # ULTRALYTICS IMAGES: "https://ultralytics.com/images/bus.jpg"
    args = parser.parse_args()
    model = YOLO("yolo11s-pose.pt")  # load an official model
    # run_pose_inference_image(model, image_path)
    run_pose_inference_mp4(model, args.input_file, args.output_file, args.movement_threshold,
                           smoothing_window_size=args.smoothing_window_size
                           )
