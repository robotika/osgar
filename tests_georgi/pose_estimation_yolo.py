import cv2
import matplotlib.pyplot as plt
from tqdm import tqdm
from ultralytics import YOLO

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

def run_pose_inference_mp4(pose_model, video_path, output_path):
    # Open the input video file 📹
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"Error: Could not open video file {video_path}")
        exit()

    # Get video properties (width, height, frames per second)
    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    # Define the codec and create a VideoWriter object to save the output 💾
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Or 'XVID', 'MJPG', etc.
    out = cv2.VideoWriter(output_path, fourcc, fps, (frame_width, frame_height))

    # --- Processing Loop ---
    # Use tqdm for a progress bar
    for _ in tqdm(range(total_frames), desc="Processing Video"):
        # Read a frame from the video
        ret, frame = cap.read()

        if not ret:
            # Break the loop if we've reached the end of the video
            break

        # Run YOLO pose estimation on the frame
        # The model handles resizing, no need to do it manually
        results = pose_model(frame, verbose=False)

        # The `plot()` method returns a BGR numpy array with the annotations drawn ✨
        annotated_frame = results[0].plot()

        # Write the annotated frame to the output video
        out.write(annotated_frame)

    # --- Cleanup ---
    print(f"✅ Processed video saved to {output_path}")
    cap.release()
    out.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description='Extract data from logfile')
    parser.add_argument('-i', '--input_file', help='recorded log file')
    parser.add_argument('-o', '--output_file', help='recorded log file')
    # ULTRALYTICS IMAGES: "https://ultralytics.com/images/bus.jpg"
    args = parser.parse_args()
    model = YOLO("yolo11s-pose.pt")  # load an official model
    # run_pose_inference_image(model, image_path)
    run_pose_inference_mp4(model, args.input_file, args.output_file)
