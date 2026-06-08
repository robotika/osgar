import numpy as np
import cv2
from tqdm import tqdm
from ultralytics import YOLO
from collections import deque
from scipy.signal import butter, filtfilt, welch
from scipy.spatial.distance import euclidean
import argparse
import re


# --- Config ---
class Config:
    # General
    VITALS_UPDATE_RATE = 5  # Process vitals every N frames for a person
    FONT = cv2.FONT_HERSHEY_SIMPLEX
    FONT_SCALE = 0.6
    FONT_THICKNESS = 2

    # Heart Rate (rPPG)
    HR_BUFFER_SECONDS = 5
    HR_MIN_HZ = 0.75  # 45 BPM
    HR_MAX_HZ = 2.5  # 150 BPM
    HR_SMOOTHING_WINDOW = 10
    HR_PSD_SNR_THRESHOLD = 2.0
    HR_MIN_FACE_ROI_PIXELS = 40 * 40  # Minimum face ROI size for a reliable signal

    # Breathing Rate
    BREATHING_BUFFER_SECONDS = 7
    BREATHING_MIN_HZ = 0.1  # 6 BrPM
    BREATHING_MAX_HZ = 0.8  # 48 BrPM
    BREATHING_MIN_SIGNAL_STD = 0.5  # Min pixel std dev to be considered a real signal
    BREATHING_CORRELATION_THRESHOLD = -0.3  # Required negative correlation between chest area and shoulder height
    BREATHING_SMOOTHING_WINDOW = 10
    BR_PSD_SNR_THRESHOLD = 1.5
    BR_SIGNAL_COHERENCE_TOLERANCE_HZ = 0.08  # Max allowed diff between signal frequencies

    # Liveness
    LIVENESS_ALIVE_THRESHOLD = 0.7
    LIVENESS_NO_VITALS_THRESHOLD = 0.1
    HR_MIN_BPM_FOR_ALIVE = 40  # If HR is detected below this, flag it

    # Keypoint Indices
    NOSE_IDX, LEFT_EYE_IDX, RIGHT_EYE_IDX, LEFT_EAR_IDX, RIGHT_EAR_IDX = 0, 1, 2, 3, 4
    LEFT_SHOULDER_IDX, RIGHT_SHOULDER_IDX, LEFT_HIP_IDX, RIGHT_HIP_IDX = 5, 6, 11, 12
    FACE_KEYPOINT_INDICES = [NOSE_IDX, LEFT_EYE_IDX, RIGHT_EYE_IDX, LEFT_EAR_IDX, RIGHT_EAR_IDX]


class TrackedPerson:
    def __init__(self, track_id, fps):
        self.id = track_id
        self.fps = fps
        self.hr_buffer_size = int(Config.HR_BUFFER_SECONDS * fps)
        self.breathing_buffer_size = int(Config.BREATHING_BUFFER_SECONDS * fps)

        # Buffers
        self.rgb_buffer_hr = deque(maxlen=self.hr_buffer_size)
        self.breathing_area_signal = deque(maxlen=self.breathing_buffer_size)
        self.breathing_shoulder_y_signal = deque(maxlen=self.breathing_buffer_size)
        self.breathing_shoulder_dist_signal = deque(maxlen=self.breathing_buffer_size)
        self.hr_history = deque(maxlen=Config.HR_SMOOTHING_WINDOW)
        self.brpm_history = deque(maxlen=Config.BREATHING_SMOOTHING_WINDOW)

        # State
        self.vitals_update_counter = 0
        self.vitals = {"hr_bpm": "N/A", "breathing": "Calculating..."}  # Default HR to N/A
        self.liveness_status = "Determining..."
        self.successful_readings = 0
        self.failed_readings = 0

        self.last_bbox = None
        self.is_static = False

        # --- NEW: Attributes for alive timer ---
        self.alive_since_frame = None
        self.alive_duration_str = ""


def get_roi_from_keypoints(keypoints, frame_shape, indices, padding=10):
    visible_kpts = keypoints[indices]
    visible_kpts = visible_kpts[np.all(visible_kpts > 0, axis=1)]
    if visible_kpts.shape[0] < 2: return None
    x1, y1 = np.min(visible_kpts, axis=0) - padding
    x2, y2 = np.max(visible_kpts, axis=0) + padding
    x1, y1 = max(0, int(x1)), max(0, int(y1))
    x2, y2 = min(frame_shape[1], int(x2)), min(frame_shape[0], int(y2))
    if (x2 - x1) < 20 or (y2 - y1) < 20: return None
    return x1, y1, x2, y2


def calculate_polygon_area(points):
    if points.shape[0] < 3: return 0.0
    x, y = points[:, 0], points[:, 1]
    return 0.5 * np.abs(np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1)))


def process_vitals_signal(signal, fps, min_hz, max_hz, snr_threshold):
    if len(signal) < int(fps * 2): return None, 0  # Need at least 2s of data
    signal = np.array(signal)
    detrended_signal = signal - np.mean(signal)
    nyquist = fps / 2.0
    low_cutoff, high_cutoff = min_hz / nyquist, max_hz / nyquist
    if low_cutoff >= high_cutoff or low_cutoff <= 0 or high_cutoff >= 1.0: return None, 0
    try:
        b, a = butter(4, [low_cutoff, high_cutoff], btype='band')
        filtered_signal = filtfilt(b, a, detrended_signal)
    except ValueError:
        return None, 0  # Handles cases where signal is too short or flat
    freqs, psd = welch(filtered_signal, fs=fps, nperseg=len(filtered_signal))
    valid_indices = np.where((freqs >= min_hz) & (freqs <= max_hz))[0]
    if len(valid_indices) == 0: return None, 0
    peak_index = valid_indices[np.argmax(psd[valid_indices])]
    peak_freq, peak_power = freqs[peak_index], psd[peak_index]
    noise_power = np.mean(psd[(psd > 0) & (psd != peak_power)])
    if noise_power < 1e-6: return None, 0
    snr = peak_power / noise_power
    if snr < snr_threshold: return None, snr
    return peak_freq * 60, snr


clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))


def update_heart_rate(frame, keypoints, person):
    face_box = get_roi_from_keypoints(keypoints, frame.shape, Config.FACE_KEYPOINT_INDICES, padding=15)
    if face_box is None: return "NO_SIGNAL"
    x1, y1, x2, y2 = face_box
    face_roi = frame[y1:y2, x1:x2]
    if face_roi.size < Config.HR_MIN_FACE_ROI_PIXELS: return "NO_SIGNAL"

    ## --- OPTIMIZATION: Only do image processing when we are about to calculate ---
    if len(person.rgb_buffer_hr) < person.hr_buffer_size:
        # Just append the mean of the raw ROI, it's faster
        person.rgb_buffer_hr.append(np.mean(face_roi, axis=(0, 1)))
        return "PROCESSING"

    # Now do the more expensive processing only when the buffer is full
    lab = cv2.cvtColor(face_roi, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    l_clahe = clahe.apply(l)
    enhanced_face_roi = cv2.cvtColor(cv2.merge((l_clahe, a, b)), cv2.COLOR_LAB2BGR)
    person.rgb_buffer_hr.append(np.mean(enhanced_face_roi, axis=(0, 1)))

    if person.vitals_update_counter != 0:
        return "PROCESSING"

    normalized_rgb = np.array(person.rgb_buffer_hr) / np.mean(person.rgb_buffer_hr, axis=0)
    projection_plane = np.array([[0, 1, -1], [-2, 1, 1]], dtype=np.float32)
    signal_components = np.dot(normalized_rgb, projection_plane.T)
    alpha = np.std(signal_components[:, 0]) / (np.std(signal_components[:, 1]) + 1e-6)
    final_signal = signal_components[:, 0] + alpha * signal_components[:, 1]

    hr_bpm, snr = process_vitals_signal(final_signal, person.fps, Config.HR_MIN_HZ, Config.HR_MAX_HZ,
                                        Config.HR_PSD_SNR_THRESHOLD)

    if hr_bpm is not None:
        person.hr_history.append(hr_bpm)
        smoothed_hr = np.mean(person.hr_history)
        person.vitals["hr_bpm"] = f"{smoothed_hr:.1f} BPM"
        return "SUCCESS"
    else:
        if isinstance(person.vitals["hr_bpm"], str): person.vitals["hr_bpm"] = "Low Signal..."
        return "LOW_SIGNAL"


def is_breathing_signal_plausible(person):
    if len(person.breathing_area_signal) < person.breathing_buffer_size:
        return True
    area_signal = np.array(person.breathing_area_signal)
    shoulder_y_signal = np.array(person.breathing_shoulder_y_signal)
    detrended_area = area_signal - np.mean(area_signal)
    detrended_shoulder_y = shoulder_y_signal - np.mean(shoulder_y_signal)
    if np.std(detrended_area) < Config.BREATHING_MIN_SIGNAL_STD: return False
    correlation_matrix = np.corrcoef(detrended_area, detrended_shoulder_y)
    correlation = correlation_matrix[0, 1]
    if correlation > Config.BREATHING_CORRELATION_THRESHOLD: return False
    return True


def update_breathing_rate(keypoints, person):
    chest_indices = [Config.LEFT_SHOULDER_IDX, Config.RIGHT_SHOULDER_IDX, Config.RIGHT_HIP_IDX, Config.LEFT_HIP_IDX]
    chest_points = keypoints[chest_indices]
    left_shoulder, right_shoulder = keypoints[Config.LEFT_SHOULDER_IDX], keypoints[Config.RIGHT_SHOULDER_IDX]
    shoulders_visible = np.all(left_shoulder > 0) and np.all(right_shoulder > 0)

    if np.all(chest_points > 0):
        person.breathing_area_signal.append(calculate_polygon_area(chest_points))
    elif len(person.breathing_area_signal) > 0:
        person.breathing_area_signal.append(person.breathing_area_signal[-1])

    if shoulders_visible:
        person.breathing_shoulder_y_signal.append((left_shoulder[1] + right_shoulder[1]) / 2)
        person.breathing_shoulder_dist_signal.append(euclidean(left_shoulder, right_shoulder))
    elif len(person.breathing_shoulder_y_signal) > 0:
        person.breathing_shoulder_y_signal.append(person.breathing_shoulder_y_signal[-1])
        person.breathing_shoulder_dist_signal.append(person.breathing_shoulder_dist_signal[-1])

    if len(person.breathing_area_signal) < person.breathing_buffer_size or person.vitals_update_counter != 0: return

    if not is_breathing_signal_plausible(person):
        person.vitals["breathing"] = "Unrealistic Motion"
        person.brpm_history.clear()
        return

    signals = {
        "area": np.array(person.breathing_area_signal),
        "shoulder_y": np.array(person.breathing_shoulder_y_signal),
        "shoulder_dist": np.array(person.breathing_shoulder_dist_signal)
    }

    peak_freqs = []
    for sig in signals.values():
        if len(sig) > int(person.fps * 2):
            brpm, _ = process_vitals_signal(sig, person.fps, Config.BREATHING_MIN_HZ, Config.BREATHING_MAX_HZ,
                                            Config.BR_PSD_SNR_THRESHOLD - 0.5)
            if brpm: peak_freqs.append(brpm / 60.0)

    if len(peak_freqs) < 2 or (len(peak_freqs) > 1 and np.std(peak_freqs) > Config.BR_SIGNAL_COHERENCE_TOLERANCE_HZ):
        if isinstance(person.vitals["breathing"], str): person.vitals["breathing"] = "Low Coherence..."
        return

    composite_signal = np.zeros(len(signals["area"]))
    for sig in signals.values():
        if np.std(sig) > 1e-6:
            composite_signal += (sig - np.mean(sig)) / np.std(sig)

    brpm, snr = process_vitals_signal(composite_signal, person.fps, Config.BREATHING_MIN_HZ, Config.BREATHING_MAX_HZ,
                                      Config.BR_PSD_SNR_THRESHOLD)
    if brpm is not None:
        person.brpm_history.append(brpm)
        smoothed_brpm = np.mean(person.brpm_history)
        person.vitals["breathing"] = f"{smoothed_brpm:.1f} BrPM"
    else:
        if isinstance(person.vitals["breathing"], str): person.vitals["breathing"] = "Low Signal..."


def calculate_robust_motion_score(roi1, roi2, motion_thresh_val=25):
    roi1_blur = cv2.GaussianBlur(roi1, (5, 5), 0)
    roi2_blur = cv2.GaussianBlur(roi2, (5, 5), 0)
    roi1_hist = cv2.equalizeHist(roi1_blur)
    roi2_hist = cv2.equalizeHist(roi2_blur)
    diff = cv2.absdiff(roi1_hist, roi2_hist)
    _, diff_thresh = cv2.threshold(diff, motion_thresh_val, 255, cv2.THRESH_BINARY)
    motion_score = np.count_nonzero(diff_thresh) / diff_thresh.size
    return motion_score


def update_liveness_status(person, hr_status, current_frame_idx):
    is_success = hr_status == "SUCCESS" or "BrPM" in str(person.vitals["breathing"])
    is_low_signal = hr_status == "LOW_SIGNAL"
    is_no_signal = hr_status == "NO_SIGNAL"

    reading_decay = 0.98
    person.successful_readings *= reading_decay
    person.failed_readings *= reading_decay

    if is_success:
        person.successful_readings += 1
    elif is_low_signal:
        person.failed_readings += 0.5
    elif is_no_signal:
        person.failed_readings += 1

    total_readings = person.successful_readings + person.failed_readings
    liveness_score = 0.0
    if total_readings > 1:
        liveness_score = person.successful_readings / total_readings

    if liveness_score > Config.LIVENESS_ALIVE_THRESHOLD:
        new_status = "Alive"
    elif liveness_score < Config.LIVENESS_NO_VITALS_THRESHOLD and total_readings > 5:
        new_status = "No Vitals Detected"
    else:
        new_status = "Determining..."

    if new_status == "Alive" and "BPM" in str(person.vitals["hr_bpm"]):
        hr_value_match = re.search(r"(\d+\.\d+)", person.vitals["hr_bpm"])
        if hr_value_match:
            hr_value = float(hr_value_match.group(1))
            if hr_value < Config.HR_MIN_BPM_FOR_ALIVE:
                new_status = "Alive (Low HR)"

    person.liveness_status = new_status

    if "Alive" in person.liveness_status:
        if person.alive_since_frame is None:
            person.alive_since_frame = current_frame_idx
        duration_frames = current_frame_idx - person.alive_since_frame
        duration_seconds = duration_frames / person.fps
        minutes, seconds = int(duration_seconds // 60), int(duration_seconds % 60)
        person.alive_duration_str = f"Alive for: {minutes}m {seconds}s"
    else:
        person.alive_since_frame = None
        person.alive_duration_str = ""


def run_pose_and_vitals_inference(args):
    pose_model = YOLO(args.model_file)
    cap = cv2.VideoCapture(args.input_file)
    if not cap.isOpened():
        print(f"Error: Could not open video file {args.input_file}");
        return

    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    if fps < 10: fps = 30
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    if args.resize_factor < 1.0:
        out_width = int(frame_width * args.resize_factor)
        out_height = int(frame_height * args.resize_factor)
    else:
        out_width, out_height = frame_width, frame_height

    out = cv2.VideoWriter(args.output_file, cv2.VideoWriter_fourcc(*'mp4v'), fps, (out_width, out_height))
    track_history = {}

    prev_gray = None
    prev_track_boxes = {}

    try:
        for frame_idx in tqdm(range(total_frames), desc="Processing Video"):
            ret, frame = cap.read()
            if not ret: break

            if frame_idx % args.skip_frames != 0:
                results = pose_model.track(frame, persist=True, verbose=False, tracker="bytetrack.yaml", conf=args.conf)
                annotated_frame = results[0].plot()
                out.write(annotated_frame)
                continue

            if args.resize_factor < 1.0:
                frame = cv2.resize(frame, (out_width, out_height), interpolation=cv2.INTER_AREA)

            current_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            results = pose_model.track(frame, persist=True, verbose=False, tracker="bytetrack.yaml", conf=args.conf)
            annotated_frame = results[0].plot()

            ## --- CHANGE 1: Keep track of IDs seen in the current frame ---
            active_track_ids = set()

            current_track_boxes = {}
            if results[0].boxes.id is not None:
                track_ids = results[0].boxes.id.int().cpu().tolist()
                active_track_ids = set(track_ids)  # Store the active IDs
                keypoints_data = results[0].keypoints.xy.cpu().numpy()
                boxes_data = results[0].boxes.data.cpu()

                for box, raw_kpts, track_id in zip(boxes_data, keypoints_data, track_ids):
                    x1, y1, x2, y2 = box[:4].int().tolist()
                    current_track_boxes[track_id] = (x1, y1, x2, y2)

                    if track_id not in track_history:
                        track_history[track_id] = TrackedPerson(track_id, fps)
                    person = track_history[track_id]
                    person.last_bbox = (x1, y1, x2, y2)

                    if prev_gray is not None and track_id in prev_track_boxes:
                        px1, py1, px2, py2 = prev_track_boxes[track_id]
                        prev_roi = prev_gray[py1:py2, px1:px2]
                        curr_roi = current_gray[y1:y2, x1:x2]
                        if prev_roi.size > 0 and curr_roi.size > 0:
                            h, w = prev_roi.shape
                            curr_roi_resized = cv2.resize(curr_roi, (w, h))
                            motion_score = calculate_robust_motion_score(prev_roi, curr_roi_resized)
                            if motion_score < args.motion_thresh:
                                person.is_static = True
                                person.liveness_status = "Static Object"
                            else:
                                person.is_static = False

                    if person.is_static: continue

                    person.vitals_update_counter = (person.vitals_update_counter + 1) % Config.VITALS_UPDATE_RATE

                    hr_status = "N/A"
                    if not args.no_hr:
                        hr_status = update_heart_rate(frame, raw_kpts, person)

                    update_breathing_rate(raw_kpts, person)
                    update_liveness_status(person, hr_status, frame_idx)

            # --- Visualization ---
            for track_id, person in track_history.items():
                ## --- CHANGE 2: Only draw the panel if the track ID is currently active ---
                if track_id not in active_track_ids:
                    continue

                if person.last_bbox is None: continue
                x1, y1, x2, y2 = person.last_bbox
                status = person.liveness_status
                if "Static" in status: status = "Static Object"
                liveness_score = person.successful_readings / (
                            person.successful_readings + person.failed_readings + 1e-6)

                text_lines = [f"ID {track_id} (Conf: {liveness_score:.2f})", f"Status: {status}", ]
                if person.alive_duration_str: text_lines.append(person.alive_duration_str)
                text_lines.append(f"Breathing: {person.vitals['breathing']}")
                if not args.no_hr: text_lines.append(f"Heart Rate: {person.vitals['hr_bpm']}")

                (line_w, line_h), _ = cv2.getTextSize(text_lines[0], Config.FONT, Config.FONT_SCALE,
                                                      Config.FONT_THICKNESS)
                panel_h = (line_h + 10) * len(text_lines) + 10
                rect_y1 = y1 - panel_h - 10 if y1 - panel_h - 10 > 0 else y1 + 10

                cv2.rectangle(annotated_frame, (x1, rect_y1), (x1 + 300, rect_y1 + panel_h), (0, 0, 0), -1)
                for i, line in enumerate(text_lines):
                    color = (255, 255, 255)
                    if "Alive for:" in line:
                        color = (150, 255, 150)
                    elif "Alive" in line:
                        color = (0, 255, 0)
                    elif "Low Heart Rate" in line:
                        color = (0, 165, 255)
                    elif "No Vitals" in line or "Static" in line:
                        color = (0, 0, 255)
                    elif "Determining" in line:
                        color = (0, 255, 255)
                    elif "BPM" in line or "BrPM" in line:
                        if "Low" not in line and "Calculating" not in line: color = (0, 255, 0)
                    cv2.putText(annotated_frame, line, (x1 + 10, rect_y1 + (i + 1) * (line_h + 10)), Config.FONT,
                                Config.FONT_SCALE, color, Config.FONT_THICKNESS)

            out.write(annotated_frame)
            prev_gray = current_gray
            prev_track_boxes = current_track_boxes

    finally:
        cap.release()
        out.release()
        print(f"✅ Processed video saved to {args.output_file}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Run Pose Estimation and Vital Signs Detection on a video.')
    parser.add_argument('-i', '--input_file', type=str, required=True, help='Path to the input video file.')
    parser.add_argument('-o', '--output_file', type=str, required=True, help='Path to save the output video file.')
    parser.add_argument('-m', '--model_file', type=str, default="yolo11s-pose.pt",
                        help='Path to the YOLO pose model file. Use yolo11n-pose.pt for max speed.')
    parser.add_argument('-c', '--conf', type=float, default=0.7, help='Object detection confidence threshold.')
    parser.add_argument('--motion_thresh', type=float, default=0.02, help='Motion detection threshold.')

    ## --- OPTIMIZATION ARGUMENTS ---
    parser.add_argument('--no-hr', action='store_true', help='Disable heart rate calculation to speed up processing.')
    parser.add_argument('--skip-frames', type=int, default=1,
                        help='Process vitals on every N-th frame (e.g., 2 means 50% less processing).')
    parser.add_argument('--resize-factor', type=float, default=0.5,
                        help='Resize video by this factor (e.g., 0.5 for half resolution) before processing.')

    args = parser.parse_args()
    print(f"Loading YOLO model: {args.model_file}")
    print(f"Processing video: {args.input_file}")
    if args.no_hr: print("Heart rate detection is DISABLED.")
    if args.skip_frames > 1: print(f"Skipping frames: Processing 1 in every {args.skip_frames} frames.")
    if args.resize_factor < 1.0: print(f"Resizing video to {args.resize_factor * 100}% of original resolution.")

    run_pose_and_vitals_inference(args)