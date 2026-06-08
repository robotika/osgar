import cv2
import depthai as dai
import numpy as np
import argparse

# --- Argument Parsing ---
parser = argparse.ArgumentParser(description='OAK-D Pro Depth Smoothing Demo')
parser.add_argument(
    '--use_dot_projector',
    action='store_true',
    help='Enable the dot projector for active stereo depth.'
)
parser.add_argument(
    '--dot_brightness',
    type=int,
    default=100,
    help='Set the dot projector brightness (0-1200).'
)
args = parser.parse_args()


# --- DepthAI Pipeline Configuration ---
def create_pipeline():
    """
    Configures a DepthAI pipeline with two StereoDepth nodes:
    1.  'raw_stereo': No post-processing filters.
    2.  'filtered_stereo': Median, Speckle, and Temporal filters enabled.
    """
    pipeline = dai.Pipeline()

    # --- THE V3 FIX (Part 1) ---
    # Create builders, call .setBoardSocket() on them, *then* call .build().
    # This solves the "...socket already used" conflict.

    # Configure Left Mono Camera
    mono_left_builder = pipeline.create(dai.node.Camera)
    mono_left = mono_left_builder.build(dai.CameraBoardSocket.CAM_B)  # mono_left is the node

    # Configure Right Mono Camera
    mono_right_builder = pipeline.create(dai.node.Camera)
    mono_right = mono_right_builder.build(dai.CameraBoardSocket.CAM_C)  # mono_right is the node

    # --- Create TWO StereoDepth nodes (as originally intended) ---
    raw_stereo = pipeline.create(dai.node.StereoDepth)
    filtered_stereo = pipeline.create(dai.node.StereoDepth)

    # --- Configure the RAW StereoDepth Node ---
    # --- THE V3 FIX (Part 2) ---
    # .setDepthAlign() belongs on the StereoDepth node.
    raw_stereo.setDepthAlign(dai.CameraBoardSocket.CAM_C)
    raw_stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
    raw_stereo.setExtendedDisparity(False)
    raw_stereo.setSubpixel(True)
    raw_stereo.setLeftRightCheck(True)
    raw_stereo.initialConfig.setMedianFilter(dai.MedianFilter.MEDIAN_OFF)
    raw_stereo.initialConfig.PostProcessing.SpeckleFilter.enable = False
    raw_stereo.initialConfig.PostProcessing.TemporalFilter.enable = False

    # --- Configure the FILTERED StereoDepth Node ---
    # --- THE V3 FIX (Part 2) ---
    # .setDepthAlign() belongs on the StereoDepth node.
    filtered_stereo.setDepthAlign(dai.CameraBoardSocket.CAM_C)
    filtered_stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.ROBOTICS)
    #filtered_stereo.setExtendedDisparity(False)
    #filtered_stereo.setSubpixel(True)
    #filtered_stereo.setLeftRightCheck(True)

    #filtered_stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
    #filtered_stereo.initialConfig.setConfidenceThreshold(200)
    #filtered_stereo.initialConfig.PostProcessing.SpatialFilter.enable = True
    #filtered_stereo.initialConfig.PostProcessing.SpatialFilter.holeFillingRadius = 5
    #filtered_stereo.initialConfig.PostProcessing.SpatialFilter.alpha = 0.1
    #filtered_stereo.initialConfig.PostProcessing.SpatialFilter.delta = 0
    #filtered_stereo.initialConfig.PostProcessing.SpatialFilter.numIterations = 2
    #filtered_stereo.initialConfig.PostProcessing.SpeckleFilter.enable = True
    #filtered_stereo.initialConfig.PostProcessing.SpeckleFilter.speckleRange = 50
    #filtered_stereo.initialConfig.PostProcessing.TemporalFilter.enable = True
    #filtered_stereo.initialConfig.PostProcessing.TemporalFilter.alpha = 0.9
    #filtered_stereo.initialConfig.PostProcessing.TemporalFilter.delta = 8

    # --- Linking Nodes ---

    # Request output from the built camera nodes
    ml_out = mono_left.requestOutput((960, 640), type=dai.ImgFrame.Type.GRAY8)
    # Link one output to BOTH stereo nodes
    ml_out.link(raw_stereo.left)
    ml_out.link(filtered_stereo.left)

    mr_out = mono_right.requestOutput((960, 640), type=dai.ImgFrame.Type.GRAY8)
    # Link one output to BOTH stereo nodes
    mr_out.link(raw_stereo.right)
    mr_out.link(filtered_stereo.right)

    # --- Create output queues (solves the very first error) ---
    raw_depth_q = raw_stereo.depth.createOutputQueue()
    filtered_depth_q = filtered_stereo.depth.createOutputQueue()

    # Return the pipeline and the queues
    return pipeline, raw_depth_q, filtered_depth_q


# --- Main Application Logic (Ported to V3) ---
if __name__ == "__main__":
    pipeline, raw_depth_q, filtered_depth_q = create_pipeline()

    pipeline.start()

    try:
        if args.use_dot_projector:
            pipeline.setIrFloodLightBrightness(0)
            pipeline.setIrDotProjectorBrightness(args.dot_brightness)
    except:
        print("Dot projector settings failed. Not an OAK-D Pro/Pro W?")

    print("Connected to OAK-D. Press 'q' to quit.")

    while pipeline.isRunning():
        in_raw = raw_depth_q.get()
        in_filtered = filtered_depth_q.get()

        raw_frame = in_raw.getFrame()
        filtered_frame = in_filtered.getFrame()

        # --- Visualization ---
        raw_normalized = cv2.normalize(raw_frame, None, 255, 0, cv2.NORM_MINMAX, cv2.CV_8U)
        filtered_normalized = cv2.normalize(filtered_frame, None, 255, 0, cv2.NORM_MINMAX, cv2.CV_8U)

        raw_colormap = cv2.applyColorMap(raw_normalized, cv2.COLORMAP_JET)
        filtered_colormap = cv2.applyColorMap(filtered_normalized, cv2.COLORMAP_JET)

        combined_view = np.hstack((raw_colormap, filtered_colormap))

        cv2.putText(combined_view, "Unfiltered Depth", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(combined_view, "Filtered Depth", (raw_colormap.shape[1] + 10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        cv2.imshow("Unfiltered vs. Filtered Depth", combined_view)

        if cv2.waitKey(1) == ord('q'):
            break

    print("Application closed.")