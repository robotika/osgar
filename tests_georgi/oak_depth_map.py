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

    # Define sources and output nodes
    mono_left = pipeline.create(dai.node.MonoCamera)
    mono_right = pipeline.create(dai.node.MonoCamera)
    xout_raw_depth = pipeline.create(dai.node.XLinkOut)
    xout_filtered_depth = pipeline.create(dai.node.XLinkOut)

    xout_raw_depth.setStreamName("raw_depth")
    xout_filtered_depth.setStreamName("filtered_depth")

    # Mono Camera Properties
    mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    mono_left.setCamera("left")
    mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    mono_right.setCamera("right")

    # --- NEW: Create TWO StereoDepth nodes ---
    raw_stereo = pipeline.create(dai.node.StereoDepth)
    filtered_stereo = pipeline.create(dai.node.StereoDepth)

    # --- Configure the RAW StereoDepth Node (No Filters) ---
    raw_stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
    raw_stereo.setDepthAlign(dai.CameraBoardSocket.CAM_C)
    raw_stereo.setExtendedDisparity(False)
    raw_stereo.setSubpixel(True)
    raw_stereo.setLeftRightCheck(True) # Keep LR-check for basic quality
    # Ensure all filters are disabled for the raw output
    raw_stereo.initialConfig.setMedianFilter(dai.MedianFilter.MEDIAN_OFF)
    raw_stereo.initialConfig.PostProcessing.SpeckleFilter.enable = False
    raw_stereo.initialConfig.PostProcessing.TemporalFilter.enable = False


    # --- Configure the FILTERED StereoDepth Node ---
    filtered_stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
    filtered_stereo.setDepthAlign(dai.CameraBoardSocket.CAM_C)
    filtered_stereo.setExtendedDisparity(False)
    filtered_stereo.setSubpixel(True)
    filtered_stereo.setLeftRightCheck(True)
    # Enable all desired filters
    # filtered_stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
    filtered_stereo.initialConfig.setConfidenceThreshold(255)

    # --- Enable and configure the Spatial Filter ---
    filtered_stereo.initialConfig.PostProcessing.SpatialFilter.enable = False
    filtered_stereo.initialConfig.PostProcessing.SpatialFilter.holeFillingRadius = 5
    filtered_stereo.initialConfig.PostProcessing.SpatialFilter.alpha = 0.1
    filtered_stereo.initialConfig.PostProcessing.SpatialFilter.delta = 0
    filtered_stereo.initialConfig.PostProcessing.SpatialFilter.numIterations = 2

    # Speckle filter removes small, noisy speckles
    filtered_stereo.initialConfig.PostProcessing.SpeckleFilter.enable = False
    filtered_stereo.initialConfig.PostProcessing.SpeckleFilter.speckleRange = 50
    # Temporal filter averages depth data over time
    filtered_stereo.initialConfig.PostProcessing.TemporalFilter.enable = False
    filtered_stereo.initialConfig.PostProcessing.TemporalFilter.alpha = 0.9
    filtered_stereo.initialConfig.PostProcessing.TemporalFilter.delta = 8


    # --- Linking Nodes ---
    # Link mono cameras to BOTH stereo nodes
    mono_left.out.link(raw_stereo.left)
    mono_right.out.link(raw_stereo.right)
    mono_left.out.link(filtered_stereo.left)
    mono_right.out.link(filtered_stereo.right)

    # Link each stereo node to its own output
    raw_stereo.depth.link(xout_raw_depth.input)
    filtered_stereo.depth.link(xout_filtered_depth.input)

    # Return the pipeline and one of the stereo nodes to get max_disparity
    return pipeline, filtered_stereo


# --- Main Application Logic ---
if __name__ == "__main__":
    pipeline, stereo_node = create_pipeline()

    with dai.Device(pipeline) as device:
        # --- NEW: Get both output queues ---
        raw_depth_queue = device.getOutputQueue(name="raw_depth", maxSize=4, blocking=False)
        filtered_depth_queue = device.getOutputQueue(name="filtered_depth", maxSize=4, blocking=False)

        max_disparity = stereo_node.initialConfig.getMaxDisparity()
        print("Connected to OAK-D. Press 'q' to quit.")
        projector_configured = False

        while True:
            # --- NEW: Get frames from both queues ---
            in_raw = raw_depth_queue.get()
            in_filtered = filtered_depth_queue.get()

            # Configure projector once after the first frame arrives
            if not projector_configured and args.use_dot_projector:
                print(f"Setting dot projector brightness to: {args.dot_brightness}")
                try:
                    # Brightness for Pro models is 0-1200
                    device.setIrLaserDotProjectorIntensity(args.dot_brightness)
                except Exception as e:
                    print(f"Failed to set IR laser dot projector brightness: {e}")
                projector_configured = True

            raw_depth_frame = in_raw.getFrame()
            filtered_depth_frame = in_filtered.getFrame()

            # --- Post-processing and Visualization ---

            # 1. Normalize both frames for visualization
            raw_norm = (raw_depth_frame * (255 / max_disparity)).astype(np.uint8)
            filtered_norm = (filtered_depth_frame * (255 / max_disparity)).astype(np.uint8)

            # 2. Apply a colormap to each for better visualization
            raw_colormap = cv2.applyColorMap(raw_norm, cv2.COLORMAP_JET)
            filtered_colormap = cv2.applyColorMap(filtered_norm, cv2.COLORMAP_JET)

            # 3. Create a combined view to compare
            combined_view = np.hstack((raw_colormap, filtered_colormap))

            # 4. Add text labels to identify each view
            cv2.putText(combined_view, "Unfiltered Depth", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.putText(combined_view, "Filtered Depth", (raw_colormap.shape[1] + 10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

            cv2.imshow("Unfiltered vs. Filtered Depth", combined_view)

            if cv2.waitKey(1) == ord('q'):
                break

    print("Application closed.")
