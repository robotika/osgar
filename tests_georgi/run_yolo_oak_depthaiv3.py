import depthai as dai
import cv2
import numpy as np

# --- Pipeline Definition ---
pipeline = dai.Pipeline()

# Define frame sizes
cam_preview_w, cam_preview_h = 640, 360
nn_input_w, nn_input_h = 640, 640

# 1. New Camera Node
cam = pipeline.create(dai.node.Camera)

# 2. Configure Camera
cam.build(
    boardSocket=dai.CameraBoardSocket.CAM_A,
    sensorResolution=(1920, 1080),
    sensorFps=15
)

# 3. Request Specific Output
# Request a 640x360 RGB stream
cam_out = cam.requestOutput((cam_preview_w, cam_preview_h), dai.ImgFrame.Type.RGB888p)

# 4. ImageManip for letterboxing
manip = pipeline.create(dai.node.ImageManip)

# Use setOutputSize with LETTERBOX mode (Replaces setResize/setResizeThumbnail)
manip.initialConfig.setFrameType(dai.ImgFrame.Type.RGB888p)
manip.initialConfig.setOutputSize(nn_input_w, nn_input_h, dai.ImageManipConfig.ResizeMode.LETTERBOX)

# Note: If setOutputSize alone doesn't letterbox correctly in your specific minor version,
# you can explicitly use the ResizeMode enum:
# manip.initialConfig.setOutputSize(nn_input_w, nn_input_h, dai.ImageManipConfig.ResizeMode.LETTERBOX)

manip.setMaxOutputFrameSize(nn_input_w * nn_input_h * 3)

# Neural Network
nn = pipeline.create(dai.node.NeuralNetwork)
nn.setBlobPath("yolo11s-pose.blob")

# Linking
cam_out.link(manip.inputImage)
manip.out.link(nn.input)

# --- Queue Creation (MUST BE BEFORE START) ---
# Queues create XLinkOut nodes internally, so they must be defined
# BEFORE the pipeline is frozen and sent to the device.
q_rgb = cam_out.createOutputQueue(maxSize=4, blocking=False)
q_nn = nn.out.createOutputQueue(maxSize=4, blocking=False)

# --- Running the Pipeline ---
# 5. Start the pipeline AFTER defining all queues
device = pipeline.start()

WINDOW_NAME = "OAK Pose Estimation"
cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
cv2.setWindowProperty(WINDOW_NAME, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

print("Pipeline started. Press 'q' to exit.")

while pipeline.isRunning():
    in_rgb = q_rgb.get()
    frame = in_rgb.getCvFrame()

    in_nn = q_nn.get()

    # specific generic access:
    layer_names = in_nn.getAllLayerNames()
    if layer_names:
        # Get the first layer's data (equivalent to old getFirstLayerFp16)
        # 1. Get all raw data from the NN output (returns uint8 numpy array)
        data_raw = in_nn.getData()

        # 2. View it as Float16 (standard for OAK blobs)
        # Note: If your model is compiled for FP32, change to np.float32
        detections_flat = data_raw.view(np.float16)
    else:
        # Fallback/Safety if no layers found
        print("No output layers found in NNData")
        continue

    CONF_THRESH = 0.5
    NMS_THRESH = 0.5
    NUM_KEYPOINTS = 17

    try:
        # Shape: (Rows, Columns). The raw blob output usually needs transposing to be [8400, 56] or similar
        # Your original code used .reshape((56, 8400)).T -> Resulting shape (8400, 56)
        detections = detections_flat.reshape((56, 8400)).T

        confidences = detections[:, 4]
        mask = confidences > CONF_THRESH

        high_conf_detections = detections[mask]

        if high_conf_detections.shape[0] == 0:
            cv2.imshow(WINDOW_NAME, frame)
            if cv2.waitKey(1) == ord('q'): break
            continue

        bboxes_center_xywh = high_conf_detections[:, :4]
        confidences_filtered = high_conf_detections[:, 4]
        keypoints_raw = high_conf_detections[:, 5:]

        x_center, y_center, w, h = bboxes_center_xywh.T
        x1 = x_center - w / 2
        y1 = y_center - h / 2

        bboxes_for_nms = np.column_stack((x1, y1, w, h)).astype(int).tolist()
        confidences_for_nms = confidences_filtered.tolist()

        indices = cv2.dnn.NMSBoxes(bboxes_for_nms, confidences_for_nms, CONF_THRESH, NMS_THRESH)

        if len(indices) > 0:
            pad_y = (nn_input_h - cam_preview_h) / 2

            for frame_id, i in enumerate(indices.flatten()):
                x_nn, y_nn, w_nn, h_nn = bboxes_for_nms[i]
                x1_display = int(x_nn)
                y1_display = int(y_nn - pad_y)
                x2_display = int(x_nn + w_nn)
                y2_display = int(y_nn + h_nn - pad_y)
                cv2.rectangle(frame, (x1_display, y1_display), (x2_display, y2_display), (0, 255, 0), 2)
                confidence = confidences_for_nms[i]
                label = f"Person {frame_id}: {confidence:.2f}"
                label_size, baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
                label_y = max(y1_display, label_size[1] + 10)
                cv2.rectangle(frame, (x1_display, label_y - label_size[1] - 10),
                              (x1_display + label_size[0], label_y + baseline - 10), (0, 255, 0), cv2.FILLED)
                cv2.putText(frame, label, (x1_display, label_y - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)

                keypoints = keypoints_raw[i].reshape((NUM_KEYPOINTS, 3))
                kp_x_nn = keypoints[:, 0]
                kp_y_nn = keypoints[:, 1]
                kp_conf = keypoints[:, 2]

                conf_mask = kp_conf > 0.5
                px_display = (kp_x_nn[conf_mask]).astype(int)
                py_display = (kp_y_nn[conf_mask] - pad_y).astype(int)

                for x, y in zip(px_display, py_display):
                    if 0 <= x < cam_preview_w and 0 <= y < cam_preview_h:
                        cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)

    except Exception as e:
        print(f"An error occurred: {e}")

    cv2.imshow(WINDOW_NAME, frame)

    if cv2.waitKey(1) == ord('q'):
        break