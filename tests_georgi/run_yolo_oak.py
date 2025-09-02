import depthai as dai
import cv2
import numpy as np

# --- Pipeline Definition ---
pipeline = dai.Pipeline()

# Define frame sizes for clarity
cam_preview_w, cam_preview_h = 640, 360
nn_input_w, nn_input_h = 640, 640

# Color Camera
cam_rgb = pipeline.create(dai.node.ColorCamera)
cam_rgb.setPreviewSize(cam_preview_w, cam_preview_h)
cam_rgb.setInterleaved(False)
cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
cam_rgb.setFps(15)

# ImageManip for letterboxing
manip = pipeline.create(dai.node.ImageManip)
manip.initialConfig.setResizeThumbnail(nn_input_w, nn_input_h)
manip.setMaxOutputFrameSize(nn_input_w * nn_input_h * 3)
manip.initialConfig.setFrameType(dai.ImgFrame.Type.RGB888p)

# Neural Network
nn = pipeline.create(dai.node.NeuralNetwork)
nn.setBlobPath("yolo11n-pose.blob")

# Outputs
nn_out = pipeline.create(dai.node.XLinkOut)
nn_out.setStreamName("nn")
xout_rgb = pipeline.create(dai.node.XLinkOut)
xout_rgb.setStreamName("rgb")

# Linking
cam_rgb.preview.link(manip.inputImage)
manip.out.link(nn.input)
cam_rgb.preview.link(xout_rgb.input)
nn.out.link(nn_out.input)

# --- Running the Pipeline ---
with dai.Device(pipeline) as device:
    q_rgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
    q_nn = device.getOutputQueue(name="nn", maxSize=4, blocking=False)

    WINDOW_NAME = "OAK Pose Estimation"
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.setWindowProperty(WINDOW_NAME, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    while True:
        in_rgb = q_rgb.get()
        frame = in_rgb.getCvFrame()

        in_nn = q_nn.get()
        detections_flat = np.array(in_nn.getFirstLayerFp16())

        CONF_THRESH = 0.5
        NMS_THRESH = 0.5
        NUM_KEYPOINTS = 17

        try:
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
                    # Draw bounding box and label
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

                    # 1. Reshape all keypoints for the current person
                    keypoints = keypoints_raw[i].reshape((NUM_KEYPOINTS, 3))

                    # 2. Extract coordinates and confidence scores into NumPy arrays
                    kp_x_nn = keypoints[:, 0]
                    kp_y_nn = keypoints[:, 1]
                    kp_conf = keypoints[:, 2]

                    # 3. Create a mask for high-confidence keypoints
                    conf_mask = kp_conf > 0.5

                    # 4. Apply coordinate mapping to all keypoints at once
                    px_display = (kp_x_nn[conf_mask]).astype(int)
                    py_display = (kp_y_nn[conf_mask] - pad_y).astype(int)

                    # 5. This loop is now only for drawing the filtered keypoints
                    for x, y in zip(px_display, py_display):
                        # Optional: A final check to ensure points are within frame bounds
                        if 0 <= x < cam_preview_w and 0 <= y < cam_preview_h:
                            cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)

        except Exception as e:
            print(f"An error occurred: {e}")

        cv2.imshow(WINDOW_NAME, frame)

        if cv2.waitKey(1) == ord('q'):
            break