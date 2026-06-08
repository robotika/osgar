from ultralytics import YOLO

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description='Export YOLO model to OAK camera')

    # ULTRALYTICS IMAGES: "https://ultralytics.com/images/bus.jpg"
    args = parser.parse_args()
    model = YOLO("yolo11n-pose.pt")  # load an official model
    model.export(format="onnx")
