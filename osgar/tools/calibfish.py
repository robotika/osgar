"""
Calibrate camera using fisheye model from opencv.

https://docs.opencv.org/trunk/db/d58/group__calib3d__fisheye.html#details

Collect images to `images/capture*.jpg` by pressing "S".
If precaptured images are found, calibration is run directly.
It is expected that chessboard calibration pattern 9x6 is on the images.
Suitable pattern can be generated using
https://calib.io/pages/camera-calibration-pattern-generator

https://stackoverflow.com/a/50876130/3185929
https://github.com/njanirudh/Aruco_Tracker/blob/master/camera_calibration.py
"""

import cv2
import glob
import numpy as np

PATTERN = (8,5)

def capture(camera=0):
    cap = cv2.VideoCapture(camera)
    if not cap.isOpened():
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

    count = 0
    while True:
        status, img = cap.read()
        if status == False:
            continue
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        gray = cv2.resize(gray, (img.shape[1]//2, img.shape[0]//2))
        found, corners = cv2.findChessboardCorners(gray, PATTERN, cv2.CALIB_CB_ADAPTIVE_THRESH+cv2.CALIB_CB_FAST_CHECK+cv2.CALIB_CB_NORMALIZE_IMAGE)
        img_copy = img.copy()
        if found:
            corners *= 2
            cv2.drawChessboardCorners(img, PATTERN, corners, found)
        cv2.imshow('img', img)
        key = cv2.waitKey(1)
        if key == 27:
            return
        if key == ord('s'):
            if found:
                filename = 'capture{:02d}.jpg'.format(count)
                print(filename)
                cv2.imwrite('images/'+filename, img_copy)
                count += 1


def calibrate(images):
    subpix_criteria = cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1
    calibration_flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC + cv2.fisheye.CALIB_CHECK_COND + cv2.fisheye.CALIB_FIX_SKEW
    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(PATTERN[0],PATTERN[1],0)
    # but for some crazy reasons it needs to have the shape (1, x, 3)
    objp = np.zeros((1, PATTERN[0]*PATTERN[1],3), np.float32)
    objp[0,:,:2] = np.mgrid[0:PATTERN[0], 0:PATTERN[1]].T.reshape(-1,2)
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.
    for image in sorted(images):
        gray = cv2.imread(image, cv2.IMREAD_GRAYSCALE)
        found, corners = cv2.findChessboardCorners(gray, PATTERN, cv2.CALIB_CB_ADAPTIVE_THRESH+cv2.CALIB_CB_FAST_CHECK+cv2.CALIB_CB_NORMALIZE_IMAGE)
        print(image, found)
        if found:
            objpoints.append(objp)
            cv2.cornerSubPix(gray, corners, (3,3), (-1,-1), subpix_criteria)
            imgpoints.append(corners)
    K = np.zeros((3, 3))
    D = np.zeros((4, 1))
    rms, _, _, _, _ = cv2.fisheye.calibrate(objpoints, imgpoints, gray.shape[::-1], K, D,
        flags=calibration_flags, criteria=(cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6))

    return len(objpoints), rms, gray.shape[::-1], K, D


def fish2perspective(images, K, D, DIM):
    NK = K.copy()
    NK[0,0] /= 2
    NK[1,1] /= 2

    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), NK, DIM, cv2.CV_16SC2)
    for fname in sorted(images):
        img = cv2.imread(fname)
        undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        cv2.imshow('a', undistorted_img)
        print(fname)
        if cv2.waitKey(0) == 27:
            return


def main():
    import argparse

    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--camera', help='camera id', type=int, default=0)
    args = parser.parse_args()

    images = glob.glob('images/capture*.jpg')
    if len(images) == 0:
        capture(args.camera)
    images = glob.glob('images/capture*.jpg')
    if len(images) == 0:
        return
    N, rms, DIM, K, D = calibrate(images)
    print("Found " + str(N) + " valid images for calibration")
    print("RMS = " + str(rms))
    print("DIM = " + str(DIM))
    print("K = np.array(" + str(K.tolist()) + ")")
    print("D = np.array(" + str(D.tolist()) + ")")

    fish2perspective(images, K, D, DIM)


if __name__ == "__main__":
    main()
