""""
  Extract robot positions from Ignition SQLite log file
"""
import cv2
import numpy as np
import re
import sqlite3
import xml.etree.ElementTree as ET

from datetime import timedelta

from subt.ign_pb2 import Pose_V

COLORS = [
    (255, 255, 255),
    (0, 255, 0),
    (0, 0, 255),
    (255, 0, 0),
    (0, 255, 255),
    (255, 0, 255),
    (255, 255, 0),
]
SCALE = 10  # 1 pixel is 1dm
BORDER_PX = 10  # extra border

MARKERS = {
    'backpack': (cv2.MARKER_SQUARE, 3),
    'rescue_randy': (cv2.MARKER_DIAMOND, 2),
    'gas': (cv2.MARKER_TRIANGLE_UP, 1),
    'vent': (cv2.MARKER_STAR, 4),
    'phone': (cv2.MARKER_TILTED_CROSS, 5),
    'artifact_origin': (cv2.MARKER_CROSS, 6),
}


def read_poses(filename, seconds=3700):
    ret = []

    con = sqlite3.connect(filename)
    cursor = con.cursor()
    cursor.execute(r"SELECT id FROM topics where name LIKE '%/dynamic_pose/info';")
    result = cursor.fetchone()
    dynamic_topic_id = result[0]

    poses = Pose_V()
    try:
        # cannot use WHERE filtering since the state.log is always corrupted
        cursor.execute("SELECT message, topic_id FROM messages")
        for m, topic_id in cursor:
            if topic_id != dynamic_topic_id:
                continue
            poses.ParseFromString(m)
            timestamp = timedelta(seconds=poses.header.stamp.sec, microseconds=poses.header.stamp.nsec / 1000)
            if timestamp > timedelta(seconds=seconds):
                return ret
            current = dict()
            for pose in poses.pose:
                if "_" in pose.name:
                    continue
                current[pose.name] = pose.position
            if len(current) > 0:
                ret.append((timestamp, current))
    except sqlite3.DatabaseError as e:
        print(f"{type(e).__name__}: {e}")

    return ret


def read_artifacts(filename):
    ret = []
    con = sqlite3.connect(filename)
    cursor = con.cursor()
    cursor.execute(f"SELECT id FROM topics WHERE name == '/logs/sdf'")
    result = cursor.fetchone()
    sdf_id = result[0]
    cursor.execute(r"SELECT message, topic_id FROM messages")
    for message, topic_id in cursor:
        if topic_id == sdf_id:
            world = message
            break
    root = ET.fromstring(world[4:])
    type_re = re.compile('^(backpack|rescue_randy|gas|vent|phone|artifact_origin)')
    for model in root.iterfind("./world/model"):
        name = model.get('name')
        match = type_re.match(name)
        if match:
            kind = match.group(1)
            x, y, z = [float(a) for a in model.find('./pose').text.split()[:3]]
            ret.append([kind, [x,y,z]])
    return ret


def draw(poses, artifacts):
    min_x, min_y = 10_000, 10_000
    max_x, max_y = -10_000, -10_000
    for timestamp, sample in poses:
        for k, v in sample.items():
            min_x = min_x if v.x > min_x else v.x
            min_y = min_y if v.y > min_y else v.y
            max_x = max_x if v.x < max_x else v.x
            max_y = max_y if v.y < max_y else v.y
    for kind, p in artifacts:
        min_x = min_x if p[0] > min_x else p[0]
        min_y = min_y if p[1] > min_y else p[1]
        max_x = max_x if p[0] < max_x else p[0]
        max_y = max_y if p[1] < max_y else p[1]

    print(f"min x: {min_x:.2f} y: {min_y:.2f}")
    print(f"max x: {max_x:.2f} y: {max_y:.2f}")

    width_px = 2*BORDER_PX + int(SCALE*(max_x - min_x))
    height_px = 2*BORDER_PX + int(SCALE*(max_y - min_y))
    print(f"width: {width_px}px height: {height_px}px")

    colors = dict()
    world = np.zeros((height_px, width_px), dtype=np.uint8)

    # draw cross at (0,0)
    #px = int(SCALE * (0 - min_x)) + BORDER_PX
    #py = int(SCALE * (0 - min_y)) + BORDER_PX
    #cv2.line(world, (px, py - 20), (px, py + 20), 255, 3)
    #cv2.line(world, (px - 20, py), (px + 20, py), 255, 3)

    for kind, p in artifacts:
        px = int(SCALE * (p[0] - min_x)) + BORDER_PX
        py = int(SCALE * (p[1] - min_y)) + BORDER_PX
        point = (px, height_px - py - 1)
        cv2.drawMarker(world, point, MARKERS[kind][1], markerType=MARKERS[kind][0], thickness=3)

    for timestamp, sample in poses:
        for k, v in sample.items():
            if k not in colors:
                colors[k] = len(colors)+1
            px = int(SCALE*(v.x - min_x)) + BORDER_PX
            py = int(SCALE*(v.y - min_y)) + BORDER_PX
            world[height_px - py - 1, px] = colors[k]

    user_color_map = np.zeros((256, 1, 3), dtype=np.uint8)
    user_color_map[0] = (0, 0, 0)
    for i in range(len(COLORS)):
        user_color_map[i+1] = COLORS[i]
    user_color_map[255] = (50, 50, 255)  # BGR -> Red
    cimg = cv2.applyColorMap(world, user_color_map)
    return cimg


def main():
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('filename', help='Ignition file from simulation (state.tlog)')
    parser.add_argument('-s', default=3700, type=float)
    parser.add_argument('-a', '--artifacts', action='store_true', default=False)
    args = parser.parse_args()

    if args.artifacts:
        artifacts = read_artifacts(args.filename)
        origin = next(filter(lambda a: a[0] == 'artifact_origin', artifacts))[1]
        for kind, p in artifacts:
            corrected = ", ".join(f"{aa-oo:.2f}".rstrip('0').rstrip('.') for aa, oo in zip(p, origin))
            print(f"{kind:<15}", f"[{corrected}]")
        return

    p = read_poses(args.filename, args.s)
    a = read_artifacts(args.filename)
    img = draw(p, a)
    cv2.imwrite(args.filename+'.png', img)


if __name__ == "__main__":
    main()
