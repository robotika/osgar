"""
  Extract log images observing given location
"""

import os

from osgar.logger import LogReader, lookup_stream_names
from osgar.lib.serialize import deserialize
from subt.trace import distance3D


def debug2dir(filename, out_dir, detector_name):
    names = lookup_stream_names(filename)
    assert detector_name + '.debug_rgbd' in names, names
    assert detector_name + '.debug_camera' in names, names
    assert detector_name + '.localized_artf' in names, names
    assert detector_name + '.debug_cv_result' in names, names
    assert 'rosmsg.sim_time_sec' in names, names
    rgbd_id = names.index(detector_name + '.debug_rgbd') + 1
    camera_id = names.index(detector_name + '.debug_camera') + 1
    artf_id = names.index(detector_name + '.localized_artf') + 1
    result_id = names.index(detector_name + '.debug_cv_result') + 1
    sim_sec_id = names.index('rosmsg.sim_time_sec') + 1
    sim_time_sec = None
    artf = None
    last_result = None
    out_json = {
        "_via_settings": {
            "ui": {
                "annotation_editor_height": 25,
                "annotation_editor_fontsize": 0.8,
                "leftsidebar_width": 18,
                "image_grid": {
                    "img_height": 80,
                    "rshape_fill": "none",
                    "rshape_fill_opacity": 0.3,
                    "rshape_stroke": "yellow",
                    "rshape_stroke_width": 2,
                    "show_region_shape": True,
                    "show_image_policy": "all"
                },
                "image": {
                    "region_label": "artifact",
                    "region_color": "artifact",
                    "region_label_font": "10px Sans",
                    "on_image_annotation_editor_placement": "NEAR_REGION"
                }
            },
            "core": {
                "buffer_size": 18,
                "filepath": {},
                "default_filepath": ""
            },
            "project": {
                "name": "subt2020"
            }
        },

        "_via_img_metadata": {
        },
            "_via_attributes": {
    "region": {
      "artifact": {
        "type": "dropdown",
        "description": "",
        "options": {
          "backpack": "",
          "phone": "",
          "survivor": "",
          "robot": "",
          "nothing": "",
          "vent": "",
          "fire_extinguisher": "",
          "rope": "",
          "helmet": "",
          "breadcrumb": "",
          "drill": "",
          "cube": ""
        },
        "default_options": {
          "helmet": True
        }
      }
    },
    "file": {}
  }
    }
    stream_ids = [rgbd_id, camera_id, artf_id, result_id, sim_sec_id]
    for dt, channel, data in LogReader(filename, only_stream_id=stream_ids):
        data = deserialize(data)
        if channel == sim_sec_id:
            sim_time_sec = data
        elif channel == artf_id:
            artf = data
        elif channel == result_id:
            last_result = data
        elif channel in [rgbd_id, camera_id]:
            # 'debug_rgbd' is the last published topic for given detection
            if channel == rgbd_id:
                robot_pose, camera_pose, rgb_compressed, depth_compressed = data
            else:  # channel == camera_id
                # image stereo artefact localization
                # expects localized pair of images [camera_name, [robot_pose, camera_pose, image], [robot_pose, camera_pose, image]]
                assert len(data) == 3, data[0]
                rgb_compressed = data[1][2]  # first image of stereo pair
            image = rgb_compressed
            assert artf is not None
            time_sec = sim_time_sec if sim_time_sec is not None else int(dt.total_seconds())
            name = os.path.basename(filename)[:-4] + '-' + artf[0] + '-' + str(time_sec) + '.jpg'
            print(name, last_result)
            with open(os.path.join(out_dir, name), 'wb') as f:
                f.write(image)
            out_json["_via_img_metadata"][name + '-1'] = get_annotation_item(name, last_result)

    with open(os.path.join(out_dir, 'annotation.json'), 'w') as f:
        json.dump(out_json, f, indent=2)


POSE3D_STREAM = 'fromrospy.pose3d'


def locimg(logfile, loc, out_dir, stream_name, radius):
    print(loc)
    names = lookup_stream_names(logfile)
    assert POSE3D_STREAM in names, (POSE3D_STREAM, names)
    assert stream_name in names, (stream_name, names)
    pose_id = names.index(POSE3D_STREAM) + 1
    camera_id = names.index(stream_name) + 1

    stream_ids = [pose_id, camera_id]
    img_count = 0
    selected = 0
    last_pose3d = None
    for dt, channel, data in LogReader(logfile, only_stream_id=stream_ids):
        data = deserialize(data)
        if channel == pose_id:
            last_pose3d = data
        elif channel == camera_id:
            if last_pose3d is not None and distance3D(last_pose3d[0], loc) < radius:
                selected += 1
                if out_dir is not None:
                    with open(os.path.join(out_dir, 'image_%04d.jpg' % img_count), 'wb') as f:
                        f.write(data)
            img_count += 1
    print(f'Images {selected}/{img_count}')


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('logfile', help='recorded log file')
    parser.add_argument('--out-dir', help='where tu dump relevant images')
    parser.add_argument('--stream', help='image source', default='logimage.image')
    parser.add_argument('--loc', help='xyz position', nargs=3, type=float, required=True)
    parser.add_argument('-r', '--radius', help='sphere radius of interest', type=float, default=5.0)
    args = parser.parse_args()

    locimg(args.logfile, args.loc, args.out_dir, args.stream, radius=args.radius)

# vim: expandtab sw=4 ts=4
