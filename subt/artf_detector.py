import numpy as np
import torch

import subt.artf_model


class Detector:
    def __init__(self, model, confidence_thresholds, categories, device,
                 max_gap, min_group_size):
        self.model = model
        self.confidence_thresholds = confidence_thresholds
        self.categories = categories
        self.device = device
        self.max_gap = max_gap
        self.min_group_size = min_group_size

    def __call__(self, img):
        with torch.set_grad_enabled(False):
            timg = torch.from_numpy(img).float().to(self.device)
            # We work with a batch of a single image.
            minibatch = timg.unsqueeze(0)
            predictions = np.ascontiguousarray(
                self.model(minibatch)[0].detach().cpu().numpy())
        scale = img.shape[0] // predictions.shape[0]
        indices = np.indices(predictions.shape[:2]) * scale

        artifacts = []
        for name, aid in self.categories.items():
            threshold = self.confidence_thresholds.get(name)
            if threshold is None:
                continue
            detected = predictions[:, :, aid] >= threshold
            ys = indices[0, detected].reshape((-1, ))
            xs = indices[1, detected].reshape((-1, ))
            for (x, y) in zip(xs, ys):
                confidence = predictions[y // scale, x // scale, aid]
                artifacts.append((x, y, name, confidence))
        return Detector.prune_small_groups(
            Detector.group_points(artifacts, self.max_gap),
            self.min_group_size)

    @staticmethod
    def group_points(artifacts, max_gap):
        belongs_to = dict()  # Maps from points to a group they belong to.
        consists_of = dict()  # Maps from group ids to a list of points.
        next_group_id = 0
        X = 0
        Y = 1
        CATEGORY = 2
        CONFIDENCE = 3
        for i in range(len(artifacts)):
            artf_i = artifacts[i]
            group_i = next_group_id
            next_group_id += 1
            belongs_to[i] = group_i
            consists_of[group_i] = [i]
            category_i = artf_i[CATEGORY]
            for j in range(i):
                artf_j = artifacts[j]
                category_j = artf_j[CATEGORY]
                dist = np.hypot(artf_i[X] - artf_j[X], artf_i[Y] - artf_j[Y])
                if dist <= max_gap and category_i == category_j:
                    group_j = belongs_to[j]
                    if group_i == group_j:
                        # The two points allready belong to the same group and
                        # there is nothing to be done.
                        continue
                    else:
                        # We need to merge two groups.
                        # We are merging a smaller group into the bigger one to
                        # pretend we care about speed. For the small groups we
                        # expect, this probably does not make much difference.
                        if (len(consists_of[group_j]) >= len(
                                consists_of[group_i])):
                            merge_to_group = group_j
                            merge_from_group = group_i
                        else:
                            merge_to_group = group_i
                            merge_from_group = group_j
                        for k in consists_of[merge_from_group]:
                            belongs_to[k] = merge_to_group
                            consists_of[merge_to_group].append(k)
                        del consists_of[merge_from_group]
                        group_i = merge_to_group

        groups = []
        for group in consists_of.values():
            # We want the most representative point in the group to go first,
            # because the neural network sometimes classifies points of the
            # object to still belong to it. In a 3D scene, these points have a
            # very different coordinate and would lead to errors!
            cx = np.mean([artifacts[i][X] for i in group])
            cy = np.mean([artifacts[i][Y] for i in group])
            group.sort(key=lambda i: np.hypot(artifacts[i][X] - cx, artifacts[
                i][Y] - cy))
            category = artifacts[group[0]][CATEGORY]
            groups.append((category, [(artifacts[i][X], artifacts[i][Y],
                                       artifacts[i][CONFIDENCE])
                                      for i in group]))
        return groups

    @staticmethod
    def prune_small_groups(groups, min_group_size):
        POINTS = 1
        return [
            group for group in groups if len(group[POINTS]) >= min_group_size
        ]


def thresholds_param(arg):
    thresholds = {}
    parts = arg.split(',')
    for part in parts:
        subparts = part.split(':')
        thresholds[subparts[0]] = float(subparts[1])
    return thresholds


if __name__ == '__main__':
    import argparse
    import cv2
    import os
    import sys

    from osgar.lib.serialize import deserialize
    from osgar.logger import LogReader, lookup_stream_id

    parser = argparse.ArgumentParser()
    parser.add_argument('--model',
                        default=None,
                        help='Path to model parameters. Example: ' +
                        'mdnet0.64.64.13.4.relu.pth')
    parser.add_argument('--logfile', help='Path to Osgar log.')
    parser.add_argument('--image-stream',
                        help='Stream id or name with image data.')
    parser.add_argument('--image', help='An image to process.')
    parser.add_argument('--output', help='Where to store detection images.')
    parser.add_argument('--confidence-thresholds',
                        type=thresholds_param,
                        default={},
                        help='Minimum confidence needed to declare a ' +
                        'detected artifact. Example: survivor:0.95,' +
                        'vent:0.878,backpack:0.977,phone:0.97,' +
                        'helmet:0.963,rope:0.99')
    parser.add_argument(
        '--max-gap',
        type=int,
        default=16,
        help='Maximum gap, in pixels, between two points that' +
        'we are still willing to consider to belong to ' +
        'the same artifact.')
    parser.add_argument('--min-group-size',
                        type=int,
                        default=2,
                        help='There need to be at least this many points in ' +
                        'an artifact for it to be considered real and ' +
                        'not noise.')
    parser.add_argument(
        '--show',
        default=False,
        action='store_true',
        help='Enable visualization of the scene and detected ' + 'artifacts.')
    args = parser.parse_args()
    assert (args.model)
    assert ((args.logfile and args.image_stream) or (args.image))

    use_cuda = torch.cuda.is_available()
    device = torch.device("cuda" if use_cuda else "cpu")
    print('Using:', device, file=sys.stderr)
    model, categories = subt.artf_model.load_model(args.model, device)
    detector = Detector(model, args.confidence_thresholds, categories, device,
                        args.max_gap, args.min_group_size)

    n = 0
    if args.logfile:
        image_stream_id = lookup_stream_id(args.logfile, args.image_stream)
        with LogReader(args.logfile,
                       only_stream_id=image_stream_id) as logreader:
            for time, stream, data in logreader:
                assert (stream == image_stream_id)
                img = cv2.imdecode(np.frombuffer(deserialize(data), np.uint8),
                                   cv2.IMREAD_COLOR)
                artifacts = detector(img)
                print('---')
                for name, group in artifacts:
                    print(name)
                    for x, y, confidence in group:
                        print(' ', confidence, x, y)
                    if args.show or args.output:
                        min_x = min(x for (x, _, _) in group)
                        min_y = min(y for (_, y, _) in group)
                        max_x = max(x for (x, _, _) in group)
                        max_y = max(y for (_, y, _) in group)
                        # Rectangle around all detected points.
                        cv2.rectangle(img, (min_x, min_y), (max_x, max_y), (0, 255, 0))
                        # Main point.
                        cv2.circle(img, group[0][:2], 5, (0, 0, 255), -1)
                if artifacts and args.output:
                    cv2.imwrite(
                        os.path.join(args.output, '{:06d}.jpg'.format(n)),
                        img)
                n += 1
                if args.show:
                    cv2.imshow('img', img)
                    KEY_Q = ord('q')
                    if artifacts:
                        key = cv2.waitKey() & 0xFF
                    else:
                        key = cv2.waitKey(3) & 0xFF
                    if key == KEY_Q:
                        break

    if args.image:
        img = cv2.imread(args.image, cv2.IMREAD_COLOR)
        artifacts = detector(img)
        for name, group in artifacts:
            print(name)
            for x, y, confidence in group:
                print('  ', confidence, x, y)
            if args.show or args.output:
                min_x = min(x for (x, _, _) in group)
                min_y = min(y for (_, y, _) in group)
                max_x = max(x for (x, _, _) in group)
                max_y = max(y for (_, y, _) in group)
                # Rectangle around all detected points.
                cv2.rectangle(img, (min_x, min_y), (max_x, max_y), (0, 255, 0))
                # Main point.
                cv2.circle(img, group[0][:2], 5, (0, 0, 255), -1)
        if args.output:
            cv2.imwrite(
                os.path.join(args.output, os.path.basename(args.image)), img)
        if args.show:
            cv2.imshow('img', img)
            cv2.waitKey()
