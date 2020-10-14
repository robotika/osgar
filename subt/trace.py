
import math

TRACE_STEP = 0.5  # meters in 3D


def distance3D(xyz1, xyz2, weights=[1.0, 1.0, 1.0]):
    return math.sqrt(sum([w * (a-b)**2 for a, b, w in zip(xyz1, xyz2, weights)]))


class Trace:
    def __init__(self, step=TRACE_STEP):
        self.trace = []  # unknown start position
        self.step = step

    def update_trace(self, pos_xyz):
        if not self.trace or distance3D(self.trace[-1], pos_xyz) >= self.step:
            self.trace.append(pos_xyz)

    def prune(self, radius=None):
        # use short-cuts and remove all cycles
        if radius is None:
            radius = self.step

        pruned = Trace(step=self.step)
        if self.trace:
            # keep the very first position in pruned version
            pruned.update_trace(self.trace[0])

        open_end = 1
        while open_end < len(self.trace):
            best = open_end
            for i, xyz in enumerate(self.trace[open_end:], start=open_end):
                if distance3D(xyz, pruned.trace[-1]) < radius:
                    best = i
            pruned.update_trace(self.trace[best])
            open_end = best + 1
        self.trace = pruned.trace


    def where_to(self, xyz, max_target_distance, z_weight=0.2):
        # looking for a target point within max_target_distance nearest to the start
        for _ in range(8):
            for target in self.trace:
                if distance3D(target, xyz, [1.0, 1.0, z_weight]) < max_target_distance:
                    return target
            # if the robot deviated too far from the trajectory, we need to look for more distant target points
            max_target_distance *= 1.5
        # robot is crazy far from the trajectory
        assert(False)

    def reverse(self):
        self.trace.reverse()

    def add_line_to(self, xyz):
        if self.trace:
            last = self.trace[-1]
            size = distance3D(last, xyz)
            dx, dy, dz = xyz[0] - last[0], xyz[1] - last[1], xyz[2] - last[2]
            for i in range(1, int(size/self.step)):
                s = self.step * i/size
                self.trace.append((last[0] + s*dx, last[1] + s*dy, last[2] + s*dz))
        self.trace.append(xyz)
