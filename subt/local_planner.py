import math
import numpy as np
import cProfile


g_pr = cProfile.Profile()
g_count = 0


def normalize_angle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


class LocalPlannerRef:
    def __init__(self, scan_right=math.radians(-135), scan_left=math.radians(135), direction_adherence=math.radians(90),
                 max_obstacle_distance=1.5, obstacle_influence=1.2, scan_subsample=1, max_considered_obstacles=None):
        self.last_scan = None
        self.scan_right = scan_right
        self.scan_left = scan_left
        self.direction_adherence = direction_adherence
        self.max_obstacle_distance = max_obstacle_distance
        self.obstacle_influence = obstacle_influence
        self.scan_subsample = scan_subsample

    def update(self, scan):
        self.last_scan = scan[::self.scan_subsample] if self.scan_subsample > 1 else scan

    def recommend(self, desired_dir):
        if self.last_scan is None:
            return 1.0, desired_dir

        obstacles = []
        for (i, measurement) in enumerate(self.last_scan):
            if measurement == 0:
                continue
            if measurement * 1e-3 > self.max_obstacle_distance:
                continue
            measurement_angle = self.scan_right + (self.scan_left - self.scan_right) * i / float(len(self.last_scan) - 1)
            measurement_vector = math.cos(measurement_angle), math.sin(measurement_angle)

            # Converting from millimeters to meters.
            obstacle_xy = [mv * (measurement * 1e-3) for mv in measurement_vector]

            obstacles.append(obstacle_xy)

        if not obstacles:
            return 1.0, normalize_angle(desired_dir)

        # Best direction points roughly in desired_dir and does not come too close to any obstacle.
        def is_desired(direction):
            direction_delta = normalize_angle(direction - desired_dir)
            return math.exp(-(direction_delta / self.direction_adherence)**2)  # Fuzzy equality

        def is_risky(direction):
            direction_vector = math.cos(direction), math.sin(direction)
            riskiness = 0.
            for obstacle_xy in obstacles:
                # Obstacles behind the robot from the perspective of the desired direction do not matter.
                # The formula below computes cos(angle between the two vectors) * their_norms. Norms are positive, so a negative result implies abs(angle) > 90deg.
                if sum(d * o for (d, o) in zip(direction_vector, obstacle_xy)) < 0:
                    continue

                # Distance between the obstacle and line defined by direction.
                # https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
                # Norm of direction_vector is 1.0, so we do not need to divide by it.
                off_track_distance = abs(direction_vector[1] * obstacle_xy[0] - direction_vector[0] * obstacle_xy[1])

                r = math.exp(-(off_track_distance / self.obstacle_influence)**2)
                if r > riskiness: # max as fuzzy OR.
                    riskiness = r

            return riskiness

        def is_safe(direction):
            return 1.0 - is_risky(direction)  # Fuzzy negation.

        def is_good(direction):
            return min(is_safe(direction), is_desired(direction))  # Fuzzy AND.

        return max((is_good(math.radians(direction)), math.radians(direction)) for direction in range(-180, 180, 3))


class LocalPlannerOpt:
    """
    Optimized version of Local Planner.
    1) ignore all directions into obstacles - there is always a way out (lidar is 270 degrees so it can go freely back)
       and way into the obstacle will have high penalization cost.
    2) leave complex evaluation as the last step for already best direction
    """
    def __init__(self, scan_right=math.radians(-135), scan_left=math.radians(135), direction_adherence=math.radians(90),
                 max_obstacle_distance=1.5, obstacle_influence=1.2, scan_subsample=1, max_considered_obstacles=None):
        self.last_scan = None
        self.scan_right = scan_right
        self.scan_left = scan_left
        self.direction_adherence = direction_adherence
        self.max_obstacle_distance = max_obstacle_distance
        self.obstacle_influence = obstacle_influence
        self.scan_subsample = scan_subsample

    def update(self, scan):
        self.last_scan = scan[::self.scan_subsample] if self.scan_subsample > 1 else scan

    def recommend(self, desired_dir):
        if self.last_scan is None:
            return 1.0, desired_dir

        # valid candidate angles for consideration
        valid = [True] * 361  # indexed -180 .. 180

        obstacles = []
        for (i, measurement) in enumerate(self.last_scan):
            if measurement == 0:
                continue
            if measurement * 1e-3 > self.max_obstacle_distance:
                continue
            measurement_angle = self.scan_right + (self.scan_left - self.scan_right) * i / float(len(self.last_scan) - 1)
            measurement_vector = math.cos(measurement_angle), math.sin(measurement_angle)

            # Converting from millimeters to meters.
            obstacle_xy = [mv * measurement * 1e-3 for mv in measurement_vector]

            obstacles.append(obstacle_xy)
            valid[int(math.degrees(normalize_angle(measurement_angle))) + 180] = False

#        print('obstacles', len(obstacles))
        if not obstacles:
            return 1.0, normalize_angle(desired_dir)

        # Best direction points roughly in desired_dir and does not come too close to any obstacle.
        def is_desired(direction):
            direction_delta = normalize_angle(direction - desired_dir)
            return math.exp(-(direction_delta / self.direction_adherence)**2)  # Fuzzy equality

        def is_risky(direction):
            direction_vector = math.cos(direction), math.sin(direction)
            min_off_track_distance = 1000.
            for obstacle_xy in obstacles:
                # Obstacles behind the robot from the perspective of the desired direction do not matter.
                # The formula below computes cos(angle between the two vectors) * their_norms. Norms are positive, so a negative result implies abs(angle) > 90deg.
                if sum(d * o for (d, o) in zip(direction_vector, obstacle_xy)) < 0:
                    continue

                # Distance between the obstacle and line defined by direction.
                # https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
                # Norm of direction_vector is 1.0, so we do not need to divide by it.
                off_track_distance = abs(direction_vector[1] * obstacle_xy[0] - direction_vector[0] * obstacle_xy[1])

                if off_track_distance < min_off_track_distance: # max as fuzzy OR.
                    min_off_track_distance = off_track_distance

            riskiness = math.exp(-(min_off_track_distance / self.obstacle_influence)**2)
            return riskiness

        def is_safe(direction):
            return 1.0 - is_risky(direction)  # Fuzzy negation.

        def is_good(direction):
            return min(is_safe(direction), is_desired(direction))  # Fuzzy AND.

        return max((is_good(math.radians(direction)), math.radians(direction)) for direction in range(-180, 180, 3) if valid[direction + 180])

class LocalPlannerNumpy:
    def __init__(self, scan_right=math.radians(-135), scan_left=math.radians(135), direction_adherence=math.radians(90),
                 max_obstacle_distance=1.5, obstacle_influence=1.2, scan_subsample=1, max_considered_obstacles=None):
        self.last_scan = None
        self.scan_right = scan_right
        self.scan_left = scan_left
        self.direction_adherence = direction_adherence
        self.max_obstacle_distance = max_obstacle_distance
        self.obstacle_influence = obstacle_influence

        self.considered_directions = np.radians(
                np.linspace(
                    -180, +180, 360//3, endpoint=False)).reshape((-1, 1))
        self.considered_directions_cos = np.cos(self.considered_directions)
        self.considered_directions_sin = np.sin(self.considered_directions)
        self.scan_subsample = scan_subsample

        # To be filled in later, once we know how many directions per scan we
        # receive.
        self.angles = None
        self.angles_cos = None
        self.angles_sin = None
        self.scan = None
        self.max_considered_obstacles = max_considered_obstacles

    def update(self, scan):
        self.scan = np.asarray(scan[::self.scan_subsample] if self.scan_subsample > 1 else scan) * 1e-3

        if self.angles is None or len(self.scan) != self.angles.shape[0]:
            n = len(self.scan)
            delta = (self.scan_left - self.scan_right) / (n - 1)
            self.angles = np.linspace(
                    self.scan_right,
                    self.scan_left,
                    n).reshape((1, -1))
            self.angles_cos = np.cos(self.angles)
            self.angles_sin = np.sin(self.angles)

    def recommend(self, desired_dir):
        if self.scan is None:
            return 0.0, desired_dir
        NO_MEASUREMENT = 0
        is_valid = np.logical_and(
                self.scan != NO_MEASUREMENT,
                self.scan <= self.max_obstacle_distance)
        valid_scan = self.scan[is_valid]
        is_valid = is_valid.reshape((1, -1))
        acoss = self.angles_cos[is_valid]
        asins = self.angles_sin[is_valid]
        x = acoss * valid_scan
        y = asins * valid_scan

        if x.shape[0] == 0:
            return 1.0, normalize_angle(desired_dir)

        if self.max_considered_obstacles is not None and x.shape[0] > self.max_considered_obstacles:
            nth = math.ceil(x.shape[0] / self.max_considered_obstacles)
            x = x[::nth]
            y = y[::nth]

        num_considered_directions = self.considered_directions.shape[0]
        xs = np.broadcast_to(x, (num_considered_directions, x.shape[0]))
        ys = np.broadcast_to(y, (num_considered_directions, y.shape[0]))
        dist_from_line = np.abs(xs * self.considered_directions_sin -
                                ys * self.considered_directions_cos)
        # Obstacles behind the robot from the perspective of the desired direction do not matter.
        # The formula below computes cos(angle between the two vectors) * their_norms. Norms are positive, so a negative result implies abs(angle) > 90deg.
        is_in_front = (xs * self.considered_directions_cos +
                       ys * self.considered_directions_sin) >= 0
        # Only valid points that can block the movement count.
        dist_from_line = np.where(is_in_front, dist_from_line, float('inf'))
        # In each direction, we only care about the nearest obstacle.
        min_dist = np.min(dist_from_line, axis=1)
        risky = np.exp(-(min_dist / self.obstacle_influence)**2)
        safe = 1.0 - risky  # Fuzzy negation.

        direction_delta = normalize_angle(self.considered_directions - desired_dir)
        desired = np.exp(-(direction_delta/self.direction_adherence)**2).reshape((-1,))  # Fuzzy equality

        good = np.minimum(safe, desired)  # Fuzzy and.
        best = np.argmax(good)
        return good[best], self.considered_directions[best][0]


class LocalPlanner:
    def __init__(self, *args, **kwargs):
        self.opt = LocalPlannerOpt(*args, **kwargs)
        self.ref = LocalPlannerRef(*args, **kwargs)
        self.nump = LocalPlannerNumpy(*args, **kwargs)

    def update(self, scan):
        self.opt.update(scan)
        self.ref.update(scan)
        self.nump.update(scan)

    def recommend(self, desired_dir):
#        global g_count
#        g_count += 1
#        g_pr.enable()
        ret = self.nump.recommend(desired_dir)
#        ret = self.opt.recommend(desired_dir)
#        ref = self.ref.recommend(desired_dir)
#        g_pr.disable()
#        if g_count % 100 == 0:
#            g_pr.print_stats()
#        assert ret == ref, (ret, ref)
        return ret

# vim: expandtab sw=4 ts=4
