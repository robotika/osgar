import math

def normalize_angle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

class LocalPlanner:
    def __init__(self, scan_right=math.radians(-135), scan_left=math.radians(135), direction_adherence=math.radians(90), max_obstacle_distance=1.5, obstacle_influence=1.2):
        self.last_scan = None
        self.scan_right = scan_right
        self.scan_left = scan_left
        self.direction_adherence = direction_adherence
        self.max_obstacle_distance = max_obstacle_distance
        self.obstacle_influence = obstacle_influence

    def update(self, scan):
        self.last_scan = scan

    def recommend(self, desired_dir):
        if self.last_scan is None:
            return 1.0, desired_dir

        obstacles = []
        for (i, measurement) in enumerate(self.last_scan):
            # Eduro hack, to reduce computations
            if i % 5 != 0:  # reduce resolution to 5deg!
                continue

            if measurement == 0:
                continue
            if measurement * 1e-3 > self.max_obstacle_distance:
                continue
            measurement_angle = self.scan_right + (self.scan_left - self.scan_right) * i / float(len(self.last_scan) - 1)
            measurement_vector = math.cos(measurement_angle), math.sin(measurement_angle)

            # Converting from tenths of milimeters to meters.
            obstacle_xy = [mv * measurement * 1e-3 for mv in measurement_vector]

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

#        return max((is_good(math.radians(direction)), math.radians(direction)) for direction in range(-180, 180, 3))
        return max((is_good(math.radians(direction)), math.radians(direction)) for direction in range(-180, 180, 10))

# vim: expandtab sw=4 ts=4
