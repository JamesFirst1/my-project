def generate_track(seed):
    import numpy as np
    import pandas as pd
    import os
    from Bezier import Bezier
    from perlin_noise import PerlinNoise
    from scipy.spatial import ConvexHull
    import matplotlib.pyplot as plt

    class CreateTrack:
        def __init__(self, num_points=10, x_bounds=[100, 100], y_bounds=[100, 100], corner_cells=10, seed=0):
            self.num_points = num_points
            self.x_bounds = x_bounds
            self.y_bounds = y_bounds
            self.corner_cells = corner_cells
            self.seed = seed
            self.height_map = None

        def generate_noise(self, octaves=1, seed=0):
            noise = PerlinNoise(octaves=octaves, seed=seed)
            xpix, ypix = 100, 100
            return [[10 * noise([i / xpix, j / ypix]) for j in range(xpix)] for i in range(ypix)]

        def curve_corners(self, points):
            def calculate_custom_point(p1, p2, perc):
                x = (1 - perc) * p1[0] + perc * p2[0]
                y = (1 - perc) * p1[1] + perc * p2[1]
                return (x, y)

            inner, outer = [], []
            for i in range(len(points)):
                inner.append(calculate_custom_point(points[i], points[(i + 1) % len(points)], np.random.uniform(0.1, 0.4)))
                outer.append(calculate_custom_point(points[i], points[(i + 1) % len(points)], np.random.uniform(0.6, 0.9)))

            curves = []
            t_vals = np.linspace(0, 1, self.corner_cells)
            for i, point in enumerate(points):
                p0 = outer[i - 1 if i != 0 else -1]
                p1 = point
                p2 = inner[i]
                curves.extend(Bezier.Curve(t_vals, np.array([p0, p1, p2])))

            curves.append(outer[-1])
            return curves

        def random_midpoint(self, points):
            center = np.mean(points, axis=0)
            indices = np.random.choice(range(1, len(points)), 4, replace=False)
            for i in indices:
                mid = (points[i] + points[(i + 1) % len(points)]) / 2
                scale = np.random.uniform(1, 0.1)
                displaced = center + scale * (mid - center)
                points = np.insert(points, i + 1, displaced, axis=0)
            return points

        def create_racetrack(self, track_3d=False):
            self.height_map = self.generate_noise(octaves=2, seed=self.seed)
            x_vals = np.random.uniform(self.x_bounds[0], self.x_bounds[1], self.num_points)
            y_vals = np.random.uniform(self.y_bounds[0], self.y_bounds[1], self.num_points)
            points = np.column_stack((x_vals, y_vals))
            hull = ConvexHull(points)
            hull_pts = self.random_midpoint(points[hull.vertices])
            return self.curve_corners(hull_pts)

    def sample_every_n_meters(points, interval=3.0):
        distances = np.sqrt(np.sum(np.diff(points, axis=0) ** 2, axis=1))
        cumulative = np.insert(np.cumsum(distances), 0, 0)
        total_length = cumulative[-1]
        target_distances = np.arange(0, total_length, interval)

        sampled_x = np.interp(target_distances, cumulative, points[:, 0])
        sampled_y = np.interp(target_distances, cumulative, points[:, 1])

        return np.column_stack((sampled_x, sampled_y))

    if seed is None:
        seed = np.random.randint(0, 2**31 - 1)

    np.random.seed(seed)
    os.makedirs("Files", exist_ok=True)

    track = CreateTrack(num_points=20, x_bounds=[0, 150], y_bounds=[0, 100], corner_cells=30, seed=seed)
    points = np.array(track.create_racetrack())

    sampled_points = sample_every_n_meters(points, interval=3.0)

    #Plot track
    plt.figure(figsize=(8, 6))
    plt.axis('equal')
    plt.axis('off')
    plt.plot(sampled_points[:, 0] + 50, sampled_points[:, 1] + 50, color='black', linewidth=2)

    plt.savefig("Files/generatedTrack.png", bbox_inches='tight', pad_inches=0)
    plt.close()

    coneLocation = []
    for i in range(len(sampled_points)):
        point = sampled_points[i]
        
        if i < len(sampled_points) - 1:
            next_point = sampled_points[i + 1]
        else:
            next_point = sampled_points[i - 1]

        dx = next_point[0] - point[0]
        dy = next_point[1] - point[1]
        length = np.hypot(dx, dy)

        nx = -dy / length
        ny = dx / length

        left_x = point[0] + nx * 1.5
        left_y = point[1] + ny * 1.5
        right_x = point[0] - nx * 1.5
        right_y = point[1] - ny * 1.5

        if i == 0:
            coneLocation.append(["car", point[0], point[1]])
            coneLocation.append(["largeOrange", left_x, left_y])
            coneLocation.append(["largeOrange", right_x, right_y])
        else:
            coneLocation.append(["smallBlue", left_x, left_y])
            coneLocation.append(["smallYellow", right_x, right_y])

    pd.DataFrame(coneLocation).to_csv("Files/Generated Track.csv", index=False, header=False)

    with open("Files/generatedSeed.txt", "w") as f:
        f.write(str(seed))
