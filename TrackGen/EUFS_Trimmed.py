def generate_track(seed,
                   min_corner_radius,
                   max_frequency,
                   amplitude,
                   starting_amplitude,
                   rel_accuracy,
                   margin,
                   starting_straight_length,
                   starting_straight_downsample,
                   min_cone_spacing,
                   max_cone_spacing,
                   track_width,
                   cone_spacing_bias,
                   starting_cone_spacing,):
    import math
    import random
    import cmath
    import numpy as np
    import matplotlib.pyplot as plt


    class TrackGenerator:
        def __init__(self, config):
            default_cfg = {
                'seed': seed,
                'min_corner_radius': min_corner_radius,
                'max_frequency': max_frequency,
                'amplitude': amplitude,
                'check_self_intersection': True,
                'starting_amplitude': starting_amplitude,
                'rel_accuracy': rel_accuracy,
                'margin': margin,
                'starting_straight_length': starting_straight_length,
                'starting_straight_downsample': starting_straight_downsample,
                'min_cone_spacing': min_cone_spacing,
                'max_cone_spacing': max_cone_spacing,
                'track_width': track_width,
                'cone_spacing_bias': cone_spacing_bias,
                'starting_cone_spacing': starting_cone_spacing
            }
            self.config = {**default_cfg, **config}

            if 'resolution' not in self.config:
                if 'length' in self.config:
                    length = self.config['length']
                else:
                    t = self.config['amplitude'] * self.config['max_frequency']
                    length = ((0.6387 * t + 43.86) * t + 123.1) * t + 35.9

                min_sep = self.config['min_cone_spacing']
                max_sep = self.config['max_cone_spacing']
                r = math.log2(length) / self.config['min_corner_radius']
                self.config['resolution'] = int(4 * length * max(1 / min_sep, r / max_sep))

            self.rng = random.Random(self.config['seed'])

        @staticmethod
        def _compute_corner_radii(dt, dPdt):
            ddPdt = np.append(np.diff(dPdt), dPdt[0] - dPdt[-1]) / dt
            return abs(dPdt)**3 / (np.conj(dPdt) * ddPdt).imag

        @staticmethod
        def generate_path_w_params(rng, n_points, min_corner_radius, max_frequency, amplitude=1/3):
            z = np.array([cmath.exp(2j * math.pi * t / n_points) for t in range(n_points)])
            waves = np.zeros(n_points, dtype=np.complex128)
            dwaves = np.zeros(n_points, dtype=np.complex128)
            for frequency in range(2, max_frequency + 1):
                phase = cmath.exp(2j * math.pi * rng.random())
                z_pow = z**frequency
                waves += z * (z_pow / (phase * (frequency + 1)) + phase / (z_pow * (frequency - 1)))
                dwaves += z_pow / phase - phase / z_pow

            points = z + amplitude * waves
            dPdt = (1j * z) * (1 + amplitude * dwaves)
            corner_radii = TrackGenerator._compute_corner_radii(2 * math.pi / n_points, dPdt)

            scale = min_corner_radius / min(abs(corner_radii))
            return scale * points, 1j * dPdt / abs(dPdt), scale * corner_radii

        @staticmethod
        def _to_edges(points):
            return np.column_stack((points, np.roll(points, -1)))

        @staticmethod
        def _intersects(p, dp, q, dq):
            q = (q - p) / dp
            dq = dq / dp
            if dq.imag == 0:
                return q.imag == 0 and q.real < 1 and q.real + dq.real > 0
            else:
                return q.imag * (q.imag + dq.imag) <= 0 and 0 < q.real - dq.real * q.imag / dq.imag < 1

        @staticmethod
        def _slf_intrsct_brute(edges):
            for i, p_i in enumerate(edges):
                for p_j in edges[i+1:]:
                    if p_j[0] == p_i[1] or p_j[1] == p_i[0]:
                        continue
                    if TrackGenerator._intersects(p_i[0], p_i[1] - p_i[0], p_j[0], p_j[1] - p_j[0]):
                        return True
            return False

        @staticmethod
        def _side(p, dp, edges):
            side0 = np.sign(((edges[:,0] - p) / dp).imag)
            side1 = np.sign(((edges[:,1] - p) / dp).imag)
            return np.sign(side0 + side1)

        @staticmethod
        def _slf_intrsct_recurse(edges):
            if len(edges) <= 8:
                return TrackGenerator._slf_intrsct_brute(edges)
            center = sum(edges[:,0] + edges[:,1]) / (2 * len(edges))
            pivot = edges[len(edges)//2][0]
            for _ in range(32):
                side = TrackGenerator._side(center, pivot - center, edges)
                left = edges[side >= 0]
                right = edges[side <= 0]
                if abs(len(left)/len(edges) - 0.5) + abs(len(right)/len(edges) - 0.5) < 0.125:
                    return TrackGenerator._slf_intrsct_recurse(left) or TrackGenerator._slf_intrsct_recurse(right)
                pivot = random.choice(edges)[0]
            return TrackGenerator._slf_intrsct_brute(edges)

        @staticmethod
        def self_intersects(points, slopes, margin):
            normals = 1j * slopes / abs(slopes)
            tmp1 = TrackGenerator._to_edges(points + margin * normals)
            tmp2 = TrackGenerator._to_edges(points - margin * normals)
            return TrackGenerator._slf_intrsct_recurse(tmp1) or TrackGenerator._slf_intrsct_recurse(tmp2)

        @staticmethod
        def pick_starting_point(positions, normals, corner_radii, starting_straight_length, downsample=2):
            curvature = abs(1 / corner_radii[::downsample])
            indices = np.argsort(curvature)[:len(curvature)//8]
            def cyclic_smooth(indices, points, values, diameter):
                distance_to_next = abs(np.append(np.diff(points), points[0] - points[-1]))
                smoothed_values = values[indices].copy()
                for n, i in enumerate(indices):
                    coef_sum = 1
                    curr = (i if i != 0 else len(values)) - 1
                    distance = distance_to_next[curr]
                    while distance < diameter:
                        coef = distance_to_next[curr] * math.sin(math.pi * distance / diameter)
                        smoothed_values[n] += coef * values[curr]
                        coef_sum += coef
                        curr = (curr if curr != 0 else len(values)) - 1
                        distance += distance_to_next[curr]
                    smoothed_values[n] /= coef_sum
                return smoothed_values

            start_index = (downsample * indices[np.argmin(cyclic_smooth(
                indices, positions, curvature, 1.5 * starting_straight_length))])

            positions = np.roll(positions, -start_index)
            normals = np.roll(normals, -start_index)
            corner_radii = np.roll(corner_radii, -start_index)

            positions -= positions[0]
            rotation = 1j / normals[0]
            positions *= rotation
            normals *= rotation

            return positions, normals, corner_radii

        @staticmethod
        def place_cones(positions, normals, corner_radii, min_corner_radius,
                        min_cone_spacing, max_cone_spacing,
                        track_width, cone_spacing_bias,
                        start_offset, starting_cone_spacing):
            min_density = 1 / max_cone_spacing
            max_density = 1 / min_cone_spacing
            density_range = max_density - min_density

            c1 = density_range / 2 * ((1 - cone_spacing_bias) * min_corner_radius - (1 + cone_spacing_bias) * track_width / 2)
            c2 = density_range / 2 * ((1 + cone_spacing_bias) * min_corner_radius - (1 - cone_spacing_bias) * track_width / 2)

            def place(points, radii, side):
                distance_to_next = abs(np.append(np.diff(points), points[0] - points[-1]))
                distance_to_prev = np.roll(distance_to_next, 1)

                cone_density = min_density + side * c1 / radii + c2 / abs(radii)
                cone_density *= distance_to_prev

                modified_length = sum(cone_density)
                threshold = modified_length / round(modified_length)

                cones = [points[0]]
                current = 0
                for i, density in enumerate(cone_density[1:]):
                    current += density
                    if current >= threshold:
                        current -= threshold
                        cones.append(points[i])
                return np.array(cones)

            l_cones = place(positions + normals * track_width / 2, corner_radii - track_width / 2, 1)
            r_cones = place(positions - normals * track_width / 2, corner_radii + track_width / 2, -1)

            start_cones = np.array([l_cones[0], r_cones[0]])
            start_cones = np.append(start_cones + starting_cone_spacing / 2,
                                    start_cones - starting_cone_spacing / 2)

            car_pos = 0
            length_accum = 0
            while length_accum < start_offset:
                length_accum += abs(positions[car_pos - 1] - positions[car_pos])
                car_pos -= 1

            l_cones -= positions[car_pos]
            r_cones -= positions[car_pos]
            start_cones -= positions[car_pos]

            rotation = 1j / normals[car_pos]
            l_cones *= rotation
            r_cones *= rotation
            start_cones *= rotation

            return start_cones, l_cones[1:], r_cones[1:]

        @staticmethod
        def write_to_csv(file_path, start_cones, l_cones, r_cones, overwrite=True):
            with open(file_path, "w") as f:
                for cone in l_cones:
                    f.write(f"smallBlue,{cone.real:.2f},{cone.imag:.2f},0,0.01,0.01,0.0\n")
                for cone in r_cones:
                    f.write(f"smallYellow,{cone.real:.2f},{cone.imag:.2f},0,0.01,0.01,0.0\n")
                for cone in start_cones:
                    f.write(f"largeOrange,{cone.real:.2f},{cone.imag:.2f},0,0.01,0.01,0.0\n")
                f.write("car,0.00,0.00,0,0.01,0.01,0.0\n")

        def __call__(self):
            margin = self.config['track_width'] / 2 + self.config['margin']
            while True:
                path = self.generate_path_w_params(
                    self.rng,
                    self.config['resolution'],
                    self.config['min_corner_radius'],
                    self.config['max_frequency'],
                    self.config['amplitude'])
                points, slopes, corner_radii = path
                if not self.config['check_self_intersection'] or not self.self_intersects(points, slopes, margin):
                    break

            positions, normals, corner_radii = self.pick_starting_point(
                points, slopes, corner_radii,
                self.config['starting_straight_length'],
                self.config['starting_straight_downsample'])

            start_cones, left_cones, right_cones = self.place_cones(
                positions, normals, corner_radii,
                self.config['min_corner_radius'],
                self.config['min_cone_spacing'],
                self.config['max_cone_spacing'],
                self.config['track_width'],
                self.config['cone_spacing_bias'],
                self.config['starting_straight_length'],
                self.config['starting_cone_spacing'])

            return start_cones, left_cones, right_cones

    # Initialize and generate cones
    if seed is None:
        seed = random.randint(0, 2**32 - 1)
    amplitude = amplitude /3
    rel_accuracy = rel_accuracy/100
    min_cone_spacing = min_cone_spacing * math.pi / 16

    tg = TrackGenerator({'seed': seed})
    start_cones, left_cones, right_cones = tg()
    # Plot cones
    plt.figure(figsize=(8, 8),facecolor='#0C1821')
    plt.scatter(left_cones.real, left_cones.imag, c='blue', s=25)
    plt.scatter(right_cones.real, right_cones.imag, c='yellow', s=25)
    plt.scatter(start_cones.real, start_cones.imag, c='orange', s=50, marker='s')
    plt.axis('off')        # turn off axes and ticks
    plt.savefig("Files/generatedTrack.png", bbox_inches='tight', pad_inches=0)
    plt.close()

    # Save CSV file
    TrackGenerator.write_to_csv("Files/Generated Track.csv", start_cones, left_cones, right_cones)
    with open("Files/generatedSeed.txt", "w") as f:
            f.write(str(seed))

