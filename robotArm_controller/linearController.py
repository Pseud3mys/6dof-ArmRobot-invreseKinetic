import numpy as np

from robotArm_controller import IKsolver, tools, config


# lie un temps à une vitesse (en respectant acceleration et vitesse max)
def vitesse(sec, Amax, Vmax):
    # ca accélaire progressivement (acce max atteint en n sec) jusqu'à atteindre l'accélération Amax
    if sec < config.temps2reachAmax:
        # si la vitesse à déja atteind sont max (mais pas l'accélération), on n'accélaire plus
        # (l'acce aura pas atteint sont max)
        if np.sqrt(2 * config.temps2reachAmax * Vmax / Amax) <= sec:
            return Vmax
        return Amax / (2 * config.temps2reachAmax) * sec ** 2

    # si l'accélération à atteint sont max mais pas la vitesse on garde acce constante pour atteindre vitesse max
    if config.temps2reachAmax <= sec:
        speed = Amax * sec + (Amax / (2*config.temps2reachAmax) * config.temps2reachAmax**2 - Amax*config.temps2reachAmax)
        # une fois qu'on atteint la vitesse max c'est bon, on garde cette vitesse
        if speed > Vmax:
            return Vmax
        return speed


# retourne un tableau de position smooth
def smooth_1axis(start, end, Amax, Vmax):

    delta = end - start
    sens = delta / abs(delta)
    position = start
    positions = []
    time = 0
    times = []
    step = 0
    # accélération
    while sens * (position - start) < sens * delta / 2:
        time += config.time_step
        step += 1
        times.append(time)
        position += vitesse(time, Amax, Vmax) * config.time_step * sens
        positions.append(position)
    # frein
    while sens * (position - start) < sens * delta:
        time -= config.time_step
        step += 1
        times.append(config.time_step * step)
        position += vitesse(time, Amax, Vmax) * config.time_step * sens
        positions.append(position)

    # plt.plot(times, positions)
    # plt.show()
    theorical_time = step*config.time_step
    return positions, len(positions), theorical_time


# crée une droite à suivre et les steps pour les angles
class linear:
    def __init__(self, start_position, end_position):
        self.point_start, self.a, self.b = tuple(start_position)
        self.point_end, self.A, self.B = tuple(end_position)

        self.vector = tools.points2vecteur(self.point_start, self.point_end)
        self.delta_length = np.linalg.norm(self.vector)
        self.delta_angle = max((self.A - self.a), (self.B - self.b))
        # prend la coordonnée la plus éloigné de 0
        index = np.argmax(list(map(abs, self.vector)))
        # donc si c'est 0 le déplacement est nul pour les coordonnées x,y,z
        if self.vector[index] == 0:
            self.k_max = self.delta_angle
        else:
            self.k_max = (self.point_end[index] - self.point_start[index]) / self.vector[index]

        self.A_step = (self.A - self.a) / self.k_max
        self.B_step = (self.B - self.b) / self.k_max

    def get_point(self, k: float):
        x = k * self.vector[0] + self.point_start[0]
        y = k * self.vector[1] + self.point_start[1]
        z = k * self.vector[2] + self.point_start[2]
        rotation_axe_Y = self.A_step * k + self.a
        rotation_axe_Z = self.B_step * k + self.b
        return [[x, y, z], rotation_axe_Y, rotation_axe_Z]


# return array of [[x, y, z], rotation_axe_Y, rotation_axe_Z]
def move_cartesian(start_position, end_position):
    start_position = np.array(start_position)
    end_position = np.array(end_position)
    if (start_position == end_position).all():
        return [], 0, 0

    axis = linear(start_position, end_position)
    # choisie entre truc en cm ou truc en degrée
    if axis.delta_length/config.cartesian_maxSpeed > axis.delta_angle/config.angular_maxSpeed:
        lenth_array, nbr_position, theorical_time = smooth_1axis(0, axis.delta_length, config.cartesian_maxAcceleration,
                                                                 config.cartesian_maxSpeed)
        k_array = list(map(lambda l: axis.k_max * l / axis.delta_length, lenth_array))

    else:
        angles_array, nbr_position, theorical_time = smooth_1axis(0, axis.delta_angle, config.angular_maxAcceleration,
                                                                  config.angular_maxSpeed)
        k_array = list(map(lambda l: axis.k_max * l / axis.delta_angle, angles_array))
    positions_array = list(map(axis.get_point, k_array))

    return positions_array, nbr_position, theorical_time


# return array of [Alpha0, Beta0, Rot0, Alpha1, Rot1]
def move_angular(start_position, end_position):
    positions_array, nbr_position, theorical_time = move_cartesian(start_position, end_position)
    angles_array = []
    for pos in positions_array:
        angles_array.append(IKsolver.cartesian2angular(pos[0], pos[1], pos[2]))
    return angles_array
