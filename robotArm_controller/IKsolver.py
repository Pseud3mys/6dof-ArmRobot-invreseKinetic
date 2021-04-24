"""
les angles sont en degrés !

"""

import numpy as np
from robotArm_controller import tools, config

"""
# angle min/max
min_Alpha0 = 0
max_Alpha0 = 180
min_Beta0 = 0
max_Beta0 = 180
min_Rot0 = 0
max_rot0 = 270
# little one
min_Alpha1 = -90
max_Alpha1 = 90
min_Rot1 = 0
max_Rot1 = 270
"""


def forward_solver(angles):
    Alpha0, Beta0, Rot0, Alpha1, Rot1 = angles

    joint1 = tools.angles2vecteur(Alpha0, Rot0, config.length_A)

    angle_HB = np.deg2rad(Beta0 - 180 + Alpha0)    # angle horizon-B
    Rot0 = np.deg2rad(Rot0)
    # coord relative à Joint1
    rela_J2_x = config.length_B * np.cos(angle_HB) * np.cos(Rot0)
    rela_J2_y = config.length_B * np.cos(angle_HB) * np.sin(Rot0)
    rela_J2_z = config.length_B * np.sin(angle_HB)
    joint2 = np.sum([joint1, [rela_J2_x, rela_J2_y, rela_J2_z]], 0)

    # point le end point:
    Alpha1 = np.deg2rad(Alpha1)
    Rot1 = np.deg2rad(Rot1)

    # crée un repère sur i,j,k
    longueur = config.length_C * np.sin(Alpha1)
    # nouveau repère (i,j,k):
    i = np.array(tools.points2vecteur(joint1, joint2), dtype=float)  # vecteur i, coolinéaire au bras
    i = i / config.length_B  # normalisé
    Xi, Yi, Zi = tuple(i)
    # pour j:
    h = (-Xi ** 2 - Yi ** 2) / Zi ** 2
    j = np.array((Xi, Yi, h * Zi))
    j = j / np.linalg.norm(j)
    # pour k (produit vectoriel de i et j:
    k = np.cross(i, j)

    end_i = (config.length_B - config.length_C * np.cos(Alpha1)) * i
    end_j = - np.cos(Rot1) * longueur * j
    end_k = np.sin(Rot1) * longueur * k
    end_point = np.sum([joint1, end_i, end_j, end_k], 0)

    return joint1, joint2, end_point


def just2D(relative_X: float, relative_Y: float):
    c = np.sqrt(relative_X ** 2 + relative_Y ** 2)

    Beta0 = np.arccos((c ** 2 - config.length_A ** 2 - config.length_B ** 2) /
                      (-2 * config.length_A * config.length_B))

    alpha_a = np.arccos((config.length_B ** 2 - config.length_A ** 2 - c ** 2) / (-2 * config.length_A * c))
    alpha_b = np.arccos(relative_X / c)
    Alpha0 = alpha_a + alpha_b

    # en degré
    Alpha0 = np.rad2deg(Alpha0)
    Beta0 = np.rad2deg(Beta0)
    return Alpha0, Beta0


def join_to_machine(join_coordinate: np.ndarray):
    x = join_coordinate[0]
    y = join_coordinate[1]
    z = join_coordinate[2]

    d = np.sqrt(x ** 2 + y ** 2)
    c = np.sqrt(z ** 2 + d ** 2)

    # pour Alpha1:
    alpha_a = np.arccos((config.length_B ** 2 - config.length_A ** 2 - c ** 2) / (-2 * config.length_A * c))
    alpha_b = np.arcsin(z/c)
    Alpha0 = alpha_a + alpha_b

    # pour Beta0
    Beta0 = np.arccos((c ** 2 - config.length_A ** 2 - config.length_B ** 2) /
                      (-2 * config.length_A * config.length_B))

    # pour rotation primaire
    Rot0 = np.arctan(y / x)

    # convertions
    Alpha0 = np.rad2deg(Alpha0)
    Beta0 = np.rad2deg(Beta0)
    Rot0 = np.rad2deg(Rot0)
    # arctan est retourne de -90 à 90 donc faut adapter sur un cercle de 360°
    if x < 0:
        Rot0 = 180 + Rot0
    return Alpha0, Beta0, Rot0


def nightmare(target_coordinate: np.ndarray, rotation_axe_Y: float, rotation_axe_Z: float):

    coordinate_offset = tools.angles2vecteur(rotation_axe_Y, rotation_axe_Z, config.length_C)
    point2 = np.sum([target_coordinate, coordinate_offset], 0)
    Alpha0, Beta0, Rot0 = join_to_machine(point2)
    # coordonné du point après le 1er tube:
    point1 = tools.angles2vecteur(Alpha0, Rot0, config.length_A)
    # nouveau repère (i,j,k):
    i = np.array(tools.points2vecteur(point1, point2))  # vecteur i, coolinéaire au bras
    i = i / config.length_B  # normalisé
    Xi, Yi, Zi = tuple(i)
    # pour j:
    h = (-Xi ** 2 - Yi ** 2) / Zi ** 2
    j = np.array((Xi, Yi, h * Zi))
    norme_j = np.linalg.norm(j)
    j = j / norme_j
    # pour k (produit vectoriel de i et j:
    k = np.cross(i, j)
    # pour Alpha1:
    Alpha1 = np.arccos(np.vdot(i, tools.points2vecteur(point2, target_coordinate)) / np.linalg.norm(
        tools.points2vecteur(point2, target_coordinate)))
    Alpha1 = 180 - np.rad2deg(Alpha1)

    # pour Rot1 (intersection droite et plan)
    X, Y, Z = target_coordinate
    h = (np.vdot(i, point1) - np.vdot(i, target_coordinate)) / (Xi ** 2 + Yi ** 2 + Zi ** 2)
    # x,y,z projection de arrivé sur plan (j,k)
    x = h * Xi + X
    y = h * Yi + Y
    z = h * Zi + Z
    # on veut l'angle entre k et vecteur(P1,projeté)
    vec = tools.points2vecteur(point1, np.array([x, y, z], float))
    Rot1 = np.arccos(np.vdot(k, vec) / (np.linalg.norm(k) * np.linalg.norm(vec)))
    Rot1 = np.rad2deg(Rot1) - 90

    return Alpha1, Rot1


def cartesian2angular(target_coordinate: np.ndarray, rotation_axe_Y: float, rotation_axe_Z: float):

    coordinate_offset = tools.angles2vecteur(rotation_axe_Y, rotation_axe_Z, config.length_C)
    new_coordinate = np.sum([target_coordinate, coordinate_offset], 0)
    print(coordinate_offset, new_coordinate)
    Alpha0, Beta0, Rot0 = join_to_machine(new_coordinate)
    Alpha1, Rot1 = nightmare(target_coordinate, rotation_axe_Y, rotation_axe_Z)
    return [Alpha0, Beta0, Rot0, Alpha1, Rot1]
