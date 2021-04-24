"""
somme use full functions
"""
import numpy as np


def points2vecteur(start_point: np.ndarray, end_point: np.ndarray):
    x = end_point[0] - start_point[0]
    y = end_point[1] - start_point[1]
    z = end_point[2] - start_point[2]
    return np.array([x, y, z], float)


def angles2vecteur(rotation_axe_Y: float, rotation_axe_Z: float, norme: float):
    rotation_axe_Y = np.deg2rad(rotation_axe_Y)
    rotation_axe_Z = np.deg2rad(rotation_axe_Z)

    x = norme * np.cos(rotation_axe_Y) * np.cos(rotation_axe_Z)
    y = norme * np.cos(rotation_axe_Y) * np.sin(rotation_axe_Z)
    z = norme * np.sin(rotation_axe_Y)
    return np.array([x, y, z], float)
