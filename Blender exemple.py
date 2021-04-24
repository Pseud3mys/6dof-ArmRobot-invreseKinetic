import bpy
import numpy as np
from robotArm_controller import linearController, config, IKsolver, tools

config.time_step = 1 / 24

# setup
Bras1 = bpy.data.objects["Bras1"]
Bras2 = bpy.data.objects["Bras2"]
Bras3 = bpy.data.objects["Bras3"]

Actual_Position = [[7, 0, 7], 90, 0]
Actual_Key = 0


def rotate(object, angles):
    x, y, z = angles
    x = np.deg2rad(x)
    y = np.deg2rad(y)
    z = np.deg2rad(z)
    object.rotation_euler = (x, y, z)


def __move_robot(angles):
    # calcule des angles:
    Alpha0, Beta0, Rot0, Alpha1, Rot1 = angles
    joint1, joint2, end_joint = IKsolver.forward_solver(angles)

    rotate(Bras1, (0, 90 - Alpha0, Rot0))

    Bras2.location = joint1
    rotate(Bras2, (0, 90 - (Beta0 - 180 + Alpha0), Rot0))

    Bras3.location = joint2
    vec = end_joint - joint2
    x = vec[0]
    y = vec[1]
    z = vec[2]

    if x == 0:
        if y > 0:
            rotation_axe_Y = 270
        else:
            rotation_axe_Y = 90
    else:
        rotation_axe_Y = np.arctan(z / x)
    if x == 0:
        rotation_axe_Z = 0
    else:
        rotation_axe_Z = np.arctan(y / x)

    # conversion:
    rotation_axe_Y = np.rad2deg(rotation_axe_Y)
    rotation_axe_Z = np.rad2deg(rotation_axe_Z)
    rotate(Bras3, (0, 90 - rotation_axe_Y, rotation_axe_Z))


def move_robot(position):
    # calcule des angles:
    coord, rotation_axe_Y, rotation_axe_Z = tuple(position)
    angles = IKsolver.cartesian2angular(coord, rotation_axe_Y, rotation_axe_Z)
    joint1, joint2, end_joint = IKsolver.forward_solver(angles)
    Alpha0, Beta0, Rot0, Alpha1, Rot1 = angles

    rotate(Bras1, (0, 90 - Alpha0, Rot0))

    Bras2.location = joint1
    rotate(Bras2, (0, 90 - (Beta0 + 180 + Alpha0), Rot0))

    Bras3.location = joint2
    rotate(Bras3, (0, - 90 - rotation_axe_Y, rotation_axe_Z))


def key(frame):
    Bras1.keyframe_insert(data_path="rotation_euler", frame=frame)
    Bras2.keyframe_insert(data_path="rotation_euler", frame=frame)
    Bras3.keyframe_insert(data_path="rotation_euler", frame=frame)
    Bras1.keyframe_insert(data_path="location", frame=frame)
    Bras2.keyframe_insert(data_path="location", frame=frame)
    Bras3.keyframe_insert(data_path="location", frame=frame)


def delete_keys():
    for i in range(1000):
        try:
            Bras1.keyframe_delete(data_path="rotation_euler", frame=i)
            Bras1.keyframe_delete(data_path="location", frame=i)
        except:
            pass

        try:
            Bras2.keyframe_delete(data_path="rotation_euler", frame=i)
            Bras2.keyframe_delete(data_path="location", frame=i)
        except:
            pass

        try:
            Bras3.keyframe_delete(data_path="rotation_euler", frame=i)
            Bras3.keyframe_delete(data_path="location", frame=i)
        except:
            pass


def line(position):
    global Actual_Position, Actual_Key
    positions, end_position, nbr_moves, time = linearController.move_cartesian(Actual_Position, position)
    print("from", Actual_Position, "to", position, "take %d moves and %s seconds." % (nbr_moves, round(time, 2)))
    for pos in positions:
        move_robot(pos)
        Actual_Key += 1
        key(Actual_Key)
    Actual_Position = end_position


delete_keys()
print("\n\n--START--")

# demo rotation:
line([[5, 5, 5], 0, 0])
line([[5, 5, 5], 360, 0])
line([[5, 5, 5], 0, 360])

line([[5, 5, 0], 90, 0])
line([[5, -5, 5], 0, 90])
line([[-5, -5, 0], 0, 0])
line([[-5, -5, 0], 0, 0])
