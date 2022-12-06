import numpy as np
from scipy.spatial.transform import Rotation

# links
tf_base1 = np.array([[0], [0], [0.06605]])
tf_12 = np.array([[0], [0], [0.04145]])
tf_23 = np.array([[-0.08285], [0], [0]])
tf_34 = np.array([[-0.08285], [0], [0]])
tf_45 = np.array([[-0.07385], [0], [0]])
tf_4cam = np.array([[-0.072], [-0.049], [0]])
tf_5tool = np.array([[0], [0], [0.11]])

def std_rotation_x(theta):
    return np.array([[1, 0, 0], [0, np.cos(theta), -np.sin(theta)], [0, np.sin(theta), np.cos(theta)]])

def std_rotation_y(theta):
    return np.array([[np.cos(theta), 0, np.sin(theta)], [0, 1, 0], [-np.sin(theta), 0, np.cos(theta)]])

def std_rotation_z(theta):
    return np.array([[np.cos(theta), -np.sin(theta), 0], [np.sin(theta), np.cos(theta), 0], [0, 0, 1]])

def rot_base_to_tool(q0, q1, q2, q3, q4):
    rot_b1 = std_rotation_z(q0)
    rot_12 = std_rotation_y(np.pi/2) @ std_rotation_z(q1)
    rot_23 = std_rotation_z(q2)
    rot_34 = std_rotation_z(q3)
    rot_4tool = std_rotation_y(-np.pi/2) @ std_rotation_x(q4)

    return rot_b1 @ rot_12 @ rot_23 @ rot_34 @ rot_4tool

def rot_base_to_cam(q0, q1, q2, q3):
    # frame 4 and camera frame are fixed
    rot_b1 = std_rotation_z(q0)
    rot_12 = std_rotation_y(np.pi/2) @ std_rotation_z(q1)
    rot_23 = std_rotation_z(q2)
    rot_34 = std_rotation_z(q3)
    rot_4cam = std_rotation_y(-np.pi/2)

    return rot_b1 @ rot_12 @ rot_23 @ rot_34 @ rot_4cam

def rotmatrix_to_quaternion(rot_matrix):
    r = Rotation.from_matrix(rot_matrix)
    return r.as_quat()


def tf_base_to_cam(q0, q1, q2, q3):
    rot_b1 = std_rotation_z(q0)
    rot_12 = std_rotation_y(np.pi/2) @ std_rotation_z(q1)
    rot_23 = std_rotation_z(q2)
    rot_34 = std_rotation_z(q3)

    return tf_base1 + rot_b1 @ (tf_12 + rot_12 @ (tf_23 + rot_23 @ (tf_34 + (rot_34 @ tf_4cam))))

def tf_base_to_tool(q0, q1, q2, q3, q4):
    rot_b1 = std_rotation_z(q0)
    rot_12 = std_rotation_y(np.pi/2) @ std_rotation_z(q1)
    rot_23 = std_rotation_z(q2)
    rot_34 = std_rotation_z(q3)
    rot_45 = std_rotation_y(-np.pi/2) @ std_rotation_x(q4)

    return tf_base1 + rot_b1 @ (tf_12 + rot_12 @ (tf_23 + rot_23 @ (tf_34 + rot_34 @ (tf_45 + rot_45 @ tf_5tool))))