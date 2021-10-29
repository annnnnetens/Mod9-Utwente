import math
# When working on the microcontroller change the numpy to ulab since numpy is not available on the microcontroller
import numpy as np

from matplotlib import pyplot as plt


def h_to_adjoint(h_matrix):
    """Turns H Matrix into Adjoint"""
    res = np.eye(3)
    res[1:3, 1:3] = h_matrix[0:2, 0:2]
    res[1, 0] = h_matrix[1, 2]
    res[2, 0] = h_matrix[0, 2] * -1
    return res


def adjoint_to_h(adjoint):
    """Turns adjoint matrix into H matrix"""
    res = np.eye(3)
    res[0:2, 0:2] = adjoint[1:3, 1:3]
    res[1, 2] = adjoint[1, 0]
    res[0, 2] = adjoint[2, 0] * -1
    return res


def create_h_matrix(angle, location):
    """Creates a H Matrix from a to b using the angle a makes with respect to b
    The location parameter is the origin of a expressed in frame b
    At least I think it does this :o
    My RKI imagination is not that good"""
    return np.array([
        [np.cos(angle * np.pi), -np.sin(angle * np.pi), location[0]],
        [np.sin(angle * np.pi), np.cos(angle * np.pi), location[1]],
        [0, 0, 1]
    ])


def get_angle_from_h_matrix(h_matrix):
    return np.arccos(h_matrix[0, 0]) / np.pi


def get_position_from_h_matrix(h_matrix):
    return h_matrix[0, 2], h_matrix[1, 2]


def brockett(h0_matrix, twistspairs):
    """
    Creates H matrix using brockett
    Inputs: H0_matrix is the H matrix in the reference configuration
    twistpairs is a list of tuples of a twist and the corresponding q (angle or translation
    If degrees is true it is assumed that q is in degrees (for example q = 45)
    If degrees is false it is assumed that q is in radians (for example q = 0.25) (so no multiplng with pi!)
    """
    res = np.eye(3)
    for twistpair in twistspairs:
        current_matrix = np.eye(3)
        twist = twistpair[0]
        q = twistpair[1]
        if twist[0] == 0:
            # Translational joint
            if (math.sqrt(twist[1] ** 2 + twist[2] ** 2) - 1) > 0.01:
                raise ValueError("There is a non unit twist in brockett")
            x = twist[1]
            y = twist[2]
            current_matrix[0, 2] = x * q
            current_matrix[1, 2] = y * q

            res = res @ current_matrix
        elif twist[0] == 1:
            # Rotational joint
            c = np.cos(q * np.pi)
            s = np.sin(q * np.pi)
            current_matrix[0:2, 0:2] = np.array([[c, -s], [s, c]])
            x = twist[2] * -1
            y = twist[1]
            current_matrix[0, 2] = x - x * c + y * s
            current_matrix[1, 2] = y - x * s - y * c
            print('current matrix')
            print(current_matrix)
            res = res @ current_matrix
        else:
            raise ValueError("There is a non unit twist in brockett")
    res = res @ h0_matrix
    return res


def twist_to_tilde(twist):
    """Returns tilde form of twist"""
    return np.array([
        [0, -twist[0], twist[1]],
        [twist[0], 0, twist[2]],
        [0, 0, 0]
    ])


def tilde_to_twist(tilde):
    """Returns the twist form of a tilde twist"""
    return np.array([tilde[1, 0], tilde[0, 2], tilde[1, 2]])


def jacobian_twists(twists):
    """Calculate jacobian using the twists"""
    return np.array(twists).T


def jacobian_angles(q1):
    """Calculate jacobian using the angles
    Keep in mind that it only needs q1 and the length of the first part is assumed to be 240mm
    Currently only uses radians"""
    J = np.zeros((3, 2))
    J[:, 0] = (1, 0, 0)
    J[:, 1] = (1, 0.24 * np.cos(q1*np.pi), 0.24 * np.sin(q1*np.pi))
    return J


def unit_twist_tranlational(vx, vy):
    magnitude = math.sqrt(vx ** 2 + vy ** 2)
    if magnitude == 1:
        return np.array([0, vx, vy])
    else:
        return np.array([0, vx / magnitude, vy / magnitude]).T


def unit_twist_rotational(px, py):
    return np.array([1, py, -px]).T


def matinv2x2(M):
    res = np.eye(2)
    det = M[0,0] * M[1, 1] - M[1, 0] * M[0, 1]
    res[0,0] = M[1, 1]/det
    res[1, 1] = M[0, 0]/det
    res[1, 0] = - M[1, 0]/det
    return res


def calculate_dq(q1, q2, x_desired, y_desired):
    L3 = 0.015
    reference_configuration = np.array([
        [1, 0, L3],
        [0, 1, 0.425],
        [0, 0, 1]
    ])
    reference_twist1 = unit_twist_rotational(0, 0)
    reference_twist2 = unit_twist_rotational(0, 0.24)
    He0 = brockett(reference_configuration, [(reference_twist1, q1), (reference_twist2, q2)])
    J = jacobian_angles(q1)

    pe0 = He0[:2, 2]
    ps = np.array([x_desired, y_desired])  # TODO: insert desired position or velocity?
    K = 10  # TODO: tune K to have good response
    F = K * (ps - pe0)
    # TODO: maybe constrict F like is done in exercise E
    Ws0 = np.array([pe0[0] * F[1] - pe0[1] * F[0], F[0], F[1]])
    tau = J.T @ Ws0

    b = [0.5, 0.25]  # TODO: tune b to have good response
    dq = tau / b
    return dq


def calculate_dq_j_inv(q1, q2, vx, vy):
    L3 = 0.015
    reference_configuration = np.array([
        [1, 0, L3],
        [0, 1, 0.425],
        [0, 0, 1]
    ])
    reference_twist1 = unit_twist_rotational(0, 0)
    reference_twist2 = unit_twist_rotational(0, 0.24)
    He0 = brockett(reference_configuration, [(reference_twist1, q1), (reference_twist2, q2)])
    J = jacobian_angles(q1)

    pe0 = He0[:2, 2]
    H_0_f = np.array([[1, 0, pe0[0]],
            [0, 1, pe0[1]],
            [0, 0, 1]])

    adj_0_f = h_to_adjoint(H_0_f)

    jprime = adj_0_f @ J

    jdoubleprime = jprime[1:, :]

    j_inv = matinv2x2(jdoubleprime)

    qdot_setp = j_inv @ (np.array([vx, vy]).T)
    return qdot_setp


if __name__ == "__main__":
    # For testing
    identity = np.eye(3)
    H4 = np.array([
        [1, 0, 0],
        [0, 1, 0.425],
        [0, 0, 1]
    ])

    T1 = np.array([1, 0, 0]).T
    T2 = np.array([1, 0.24, 0]).T

    plt.figure()
    angle1 = 0.25
    angle2 = 0.3
    H5 = brockett(H4, [(T1, angle1), (T2, angle2)])
    print(H5)
    x = [0, -0.24 * np.sin(angle1 * np.pi), -0.185 * np.sin((angle1 + angle2) * np.pi) - 0.24 * np.sin(angle1 * np.pi)]
    y = [0, 0.24 * np.cos(angle1 * np.pi), 0.185 * np.cos((angle1 + angle2) * np.pi) + 0.24 * np.cos(angle1 * np.pi)]
    plt.plot(x, y, 'r+', markersize = 10)
    plt.plot(H5[0, 2], H5[1, 2], 'bo')
    plt.show()
