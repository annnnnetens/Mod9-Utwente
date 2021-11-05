import math
# When working on the microcontroller change the numpy to ulab since numpy is not available on the microcontroller

try:
    import numpy as np
    on_microcontroller = False
    from matplotlib import pyplot as plt
except ImportError:
    import ulab as np
    on_microcontroller = True
pi = 3.141592653589793


def h_to_adjoint(h_matrix):
    """Turns H Matrix into Adjoint"""
    res = np.eye(3)
    res[1:3, 1:3] = h_matrix[0:2, 0:2]
    res[1, 0] = h_matrix[1, 2]
    res[2, 0] = h_matrix[0, 2] * -1
    return res


def brockett(h0_matrix, twistspairs):
    """
    Creates H matrix using brockett
    Inputs: H0_matrix is the H matrix in the reference configuration
    twistpairs is a list of tuples of a twist and the corresponding q (angle or translation
    If degrees is true it is assumed that q is in degrees (for example q = 45)
    If degrees is false it is assumed that q is in radians (for example q = 0.25) (so no multiplng with pi!)
    """
    result = np.eye(3)
    for twistpair in twistspairs:
        current_matrix = np.eye(3)
        twist = twistpair[0]
        q = twistpair[1]
        if twist[0] == 0:
            # Translational joint
            x = twist[1]
            y = twist[2]
            current_matrix[0, 2] = x * q
            current_matrix[1, 2] = y * q

            result = matrix_multiplication(result, current_matrix)
        elif twist[0] == 1:
            # Rotational joint
            c = math.cos(q * pi)
            s = math.sin(q * pi)
            current_matrix[0:2, 0:2] = np.array([[c, -s], [s, c]])
            x = twist[2] * -1
            y = twist[1]
            current_matrix[0, 2] = x - x * c + y * s
            current_matrix[1, 2] = y - x * s - y * c
            result = matrix_multiplication(result, current_matrix)
    result = matrix_multiplication(result, h0_matrix)
    return result


def jacobian_angles(q1):
    """Calculate jacobian using the angles
    Keep in mind that it only needs q1 and the length of the first part is assumed to be 240mm
    Currently only uses radians"""
    J = np.zeros((3, 2))
    J[0] = np.array([1, 1])
    J[1] = np.array([0, 0.24 * math.cos(q1 * pi)])
    J[2] = np.array([0, 0.24 * math.sin(q1 * pi)])
    return J


def unit_twist_rotational(px, py):
    return np.array([1, py, -px])


def matinv2x2(M):
    res = np.eye(2)
    det = M[0, 0] * M[1, 1] - M[1, 0] * M[0, 1]
    res[0, 0] = M[1, 1]/det
    res[1, 1] = M[0, 0]/det
    res[1, 0] = - M[1, 0]/det
    res[0, 1] = - M[0, 1]/det
    return res


def calculate_dq_j_inv(q1, q2, x_des, y_des):
    L3 = 0.028
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


    if on_microcontroller:
        vx = x_des - pe0[0][0]
        vy = y_des - pe0[1][0]
        H_0_f = np.array([[1, 0, -pe0[0][0]],
                          [0, 1, -pe0[1][0]],
                          [0, 0, 1]])
    else:
        vx = x_des - pe0[0]
        vy = y_des - pe0[1]
        H_0_f = np.array([[1, 0, -pe0[0]],
                        [0, 1, -pe0[1]],
                        [0, 0, 1]])

    adj_0_f = h_to_adjoint(H_0_f)

    jprime = matrix_multiplication(adj_0_f, J)

    jdoubleprime = jprime[1:, :]

    j_inv = matinv2x2(jdoubleprime)

    qdot_setp = matrix_multiplication(j_inv, np.array([vx, vy]).transpose())
    return qdot_setp

def matrix_multiplication(a, b):
    if on_microcontroller:
        return np.linalg.dot(a,b)
    else:
        return a @ b


if __name__ == "__main__":
    if on_microcontroller:
        pass
    else:
        # For testing
        identity = np.eye(3)
        H4 = np.array([
            [1, 0, 0],
            [0, 1, 0.425],
            [0, 0, 1]
        ])

        T1 = np.array([1, 0, 0])
        T2 = np.array([1, 0.24, 0])

        plt.figure()

        angle1 = 0
        angle2 = 0
        timestep = 0.01
        vx = 0.5
        vy = -1

        H5 = brockett(H4, [(T1, angle1), (T2, angle2)])
        print(H5)
        x = [0, -0.24 * math.sin(angle1 * pi), -0.185 * math.sin((angle1 + angle2) * pi) - 0.24 * math.sin(angle1 * pi)]
        y = [0, 0.24 * math.cos(angle1 * pi), 0.185 * math.cos((angle1 + angle2) * pi) + 0.24 * math.cos(angle1 * pi)]
        plt.plot(x, y, 'r+', markersize=10)

        des_x = x[2] + vx * timestep
        des_y = y[2] + vy * timestep


        for i in range(100):

            [dq1, dq2] = calculate_dq_j_inv(angle1, angle2, des_x, des_y)
            angle1 = angle1 + timestep * dq1
            angle2 = angle2 + timestep * dq2

            # xx = [0, -0.24 * math.sin(angle1 * pi), -0.185 * math.sin((angle1 + angle2) * pi) - 0.24 * math.sin(angle1 * pi)]
            # yy = [0, 0.24 * math.cos(angle1 * pi), 0.185 * math.cos((angle1 + angle2) * pi) + 0.24 * math.cos(angle1 * pi)]
            #
            # plt.plot(xx, yy)

            xx = [-0.185 * math.sin((angle1 + angle2) * pi) - 0.24 * math.sin(angle1 * pi)]
            yy = [0.185 * math.cos((angle1 + angle2) * pi) + 0.24 * math.cos(angle1 * pi)]

            plt.scatter(xx, yy)

            des_x = des_x + vx * timestep
            des_y = des_y + vy * timestep

        plt.xticks([-0.5, -0.4, -0.3, -0.2, -0.1, 0, 0.1, 0.2, 0.3, 0.4, 0.5])
        plt.yticks([-0.5, -0.4, -0.3, -0.2, -0.1, 0, 0.1, 0.2, 0.3, 0.4, 0.5])

        plt.show()

