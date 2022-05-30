import sympy


def quat_mult(p,q):
    r = sympy.Matrix([p[0] * q[0] - p[1] * q[1] - p[2] * q[2] - p[3] * q[3],
                p[0] * q[1] + p[1] * q[0] + p[2] * q[3] - p[3] * q[2],
                p[0] * q[2] - p[1] * q[3] + p[2] * q[0] + p[3] * q[1],
                p[0] * q[3] + p[1] * q[2] - p[2] * q[1] + p[3] * q[0]])

    return r


def quat2Rot(q):
    q0 = q[0]
    q1 = q[1]
    q2 = q[2]
    q3 = q[3]

    Rot = sympy.Matrix([[1 - 2*q2**2 - 2*q3**2, 2*(q1*q2 - q0*q3), 2*(q1*q3 + q0*q2)],
                  [2*(q1*q2 + q0*q3), 1 - 2*q1**2 - 2*q3**2, 2*(q2*q3 - q0*q1)],
                   [2*(q1*q3-q0*q2), 2*(q2*q3 + q0*q1), 1 - 2*q1**2 - 2*q2**2]])

    return Rot


def create_cov_matrix(i, j):
    if j >= i:
        return sympy.Symbol("P(" + str(i) + "," + str(j) + ")", real=True)
        # legacy array format
        # return Symbol("P[" + str(i) + "][" + str(j) + "]", real=True)
    else:
        return 0


def create_symmetric_cov_matrix(size):
    # define a symbolic covariance matrix
    P = sympy.Matrix(size[0],size[1],create_cov_matrix)

    for index in range(size[0]):
        for j in range(size[0]):
            if index > j:
                P[index,j] = P[j,index]

    return P
