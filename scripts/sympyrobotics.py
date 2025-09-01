from sympy import symbols, cos, sin, Matrix

def Rz(yaw):
    return Matrix([
        [cos(yaw), -sin(yaw), 0],
        [sin(yaw),  cos(yaw), 0],
        [0,         0,        1],
    ])

def Ry(pitch):
    return Matrix([
        [cos(pitch),  0, sin(pitch)],
        [0,           1, 0         ],
        [-sin(pitch), 0, cos(pitch)],
    ])

def Rx(roll):
    return Matrix([
        [1,         0,          0],
        [0, cos(roll), -sin(roll)],
        [0, sin(roll),  cos(roll)],
    ])