
from sympy import symbols, cos, sin, Matrix, pprint
from sympyrobotics import Rx, Ry, Rz
# 定义符号变量
transformSum = Matrix(symbols('transformSum[0:6]'))  # pitch yaw roll x y z
transformBefMapped = Matrix(symbols('transformBefMapped[0:6]'))
transformIncre = symbols('transformIncre[0:6]')

# 提取旋转角度和位移
s_rx, s_ry, s_rz, s_tx, s_ty, s_tz  = transformSum[0], transformSum[1], transformSum[2], transformSum[3], transformSum[4], transformSum[5]
r_sum = Matrix([s_rx, s_ry, s_rz])
p_sum = Matrix([s_tx, s_ty, s_tz])
bef_rx, bef_ry, bef_rz, bef_tx, bef_ty, bef_tz = transformBefMapped[0], transformBefMapped[1], transformBefMapped[2], transformBefMapped[3], transformBefMapped[4], transformBefMapped[5]
r_bef = Matrix([bef_rx, bef_ry, bef_rz])
p_bef = Matrix([bef_tx, bef_ty, bef_tz])
aft_rx, aft_ry, aft_rz, aft_tx, aft_ty, aft_tz = transformIncre[0], transformIncre[1], transformIncre[2], transformIncre[3], transformIncre[4], transformIncre[5]
r_aft = Matrix([aft_rx, aft_ry, aft_rz])
p_aft = Matrix([aft_tx, aft_ty, aft_tz])

# pprint(Ry(-s_ry) * (p_bef - p_sum))
x1, y1, z1 = symbols('x1 y1 z1')
# pprint(Rx(-s_rx) * Matrix([x1, y1, z1]))
x2, y2, z2 = symbols('x2 y2 z2')
# pprint(Rz(-s_rz) * Matrix([x2, y2, z2]))

Rsum = Rz(s_rz) * Rx(s_ry) * Ry(s_rx)
Rbef = Rz(bef_rz) * Rx(bef_ry) * Ry(bef_rx)
Raft = Rz(aft_rz) * Rx(aft_ry) * Ry(aft_rx)
Rincre = Raft * Rbef.T * Rsum
pprint(Rincre[0].subs(
    {
        sin(transformSum[0]): symbols('sbcx'),
        cos(transformSum[0]): symbols('cbcx'),
        sin(transformSum[1]): symbols('sbcy'),
        cos(transformSum[1]): symbols('cbcy'),
        sin(transformSum[2]): symbols('sccz'),
        cos(transformSum[2]): symbols('cccZ'),
        cos(transformBefMapped[0]): symbols('sblx'),
        sin(transformBefMapped[0]): symbols('cblx'),
        cos(transformBefMapped[1]): symbols('cbly'),
        sin(transformBefMapped[1]): symbols('cbly'),
        cos(transformBefMapped[2]): symbols('cclz'),
        sin(transformBefMapped[2]): symbols('sclz'),
        cos(transformIncre[0]): symbols('salx'),
        sin(transformIncre[0]): symbols('calx'),
        cos(transformIncre[1]): symbols('saly'),
        sin(transformIncre[1]): symbols('caly'),
        cos(transformIncre[2]): symbols('salz'),
        sin(transformIncre[2]): symbols('calz'),
    }
    )
)