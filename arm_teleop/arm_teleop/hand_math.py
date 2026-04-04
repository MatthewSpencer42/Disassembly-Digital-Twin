from __future__ import annotations

import math
from typing import Iterable


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def vec_add(a: Iterable[float], b: Iterable[float]) -> list[float]:
    return [float(x) + float(y) for x, y in zip(a, b)]


def vec_sub(a: Iterable[float], b: Iterable[float]) -> list[float]:
    return [float(x) - float(y) for x, y in zip(a, b)]


def vec_scale(a: Iterable[float], scalar: float) -> list[float]:
    return [float(x) * float(scalar) for x in a]


def dot(a: Iterable[float], b: Iterable[float]) -> float:
    return sum(float(x) * float(y) for x, y in zip(a, b))


def cross(a: Iterable[float], b: Iterable[float]) -> list[float]:
    ax, ay, az = [float(v) for v in a]
    bx, by, bz = [float(v) for v in b]
    return [
        ay * bz - az * by,
        az * bx - ax * bz,
        ax * by - ay * bx,
    ]


def norm(a: Iterable[float]) -> float:
    return math.sqrt(dot(a, a))


def normalize(a: Iterable[float]) -> list[float] | None:
    magnitude = norm(a)
    if magnitude < 1e-8:
        return None
    return [float(v) / magnitude for v in a]


def mat_vec_mul(matrix: list[list[float]], vector: Iterable[float]) -> list[float]:
    vx, vy, vz = [float(v) for v in vector]
    return [
        matrix[0][0] * vx + matrix[0][1] * vy + matrix[0][2] * vz,
        matrix[1][0] * vx + matrix[1][1] * vy + matrix[1][2] * vz,
        matrix[2][0] * vx + matrix[2][1] * vy + matrix[2][2] * vz,
    ]


def quat_normalize(q: Iterable[float]) -> list[float]:
    qx, qy, qz, qw = [float(v) for v in q]
    mag = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if mag < 1e-8:
        return [0.0, 0.0, 0.0, 1.0]
    return [qx / mag, qy / mag, qz / mag, qw / mag]


def quat_conjugate(q: Iterable[float]) -> list[float]:
    qx, qy, qz, qw = [float(v) for v in q]
    return [-qx, -qy, -qz, qw]


def quat_multiply(a: Iterable[float], b: Iterable[float]) -> list[float]:
    ax, ay, az, aw = [float(v) for v in a]
    bx, by, bz, bw = [float(v) for v in b]
    return quat_normalize(
        [
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        ]
    )


def quat_rotate_vector(q: Iterable[float], vector: Iterable[float]) -> list[float]:
    rotated = quat_multiply(quat_multiply(q, [*vector, 0.0]), quat_conjugate(q))
    return rotated[:3]


def quat_to_matrix(q: Iterable[float]) -> list[list[float]]:
    x, y, z, w = quat_normalize(q)
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z
    return [
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ]


def matrix_to_quat(matrix: list[list[float]]) -> list[float]:
    m00, m01, m02 = matrix[0]
    m10, m11, m12 = matrix[1]
    m20, m21, m22 = matrix[2]
    trace = m00 + m11 + m22
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (m21 - m12) / s
        qy = (m02 - m20) / s
        qz = (m10 - m01) / s
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        qw = (m21 - m12) / s
        qx = 0.25 * s
        qy = (m01 + m10) / s
        qz = (m02 + m20) / s
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        qw = (m02 - m20) / s
        qx = (m01 + m10) / s
        qy = 0.25 * s
        qz = (m12 + m21) / s
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        qw = (m10 - m01) / s
        qx = (m02 + m20) / s
        qy = (m12 + m21) / s
        qz = 0.25 * s
    return quat_normalize([qx, qy, qz, qw])


def quaternion_slerp(q0: Iterable[float], q1: Iterable[float], alpha: float) -> list[float]:
    ax, ay, az, aw = quat_normalize(q0)
    bx, by, bz, bw = quat_normalize(q1)
    cos_half_theta = ax * bx + ay * by + az * bz + aw * bw
    if cos_half_theta < 0.0:
        bx, by, bz, bw = -bx, -by, -bz, -bw
        cos_half_theta = -cos_half_theta
    if cos_half_theta > 0.9995:
        return quat_normalize(
            [
                ax + alpha * (bx - ax),
                ay + alpha * (by - ay),
                az + alpha * (bz - az),
                aw + alpha * (bw - aw),
            ]
        )
    half_theta = math.acos(clamp(cos_half_theta, -1.0, 1.0))
    sin_half_theta = math.sqrt(max(1.0 - cos_half_theta * cos_half_theta, 0.0))
    if sin_half_theta < 1e-6:
        return [ax, ay, az, aw]
    ratio_a = math.sin((1.0 - alpha) * half_theta) / sin_half_theta
    ratio_b = math.sin(alpha * half_theta) / sin_half_theta
    return quat_normalize(
        [
            ax * ratio_a + bx * ratio_b,
            ay * ratio_a + by * ratio_b,
            az * ratio_a + bz * ratio_b,
            aw * ratio_a + bw * ratio_b,
        ]
    )
