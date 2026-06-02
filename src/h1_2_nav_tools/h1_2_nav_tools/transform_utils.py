import math

from geometry_msgs.msg import PoseStamped, TransformStamped


def quat_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_conjugate(q):
    return (-q[0], -q[1], -q[2], q[3])


def quat_normalize(q):
    norm = math.sqrt(sum(v * v for v in q))
    if norm < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(v / norm for v in q)


def rotate_vector(q, v):
    q = quat_normalize(q)
    vx, vy, vz, _ = quat_multiply(quat_multiply(q, (v[0], v[1], v[2], 0.0)), quat_conjugate(q))
    return vx, vy, vz


def transform_pose(pose: PoseStamped, transform: TransformStamped) -> PoseStamped:
    q = transform.transform.rotation
    t = transform.transform.translation
    rot = (q.x, q.y, q.z, q.w)

    p = pose.pose.position
    rx, ry, rz = rotate_vector(rot, (p.x, p.y, p.z))

    out = PoseStamped()
    out.header.stamp = transform.header.stamp
    out.header.frame_id = transform.header.frame_id
    out.pose.position.x = rx + t.x
    out.pose.position.y = ry + t.y
    out.pose.position.z = rz + t.z

    oq = pose.pose.orientation
    ox, oy, oz, ow = quat_normalize(quat_multiply(rot, (oq.x, oq.y, oq.z, oq.w)))
    out.pose.orientation.x = ox
    out.pose.orientation.y = oy
    out.pose.orientation.z = oz
    out.pose.orientation.w = ow
    return out
