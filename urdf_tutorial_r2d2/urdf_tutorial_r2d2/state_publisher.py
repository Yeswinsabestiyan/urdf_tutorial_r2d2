#!/usr/bin/env python3
from math import sin, cos, pi, atan2, sqrt

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, Point
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


def quat_mul(a, b):
    """Hamilton product of two (x, y, z, w) quaternions."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_yaw(yaw):
    return (0.0, 0.0, sin(yaw / 2.0), cos(yaw / 2.0))


def quat_pitch(pitch):
    return (0.0, sin(pitch / 2.0), 0.0, cos(pitch / 2.0))


def smoothstep(t):
    t = max(0.0, min(1.0, t))
    return t * t * (3.0 - 2.0 * t)


GROUND_CLEARANCE = 0.6  # base_link height above ground on flat terrain


class StraightEdge:
    """A straight leg of the loop. offset_fn(u) adds a sideways wiggle
    (cones / maze zig-zag); height_fn(u) adds height + pitch (the ramp).
    Both MUST return 0 / (GROUND_CLEARANCE, 0) at u=0 and u=length, so
    edges join up cleanly with the flat, straight corners -- that's what
    keeps the whole loop free of position/orientation jumps."""

    def __init__(self, start, heading, length, offset_fn=None, height_fn=None):
        self.start = start
        self.heading = heading
        self.length = length
        self.offset_fn = offset_fn
        self.height_fn = height_fn

    def point_at(self, u, lateral=0.0):
        fx, fy = cos(self.heading), sin(self.heading)
        lx, ly = -fy, fx
        return (self.start[0] + fx * u + lx * lateral,
                self.start[1] + fy * u + ly * lateral)

    def sample(self, u):
        u = max(0.0, min(self.length, u))
        off = self.offset_fn(u) if self.offset_fn else 0.0
        x, y = self.point_at(u, off)
        z, pitch = self.height_fn(u) if self.height_fn else (GROUND_CLEARANCE, 0.0)
        return x, y, z, pitch

    def end_point(self):
        return self.point_at(self.length, 0.0)


class CornerArc:
    """A flat, constant-radius 90-degree LEFT turn joining two straight
    edges. Every corner in this course is one of these, so four of them
    in a row always add up to exactly 360 degrees with zero net drift."""

    def __init__(self, start, start_heading, radius):
        self.start = start
        self.start_heading = start_heading
        self.radius = radius
        self.length = radius * (pi / 2.0)
        lx, ly = -sin(start_heading), cos(start_heading)  # left normal
        self.center = (start[0] + radius * lx, start[1] + radius * ly)
        self.v0 = (start[0] - self.center[0], start[1] - self.center[1])

    def sample(self, u):
        u = max(0.0, min(self.length, u))
        theta = u / self.radius
        ct, st = cos(theta), sin(theta)
        vx = self.v0[0] * ct - self.v0[1] * st
        vy = self.v0[0] * st + self.v0[1] * ct
        return (self.center[0] + vx, self.center[1] + vy, GROUND_CLEARANCE, 0.0)

    def end_point(self):
        x, y, _, _ = self.sample(self.length)
        return (x, y)

    def end_heading(self):
        return self.start_heading + pi / 2.0


class Path:
    """Stitches segments into one closed loop, addressed by total arc
    length s. sample() wraps s with a plain modulo, so the robot just
    keeps driving forward forever -- no direction flag, no reversing."""

    def __init__(self, segments):
        self.segments = segments
        self.cum = [0.0]
        for seg in segments:
            self.cum.append(self.cum[-1] + seg.length)
        self.total_length = self.cum[-1]

    def sample(self, s):
        s = s % self.total_length
        for i, seg in enumerate(self.segments):
            if s <= self.cum[i + 1] + 1e-9 or i == len(self.segments) - 1:
                return seg.sample(s - self.cum[i])
        return self.segments[-1].sample(self.segments[-1].length)


class StatePublisher(Node):
    """Drives R2D2 forever around one closed rectangular loop, always
    moving forward:
      1) south leg   - weaves through 3 cones, climbs the ramp onto a
                        box, crosses the top, comes straight back down,
      2) left turn -> bottom leg (plain straight, "normal moving"),
      3) left turn -> east leg: a maze of square walls + round pillars,
      4) left turn -> top leg (plain straight) back to the start point,
      5) left turn -> back onto the south leg, and it repeats forever.
    All four corners are identical 90-degree left turns, and the two
    vertical legs / two horizontal legs are each forced to equal length
    in code, so the loop always closes exactly -- no drift, no gaps.
    """

    # --- south leg: cones + ramp -------------------------------------
    CONE_US = [1.0, 2.0, 3.0]        # 3 cones (down from 6)
    WEAVE_AMPLITUDE = 0.6
    WEAVE_TAPER_ZONE = 0.6
    WEAVE_FADE_IN = 0.6

    RAMP_UP_START = 3.7
    RAMP_UP_END = 4.7
    TOP_END = 5.7
    RAMP_DOWN_END = 6.7
    LEAD_OUT = 0.5
    EDGE_A_LEN = RAMP_DOWN_END + LEAD_OUT       # 7.2

    BOX_HEIGHT = 0.6
    PITCH_BLEND = 0.25

    # --- loop shape ---------------------------------------------------
    CORNER_RADIUS = 1.0
    EDGE_B_LEN = 3.0                            # bottom straight (ramp <-> maze gap)
    EDGE_D_LEN = EDGE_B_LEN                     # top straight (must match B)
    EDGE_C_LEN = EDGE_A_LEN                     # maze leg (must match A)

    # --- speed bumps on the two plain straights, so they don't sit empty --
    BUMP_FRACS = [0.3, 0.6, 0.85]     # positions along each straight, as a fraction of its length
    BUMP_WIDTH = 0.5                  # meters the bump spans along the direction of travel
    BUMP_HEIGHT = 0.05                # meters the body rises over each bump (subtle, not a jump)

    # --- maze (east leg) -----------------------------------------------
    MAZE_AMPLITUDE = 0.9
    MAZE_KEYS = [(0.0, 0.0), (0.22, 1.0), (0.5, -1.0), (0.78, 1.0), (1.0, 0.0)]
    MAZE_WALL_GAP = 0.5
    MAZE_WALL_SAMPLES = 18

    SPEED = 0.55  # m/s

    def __init__(self):
        super().__init__('r2d2_state_publisher')

        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'obstacle_markers', 10)
        self.trail_pub = self.create_publisher(Marker, 'robot_trail', 10)
        self.broadcaster = TransformBroadcaster(self)

        self.wheel_angle = 0.0
        self.gripper_pos = 0.0
        self.gripper_dir = 1
        self.head_phase = 0.0

        ramp_len = self.RAMP_UP_END - self.RAMP_UP_START
        self.ramp_angle = atan2(self.BOX_HEIGHT, ramp_len)

        # ---- build the closed loop: 4 straight edges + 4 left corners ----
        heading_s = -pi / 2.0   # south
        start = (0.0, 0.0)

        self.edge_cones_ramp = StraightEdge(
            start, heading_s, self.EDGE_A_LEN,
            offset_fn=self.edgeA_offset, height_fn=self.edgeA_height)

        c1 = CornerArc(self.edge_cones_ramp.end_point(), heading_s, self.CORNER_RADIUS)
        heading_e = c1.end_heading()

        self.edge_bottom = StraightEdge(
            c1.end_point(), heading_e, self.EDGE_B_LEN,
            height_fn=self.make_bump_height(self.EDGE_B_LEN))

        c2 = CornerArc(self.edge_bottom.end_point(), heading_e, self.CORNER_RADIUS)
        heading_n = c2.end_heading()

        self.edge_maze = StraightEdge(
            c2.end_point(), heading_n, self.EDGE_C_LEN,
            offset_fn=self.edgeC_offset)

        c3 = CornerArc(self.edge_maze.end_point(), heading_n, self.CORNER_RADIUS)
        heading_w = c3.end_heading()

        self.edge_top = StraightEdge(
            c3.end_point(), heading_w, self.EDGE_D_LEN,
            height_fn=self.make_bump_height(self.EDGE_D_LEN))

        c4 = CornerArc(self.edge_top.end_point(), heading_w, self.CORNER_RADIUS)

        self.path = Path([self.edge_cones_ramp, c1, self.edge_bottom, c2,
                           self.edge_maze, c3, self.edge_top, c4])

        self.s = 0.0
        self.prev_x, self.prev_y, self.prev_z = start[0], start[1], GROUND_CLEARANCE
        self.prev_yaw = heading_s
        self.trail_points = []

        self.get_logger().info('r2d2_state_publisher started (looping course)')
        self.timer = self.create_timer(1.0 / 30.0, self.update)
        self.marker_timer = self.create_timer(1.0, self.publish_scene)

    # ------------------------------------------------------------------
    # south-leg cone weave + ramp (pure functions of u along that edge)
    # ------------------------------------------------------------------
    def edgeA_offset(self, u):
        if u >= self.RAMP_UP_START:
            return 0.0
        raw = self.WEAVE_AMPLITUDE * cos(pi * (u - self.CONE_US[0]))
        if u < self.WEAVE_FADE_IN:
            raw *= max(0.0, u / self.WEAVE_FADE_IN)
        fade_start = self.RAMP_UP_START - self.WEAVE_TAPER_ZONE
        if u > fade_start:
            taper = max(0.0, (self.RAMP_UP_START - u) / self.WEAVE_TAPER_ZONE)
            raw *= taper
        return raw

    def edgeA_height(self, u):
        base = GROUND_CLEARANCE
        if u < self.RAMP_UP_START:
            return base, 0.0
        elif u < self.RAMP_UP_END:
            t = (u - self.RAMP_UP_START) / (self.RAMP_UP_END - self.RAMP_UP_START)
            height = base + self.BOX_HEIGHT * t
            ease_in = smoothstep((u - self.RAMP_UP_START) / self.PITCH_BLEND)
            ease_out = smoothstep((self.RAMP_UP_END - u) / self.PITCH_BLEND)
            pitch = -self.ramp_angle * min(ease_in, ease_out)
            return height, pitch
        elif u < self.TOP_END:
            return base + self.BOX_HEIGHT, 0.0
        elif u < self.RAMP_DOWN_END:
            t = (u - self.TOP_END) / (self.RAMP_DOWN_END - self.TOP_END)
            return base + self.BOX_HEIGHT * (1.0 - t), 0.0
        else:
            return base, 0.0

    def make_bump_height(self, length):
        """A straight leg's height_fn: flat ground with a very low, wide
        ripple (no pitch -- body stays level) over each speed bump."""
        def h(u):
            base = GROUND_CLEARANCE
            bump = 0.0
            half_w = self.BUMP_WIDTH / 2.0
            for frac in self.BUMP_FRACS:
                d = abs(u - frac * length)
                if d < half_w:
                    bump += self.BUMP_HEIGHT * 0.5 * (1.0 + cos(pi * d / half_w))
            return base + bump, 0.0
        return h

    # ------------------------------------------------------------------
    # east-leg maze zig-zag: eased between key offsets (not a smooth
    # sine), so it reads as distinct corridor turns, not a weave
    # ------------------------------------------------------------------
    def edgeC_offset(self, u):
        frac = u / self.EDGE_C_LEN
        keys = self.MAZE_KEYS
        for i in range(len(keys) - 1):
            u0, v0 = keys[i]
            u1, v1 = keys[i + 1]
            if frac <= u1 or i == len(keys) - 2:
                local = (frac - u0) / (u1 - u0) if u1 > u0 else 0.0
                local = smoothstep(local)
                return self.MAZE_AMPLITUDE * (v0 + (v1 - v0) * local)
        return 0.0

    # ------------------------------------------------------------------
    def publish_scene(self):
        stamp = self.get_clock().now().to_msg()
        marker_array = MarkerArray()
        heading_s = self.edge_cones_ramp.heading

        for i, u in enumerate(self.CONE_US):
            x, y = self.edge_cones_ramp.point_at(u, 0.0)
            m = Marker()
            m.header.frame_id = 'odom'
            m.header.stamp = stamp
            m.ns = 'cones'
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = 0.25
            m.pose.orientation.w = 1.0
            m.scale.x = 0.2
            m.scale.y = 0.2
            m.scale.z = 0.5
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.5, 0.0, 1.0
            marker_array.markers.append(m)

        # raised platform (top of the box)
        top_x, top_y = self.edge_cones_ramp.point_at(
            (self.RAMP_UP_END + self.TOP_END) / 2.0, 0.0)
        qx, qy, qz, qw = quat_yaw(heading_s)
        top = Marker()
        top.header.frame_id = 'odom'
        top.header.stamp = stamp
        top.ns = 'box'
        top.id = 0
        top.type = Marker.CUBE
        top.action = Marker.ADD
        top.pose.position.x = top_x
        top.pose.position.y = top_y
        top.pose.position.z = self.BOX_HEIGHT / 2.0
        top.pose.orientation.x, top.pose.orientation.y = qx, qy
        top.pose.orientation.z, top.pose.orientation.w = qz, qw
        top.scale.x = self.TOP_END - self.RAMP_UP_END
        top.scale.y = 1.0
        top.scale.z = self.BOX_HEIGHT
        top.color.r, top.color.g, top.color.b, top.color.a = 0.55, 0.55, 0.6, 1.0
        marker_array.markers.append(top)

        ramp_len = self.RAMP_UP_END - self.RAMP_UP_START
        slope_len = sqrt(ramp_len ** 2 + self.BOX_HEIGHT ** 2)

        def ramp_slab(marker_id, mid_u, pitch_sign):
            mx, my = self.edge_cones_ramp.point_at(mid_u, 0.0)
            slab = Marker()
            slab.header.frame_id = 'odom'
            slab.header.stamp = stamp
            slab.ns = 'box'
            slab.id = marker_id
            slab.type = Marker.CUBE
            slab.action = Marker.ADD
            slab.pose.position.x = mx
            slab.pose.position.y = my
            slab.pose.position.z = self.BOX_HEIGHT / 2.0
            sqx, sqy, sqz, sqw = quat_mul(quat_yaw(heading_s), quat_pitch(pitch_sign * self.ramp_angle))
            slab.pose.orientation.x, slab.pose.orientation.y = sqx, sqy
            slab.pose.orientation.z, slab.pose.orientation.w = sqz, sqw
            slab.scale.x = slope_len
            slab.scale.y = 1.0
            slab.scale.z = 0.06
            slab.color.r, slab.color.g, slab.color.b, slab.color.a = 0.45, 0.45, 0.5, 1.0
            marker_array.markers.append(slab)

        ramp_slab(1, (self.RAMP_UP_START + self.RAMP_UP_END) / 2.0, -1.0)
        ramp_slab(2, (self.TOP_END + self.RAMP_DOWN_END) / 2.0, 1.0)

        # maze walls: sample the winding corridor, drop a wall cube on
        # each side, oriented along the corridor's local direction there
        du = 0.05
        for i in range(self.MAZE_WALL_SAMPLES + 1):
            u = self.EDGE_C_LEN * i / self.MAZE_WALL_SAMPLES
            u0 = max(0.0, u - du)
            u1 = min(self.EDGE_C_LEN, u + du)
            x0, y0 = self.edge_maze.point_at(u0, self.edgeC_offset(u0))
            x1, y1 = self.edge_maze.point_at(u1, self.edgeC_offset(u1))
            tx, ty = x1 - x0, y1 - y0
            tlen = sqrt(tx * tx + ty * ty) or 1.0
            tx, ty = tx / tlen, ty / tlen
            lx, ly = -ty, tx
            cx, cy = self.edge_maze.point_at(u, self.edgeC_offset(u))
            wqx, wqy, wqz, wqw = quat_yaw(atan2(ty, tx))

            for side, wid in ((1.0, 100 + 2 * i), (-1.0, 100 + 2 * i + 1)):
                w = Marker()
                w.header.frame_id = 'odom'
                w.header.stamp = stamp
                w.ns = 'maze_walls'
                w.id = wid
                w.type = Marker.CUBE
                w.action = Marker.ADD
                w.pose.position.x = cx + lx * self.MAZE_WALL_GAP * side
                w.pose.position.y = cy + ly * self.MAZE_WALL_GAP * side
                w.pose.position.z = 0.2
                w.pose.orientation.x, w.pose.orientation.y = wqx, wqy
                w.pose.orientation.z, w.pose.orientation.w = wqz, wqw
                w.scale.x = 0.4
                w.scale.y = 0.08
                w.scale.z = 0.4
                w.color.r, w.color.g, w.color.b, w.color.a = 0.5, 0.5, 0.55, 1.0
                marker_array.markers.append(w)

        # speed bumps across the two plain straights (bottom + top): very
        # low, wide boxes flat on the grid, so they read as ground-level
        # hazards rather than empty road
        bid = 0
        for edge in (self.edge_bottom, self.edge_top):
            for frac in self.BUMP_FRACS:
                u = frac * edge.length
                bx, by = edge.point_at(u, 0.0)
                bqx, bqy, bqz, bqw = quat_yaw(edge.heading)
                b = Marker()
                b.header.frame_id = 'odom'
                b.header.stamp = stamp
                b.ns = 'speed_bumps'
                b.id = bid
                bid += 1
                b.type = Marker.CUBE
                b.action = Marker.ADD
                b.pose.position.x = bx
                b.pose.position.y = by
                b.pose.position.z = 0.03
                b.pose.orientation.x, b.pose.orientation.y = bqx, bqy
                b.pose.orientation.z, b.pose.orientation.w = bqz, bqw
                b.scale.x = self.BUMP_WIDTH
                b.scale.y = 0.9
                b.scale.z = 0.06
                b.color.r, b.color.g, b.color.b, b.color.a = 0.9, 0.75, 0.1, 1.0
                marker_array.markers.append(b)

        self.marker_pub.publish(marker_array)

    def publish_trail(self, x, y, z):
        self.trail_points.append(Point(x=x, y=y, z=z))
        if len(self.trail_points) > 500:
            self.trail_points.pop(0)

        m = Marker()
        m.header.frame_id = 'odom'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'trail'
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.03
        m.color.r, m.color.g, m.color.b, m.color.a = 0.2, 0.8, 1.0, 0.9
        m.pose.orientation.w = 1.0
        m.points = self.trail_points
        self.trail_pub.publish(m)

    # ------------------------------------------------------------------
    def update(self):
        now = self.get_clock().now()
        dt = 1.0 / 30.0

        self.s = (self.s + self.SPEED * dt) % self.path.total_length
        x, y, z, pitch = self.path.sample(self.s)

        dx, dy = x - self.prev_x, y - self.prev_y
        yaw = atan2(dy, dx) if (abs(dx) > 1e-7 or abs(dy) > 1e-7) else self.prev_yaw
        self.prev_x, self.prev_y, self.prev_z = x, y, z
        self.prev_yaw = yaw

        # --- animate joints (always forward, never reverses) ---
        self.wheel_angle = (self.wheel_angle + 8 * pi / 180.0) % (2 * pi)
        self.head_phase += 0.05
        head_angle = 0.5 * sin(self.head_phase)

        self.gripper_pos += self.gripper_dir * 0.002
        if self.gripper_pos > 0.15 or self.gripper_pos < 0.0:
            self.gripper_dir *= -1

        joint_state = JointState()
        joint_state.header.stamp = now.to_msg()
        joint_state.name = [
            'left_wheel_joint', 'right_wheel_joint',
            'head_swivel',
            'gripper_extension',
            'left_gripper_joint', 'right_gripper_joint',
        ]
        joint_state.position = [
            self.wheel_angle, self.wheel_angle,
            head_angle,
            -self.gripper_pos,
            self.gripper_pos * 3.5,
            -self.gripper_pos * 3.5,
        ]
        self.joint_pub.publish(joint_state)

        qx, qy, qz, qw = quat_mul(quat_yaw(yaw), quat_pitch(pitch))

        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.broadcaster.sendTransform(t)

        self.publish_trail(x, y, z)


def main():
    rclpy.init()
    node = StatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
