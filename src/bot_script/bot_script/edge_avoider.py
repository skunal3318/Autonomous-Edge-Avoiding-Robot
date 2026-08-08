#!/usr/bin/env python3
"""
Edge-avoidance behavior node (v2) -- layered reactive/deliberative design.

Why not a plain forward -> reverse -> turn loop?
Because it has no memory: with a fixed reverse/turn duration, two edges
close enough together (or one edge encountered at a repeatable angle)
reliably send the robot right back into the same trigger, forever. The
symptom is a robot that "works" (state machine runs, wheels turn) but
never covers the floor.

Architecture
------------
EXPLORE   Forward driving with *proportional* steering away from edges
          detected in the scan's periphery -- a glancing edge on one
          side gets steered away from smoothly, no stop required.
AVOID     Triggered when an edge is under the robot's center bins
          (hard_stop). Reverses, then rotates away from the edge side.
          Duration escalates with how many AVOID episodes have fired
          in the last few seconds -- a sign the previous, shorter
          maneuver wasn't enough to clear whatever trap this is.
RECOVER   Triggered by either too many AVOID episodes in a short
          window, or by StuckDetector noticing the robot hasn't made
          net progress despite trying. Performs one large randomized
          rotation -- breaking the geometric symmetry that produced
          the trap -- followed by an extended forward run.

All scan interpretation lives in scan_utils.analyze_scan(): a pure
function, unit-tested independently of ROS. Stuck detection lives in
stuck_detector.StuckDetector, likewise pure and unit-tested. Every
threshold is a ROS parameter with a live-reconfigure callback, so
behavior can be tuned with `ros2 param set` while it's running.
"""
import functools
import math
import random
from collections import deque
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from bot_script.scan_utils import analyze_scan, is_cliff, SteeringDecision
from bot_script.stuck_detector import StuckDetector
from bot_script.hazard_memory import HazardMemory

# Corner name -> IR/cliff-sensor topic (see bot_description/urdf/bot.urdf.xacro
# and bot_controller/config/bridge.yaml). Order matters only for iteration
# stability; the mapping itself is what the node subscribes to.
IR_TOPICS = {
    'front_left': 'ir_front_left',
    'front_right': 'ir_front_right',
    'rear_left': 'ir_rear_left',
    'rear_right': 'ir_rear_right',
}


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class State(Enum):
    EXPLORE = auto()
    AVOID = auto()
    RECOVER = auto()


class EdgeAvoider(Node):
    def __init__(self):
        super().__init__('edge_avoider')

        self._declare_parameters()
        self._read_parameters()
        self.add_on_set_parameters_callback(self._on_parameters_set)

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.state_pub = self.create_publisher(String, '~/state', 10)
        self.marker_pub = self.create_publisher(Marker, '~/steering_marker', 10)
        self.hazard_marker_pub = self.create_publisher(MarkerArray, '~/hazard_map', 10)
        self.cliff_marker_pub = self.create_publisher(MarkerArray, '~/cliff_sensors', 10)

        self.create_subscription(LaserScan, self.scan_topic, self._on_scan, 10)
        self.create_subscription(Odometry, self.odom_topic, self._on_odom, 10)
        self._ir_ranges = {corner: None for corner in IR_TOPICS}
        for corner, topic in IR_TOPICS.items():
            self.create_subscription(
                LaserScan, topic, functools.partial(self._on_ir, corner=corner), 10)

        self.stuck_detector = StuckDetector(
            window_sec=self.stuck_window_sec,
            min_displacement=self.stuck_min_displacement)
        self.hazard_memory = HazardMemory(max_points=self.hazard_memory_size)

        self.state = State.EXPLORE
        self._state_entered = self._now()
        self._last_scan = None
        self._pos = (0.0, 0.0)
        self._yaw = 0.0
        self._geofence_breached = False
        self._cornered = False
        self._avoid_events = deque()   # timestamps of recent AVOID entries
        self._last_turn_dir = 0.0
        self._avoid_sub_state = 'REVERSE'
        self._avoid_turn_dir = 1.0
        self._avoid_reverse_time = self.reverse_time_base
        self._avoid_turn_time = self.turn_time_base
        self._recover_sub_state = 'TURN'
        self._recover_turn_dir = 1.0
        self._recover_turn_time = self.recover_turn_time_min

        self.timer = self.create_timer(1.0 / self.control_rate_hz, self._control_loop)
        self.get_logger().info('edge_avoider v2 (EXPLORE/AVOID/RECOVER) started')

    # ------------------------------------------------------------ params --
    def _declare_parameters(self):
        p = self.declare_parameter
        p('edge_range_threshold', 0.65)   # m; tune to your LIDAR mount, see bot.urdf.xacro
        p('num_scan_bins', 5)
        p('center_bins', 1)
        p('forward_speed', 0.15)
        p('steer_gain', 1.2)
        p('turn_speed', 0.6)
        p('reverse_speed', 0.15)
        # Backing up 0.5s @ old 0.12 m/s only cleared ~6cm -- not enough
        # for the chassis to safely rotate away from an edge without a
        # front corner swinging back over it. 1.1s @ 0.15 m/s clears
        # ~16.5cm, comfortably past the chassis's turn-sweep radius.
        # See bot.urdf.xacro's header comment for the geometry this
        # margin is sized against.
        p('reverse_time_base', 1.1)
        p('turn_time_base', 0.7)
        p('avoid_escalation', 0.35)        # extra seconds per repeat AVOID within the window
        p('avoid_window_sec', 8.0)
        p('avoid_trap_count', 3)            # N avoids within window -> escalate to RECOVER
        p('recover_turn_time_min', 1.2)
        p('recover_turn_time_max', 2.5)
        p('recover_forward_time', 3.0)
        p('stuck_window_sec', 6.0)
        p('stuck_min_displacement', 0.3)
        p('control_rate_hz', 15.0)
        p('scan_topic', '/scan')
        p('odom_topic', '/odom')
        p('cmd_vel_topic', '/cmd_vel')

        # Hazard memory: world-frame locations of past AVOID/RECOVER
        # triggers, used to steer future exploration away from spots
        # already known to be dangerous instead of reacting fresh every
        # time. See hazard_memory.py.
        p('hazard_memory_size', 200)
        p('hazard_memory_radius', 0.9)      # m; how far a remembered hazard's repulsion reaches
        p('hazard_repulsion_gain', 1.0)     # weight of hazard repulsion in the steering mix
        p('danger_slowdown_factor', 0.5)    # forward-speed multiplier while any edge is flagged

        # Corner cliff sensors: the forward LIDAR, however wide, can
        # only ever look where the robot is facing, so it structurally
        # cannot resolve an actual corner (two edges meeting) -- turning
        # away from one edge just points it at the other. A direct
        # "is there floor under this corner" reading at all four corners
        # is what most real edge-avoiding robots use instead of a single
        # forward sensor, and is authoritative: no geometry inference
        # involved. 0.20 m matches the sensors' own max range in the
        # urdf (~0.12 m expected on-floor reading + margin).
        p('cliff_range_threshold', 0.20)
        # For placing the cliff-status debug markers only; must match
        # bot.urdf.xacro's cliff_x/cliff_y if the chassis size changes.
        p('cliff_sensor_x_offset', 0.45)
        p('cliff_sensor_y_offset', 0.25)

        # Geofence: this project is scoped to a single 4x4 m test
        # platform centered at the odom origin (see edge_world.sdf). With
        # real cliff sensors now doing the actual detection work, this is
        # a last-resort backstop only (e.g. sensor/bridge dropout) rather
        # than the primary safety mechanism -- it doesn't depend on any
        # sensor at all. 1.3 m leaves ~0.7 m of margin from the
        # platform's true 2.0 m half-extent. 0 disables it for use on a
        # different world.
        p('geofence_half_extent', 1.3)

    def _read_parameters(self):
        g = lambda name: self.get_parameter(name).value
        self.edge_threshold = float(g('edge_range_threshold'))
        self.num_scan_bins = int(g('num_scan_bins'))
        self.center_bins = int(g('center_bins'))
        self.forward_speed = float(g('forward_speed'))
        self.steer_gain = float(g('steer_gain'))
        self.turn_speed = float(g('turn_speed'))
        self.reverse_speed = float(g('reverse_speed'))
        self.reverse_time_base = float(g('reverse_time_base'))
        self.turn_time_base = float(g('turn_time_base'))
        self.avoid_escalation = float(g('avoid_escalation'))
        self.avoid_window_sec = float(g('avoid_window_sec'))
        self.avoid_trap_count = int(g('avoid_trap_count'))
        self.recover_turn_time_min = float(g('recover_turn_time_min'))
        self.recover_turn_time_max = float(g('recover_turn_time_max'))
        self.recover_forward_time = float(g('recover_forward_time'))
        self.stuck_window_sec = float(g('stuck_window_sec'))
        self.stuck_min_displacement = float(g('stuck_min_displacement'))
        self.control_rate_hz = float(g('control_rate_hz'))
        self.scan_topic = g('scan_topic')
        self.odom_topic = g('odom_topic')
        self.cmd_vel_topic = g('cmd_vel_topic')
        self.hazard_memory_size = int(g('hazard_memory_size'))
        self.hazard_memory_radius = float(g('hazard_memory_radius'))
        self.hazard_repulsion_gain = float(g('hazard_repulsion_gain'))
        self.danger_slowdown_factor = float(g('danger_slowdown_factor'))
        self.cliff_range_threshold = float(g('cliff_range_threshold'))
        self.cliff_sensor_x_offset = float(g('cliff_sensor_x_offset'))
        self.cliff_sensor_y_offset = float(g('cliff_sensor_y_offset'))
        self.geofence_half_extent = float(g('geofence_half_extent'))

    def _on_parameters_set(self, params):
        # rclpy has already applied the new values to the parameter server;
        # just refresh our cached copies so the change takes effect live.
        self._read_parameters()
        if hasattr(self, 'stuck_detector'):
            self.stuck_detector.window_sec = self.stuck_window_sec
            self.stuck_detector.min_displacement = self.stuck_min_displacement
        return SetParametersResult(successful=True)

    # --------------------------------------------------------- callbacks --
    def _on_scan(self, msg: LaserScan):
        self._last_scan = msg

    def _on_ir(self, msg: LaserScan, corner: str):
        if msg.ranges:
            self._ir_ranges[corner] = msg.ranges[0]

    def _on_odom(self, msg: Odometry):
        t = self._now()
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self._pos = (x, y)
        q = msg.pose.pose.orientation
        self._yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.stuck_detector.update(t, x, y)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _elapsed_in_state(self) -> float:
        return self._now() - self._state_entered

    def _enter_state(self, new_state: State):
        self.state = new_state
        self._state_entered = self._now()
        self.state_pub.publish(String(data=new_state.name))
        self.get_logger().info(f'-> {new_state.name}')

    # --------------------------------------------------- cliff sensors --
    def _cliff(self, corner: str) -> bool:
        r = self._ir_ranges.get(corner)
        if r is None:
            return False  # no reading yet -- don't assume the worst
        return is_cliff(r, self.cliff_range_threshold)

    def _rear_blocked(self) -> bool:
        """True if either rear corner sensor reports no floor -- checked
        while reversing, since the front-mounted LIDAR can't see behind
        the robot at all."""
        return self._cliff('rear_left') or self._cliff('rear_right')

    def _apply_cliff_sensors(self, decision: SteeringDecision):
        """Fuse the corner cliff sensors into `decision`. They're
        authoritative and close-range (unlike the forward LIDAR, no
        geometry inference needed: floor is either under a given corner
        right now or it isn't), so they take priority over the scan's
        own strongest_side call when both fire. Returns (decision,
        cornered) -- `cornered` is True when *both* front corners have
        lost the floor at once, the signature of being wedged into an
        actual corner rather than facing a single glancing edge."""
        cliff_fl = self._cliff('front_left')
        cliff_fr = self._cliff('front_right')
        if not (cliff_fl or cliff_fr):
            return decision, False

        if cliff_fl and cliff_fr:
            side = 'center'
            cornered = True
        elif cliff_fl:
            side = 'left'
            cornered = False
        else:
            side = 'right'
            cornered = False

        return SteeringDecision(hard_stop=True, danger=True,
                                 angular_bias=decision.angular_bias,
                                 strongest_side=side), cornered

    # ------------------------------------------------------- geofence --
    def _outside_geofence(self) -> bool:
        """Hard boundary check independent of LIDAR geometry -- this
        project is scoped to the single known platform, so a square
        keep-in region around the odom origin is a valid backstop even
        if a sensor gap ever lets a real edge go unflagged."""
        if self.geofence_half_extent <= 0:
            return False
        x, y = self._pos
        return max(abs(x), abs(y)) > self.geofence_half_extent

    # --------------------------------------------------- hazard memory --
    def _hazard_lateral_bias(self) -> float:
        """Steering contribution (same sign convention as
        SteeringDecision.angular_bias: positive -> steer left) that
        pushes the robot away from remembered hazard locations near its
        current position, rotated from the world frame into the robot's
        body frame using the latest odometry yaw."""
        rx, ry = self.hazard_memory.repulsion(
            self._pos[0], self._pos[1], self.hazard_memory_radius)
        if rx == 0.0 and ry == 0.0:
            return 0.0
        local_y = -rx * math.sin(self._yaw) + ry * math.cos(self._yaw)
        return local_y

    def _record_hazard(self):
        self.hazard_memory.record(*self._pos)
        self._publish_hazard_map()

    # --------------------------------------------------------- main loop --
    def _control_loop(self):
        if self._last_scan is None:
            return

        decision = analyze_scan(
            self._last_scan.ranges, self.edge_threshold,
            num_bins=self.num_scan_bins, center_bins=self.center_bins)

        # Fold the geofence into the decision itself so every state
        # (EXPLORE/AVOID/RECOVER) reacts to it exactly like a real edge,
        # with no separate code path to keep in sync.
        breached = self._outside_geofence()
        if breached and not self._geofence_breached:
            self.get_logger().warn('Past geofence boundary -> treating as edge')
        self._geofence_breached = breached
        if breached and not decision.hard_stop:
            decision = SteeringDecision(
                hard_stop=True, danger=True,
                angular_bias=decision.angular_bias,
                strongest_side=decision.strongest_side if decision.danger else 'center')

        # Corner cliff sensors are close-range and authoritative (no
        # geometry inference needed), so they take priority over the
        # forward scan's own side call when they disagree.
        decision, self._cornered = self._apply_cliff_sensors(decision)

        self._publish_marker(decision)
        self._publish_cliff_markers()

        if self.state == State.EXPLORE:
            self._run_explore(decision)
        elif self.state == State.AVOID:
            self._run_avoid()
        elif self.state == State.RECOVER:
            self._run_recover(decision)

    def _react_to_hard_stop(self, decision):
        """Shared by EXPLORE and RECOVER's forward leg: a plain edge
        gets the normal AVOID maneuver, but both front corners losing
        the floor at once is the signature of being wedged into an
        actual corner, where a small AVOID turn tends to just point the
        chassis at the *other* edge. Go straight to RECOVER's larger,
        randomized reorientation instead of waiting for the AVOID trap
        counter to notice the same thing several bounces later."""
        if self._cornered:
            self.get_logger().warn('Cornered (both front cliff sensors tripped) -> RECOVER')
            self._begin_recover()
        else:
            self._begin_avoid(decision)

    # ----------------------------------------------------------- EXPLORE --
    def _run_explore(self, decision):
        if self.stuck_detector.is_stuck():
            self.get_logger().warn('No net progress despite driving -> RECOVER')
            self._begin_recover()
            return

        if decision.hard_stop:
            self._react_to_hard_stop(decision)
            return

        # Blend the live scan's steering bias with a pull away from
        # remembered hazard locations -- this is what keeps the robot
        # from drifting back toward an edge it already backed away from,
        # even before the current scan re-flags it.
        hazard_bias = self._hazard_lateral_bias()
        steering_bias = decision.angular_bias + self.hazard_repulsion_gain * hazard_bias

        cmd = Twist()
        # Slow down while steering hard away from danger -- less distance
        # covered per turn-radian means less arc for a corner to swing
        # over an edge before the heading actually changes.
        danger = decision.danger or abs(hazard_bias) > 0.05
        cmd.linear.x = self.forward_speed * (self.danger_slowdown_factor if danger else 1.0)
        # Steer away from peripheral edges *before* they become a hard
        # stop -- this is what lets the robot actually cover the floor
        # instead of walking straight up to every edge before reacting.
        cmd.angular.z = clamp(self.steer_gain * steering_bias,
                               -self.turn_speed, self.turn_speed)
        self.cmd_pub.publish(cmd)

    # ------------------------------------------------------------- AVOID --
    def _begin_avoid(self, decision):
        self._record_hazard()

        now = self._now()
        self._avoid_events.append(now)
        while self._avoid_events and (now - self._avoid_events[0]) > self.avoid_window_sec:
            self._avoid_events.popleft()

        if len(self._avoid_events) >= self.avoid_trap_count:
            self.get_logger().warn(
                f'{len(self._avoid_events)} avoids in {self.avoid_window_sec:.0f}s -> RECOVER')
            self._begin_recover()
            return

        # Turn away from whichever side triggered it. If it's dead
        # center, prefer turning away from nearby remembered hazards; if
        # memory has no opinion either, alternate away from the last
        # turn direction instead of always defaulting the same way --
        # always defaulting the same way is exactly what produces a
        # deterministic ping-pong.
        hazard_bias = self._hazard_lateral_bias()
        if decision.strongest_side == 'left':
            self._avoid_turn_dir = -1.0
        elif decision.strongest_side == 'right':
            self._avoid_turn_dir = 1.0
        elif abs(hazard_bias) > 0.05:
            self._avoid_turn_dir = 1.0 if hazard_bias > 0 else -1.0
        else:
            self._avoid_turn_dir = (-self._last_turn_dir if self._last_turn_dir != 0
                                     else random.choice([-1.0, 1.0]))
        self._last_turn_dir = self._avoid_turn_dir

        # Escalate maneuver duration with how "trapped" this looks.
        escalation = self.avoid_escalation * (len(self._avoid_events) - 1)
        self._avoid_reverse_time = self.reverse_time_base + escalation
        self._avoid_turn_time = self.turn_time_base + escalation
        self._avoid_sub_state = 'REVERSE'
        self._enter_state(State.AVOID)

    def _run_avoid(self):
        cmd = Twist()
        elapsed = self._elapsed_in_state()
        if self._avoid_sub_state == 'REVERSE':
            # The LIDAR is front-mounted and tilted forward, so it can't
            # see anything behind the robot while reversing -- the rear
            # cliff sensors are what actually cover that blind spot, with
            # the geofence as a last-resort backup for the case a sensor
            # reading is missing entirely. If either trips, stop
            # reversing immediately rather than run out the fixed
            # duration further into trouble.
            if self._rear_blocked() or self._outside_geofence():
                self._avoid_sub_state = 'TURN'
                self._state_entered = self._now()
            else:
                cmd.linear.x = -self.reverse_speed
                if elapsed > self._avoid_reverse_time:
                    self._avoid_sub_state = 'TURN'
                    self._state_entered = self._now()
        else:  # TURN
            cmd.angular.z = self.turn_speed * self._avoid_turn_dir
            if elapsed > self._avoid_turn_time:
                self._enter_state(State.EXPLORE)
                return
        self.cmd_pub.publish(cmd)

    # ----------------------------------------------------------- RECOVER --
    def _begin_recover(self):
        self._record_hazard()

        hazard_bias = self._hazard_lateral_bias()
        if abs(hazard_bias) > 0.05:
            self._recover_turn_dir = 1.0 if hazard_bias > 0 else -1.0
        else:
            self._recover_turn_dir = (-self._last_turn_dir if self._last_turn_dir != 0
                                       else random.choice([-1.0, 1.0]))
        self._last_turn_dir = self._recover_turn_dir
        self._recover_turn_time = random.uniform(
            self.recover_turn_time_min, self.recover_turn_time_max)
        self._recover_sub_state = 'TURN'
        self._avoid_events.clear()
        self.stuck_detector.reset()
        self._enter_state(State.RECOVER)

    def _run_recover(self, decision):
        cmd = Twist()
        elapsed = self._elapsed_in_state()
        if self._recover_sub_state == 'TURN':
            cmd.angular.z = self.turn_speed * self._recover_turn_dir
            if elapsed > self._recover_turn_time:
                self._recover_sub_state = 'FORWARD'
                self._state_entered = self._now()
        else:  # FORWARD
            if decision.hard_stop:
                # Recovery still respects a real edge dead ahead.
                self._react_to_hard_stop(decision)
                return
            hazard_bias = self._hazard_lateral_bias()
            steering_bias = decision.angular_bias + self.hazard_repulsion_gain * hazard_bias
            danger = decision.danger or abs(hazard_bias) > 0.05
            cmd.linear.x = self.forward_speed * (self.danger_slowdown_factor if danger else 1.0)
            cmd.angular.z = clamp(self.steer_gain * steering_bias,
                                   -self.turn_speed, self.turn_speed)
            if elapsed > self.recover_forward_time:
                self._enter_state(State.EXPLORE)
                return
        self.cmd_pub.publish(cmd)

    # ------------------------------------------------------------- debug --
    def _publish_marker(self, decision):
        """Arrow in RViz: green = clear, orange = edge in periphery,
        red = hard stop. Rotated toward the side that triggered it."""
        m = Marker()
        m.header.frame_id = 'base_link'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'edge_avoider'
        m.id = 0
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.scale.x, m.scale.y, m.scale.z = 0.4, 0.04, 0.04
        m.color.a = 1.0
        if decision.hard_stop:
            m.color.r, m.color.g, m.color.b = 1.0, 0.0, 0.0
        elif decision.danger:
            m.color.r, m.color.g, m.color.b = 1.0, 0.6, 0.0
        else:
            m.color.r, m.color.g, m.color.b = 0.0, 0.8, 0.2
        yaw = decision.angular_bias * 1.2
        m.pose.position.x = 0.4
        m.pose.position.z = 0.2
        m.pose.orientation.z = math.sin(yaw / 2.0)
        m.pose.orientation.w = math.cos(yaw / 2.0)
        self.marker_pub.publish(m)

    def _publish_hazard_map(self):
        """Red spheres in RViz at every remembered hazard location --
        visible, growing proof that the robot is actually learning where
        the edges are instead of reacting fresh each time."""
        arr = MarkerArray()
        for i, (hx, hy) in enumerate(self.hazard_memory):
            m = Marker()
            m.header.frame_id = 'odom'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'hazard_map'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.scale.x = m.scale.y = m.scale.z = 0.12
            m.color.a = 0.8
            m.color.r, m.color.g, m.color.b = 1.0, 0.15, 0.0
            m.pose.position.x = hx
            m.pose.position.y = hy
            m.pose.position.z = 0.05
            m.pose.orientation.w = 1.0
            arr.markers.append(m)
        self.hazard_marker_pub.publish(arr)

    def _publish_cliff_markers(self):
        """One small sphere per chassis corner, in base_link, green when
        that corner's IR sensor sees floor and red when it doesn't --
        lets you watch the cliff sensors work in real time in RViz
        instead of only inferring it from the robot's motion."""
        arr = MarkerArray()
        corners = (
            ('front_left', self.cliff_sensor_x_offset, self.cliff_sensor_y_offset),
            ('front_right', self.cliff_sensor_x_offset, -self.cliff_sensor_y_offset),
            ('rear_left', -self.cliff_sensor_x_offset, self.cliff_sensor_y_offset),
            ('rear_right', -self.cliff_sensor_x_offset, -self.cliff_sensor_y_offset),
        )
        for i, (corner, x, y) in enumerate(corners):
            m = Marker()
            m.header.frame_id = 'base_link'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'cliff_sensors'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.scale.x = m.scale.y = m.scale.z = 0.08
            m.color.a = 1.0
            if self._cliff(corner):
                m.color.r, m.color.g, m.color.b = 1.0, 0.0, 0.0
            else:
                m.color.r, m.color.g, m.color.b = 0.0, 0.8, 0.2
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = 0.02
            m.pose.orientation.w = 1.0
            arr.markers.append(m)
        self.cliff_marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = EdgeAvoider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
