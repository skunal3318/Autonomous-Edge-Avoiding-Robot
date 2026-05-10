#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from enum import Enum
import math
import random


class State(Enum):
    WANDER    = 0  
    REVERSING = 1   
    TURNING   = 2  


class EdgeAvoidanceBot(Node):
    def __init__(self):
        super().__init__('edge_avoid')

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10)
        self.timer = self.create_timer(0.1, self.control_loop)  

        self.angle_min = None
        self.angle_inc = None
        self.n         = None
        self.scan_ready = False

        self.front_edge = False
        self.left_edge  = False
        self.right_edge = False

        self.EDGE_THRESHOLD = 1.5  

        self.state  = State.WANDER
        self.ticks  = 0            
        self.target_ticks = 0   

        self.wander_linear  = 0.20 
        self.wander_angular = 0.0  
        self.wander_change_ticks = 0
        self.WANDER_CHANGE_INTERVAL = 30  

        self.get_logger().info("Wandering.......!")
        self._new_wander_direction()


    def _new_wander_direction(self):
        """Pick a new gentle random curve direction."""
        self.wander_angular = random.uniform(-0.4, 0.4)
        self.wander_change_ticks = 0
        self.get_logger().info(
            f"Wander: angular={self.wander_angular:.2f} rad/s")

    def angle_to_index(self, angle_rad):
        idx = int((angle_rad - self.angle_min) / self.angle_inc)
        return max(0, min(self.n - 1, idx))

    def sector_max(self, ranges, center_angle, half_width):
        i1 = self.angle_to_index(center_angle - half_width)
        i2 = self.angle_to_index(center_angle + half_width)
        if i1 > i2:
            i1, i2 = i2, i1
        sector = ranges[i1:i2 + 1]
        valid  = [r for r in sector
                  if not math.isnan(r) and not math.isinf(r) and r > 0.0]
        return max(valid) if valid else 0.0

    def lidar_callback(self, msg):
        if self.angle_min is None:
            self.angle_min = msg.angle_min
            self.angle_inc = msg.angle_increment
            self.n         = len(msg.ranges)
            self.scan_ready = True
            self.get_logger().info(
                f"Scan ready | samples={self.n} | "
                f"min={math.degrees(msg.angle_min):.0f}° "
                f"max={math.degrees(msg.angle_max):.0f}° | "
                f"range_min={msg.range_min:.2f}m "
                f"range_max={msg.range_max:.2f}m"
            )

        ranges = list(msg.ranges)
        hw = math.radians(30) 

        front = self.sector_max(ranges,  0.0,       hw)
        left  = self.sector_max(ranges,  math.pi/2, hw)
        right = self.sector_max(ranges, -math.pi/2, hw)

        self.front_edge = front > self.EDGE_THRESHOLD
        self.left_edge  = left  > self.EDGE_THRESHOLD
        self.right_edge = right > self.EDGE_THRESHOLD

        self.get_logger().info(
            f"F={front:.2f}m({'EDGE' if self.front_edge else 'ok':4}) "
            f"L={left:.2f}m({'EDGE' if self.left_edge  else 'ok':4}) "
            f"R={right:.2f}m({'EDGE' if self.right_edge else 'ok':4}) "
            f"| state={self.state.name}"
        )

    def control_loop(self):
        cmd = Twist()

        if self.state == State.WANDER:
            cmd.linear.x  = self.wander_linear
            cmd.angular.z = self.wander_angular

            self.wander_change_ticks += 1
            if self.wander_change_ticks >= self.WANDER_CHANGE_INTERVAL:
                self._new_wander_direction()

            if self.scan_ready:
                any_edge = self.front_edge or self.left_edge or self.right_edge

                if self.front_edge:
                    self.get_logger().warn("⚠ FRONT edge — reversing!")
                    self._start_reversing()

                elif self.left_edge and not self.right_edge:
                    self.get_logger().warn("⚠ LEFT edge — turning right")
                    self._start_turning(direction=-1)

                elif self.right_edge and not self.left_edge:
                    self.get_logger().warn("⚠ RIGHT edge — turning left")
                    self._start_turning(direction=1)

                elif self.left_edge and self.right_edge:
                    self.get_logger().warn("⚠ BOTH sides — reversing!")
                    self._start_reversing()

    
        elif self.state == State.REVERSING:
            cmd.linear.x  = -0.20
            cmd.angular.z =  0.0
            self.ticks += 1
            if self.ticks >= self.target_ticks:
                self.get_logger().info("Reverse done — now turning")
                self._start_turning(direction=random.choice([-1, 1]))

        elif self.state == State.TURNING:
            cmd.linear.x  =  0.0
            cmd.angular.z =  self.turn_rate
            self.ticks += 1
            if self.ticks >= self.target_ticks:
                self.get_logger().info("Turn done — wandering again")
                self.state = State.WANDER
                self._new_wander_direction()

        self.publisher.publish(cmd)

    def _start_reversing(self):
        self.state        = State.REVERSING
        self.ticks        = 0
        self.target_ticks = random.randint(15, 25) 

    def _start_turning(self, direction):
        """direction: +1 = left, -1 = right"""
        self.state        = State.TURNING
        self.ticks        = 0
        self.target_ticks = random.randint(10, 25)  
        self.turn_rate    = direction * random.uniform(0.8, 1.5)  


def main(args=None):
    rclpy.init(args=args)
    node = EdgeAvoidanceBot()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()