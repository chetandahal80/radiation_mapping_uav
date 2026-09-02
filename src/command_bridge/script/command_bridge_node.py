#!/usr/bin/env python3
# xy_vel_with_fixed_z.py
import rospy, math, tf
from geometry_msgs.msg import Twist
from mavros_msgs.msg import PositionTarget

class XYVelWithFixedZ:
    def __init__(self):
        # --- Params ---
        self.rate_hz   = rospy.get_param("~rate_hz", 20.0)
        self.z_hold    = rospy.get_param("~z_hold", 1.8)
        self.max_xy    = rospy.get_param("~max_xy", 2.0)
        self.max_r     = rospy.get_param("~max_yaw_rate", 1.5)
        self.stale_t   = rospy.get_param("~stale_timeout", 10.0)
        self.cmd_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.pub_topic = rospy.get_param("~target_topic", "/mavros/setpoint_raw/local")
        self.world_frame = rospy.get_param("~world_frame", "map")        # or "odom"
        self.base_frame  = rospy.get_param("~base_frame",  "base_footprint")  # or "base_footprint"

        # --- State ---
        self._last_cmd = Twist()
        self._t_last   = rospy.Time(0)

        # --- I/O ---
        self.listener = tf.TransformListener()
        rospy.Subscriber(self.cmd_topic, Twist, self._cmd_cb, queue_size=10)
        self.pub = rospy.Publisher(self.pub_topic, PositionTarget, queue_size=50)

        # --- Timer publisher ---
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self._tick)

        rospy.loginfo("XYVelWithFixedZ: -> %s at %.1f Hz, z_hold=%.2f m (world=%s, base=%s)",
                      self.pub_topic, self.rate_hz, self.z_hold, self.world_frame, self.base_frame)

    @staticmethod
    def _clamp(v, lo, hi):
        return max(lo, min(hi, v))

    def _cmd_cb(self, msg: Twist):
        self._last_cmd = msg
        self._t_last   = rospy.Time.now()

    def _get_yaw_world_of_base(self) -> float:
        try:
            self.listener.waitForTransform(self.world_frame, self.base_frame, rospy.Time(0), rospy.Duration(0.05))
            (_, q) = self.listener.lookupTransform(self.world_frame, self.base_frame, rospy.Time(0))
            return tf.transformations.euler_from_quaternion(q)[2]
        except Exception:
            return 0.0  # fall back (better than nothing)

    def _tick(self, _evt):
        now = rospy.Time.now()
        stale = (now - self._t_last).to_sec() > self.stale_t

        # last command (in base_link frame)
        vx_b = self._last_cmd.linear.x if not stale else 0.0
        vy_b = self._last_cmd.linear.y if not stale else 0.0
        wz   = self._last_cmd.angular.z if not stale else 0.0

        # clamp XY speed & yaw-rate
        spd = math.hypot(vx_b, vy_b)
        if spd > self.max_xy and spd > 1e-6:
            scale = self.max_xy / spd
            vx_b *= scale; vy_b *= scale
        wz = self._clamp(wz, -self.max_r, self.max_r)

        # rotate body -> world (LOCAL ENU) so it matches FRAME_LOCAL_NED convention in MAVROS
        yaw = self._get_yaw_world_of_base()
        c, s = math.cos(yaw), math.sin(yaw)
        vx_w = c*vx_b - s*vy_b
        vy_w = s*vx_b + c*vy_b

        # Compose PositionTarget: use Z position, XY velocity, yaw_rate
        sp = PositionTarget()
        sp.header.stamp = now
        sp.header.frame_id = self.world_frame  # informational; MAVROS uses coordinate_frame enum below
        sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED  # MAVROS converts ENU<->NED under the hood

        sp.position.z = float(self.z_hold)   # hold altitude (ENU up)
        sp.velocity.x = float(vx_w)          # world ENU X
        sp.velocity.y = float(vy_w)          # world ENU Y
        sp.velocity.z = 0.0
        sp.yaw_rate   = float(wz)            # rad/s

        # Type mask: use pz, vx, vy, yaw_rate; ignore px, py, vz, accel, yaw
        IGNORE_PX=1; IGNORE_PY=2; IGNORE_VZ=32
        IGNORE_AFX=64; IGNORE_AFY=128; IGNORE_AFZ=256; IGNORE_YAW=1024
        sp.type_mask = (IGNORE_PX | IGNORE_PY | IGNORE_VZ |
                        IGNORE_AFX | IGNORE_AFY | IGNORE_AFZ |
                        IGNORE_YAW)

        self.pub.publish(sp)

if __name__ == "__main__":
    rospy.init_node("xy_vel_with_fixed_z")
    XYVelWithFixedZ()
    rospy.spin()
