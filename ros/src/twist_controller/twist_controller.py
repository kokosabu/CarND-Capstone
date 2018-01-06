from yaw_controller import YawController
from lowpass import LowPassFilter
from pid import PID
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

class Controller(object):
    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel,
            max_steer_angle, decel_limit, accel_limit):
        self.yaw_controller = YawController(wheel_base, steer_ratio,
                min_speed, max_lat_accel, max_steer_angle)
        self.steer_filter = LowPassFilter(0.0, 1.0)
        self.steer_pid = PID(0.75, 0.005, 1.0, -max_steer_angle, max_steer_angle)
        self.velocity_pid = PID(1.4, 0, 0, decel_limit, accel_limit)
        self.velocity_filter = LowPassFilter(1.0, 3.0)
        self.last_time = rospy.get_time()

    def control(self, proposed_linear, proposed_angular, current_linear,
            dbw_status, rate):
        if not dbw_status:
            self.steer_pid.reset()
            return 0.0, 0.0, 0.0

        now = rospy.get_time()
        delta = now - self.last_time
        self.last_time = now

        steer_raw = self.yaw_controller.get_steering(proposed_linear.x,
                proposed_angular.z, current_linear.x)
        steer = self.steer_pid.step(steer_raw, delta)
        #steer = self.steer_pid.step(steer_raw, 0.02)
        #steer = self.steer_filter.filt(steer_raw)

        #throttle = 0.2 * (proposed_linear.x - current_linear.x)
        throttle_raw = self.velocity_pid.step(proposed_linear.x - current_linear.x, delta)
        throttle = self.velocity_filter.filt(throttle_raw)
        if throttle <= 0:
            throttle = 0
            brake = 0.2
        else:
            brake = 0.0

        return throttle, brake, steer
