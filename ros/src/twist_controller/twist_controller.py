from yaw_controller import YawController
from lowpass import LowPassFilter
#    def __init__(self, tau, ts):
#    def get(self):
#    def filt(self, val):

from pid import PID
#    def __init__(self, kp, ki, kd, mn=MIN_NUM, mx=MAX_NUM):
#    def reset(self):
#    def step(self, error, sample_time):

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

class Controller(object):
    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle):
        self.yaw_controller = YawController(wheel_base, steer_ratio,
                #min_speed, max_lat_accel, max_steer_angle)
                1.0, max_lat_accel, max_steer_angle)
        self.steer_filter = LowPassFilter(0.0, 1.0)
        self.steer_pid = PID(0.13, 0.00012, 3.2, -max_steer_angle, max_steer_angle)
        # def __init__(self, kp, ki, kd, mn=MIN_NUM, mx=MAX_NUM):

    def control(self, proposed_linear, proposed_angular, current_linear,
            dbw_status, rate):
        if not dbw_status:
            self.steer_pid.reset()
            return 0.0, 0.0, 0.0

        steer_raw = self.yaw_controller.get_steering(proposed_linear.x,
                proposed_angular.z, current_linear.x)
        #steer = self.steer_pid.step(steer_raw, rate)
        #steer = self.steer_pid.step(steer_raw, 0.02)
        steer = self.steer_filter.filt(steer_raw)

        # Return throttle, brake, steer
        return 0.2, 0., steer
