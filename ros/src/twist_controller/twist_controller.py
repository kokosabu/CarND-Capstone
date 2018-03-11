from yaw_controller import YawController
from lowpass import LowPassFilter
from pid import PID
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

class Controller(object):
    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel,
            max_steer_angle, decel_limit, accel_limit, vehicle_mass,
            fuel_capacity, wheel_radius):
        self.yaw_controller = YawController(wheel_base, steer_ratio,
                min_speed, max_lat_accel, max_steer_angle)
        #self.steer_filter = LowPassFilter(0.0, 1.0)
        self.steer_filter = LowPassFilter(3.0, 1.0)
        #self.steer_pid = PID(0.8, 0.005, 1.3, -max_steer_angle, max_steer_angle)
        self.steer_pid = PID(1.3, 0.01, 1.9, -max_steer_angle, max_steer_angle)
        self.velocity_pid = PID(0.4, 0.1, 0.0, decel_limit, accel_limit)
        self.velocity_filter = LowPassFilter(0.5, 0.02)
        self.last_time = rospy.get_time()
        self.vehicle_mass = vehicle_mass + fuel_capacity * GAS_DENSITY
        self.wheel_radius = wheel_radius

    def control(self, proposed_linear, proposed_angular, current_linear,
            dbw_status, rate):
        if not dbw_status:
            self.steer_pid.reset()
            return 0.0, 0.0, 0.0

        now = rospy.get_time()
        delta = now - self.last_time
        self.last_time = now
        #rospy.logwarn("now %f, delta %f, last_time %f", 
        #        now, delta, self.last_time)


        #if proposed_linear.x <= 0:
        steer_raw = self.yaw_controller.get_steering(proposed_linear.x,
                proposed_angular.z, current_linear.x)
        steer = self.steer_pid.step(steer_raw, delta)
        steer = self.steer_filter.filt(steer)

        #brake = (proposed_linear.x - current_linear.x) + 0.05 < 0
        brake = proposed_linear.x < 0

        #if abs(proposed_linear.x) < 1.0 * ONE_MPH:
        #    self.velocity_pid.reset()

        #throttle = 0.2 * (proposed_linear.x - current_linear.x)
        throttle_raw = self.velocity_pid.step(proposed_linear.x - current_linear.x, delta)
        throttle = self.velocity_filter.filt(throttle_raw) * 0.7
        #throttle = min(throttle, 0.65)
        rospy.logwarn("linear %f, current %f, throttle %f( -> %f )",
                proposed_linear.x, current_linear.x, throttle_raw, throttle)

        if throttle < 0.1 or brake:
        #if brake:
            brake = -throttle * self.vehicle_mass * self.wheel_radius
            throttle = 0
            self.velocity_pid.reset()
        else:
            brake = 0.0

        return throttle, brake, steer
