import rospy
GAS_DENSITY = 2.858
ONE_MPH = 0.44704
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit,
                accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        # TODO: Implement
        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)
        kp = 0.3
        ki = 0.1
        kd = 0.0
        mn = 0 # minimum throttle value
        mx = 0.2 #maximum throttle value
        # using PID controller classW
        self.throttle_controller = PID(kp, ki, kd, mn, mx)

        tau = 0.5 # 1/(2pi*tau) = cutoff freqency for low pass filter
        ts = .02 # sample time for 50 hz
        self.vel_lpf = LowPassFilter(tau, ts)

        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius

        self.last_time = rospy.get_time()

    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        
        # if drive by wire is not enabled, it means human driver drives the car. In this step, controller
        #shouldn't keep running. Since the pid controller has integration part, it accumulate errors.
        # If we don't reset, then, when human switch to autonomous system, it run with accumulated errors.
        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0., 0., 0.
        
        #using low pass filter for filtering the velocity
        current_vel = self.vel_lpf.filt(current_vel)

        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)

        vel_error = linear_vel - current_vel
        self.last_vel = current_vel

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        throttle = self.throttle_controller.step(vel_error, sample_time)
        brake = 0

        # below is a logic about making vehicle stop when speed is lower than 0.1. It depends on vehicle's 
        # mass to calculate the brake torque.
        if linear_vel == 0. and current_vel < 0.1 :
            throttle = 0
            brake = 400 #N*m  . this is to hold the car in place if we are stopped at a light. Acceleration - 1m/s^2
        elif throttle < .1 and vel_error < 0 :
            throttle = 0
            decel =  max(vel_error, self.decel_limit)
            brake = abs(decel) * self.vehicle_mass * self.wheel_radius # Torque N*m

        return throttle, brake, steering
