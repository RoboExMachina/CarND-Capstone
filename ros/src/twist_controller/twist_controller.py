from pid import PID
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

class Controller(object):
    def __init__(self, kp_t,ki_t,kd_t, kp_b,ki_b,kd_b, wheel_base, steer_ratio, min_speed 
                             ,max_lat_accel, max_steer_angle,brake_deadband):
        # TODO: Implement
        # Initialize the controllers
        self.throttle_pid = PID(kp_t,ki_t,kd_t,0,1)
        self.brake_pid    = PID(kp_b,ki_b,kd_b,brake_deadband,1000)
        self.steering_yaw = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)

    # Inputs: linear_velocity  in M/H
    #         curr_speed in m/s
    
    def control(self, linear_velocity, angular_velocity, curr_speed,dbw_state):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        
        # Velocity error
        error = linear_velocity - curr_speed
        
        throttle_cmd = self.throttle_pid.step(error,0.05)
        # Brake command
        if(error<0):
             brake_cmd = self.brake_pid.step(-error,0.05)
        else:
             brake_cmd = 0.
             self.brake_pid.reset()
        # Steering command
        steering_cmd = self.steering_yaw.get_steering(linear_velocity, angular_velocity, curr_speed)

        # reset the commands and the pid integration to avoid error accumulation when the DBW is disengaged
        if(dbw_state == False):
             throttle_cmd = 0.
             steering_cmd = 0.
             brake_cmd = 0.
             self.throttle_pid.reset()
             self.brake_pid.reset()
        return throttle_cmd,brake_cmd, steering_cmd

