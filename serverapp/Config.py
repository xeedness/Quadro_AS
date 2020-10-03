import configparser
import os

class Config():
    def __init__(self, filepath):
        self.filepath = filepath
        self.load_defaults()
        self.load_config()

    def load_defaults(self):
        self.pid_factor = 2
        self.p_angle_factor = 0.5
        self.i_angle_factor = 0.1
        self.d_angle_factor = 0.1
        self.p_rate_factor = 0.5
        self.i_rate_factor = 0.1
        self.d_rate_factor = 0.1
        self.p_vertical_velocity_factor = 1
        self.i_vertical_velocity_factor = 0.2
        self.d_vertical_velocity_factor = 0
        self.pid_interval = 20

        self.min_speed = 0.1
        self.max_speed = 0.8
        self.vertical_velocity_amplifier = 1
        self.velocity_limit = 0.1
        self.speed_interval = 20

        self.orientation_enabled = 1
        self.pid_enabled = 1
        self.speed_enabled = 1
        self.altitude_enabled = 1
        self.log_interval = 50

        self.acceleration_weight = 0.01
        self.measurement_error_angle = 0.1
        self.measurement_error_angular_velocity = 0.001
        self.estimate_error_angle = 0.001
        self.estimate_error_angular_velocity = 0.1
        self.altitude_gain = -1.1547
        self.speed_gain = -0.6667
        self.use_kalman_orientation = 1
        self.sensor_enabled = 1

    def load_config(self):
        if os.path.exists(self.filepath):
            config = configparser.ConfigParser()
            config.read(self.filepath)

            self.pid_factor = float(config['DEFAULT']['PID-Factor'])
            self.p_angle_factor = float(config['DEFAULT']['P-Angle-Factor'])
            self.i_angle_factor = float(config['DEFAULT']['I-Angle-Factor'])
            self.d_angle_factor = float(config['DEFAULT']['D-Angle-Factor'])
            self.p_rate_factor = float(config['DEFAULT']['P-Rate-Factor'])
            self.i_rate_factor = float(config['DEFAULT']['I-Rate-Factor'])
            self.d_rate_factor = float(config['DEFAULT']['D-Rate-Factor'])
            self.p_vertical_velocity_factor = float(config['DEFAULT']['P-Velocity-Factor'])
            self.i_vertical_velocity_factor = float(config['DEFAULT']['I-Velocity-Factor'])
            self.d_vertical_velocity_factor = float(config['DEFAULT']['D-Velocity-Factor'])
            self.pid_interval = int(config['DEFAULT']['PID-Interval'])
            
            self.min_speed = float(config['DEFAULT']['Min-Speed'])
            self.max_speed = float(config['DEFAULT']['Max-Speed'])
            self.vertical_velocity_amplifier = float(config['DEFAULT']['Velocity-Amp'])
            self.velocity_limit = float(config['DEFAULT']['Velocity-Limit'])
            self.speed_interval = int(config['DEFAULT']['Speed-Interval'])

            self.orientation_enabled = int(config['DEFAULT']['Orientation-Enabled'])
            self.pid_enabled = int(config['DEFAULT']['PID-Enabled'])
            self.speed_enabled = int(config['DEFAULT']['Speed-Enabled'])
            self.altitude_enabled = int(config['DEFAULT']['Altitude-Enabled'])
            self.log_interval = int(config['DEFAULT']['Log-Interval'])

            self.acceleration_weight = float(config['DEFAULT']['Acceleration-Weight'])
            self.measurement_error_angle = float(config['DEFAULT']['Measurement-Error-Angle'])
            self.measurement_error_angular_velocity = float(config['DEFAULT']['Measurement-Error-Angular-Velocity'])
            self.estimate_error_angle = float(config['DEFAULT']['Estimate-Error-Angle'])
            self.estimate_error_angular_velocity = float(config['DEFAULT']['Estimate-Error-Angular-Velocity'])
            self.altitude_gain = float(config['DEFAULT']['Altitude-Gain'])
            self.speed_gain = float(config['DEFAULT']['Speed-Gain'])
            self.acceleration_weight = float(config['DEFAULT']['Acceleration-Weight'])
            self.use_kalman_orientation = int(config['DEFAULT']['Use-Kalman-Orientation'])
            self.sensor_enabled = int(config['DEFAULT']['Sensor-Enabled'])

            print("Read Config")


    def save_config(self):
        config = configparser.ConfigParser()
        
        config['DEFAULT'] = {}
        config['DEFAULT']['PID-Factor'] = str(self.pid_factor)
        config['DEFAULT']['P-Angle-Factor'] = str(self.p_angle_factor)
        config['DEFAULT']['I-Angle-Factor'] = str(self.i_angle_factor)
        config['DEFAULT']['D-Angle-Factor'] = str(self.d_angle_factor)
        config['DEFAULT']['P-Rate-Factor'] = str(self.p_rate_factor)
        config['DEFAULT']['I-Rate-Factor'] = str(self.i_rate_factor)
        config['DEFAULT']['D-Rate-Factor'] = str(self.d_rate_factor)
        config['DEFAULT']['P-Velocity-Factor'] = str(self.p_vertical_velocity_factor)
        config['DEFAULT']['I-Velocity-Factor'] = str(self.i_vertical_velocity_factor)
        config['DEFAULT']['D-Velocity-Factor'] = str(self.d_vertical_velocity_factor)
        config['DEFAULT']['PID-Interval'] = str(self.pid_interval)
        
        config['DEFAULT']['Min-Speed'] = str(self.min_speed)
        config['DEFAULT']['Max-Speed'] = str(self.max_speed)        
        config['DEFAULT']['Velocity-Amp'] = str(self.vertical_velocity_amplifier)
        config['DEFAULT']['Velocity-Limit'] = str(self.velocity_limit)
        config['DEFAULT']['Speed-Interval'] = str(self.speed_interval)

        config['DEFAULT']['Orientation-Enabled'] = str(self.orientation_enabled)
        config['DEFAULT']['PID-Enabled'] = str(self.pid_enabled)
        config['DEFAULT']['Speed-Enabled'] = str(self.speed_enabled)
        config['DEFAULT']['Altitude-Enabled'] = str(self.altitude_enabled)
        config['DEFAULT']['Log-Interval'] = str(self.log_interval)

        config['DEFAULT']['Acceleration-Weight'] = str(self.acceleration_weight)
        config['DEFAULT']['Measurement-Error-Angle'] = str(self.measurement_error_angle)
        config['DEFAULT']['Measurement-Error-Angular-Velocity'] = str(self.measurement_error_angular_velocity)
        config['DEFAULT']['Estimate-Error-Angle'] = str(self.estimate_error_angle)
        config['DEFAULT']['Estimate-Error-Angular-Velocity'] = str(self.estimate_error_angular_velocity)
        config['DEFAULT']['Altitude-Gain'] = str(self.altitude_gain)
        config['DEFAULT']['Speed-Gain'] = str(self.speed_gain)
        config['DEFAULT']['Use-Kalman-Orientation'] = str(self.use_kalman_orientation)
        config['DEFAULT']['Sensor-Enabled'] = str(self.sensor_enabled)

        with open(self.filepath, 'w') as configfile:
            config.write(configfile)
