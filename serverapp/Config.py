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
        self.pid_interval = 20
        self.landing_speed = 0.2
        self.hover_speed = 0.35
        self.max_speed = 0.8
        self.min_speed = 0.1
        self.max_ratio = 2
        self.speed_interval = 20

        self.orientation_enabled = 1
        self.pid_enabled = 1
        self.speed_enabled = 1
        self.altitude_enabled = 1
        self.log_interval = 50

        self.acceleration_weight = 0.01
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
            self.pid_interval = int(config['DEFAULT']['PID-Interval'])
            
            self.landing_speed = float(config['DEFAULT']['Landing-Speed'])
            self.hover_speed = float(config['DEFAULT']['Hover-Speed'])
            self.max_speed = float(config['DEFAULT']['Max-Speed'])
            self.min_speed = float(config['DEFAULT']['Min-Speed'])
            self.max_ratio = float(config['DEFAULT']['Max-Ratio'])
            self.speed_interval = int(config['DEFAULT']['Speed-Interval'])

            self.orientation_enabled = int(config['DEFAULT']['Orientation-Enabled'])
            self.pid_enabled = int(config['DEFAULT']['PID-Enabled'])
            self.speed_enabled = int(config['DEFAULT']['Speed-Enabled'])
            self.altitude_enabled = int(config['DEFAULT']['Altitude-Enabled'])
            self.log_interval = int(config['DEFAULT']['Log-Interval'])

            self.acceleration_weight = float(config['DEFAULT']['Acceleration-Weight'])
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
        config['DEFAULT']['PID-Interval'] = str(self.pid_interval)
        
        config['DEFAULT']['Landing-Speed'] = str(self.landing_speed)
        config['DEFAULT']['Hover-Speed'] = str(self.hover_speed)
        config['DEFAULT']['Max-Speed'] = str(self.max_speed)
        config['DEFAULT']['Min-Speed'] = str(self.min_speed)
        config['DEFAULT']['Max-Ratio'] = str(self.max_ratio)
        config['DEFAULT']['Speed-Interval'] = str(self.speed_interval)

        config['DEFAULT']['Orientation-Enabled'] = str(self.orientation_enabled)
        config['DEFAULT']['PID-Enabled'] = str(self.pid_enabled)
        config['DEFAULT']['Speed-Enabled'] = str(self.speed_enabled)
        config['DEFAULT']['Altitude-Enabled'] = str(self.altitude_enabled)
        config['DEFAULT']['Log-Interval'] = str(self.log_interval)

        config['DEFAULT']['Acceleration-Weight'] = str(self.acceleration_weight)
        config['DEFAULT']['Sensor-Enabled'] = str(self.sensor_enabled)

        with open(self.filepath, 'w') as configfile:
            config.write(configfile)
