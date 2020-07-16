import configparser
import os

class Config():
    def __init__(self, filepath):
        self.filepath = filepath
        self.load_defaults()
        self.load_config()

    def load_defaults(self):
        self.pid_factor = 0.5
        self.p_factor = 0.1
        self.i_factor = 0.01
        self.d_factor = 0.01
        self.pid_interval = 20

        self.landing_speed = 1200
        self.hover_speed = 1350
        self.max_speed = 1800
        self.min_speed = 1100
        self.speed_interval = 20

        self.orientation_enabled = 1
        self.pid_enabled = 1
        self.speed_enabled = 1
        self.log_interval = 50

    def load_config(self):
        if os.path.exists(self.filepath):
            config = configparser.ConfigParser()
            config.read(self.filepath)

            self.pid_factor = config['DEFAULT']['PID-Factor']
            self.p_factor = config['DEFAULT']['P-Factor']
            self.i_factor = config['DEFAULT']['I-Factor']
            self.d_factor = config['DEFAULT']['D-Factor']
            self.pid_interval = config['DEFAULT']['PID-Interval']
            
            self.landing_speed = config['DEFAULT']['Landing-Speed']
            self.hover_speed = config['DEFAULT']['Hover-Speed']
            self.max_speed = config['DEFAULT']['Max-Speed']
            self.min_speed = config['DEFAULT']['Min-Speed']
            self.speed_interval = config['DEFAULT']['Speed-Interval']

            self.orientation_enabled = config['DEFAULT']['Orientation-Enabled']
            self.pid_enabled = config['DEFAULT']['PID-Enabled']
            self.speed_enabled = config['DEFAULT']['Speed-Enabled']
            self.log_interval = config['DEFAULT']['Log-Interval']

            print("Read Config")


    def save_config(self):
        config = configparser.ConfigParser()
        
        config['DEFAULT'] = {}
        config['DEFAULT']['PID-Factor'] = str(self.pid_factor)
        config['DEFAULT']['P-Factor'] = str(self.p_factor)
        config['DEFAULT']['I-Factor'] = str(self.i_factor)
        config['DEFAULT']['D-Factor'] = str(self.d_factor)
        config['DEFAULT']['PID-Interval'] = str(self.pid_interval)
        
        config['DEFAULT']['Landing-Speed'] = str(self.landing_speed)
        config['DEFAULT']['Hover-Speed'] = str(self.hover_speed)
        config['DEFAULT']['Max-Speed'] = str(self.max_speed)
        config['DEFAULT']['Min-Speed'] = str(self.min_speed)
        config['DEFAULT']['Speed-Interval'] = str(self.speed_interval)

        config['DEFAULT']['Orientation-Enabled'] = str(self.orientation_enabled)
        config['DEFAULT']['PID-Enabled'] = str(self.pid_enabled)
        config['DEFAULT']['Speed-Enabled'] = str(self.speed_enabled)
        config['DEFAULT']['Log-Interval'] = str(self.log_interval)

        with open(self.filepath, 'w') as configfile:
            config.write(configfile)
