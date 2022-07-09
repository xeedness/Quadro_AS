import socket
from base64 import b85encode
from tkinter import Tk, Label, Frame, X, RIGHT, LEFT, RAISED, Button, OptionMenu, Entry, StringVar, IntVar, Grid, Scale, HORIZONTAL, DoubleVar
import MessageTypes
import Sender
from Config import Config
import struct
import threading
import time
import ControllerInput

class Controller(Frame):
    def __init__(self, root, fill, side, expand, sender: Sender, config: Config):
        super().__init__(root, relief=RAISED, borderwidth=1, padx = 5, pady = 5)
        self.root = root
        self.sender = sender
        self.config = config
        self.side = side
        self.fill = fill
        self.expand = expand
        self.initUI()
        self.start_alive_thread()
        self.start_xbox_controller()

    def initUI(self):
        self.pack(fill=self.fill, side=self.side, expand=self.expand)
        self.__init_basic__()
        self.__init_config__()
        self.__init_controls__()
        # self.send_btn = Button(self, text="Send", command=self.send)
        # self.send_btn.pack(side=RIGHT, padx=1, pady=1)

        # self.arg_encoding = StringVar(self)
        # self.arg_encoding.set("int") 
        # self.arg_menu = OptionMenu(self, self.arg_encoding, "int", "float", "hex")
        # self.arg_menu.pack(side=RIGHT, padx=1, pady=1)
    
        # self.args = StringVar(self)
        # self.arg_input = Entry(self, textvariable=self.args)
        # self.arg_input.pack(side=RIGHT, padx=1, pady=1)
        
        # self.arg_input_label = Label(self, text="Arguments")
        # self.arg_input_label.pack(side=RIGHT, padx=1, pady=1) 

        # self.msg_type = IntVar(self)
        # self.msg_type_input = Entry(self, textvariable=self.msg_type)
        # self.msg_type_input.pack(side=RIGHT, padx=1, pady=1) 

        # self.msg_type_label = Label(self, text="Type")
        # self.msg_type_label.pack(side=RIGHT, padx=1, pady=1) 

        # self.start_btn = Button(self, text="Start", command=self.start)
        # self.start_btn.pack(side=LEFT, padx=1, pady=1)

        # self.stop_btn = Button(self, text="Stop", command=self.stop)
        # self.stop_btn.pack(side=LEFT, padx=1, pady=1)

    def __init_basic__(self):
        self.start_btn = Button(self, text="Start", command=self.start)
        self.start_btn.grid(row=1, column=0, padx=1, pady=1)

        self.stop_btn = Button(self, text="Stop", command=self.stop)
        self.stop_btn.grid(row=1, column=1, padx=1, pady=1)

        self.msg_type_label = Label(self, text="Type")
        self.msg_type_label.grid(row=1, column=2, padx=1, pady=1) 

        self.msg_type = IntVar(self)
        self.msg_type_input = Entry(self, textvariable=self.msg_type)
        self.msg_type_input.grid(row=1, column=3, padx=1, pady=1) 

        self.arg_input_label = Label(self, text="Arguments")
        self.arg_input_label.grid(row=1, column=4, padx=1, pady=1) 

        self.args = StringVar(self)
        self.arg_input = Entry(self, textvariable=self.args)
        self.arg_input.grid(row=1, column=5, padx=1, pady=1)

        self.arg_encoding = StringVar(self)
        self.arg_encoding.set("int") 
        self.arg_menu = OptionMenu(self, self.arg_encoding, "int", "float", "hex")
        self.arg_menu.grid(row=1, column=6, padx=1, pady=1)

        self.send_btn = Button(self, text="Send", command=self.send)
        self.send_btn.grid(row=1, column=7, padx=1, pady=1)
    
    def __init_config__(self):
        # PID
        self.pid_factor_label = Label(self, text="PID-Factor")
        self.pid_factor = StringVar(self, value=self.config.pid_factor)
        self.pid_factor_input = Entry(self, textvariable=self.pid_factor)
        
        self.p_angle_factor_label = Label(self, text="P-Angle-Factor")
        self.p_angle_factor = StringVar(self, value=self.config.p_angle_factor)
        self.p_angle_factor_input = Entry(self, textvariable=self.p_angle_factor)

        self.i_angle_factor_label = Label(self, text="I-Angle-Factor")
        self.i_angle_factor = StringVar(self, value=self.config.i_angle_factor)
        self.i_angle_factor_input = Entry(self, textvariable=self.i_angle_factor)

        self.d_angle_factor_label = Label(self, text="D-Angle-Factor")
        self.d_angle_factor = StringVar(self, value=self.config.d_angle_factor)
        self.d_angle_factor_input = Entry(self, textvariable=self.d_angle_factor)

        self.p_rate_factor_label = Label(self, text="P-Rate-Factor")
        self.p_rate_factor = StringVar(self, value=self.config.p_rate_factor)
        self.p_rate_factor_input = Entry(self, textvariable=self.p_rate_factor)

        self.i_rate_factor_label = Label(self, text="I-Rate-Factor")
        self.i_rate_factor = StringVar(self, value=self.config.i_rate_factor)
        self.i_rate_factor_input = Entry(self, textvariable=self.i_rate_factor)

        self.d_rate_factor_label = Label(self, text="D-Rate-Factor")
        self.d_rate_factor = StringVar(self, value=self.config.d_rate_factor)
        self.d_rate_factor_input = Entry(self, textvariable=self.d_rate_factor)

        self.p_vertical_velocity_factor_label = Label(self, text="P-Velocity-Factor")
        self.p_vertical_velocity_factor = StringVar(self, value=self.config.p_vertical_velocity_factor)
        self.p_vertical_velocity_factor_input = Entry(self, textvariable=self.p_vertical_velocity_factor)

        self.i_vertical_velocity_factor_label = Label(self, text="I-Velocity-Factor")
        self.i_vertical_velocity_factor = StringVar(self, value=self.config.i_vertical_velocity_factor)
        self.i_vertical_velocity_factor_input = Entry(self, textvariable=self.i_vertical_velocity_factor)

        self.d_vertical_velocity_factor_label = Label(self, text="D-Velocity-Factor")
        self.d_vertical_velocity_factor = StringVar(self, value=self.config.d_vertical_velocity_factor)
        self.d_vertical_velocity_factor_input = Entry(self, textvariable=self.d_vertical_velocity_factor)

        self.pid_interval_label = Label(self, text="PID-Interval")
        self.pid_interval = StringVar(self, value=self.config.pid_interval)
        self.pid_interval_input = Entry(self, textvariable=self.pid_interval)

        # Speed
        self.min_speed_label = Label(self, text="Min-Speed")
        self.min_speed = StringVar(self, value=self.config.min_speed)
        self.min_speed_input = Entry(self, textvariable=self.min_speed)

        self.max_speed_label = Label(self, text="Max-Speed")
        self.max_speed = StringVar(self, value=self.config.max_speed)
        self.max_speed_input = Entry(self, textvariable=self.max_speed)

        self.vertical_velocity_amplifier_label = Label(self, text="Velocity-Amp")
        self.vertical_velocity_amplifier = StringVar(self, value=self.config.vertical_velocity_amplifier)
        self.vertical_velocity_amplifier_input = Entry(self, textvariable=self.vertical_velocity_amplifier)

        self.velocity_limit_label = Label(self, text="Velocity-Limit")
        self.velocity_limit = StringVar(self, value=self.config.velocity_limit)
        self.velocity_limit_input = Entry(self, textvariable=self.velocity_limit)

        self.speed_interval_label = Label(self, text="Speed-Interval")
        self.speed_interval = StringVar(self, value=self.config.speed_interval)
        self.speed_interval_input = Entry(self, textvariable=self.speed_interval)

        # Sensor

        self.acceleration_weight_label = Label(self, text="Acceleration-Weight")
        self.acceleration_weight = StringVar(self, value=self.config.acceleration_weight)
        self.acceleration_weight_input = Entry(self, textvariable=self.acceleration_weight)

        self.measurement_error_angle_label = Label(self, text="Err-Mea-Angle")
        self.measurement_error_angle = StringVar(self, value=self.config.measurement_error_angle)
        self.measurement_error_angle_input = Entry(self, textvariable=self.measurement_error_angle)

        self.measurement_error_angular_velocity_label = Label(self, text="Err-Mea-Angle-V")
        self.measurement_error_angular_velocity = StringVar(self, value=self.config.measurement_error_angular_velocity)
        self.measurement_error_angular_velocity_input = Entry(self, textvariable=self.measurement_error_angular_velocity)

        self.estimate_error_angle_label = Label(self, text="Err-Est-Angle")
        self.estimate_error_angle = StringVar(self, value=self.config.estimate_error_angle)
        self.estimate_error_angle_input = Entry(self, textvariable=self.estimate_error_angle)

        self.estimate_error_angular_velocity_label = Label(self, text="Err-Est-Angle-V")
        self.estimate_error_angular_velocity = StringVar(self, value=self.config.estimate_error_angular_velocity)
        self.estimate_error_angular_velocity_input = Entry(self, textvariable=self.estimate_error_angular_velocity)

        self.altitude_gain_label = Label(self, text="Altitude-Gain")
        self.altitude_gain = StringVar(self, value=self.config.altitude_gain)
        self.altitude_gain_input = Entry(self, textvariable=self.altitude_gain)

        self.speed_gain_label = Label(self, text="Speed-Gain")
        self.speed_gain = StringVar(self, value=self.config.speed_gain)
        self.speed_gain_input = Entry(self, textvariable=self.speed_gain)

        self.use_kalman_orientation_label = Label(self, text="Use-Kalman-Orientation")
        self.use_kalman_orientation = StringVar(self, value=self.config.use_kalman_orientation)
        self.use_kalman_orientation_input = Entry(self, textvariable=self.use_kalman_orientation)                

        self.sensor_enabled_label = Label(self, text="Sensor-Enabled")
        self.sensor_enabled = StringVar(self, value=self.config.sensor_enabled)
        self.sensor_enabled_input = Entry(self, textvariable=self.sensor_enabled)                

        # Log

        self.orientation_enabled_label = Label(self, text="Log-Orientation")
        self.orientation_enabled = StringVar(self, value=self.config.orientation_enabled)
        self.orientation_enabled_input = Entry(self, textvariable=self.orientation_enabled)

        self.pid_enabled_label = Label(self, text="Log-PID")
        self.pid_enabled = StringVar(self, value=self.config.pid_enabled)
        self.pid_enabled_input = Entry(self, textvariable=self.pid_enabled)

        self.speed_enabled_label = Label(self, text="Log-Speed")
        self.speed_enabled = StringVar(self, value=self.config.speed_enabled)
        self.speed_enabled_input = Entry(self, textvariable=self.speed_enabled)

        self.altitude_enabled_label = Label(self, text="Log-Altitude")
        self.altitude_enabled = StringVar(self, value=self.config.altitude_enabled)
        self.altitude_enabled_input = Entry(self, textvariable=self.altitude_enabled)

        self.log_interval_label = Label(self, text="Log-Interval")
        self.log_interval = StringVar(self, value=self.config.log_interval)
        self.log_interval_input = Entry(self, textvariable=self.log_interval)

        
        # Controls

        self.send_config_btn = Button(self, text="Send Config", command=self.send_config)
        self.save_config_btn = Button(self, text="Save Config", command=self.save_config)

        self.pid_factor_label.grid(row=2, column=0, padx=1, pady=1) 
        self.pid_factor_input.grid(row=2, column=1, padx=1, pady=1)
        self.p_angle_factor_label.grid(row=3, column=0, padx=1, pady=1)
        self.p_angle_factor_input.grid(row=3, column=1, padx=1, pady=1)
        self.i_angle_factor_label.grid(row=4, column=0, padx=1, pady=1)
        self.i_angle_factor_input.grid(row=4, column=1, padx=1, pady=1)
        self.d_angle_factor_label.grid(row=5, column=0, padx=1, pady=1)
        self.d_angle_factor_input.grid(row=5, column=1, padx=1, pady=1)

        self.p_rate_factor_label.grid(row=6, column=0, padx=1, pady=1)
        self.p_rate_factor_input.grid(row=6, column=1, padx=1, pady=1)
        self.i_rate_factor_label.grid(row=7, column=0, padx=1, pady=1)
        self.i_rate_factor_input.grid(row=7, column=1, padx=1, pady=1)
        self.d_rate_factor_label.grid(row=8, column=0, padx=1, pady=1)
        self.d_rate_factor_input.grid(row=8, column=1, padx=1, pady=1)

        self.pid_interval_label.grid(row=9, column=0, padx=1, pady=1)
        self.pid_interval_input.grid(row=9, column=1, padx=1, pady=1)

        self.p_vertical_velocity_factor_label.grid(row=2, column=2, padx=1, pady=1)
        self.p_vertical_velocity_factor_input.grid(row=2, column=3, padx=1, pady=1)
        self.i_vertical_velocity_factor_label.grid(row=3, column=2, padx=1, pady=1)
        self.i_vertical_velocity_factor_input.grid(row=3, column=3, padx=1, pady=1)
        self.d_vertical_velocity_factor_label.grid(row=4, column=2, padx=1, pady=1)
        self.d_vertical_velocity_factor_input.grid(row=4, column=3, padx=1, pady=1)

        self.min_speed_label.grid(row=5, column=2, padx=1, pady=1)
        self.min_speed_input.grid(row=5, column=3, padx=1, pady=1)
        self.max_speed_label.grid(row=6, column=2, padx=1, pady=1)
        self.max_speed_input.grid(row=6, column=3, padx=1, pady=1)
        self.vertical_velocity_amplifier_label.grid(row=7, column=2, padx=1, pady=1)
        self.vertical_velocity_amplifier_input.grid(row=7, column=3, padx=1, pady=1)
        self.velocity_limit_label.grid(row=8, column=2, padx=1, pady=1)
        self.velocity_limit_input.grid(row=8, column=3, padx=1, pady=1)

        self.speed_interval_label.grid(row=9, column=2, padx=1, pady=1)
        self.speed_interval_input.grid(row=9, column=3, padx=1, pady=1)

        self.orientation_enabled_label.grid(row=2, column=4, padx=1, pady=1)
        self.orientation_enabled_input.grid(row=2, column=5, padx=1, pady=1)
        self.pid_enabled_label.grid(row=3, column=4, padx=1, pady=1)
        self.pid_enabled_input.grid(row=3, column=5, padx=1, pady=1)
        self.speed_enabled_label.grid(row=4, column=4, padx=1, pady=1)
        self.speed_enabled_input.grid(row=4, column=5, padx=1, pady=1)
        self.altitude_enabled_label.grid(row=5, column=4, padx=1, pady=1)
        self.altitude_enabled_input.grid(row=5, column=5, padx=1, pady=1)
        self.log_interval_label.grid(row=6, column=4, padx=1, pady=1)
        self.log_interval_input.grid(row=6, column=5, padx=1, pady=1)

        self.acceleration_weight_label.grid(row=2, column=6, padx=1, pady=1)
        self.acceleration_weight_input.grid(row=2, column=7, padx=1, pady=1)

        self.measurement_error_angle_label.grid(row=3, column=6, padx=1, pady=1)
        self.measurement_error_angle_input.grid(row=3, column=7, padx=1, pady=1)
        self.measurement_error_angular_velocity_label.grid(row=4, column=6, padx=1, pady=1)
        self.measurement_error_angular_velocity_input.grid(row=4, column=7, padx=1, pady=1)

        self.estimate_error_angle_label.grid(row=5, column=6, padx=1, pady=1)
        self.estimate_error_angle_input.grid(row=5, column=7, padx=1, pady=1)
        self.estimate_error_angular_velocity_label.grid(row=6, column=6, padx=1, pady=1)
        self.estimate_error_angular_velocity_input.grid(row=6, column=7, padx=1, pady=1)
        
        self.altitude_gain_label.grid(row=7, column=6, padx=1, pady=1)
        self.altitude_gain_input.grid(row=7, column=7, padx=1, pady=1)
        self.speed_gain_label.grid(row=8, column=6, padx=1, pady=1)
        self.speed_gain_input.grid(row=8, column=7, padx=1, pady=1)

        self.use_kalman_orientation_label.grid(row=9, column=6, padx=1, pady=1)
        self.use_kalman_orientation_input.grid(row=9, column=7, padx=1, pady=1)
        
        self.sensor_enabled_label.grid(row=10, column=6, padx=1, pady=1)
        self.sensor_enabled_input.grid(row=10, column=7, padx=1, pady=1)

        self.send_config_btn.grid(row=7, column=4, padx=1, pady=1)
        self.save_config_btn.grid(row=7, column=5, padx=1, pady=1)
        
    def __init_controls__(self):

        self.throttle = DoubleVar()
        self.throttle_input = Scale(self, from_=-100, to=100,tickinterval=1, variable=self.throttle, orient=HORIZONTAL, command=self.update_throttle)
        self.throttle_input.set(0)

        self.throttle_input.grid(row=0, column=0, columnspan=2,sticky='NSEW')

        self.angle_x = DoubleVar()
        self.angle_x_input = Scale(self, from_=-180, to=180,tickinterval=1, variable=self.angle_x, orient=HORIZONTAL, command=self.update_control)
        self.angle_x_input.set(0)

        self.angle_x_input.grid(row=0, column=2, columnspan=3,sticky='NSEW')

        self.angle_y = DoubleVar()
        self.angle_y_input = Scale(self, from_=-180, to=180,tickinterval=1, variable=self.angle_y, orient=HORIZONTAL, command=self.update_control)
        self.angle_y_input.set(0)

        self.angle_y_input.grid(row=0, column=5, columnspan=3,sticky='NSEW')

    def start_alive_thread(self):
        self.alive_thread = threading.Thread(target=self.alive_loop, args=())
        self.alive_thread.start()

    def start_xbox_controller(self):
        self.xbox = ControllerInput.XboxController()
        if(self.xbox.active):
            self.xbox_thread = threading.Thread(target=self.xbox_control_thread, args=())
            self.xbox_thread.start()

    def xbox_control_thread(self):
        time.sleep(5)
        while(True):
            time.sleep(0.11)
            if(abs(self.xbox.LeftJoystickY) > 0.2):
                self.throttle.set(self.xbox.LeftJoystickY*100)
            else:
                self.throttle.set(0)

            if(abs(self.xbox.RightJoystickY) > 0.5):
                self.angle_x.set(self.xbox.RightJoystickY*180)
            else:
                self.angle_x.set(0)
            if(abs(self.xbox.RightJoystickX) > 0.2):
                self.angle_y.set(self.xbox.RightJoystickX*180)
            else:
                self.angle_y.set(0)
            self.update_control(1)

    def start(self):
        self.send_msg(MessageTypes.START, None)

    def stop(self):
        self.send_msg(MessageTypes.STOP, None)

    def send(self):
        try:
            arg_list = self.args.get().split(" ")
            payload = bytearray()
            if(self.arg_encoding.get() == "int"):
                for arg in arg_list:
                    payload = payload + int(arg).to_bytes(4, "big")
            elif(self.arg_encoding.get() == "float"):
                for arg in arg_list:
                    payload= payload + struct.pack("f", float(arg))
            else:
                payload = bytearray.fromhex(self.args.get().strip(" "))

            self.send_msg(self.msg_type.get(), payload)
        except Exception as e:
            print("Couldn't send inputs \""+ self.args.get() +"\". Reason: "+str(e))

    def send_msg(self, msg_type, payload: bytearray):
        if payload:
            self.sender.send(msg_type.to_bytes(1, "big") + payload)
        else:
            self.sender.send(msg_type.to_bytes(1, "big"))

    def alive_loop(self):
        while(True):
            time.sleep(0.5)
            self.send_msg(MessageTypes.ALIVE, None)

    def apply_config(self):
        self.config.pid_factor = float(self.pid_factor.get())
        self.config.p_angle_factor = float(self.p_angle_factor.get())
        self.config.i_angle_factor = float(self.i_angle_factor.get())
        self.config.d_angle_factor = float(self.d_angle_factor.get())

        self.config.p_rate_factor = float(self.p_rate_factor.get())
        self.config.i_rate_factor = float(self.i_rate_factor.get())
        self.config.d_rate_factor = float(self.d_rate_factor.get())

        self.config.p_vertical_velocity_factor = float(self.p_vertical_velocity_factor.get())
        self.config.i_vertical_velocity_factor = float(self.i_vertical_velocity_factor.get())
        self.config.d_vertical_velocity_factor = float(self.d_vertical_velocity_factor.get())

        self.config.pid_interval = int(self.pid_interval.get())

        self.config.min_speed = float(self.min_speed.get())
        self.config.max_speed = float(self.max_speed.get())
        self.config.vertical_velocity_amplifier = float(self.vertical_velocity_amplifier.get())
        self.config.velocity_limit = float(self.velocity_limit.get())
        self.config.speed_interval = int(self.speed_interval.get())

        self.config.orientation_enabled = int(self.orientation_enabled.get())
        self.config.pid_enabled = int(self.pid_enabled.get())
        self.config.speed_enabled = int(self.speed_enabled.get())
        self.config.altitude_enabled = int(self.altitude_enabled.get())
        self.config.log_interval = int(self.log_interval.get())

        self.config.acceleration_weight = float(self.acceleration_weight.get())
        self.config.measurement_error_angle = float(self.measurement_error_angle.get())
        self.config.measurement_error_angular_velocity = float(self.measurement_error_angular_velocity.get())
        self.config.estimate_error_angle = float(self.estimate_error_angle.get())
        self.config.estimate_error_angular_velocity = float(self.estimate_error_angular_velocity.get())
        self.config.altitude_gain = float(self.altitude_gain.get())
        self.config.speed_gain = float(self.speed_gain.get())
        self.config.use_kalman_orientation = int(self.use_kalman_orientation.get())
        self.config.sensor_enabled = int(self.sensor_enabled.get())
    
    def send_config(self):
        self.apply_config()

        payload = bytearray()
        payload = payload + struct.pack("f", self.config.pid_factor)
        payload = payload + struct.pack("f", self.config.p_angle_factor)
        payload = payload + struct.pack("f", self.config.i_angle_factor)
        payload = payload + struct.pack("f", self.config.d_angle_factor)
        payload = payload + struct.pack("f", self.config.p_rate_factor)
        payload = payload + struct.pack("f", self.config.i_rate_factor)
        payload = payload + struct.pack("f", self.config.d_rate_factor)
        payload = payload + struct.pack("f", self.config.p_vertical_velocity_factor)
        payload = payload + struct.pack("f", self.config.i_vertical_velocity_factor)
        payload = payload + struct.pack("f", self.config.d_vertical_velocity_factor)
        payload = payload + struct.pack("<i", self.config.pid_interval)
        
        payload = payload + struct.pack("B", self.config.orientation_enabled)
        payload = payload + struct.pack("B", self.config.pid_enabled)
        payload = payload + struct.pack("B", self.config.speed_enabled)
        payload = payload + struct.pack("B", self.config.altitude_enabled)
        payload = payload + struct.pack("<i", self.config.log_interval)

        payload = payload + struct.pack("f", self.config.min_speed)
        payload = payload + struct.pack("f", self.config.max_speed)
        payload = payload + struct.pack("f", self.config.vertical_velocity_amplifier)
        payload = payload + struct.pack("f", self.config.velocity_limit)
        payload = payload + struct.pack("<i", self.config.speed_interval)

        payload = payload + struct.pack("f", self.config.acceleration_weight)
        payload = payload + struct.pack("f", self.config.measurement_error_angle)
        payload = payload + struct.pack("f", self.config.measurement_error_angular_velocity)
        payload = payload + struct.pack("f", self.config.estimate_error_angle)
        payload = payload + struct.pack("f", self.config.estimate_error_angular_velocity)
        payload = payload + struct.pack("f", self.config.altitude_gain)
        payload = payload + struct.pack("f", self.config.speed_gain)
        payload = payload + struct.pack("B", self.config.use_kalman_orientation)
        payload = payload + struct.pack("B", self.config.sensor_enabled)
        payload = payload + struct.pack("B", self.config.sensor_enabled) # Padding
        payload = payload + struct.pack("B", self.config.sensor_enabled) # Padding

        self.send_msg(MessageTypes.INIT, payload)


    def save_config(self):
        self.apply_config()
        self.config.save_config()
            
    def update_throttle(self, value):
        payload = bytearray()
        payload = payload + struct.pack("f", float(value)/100)
        self.send_msg(MessageTypes.THROTTLE, payload)

    def update_control(self, value):
        payload = bytearray()
        payload = payload + struct.pack("f", float(self.throttle.get())/100)
        payload = payload + struct.pack("f", float(self.angle_x.get())*3.14/180)
        payload = payload + struct.pack("f", float(self.angle_y.get())*3.14/180)
        self.send_msg(MessageTypes.CONTROL, payload)        