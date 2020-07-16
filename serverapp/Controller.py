import socket
from base64 import b85encode
from tkinter import Tk, Label, Frame, X, RIGHT, LEFT, RAISED, Button, OptionMenu, Entry, StringVar, IntVar, Grid, Scale, HORIZONTAL, DoubleVar
import MessageTypes
import Sender
from Config import Config
import struct
import threading
import time

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

    def initUI(self):
        self.pack(fill=self.fill, side=self.side, expand=self.expand)
        self.__init_basic__()
        self.__init_config__()
        self.__init_controls__()
        # self.send_btn = Button(self, text="Send", command=self.send)
        # self.send_btn.pack(side=RIGHT, padx=5, pady=5)

        # self.arg_encoding = StringVar(self)
        # self.arg_encoding.set("int") 
        # self.arg_menu = OptionMenu(self, self.arg_encoding, "int", "float", "hex")
        # self.arg_menu.pack(side=RIGHT, padx=5, pady=5)
    
        # self.args = StringVar(self)
        # self.arg_input = Entry(self, textvariable=self.args)
        # self.arg_input.pack(side=RIGHT, padx=5, pady=5)
        
        # self.arg_input_label = Label(self, text="Arguments")
        # self.arg_input_label.pack(side=RIGHT, padx=5, pady=5) 

        # self.msg_type = IntVar(self)
        # self.msg_type_input = Entry(self, textvariable=self.msg_type)
        # self.msg_type_input.pack(side=RIGHT, padx=5, pady=5) 

        # self.msg_type_label = Label(self, text="Type")
        # self.msg_type_label.pack(side=RIGHT, padx=5, pady=5) 

        # self.start_btn = Button(self, text="Start", command=self.start)
        # self.start_btn.pack(side=LEFT, padx=5, pady=5)

        # self.stop_btn = Button(self, text="Stop", command=self.stop)
        # self.stop_btn.pack(side=LEFT, padx=5, pady=5)

    def __init_basic__(self):
        self.start_btn = Button(self, text="Start", command=self.start)
        self.start_btn.grid(row=1, column=0, padx=5, pady=5)

        self.stop_btn = Button(self, text="Stop", command=self.stop)
        self.stop_btn.grid(row=1, column=1, padx=5, pady=5)

        self.msg_type_label = Label(self, text="Type")
        self.msg_type_label.grid(row=1, column=2, padx=5, pady=5) 

        self.msg_type = IntVar(self)
        self.msg_type_input = Entry(self, textvariable=self.msg_type)
        self.msg_type_input.grid(row=1, column=3, padx=5, pady=5) 

        self.arg_input_label = Label(self, text="Arguments")
        self.arg_input_label.grid(row=1, column=4, padx=5, pady=5) 

        self.args = StringVar(self)
        self.arg_input = Entry(self, textvariable=self.args)
        self.arg_input.grid(row=1, column=5, padx=5, pady=5)

        self.arg_encoding = StringVar(self)
        self.arg_encoding.set("int") 
        self.arg_menu = OptionMenu(self, self.arg_encoding, "int", "float", "hex")
        self.arg_menu.grid(row=1, column=6, padx=5, pady=5)

        self.send_btn = Button(self, text="Send", command=self.send)
        self.send_btn.grid(row=1, column=7, padx=5, pady=5)
    
    def __init_config__(self):
        # PID
        self.pid_factor_label = Label(self, text="PID-Factor")
        self.pid_factor = StringVar(self, value=self.config.pid_factor)
        self.pid_factor_input = Entry(self, textvariable=self.pid_factor)
        
        self.p_factor_label = Label(self, text="P-Factor")
        self.p_factor = StringVar(self, value=self.config.p_factor)
        self.p_factor_input = Entry(self, textvariable=self.p_factor)

        self.i_factor_label = Label(self, text="I-Factor")
        self.i_factor = StringVar(self, value=self.config.i_factor)
        self.i_factor_input = Entry(self, textvariable=self.i_factor)

        self.d_factor_label = Label(self, text="D-Factor")
        self.d_factor = StringVar(self, value=self.config.d_factor)
        self.d_factor_input = Entry(self, textvariable=self.d_factor)

        self.pid_interval_label = Label(self, text="PID-Interval")
        self.pid_interval = StringVar(self, value=self.config.pid_interval)
        self.pid_interval_input = Entry(self, textvariable=self.pid_interval)

        # Speed
        self.landing_speed_label = Label(self, text="Landing-Speed")
        self.landing_speed = StringVar(self, value=self.config.landing_speed)
        self.landing_speed_input = Entry(self, textvariable=self.landing_speed)

        self.hover_speed_label = Label(self, text="Hover-Speed")
        self.hover_speed = StringVar(self, value=self.config.hover_speed)
        self.hover_speed_input = Entry(self, textvariable=self.hover_speed)

        self.min_speed_label = Label(self, text="Min-Speed")
        self.min_speed = StringVar(self, value=self.config.min_speed)
        self.min_speed_input = Entry(self, textvariable=self.min_speed)

        self.max_speed_label = Label(self, text="Max-Speed")
        self.max_speed = StringVar(self, value=self.config.max_speed)
        self.max_speed_input = Entry(self, textvariable=self.max_speed)

        self.speed_interval_label = Label(self, text="Speed-Interval")
        self.speed_interval = StringVar(self, value=self.config.speed_interval)
        self.speed_interval_input = Entry(self, textvariable=self.speed_interval)

        # Sensor

        self.acceleration_weight_label = Label(self, text="Acceleration-Weight")
        self.acceleration_weight = StringVar(self, value=self.config.acceleration_weight)
        self.acceleration_weight_input = Entry(self, textvariable=self.acceleration_weight)        

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

        self.log_interval_label = Label(self, text="Log-Interval")
        self.log_interval = StringVar(self, value=self.config.log_interval)
        self.log_interval_input = Entry(self, textvariable=self.log_interval)

        


        # Controls

        self.send_config_btn = Button(self, text="Send Config", command=self.send_config)
        self.save_config_btn = Button(self, text="Save Config", command=self.save_config)

        self.pid_factor_label.grid(row=2, column=0, padx=5, pady=5) 
        self.pid_factor_input.grid(row=2, column=1, padx=5, pady=5)
        self.p_factor_label.grid(row=3, column=0, padx=5, pady=5)
        self.p_factor_input.grid(row=3, column=1, padx=5, pady=5)
        self.i_factor_label.grid(row=4, column=0, padx=5, pady=5)
        self.i_factor_input.grid(row=4, column=1, padx=5, pady=5)
        self.d_factor_label.grid(row=5, column=0, padx=5, pady=5)
        self.d_factor_input.grid(row=5, column=1, padx=5, pady=5)
        self.pid_interval_label.grid(row=6, column=0, padx=5, pady=5)
        self.pid_interval_input.grid(row=6, column=1, padx=5, pady=5)

        self.landing_speed_label.grid(row=2, column=2, padx=5, pady=5)
        self.landing_speed_input.grid(row=2, column=3, padx=5, pady=5)
        self.hover_speed_label.grid(row=3, column=2, padx=5, pady=5)
        self.hover_speed_input.grid(row=3, column=3, padx=5, pady=5)
        self.min_speed_label.grid(row=4, column=2, padx=5, pady=5)
        self.min_speed_input.grid(row=4, column=3, padx=5, pady=5)
        self.max_speed_label.grid(row=5, column=2, padx=5, pady=5)
        self.max_speed_input.grid(row=5, column=3, padx=5, pady=5)
        self.speed_interval_label.grid(row=6, column=2, padx=5, pady=5)
        self.speed_interval_input.grid(row=6, column=3, padx=5, pady=5)

        self.orientation_enabled_label.grid(row=2, column=4, padx=5, pady=5)
        self.orientation_enabled_input.grid(row=2, column=5, padx=5, pady=5)
        self.pid_enabled_label.grid(row=3, column=4, padx=5, pady=5)
        self.pid_enabled_input.grid(row=3, column=5, padx=5, pady=5)
        self.speed_enabled_label.grid(row=4, column=4, padx=5, pady=5)
        self.speed_enabled_input.grid(row=4, column=5, padx=5, pady=5)
        self.log_interval_label.grid(row=5, column=4, padx=5, pady=5)
        self.log_interval_input.grid(row=5, column=5, padx=5, pady=5)

        self.acceleration_weight_label.grid(row=2, column=6, padx=5, pady=5)
        self.acceleration_weight_input.grid(row=2, column=7, padx=5, pady=5)

        self.send_config_btn.grid(row=6, column=4, padx=5, pady=5)
        self.save_config_btn.grid(row=6, column=5, padx=5, pady=5)
        
    def __init_controls__(self):

        self.throttle = DoubleVar()
        self.throttle_input = Scale(self, from_=-100, to=100,tickinterval=1, variable=self.throttle, orient=HORIZONTAL, command=self.update_throttle)
        self.throttle_input.set(0)

        self.throttle_input.grid(row=0, column=0, columnspan=8,sticky='NSEW')

    def start_alive_thread(self):
        self.alive_thread = threading.Thread(target=self.alive_loop, args=())
        self.alive_thread.start()

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
        self.config.p_factor = float(self.p_factor.get())
        self.config.i_factor = float(self.i_factor.get())
        self.config.d_factor = float(self.d_factor.get())
        self.config.pid_interval = int(self.pid_interval.get())

        self.config.landing_speed = int(self.landing_speed.get())
        self.config.hover_speed = int(self.hover_speed.get())
        self.config.max_speed = int(self.max_speed.get())
        self.config.min_speed = int(self.min_speed.get())
        self.config.speed_interval = int(self.speed_interval.get())

        self.config.orientation_enabled = int(self.orientation_enabled.get())
        self.config.pid_enabled = int(self.pid_enabled.get())
        self.config.speed_enabled = int(self.speed_enabled.get())
        self.config.log_interval = int(self.log_interval.get())

        self.config.acceleration_weight = float(self.acceleration_weight.get())
    
    def send_config(self):
        self.apply_config()

        payload = bytearray()
        payload = payload + struct.pack("f", self.config.pid_factor)
        payload = payload + struct.pack("f", self.config.p_factor)
        payload = payload + struct.pack("f", self.config.i_factor)
        payload = payload + struct.pack("f", self.config.d_factor)
        payload = payload + struct.pack("<i", self.config.pid_interval)
        
        payload = payload + struct.pack("B", self.config.orientation_enabled)
        payload = payload + struct.pack("B", self.config.pid_enabled)
        payload = payload + struct.pack("B", self.config.speed_enabled)
        payload = payload + struct.pack("B", self.config.speed_enabled)
        payload = payload + struct.pack("<i", self.config.log_interval)

        payload = payload + struct.pack("<H", self.config.landing_speed)
        payload = payload + struct.pack("<H", self.config.hover_speed)
        payload = payload + struct.pack("<H", self.config.max_speed)
        payload = payload + struct.pack("<H", self.config.min_speed)
        payload = payload + struct.pack("<i", self.config.speed_interval)

        payload = payload + struct.pack("f", self.config.acceleration_weight)

        self.send_msg(MessageTypes.INIT, payload)


    def save_config(self):
        self.apply_config()
        self.config.save_config()
            
    def update_throttle(self, value):
        payload = bytearray()
        payload = payload + struct.pack("f", float(value)/100)
        self.send_msg(MessageTypes.THROTTLE, payload)