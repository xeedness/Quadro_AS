from base64 import b85decode
import binascii
import struct
import threading
import socket
from PlotHandler import PlotHandler
from Recorder import Recorder
from Controller import Controller
from ClientSocket import ClientSocket
import MessageTypes
import time

START_BYTE = 2
END_BYTE = 3

class Receiver():

    def __init__(self, client_socket: ClientSocket, plot_handler: PlotHandler, recorder: Recorder, controller: Controller):
        self.client_socket = client_socket
        self.plot_handler = plot_handler
        self.recorder = recorder
        self.controller = controller
        self.start_recv_thread()

    def start_recv_thread(self):
        self.thread = threading.Thread(target=self.recv_thread_func, args=())
        self.thread.start()
        
    def recv_thread_func(self):
        buffer = bytearray()
        while(1):              
            byte = self.client_socket.recv(1)
            if byte == b'':
                # Sleep a little to not bombard the client socket with recv calls
                time.sleep(0.1)
            else:
                if(self.append_byte(byte[0], buffer)):
                    self.receive(buffer[1:-1])
                    del buffer[:]      
           
    def append_byte(self, byte, buffer):
        if byte == START_BYTE and len(buffer) > 0:
            print("Another start byte received. Throwing away: "+buffer.hex())
            del buffer[:]
        buffer.append(byte)
        return byte == END_BYTE    

    def receive(self, packet):
        try:
            if self.recorder:
                self.recorder.record_packet(packet)
            payload = b85decode(packet)
            msg_type = payload[0]
            data = payload[1:]
            # print("Received ["+hex(msg_type)+"] "+ data.hex())

            if msg_type == 0:
                value = int.from_bytes(data, "big")
                print("Interpreted data as integer: "+str(value))
            elif msg_type == MessageTypes.INIT:
                print("Init Request received")
                self.send_init()
            elif msg_type == MessageTypes.SENSOR:
                self.sensor_update(data)
            elif msg_type == MessageTypes.ANGULAR_VELOCITY_LOG:
                self.angular_velocity_update(data)
            elif msg_type == MessageTypes.PID:
                self.pid_update(data)
            elif msg_type == MessageTypes.THROTTLE_LOG:
                self.throttle_update(data)
            elif msg_type == MessageTypes.ALTIMETER_DATA_LOG:
                self.altimeter_data_update(data)
            else:
                print("Could not interpret msg_type: "+str(msg_type))
        except:
            print("Decoding exception. Throwing away packet")

    def sensor_update(self, data):
        [x_angle, y_angle, z_angle] = struct.unpack('fff', data[0:12])
        self.plot_handler.record_x_angle(x_angle)
        self.plot_handler.record_y_angle(y_angle)
        self.plot_handler.record_z_angle(z_angle)
        # print("x angle: "+str(x_angle))
        # print("y angle: "+str(y_angle))
        # print("z angle: "+str(z_angle))

    def angular_velocity_update(self, data):
        [x, y, z] = struct.unpack('fff', data[0:12])
        self.plot_handler.record_x_av(x)
        self.plot_handler.record_y_av(y)
        self.plot_handler.record_z_av(z)

    def pid_update(self, data):
        [x_pid, y_pid] = struct.unpack('ff', data[0:8])
        self.plot_handler.record_x_pid(x_pid)
        self.plot_handler.record_y_pid(y_pid)
        # print("x pid: "+str(x_pid))
        # print("y pid: "+str(y_pid))

    def throttle_update(self, data):
        [lf, rf, lr, rr] = struct.unpack('ffff', data[0:16])
        self.plot_handler.record_lf_throttle(lf)
        self.plot_handler.record_rf_throttle(rf)
        self.plot_handler.record_lr_throttle(lr)
        self.plot_handler.record_rr_throttle(rr)
    
    def altimeter_data_update(self, data):
        [altitude, vertical_velocity, vertical_acceleration] = struct.unpack('fff', data[0:12])
        self.plot_handler.record_altitude(altitude)
        self.plot_handler.record_vertical_velocity(vertical_velocity)
        self.plot_handler.record_vertical_acceleration(vertical_acceleration)

    def send_init(self):
        self.controller.send_config()



    

