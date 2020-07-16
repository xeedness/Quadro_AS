from base64 import b85decode
import binascii
import struct
import threading
import socket
from PlotHandler import PlotHandler
from Recorder import Recorder
from Controller import Controller
import MessageTypes

START_BYTE = 2
END_BYTE = 3

class Receiver():

    def __init__(self, host, port, plot_handler: PlotHandler, recorder: Recorder, controller: Controller):
        self.host = host
        self.port = port
        self.plot_handler = plot_handler
        self.recorder = recorder
        self.controller = controller
        self.start_listen_thread()

    def start_listen_thread(self):
        self.thread = threading.Thread(target=self.listen_thread_func, args=())
        self.thread.start()

    def start_recv_thread(self, conn):
        self.thread = threading.Thread(target=self.recv_thread_func, args=[conn])
        self.thread.start()

    def listen_thread_func(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            print("Accepting RX on "+str(self.port))
            s.bind((self.host, self.port))
            s.listen()
            while(True):
                conn, addr = s.accept()
                print('RX Connected to ', addr)
                self.start_recv_thread(conn)
        
    def recv_thread_func(self, conn):
        buffer = bytearray()
        while(1):              
            byte = conn.recv(1)
            if(self.append_byte(byte[0], buffer)):
                self.receive(buffer[1:-1])
                del buffer[:]
        print('Connection aborted.')      
           
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
                self.send_init()
            elif msg_type == MessageTypes.SENSOR:
                self.sensor_update(data)
            elif msg_type == MessageTypes.PID:
                self.pid_update(data)
            elif msg_type == MessageTypes.THROTTLE_LOG:
                self.throttle_update(data)
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

    def pid_update(self, data):
        [x_pid, y_pid] = struct.unpack('ff', data[0:8])
        self.plot_handler.record_x_pid(x_pid)
        self.plot_handler.record_y_pid(y_pid)
        # print("x pid: "+str(x_pid))
        # print("y pid: "+str(y_pid))

    def throttle_update(self, data):
        [lf, rf, lr, rr] = struct.unpack('HHHH', data[0:8])
        self.plot_handler.record_lf_throttle(lf)
        self.plot_handler.record_rf_throttle(rf)
        self.plot_handler.record_lr_throttle(lr)
        self.plot_handler.record_rr_throttle(rr)
        # print("lf: " + str(lf))
        # print("rf: " + str(rf))
        # print("lr: " + str(lr))
        # print("rr: " + str(rr))

    def send_init(self):
        self.controller.send_config()



    
