from base64 import b85decode
import binascii
import struct
import threading
import socket
from PlotHandler import PlotHandler
from Recorder import Recorder

START_BYTE = 2
END_BYTE = 3

class Receiver():

    def __init__(self, host, port, plot_handler: PlotHandler, recorder: Recorder):
        self.host = host
        self.port = port
        self.plot_handler = plot_handler
        self.recorder = recorder
        self.start_recv_thread()

    def start_recv_thread(self):
        self.thread = threading.Thread(target=self.recv_thread_func, args=())
        self.thread.start()

    def recv_thread_func(self):
        while(True):
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                print("Accepting RX on "+str(self.port))
                s.bind((self.host, self.port))
                s.listen()
                conn, addr = s.accept()
                print('RX Connected to ', addr)
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
        print('Connection aborted.')

    def receive(self, packet):
        if self.recorder:
            self.recorder.record_packet(packet)
        payload = b85decode(packet)
        msg_type = payload[0]
        data = payload[1:]
        # print("Received ["+hex(msg_type)+"] "+ data.hex())

        if msg_type == 0:
            value = int.from_bytes(data, "big")
            print("Interpreted data as integer: "+str(value))
        elif msg_type == 42:
            self.sensor_update(data)
        else:
            print("Could not interpret msg_type: "+str(msg_type))

    def sensor_update(self, data):
        # [x_angle, y_angle, z_angle] = struct.unpack('f', data[0:12])
        x_angle = struct.unpack('f', data[0:4])[0]
        y_angle = struct.unpack('f', data[4:8])[0]
        z_angle = struct.unpack('f', data[8:12])[0]
        self.plot_handler.record_x_angle(x_angle)
        self.plot_handler.record_y_angle(y_angle)
        self.plot_handler.record_z_angle(z_angle)
        # print("x: "+str(x_angle))
        # print("y: "+str(y_angle))
        # print("z: "+str(z_angle))

