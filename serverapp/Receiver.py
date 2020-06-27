from base64 import b85decode
import binascii
import struct
import Plot
from Recorder import Recorder

class Receiver():

    def __init__(self, plot_handler, recorder: Recorder):
        self.plot_handler = plot_handler
        self.recorder = recorder

    def receive(self, packet):
        if self.recorder:
            self.recorder.record_packet(packet)
        payload = b85decode(packet)
        msg_type = payload[0]
        data = payload[1:]
        print("Received ["+hex(msg_type)+"] "+ data.hex())

        if msg_type == 0:
            value = int.from_bytes(data, "big")
            print("Interpreted data as integer: "+str(value))
        elif msg_type == 42:
            self.sensor_update(data)
        else:
            print("Could not interpret msg_type: "+str(msg_type))

    def sensor_update(self, data):
        #[x_angle, y_angle, z_angle] = struct.unpack('f', data)
        x_angle = struct.unpack('f', data[0:4])[0]
        y_angle = struct.unpack('f', data[4:8])[0]
        z_angle = struct.unpack('f', data[8:12])[0]
        self.plot_handler.record_x_angle(x_angle)
        self.plot_handler.record_y_angle(y_angle)
        self.plot_handler.record_z_angle(z_angle)
        print("x: "+str(x_angle))
        print("y: "+str(y_angle))
        print("z: "+str(z_angle))

