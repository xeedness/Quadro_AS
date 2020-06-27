import os
import base64
import time
from datetime import datetime
import threading

class Recorder():
    def __init__(self, path):
        self.path = path

    def start_recording(self):
        
        timestr = datetime.utcnow().strftime("%Y-%m-%d %H-%M-%S")
        self.filepath = os.path.join(self.path, "record_"+timestr+".csv")
        self.file = open(self.filepath, "w")

    def record_packet(self, packet):
        timestr = datetime.utcnow().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        packet_str = base64.b64encode(base64.b85decode(packet)).decode("utf-8")
        self.file.write(timestr+";"+packet_str+"\n")
        self.file.flush()

    def end_recording(self):
        self.file.close()

    def playback_recording(self, receiver, path):
        self.receiver = receiver
        self.packets = self.read_packets(path)
        self.base_packet_time = self.packets[0]["timestamp"]
        self.base_real_time = datetime.utcnow()
        self.playback_running = True
        self.playback_paused = False
        self.thread = threading.Thread(target=self.playback_thread_func, args=())
        self.thread.start()

    def stop_playback(self):
        self.playback_running = False
        self.packets = []

    def pause_playback(self):
        self.playback_paused = True

    def continue_playback(self):
        self.playback_paused = False

    def playback_thread_func(self):
        while(self.playback_running and len(self.packets) > 0):
            if not self.playback_paused:
                packet_diff = self.packets[0]["timestamp"] - self.base_packet_time
                real_diff = datetime.utcnow() - self.base_real_time
                if packet_diff < real_diff:
                    packet85 = base64.b85encode(base64.b64decode(bytearray(self.packets[0]["packet"], "utf-8")[:-1]))
                    self.receiver.receive(packet85)
                    self.packets[:] = self.packets[1:]

                time.sleep(0.001)



    def read_packets(self, path):
        packets = []
        f = open(path, "r")
        lines = f.readlines() 
        for line in lines:
            columns = line.split(";")
            timestamp = datetime.strptime(columns[0], "%Y-%m-%d %H:%M:%S.%f")
            packet = columns[1]

            packets.append({"timestamp": timestamp, "packet": packet})

        return packets



