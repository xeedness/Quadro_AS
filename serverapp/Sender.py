import socket
from base64 import b85encode
import threading
import MessageTypes

class Sender():
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.start_open_socket_thread()

    def start_open_socket_thread(self):
        self.thread = threading.Thread(target=self.open_socket, args=())
        self.thread.start()

    def open_socket(self):
        self.listening = True
        self.send_listen_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.send_listen_socket.bind((self.host, self.port))

        print("Accepting TX Connection on "+str(self.port))
        self.send_listen_socket.listen()
        self.send_socket, self.conn_addr = self.send_listen_socket.accept()
        print('TX Connected to ', self.conn_addr)
        self.send_listen_socket.close()

        self.listening = False

    def send(self, payload):
        encoded_payload = b85encode(payload)
        msg = b'\x02' + encoded_payload + b'\x03'

        try:
            sent = self.send_socket.send(msg)
            if sent:
                print("Sent "+payload.hex()+ "(" + msg.hex() + ")")
            else:
                raise RuntimeError("socket connection broken")
        except: 
            print("Could not send "+payload.hex()+ "(" + msg.hex() + ")")
            if(not self.listening):
                self.start_open_socket_thread()

# def sendLoop(socket):
#     while(1):
#         try:
#             x = input()
#             values = x.split(" ")
#             msg_type = int(values[0])

#             if msg_type == 44:
#                 intensity = float(values[1])
#                 payload = msg_type.to_bytes(1, "big") + intensity.to_bytes(4, "big")
#             else:
#                 payload = msg_type.to_bytes(1, "big")
#         except:
#             print("Invalid input")
#             continue

#         print("Sending bytes: "+payload.hex())
#         encoded_payload = b85encode(payload)
#         print("Sending b85: "+encoded_payload.hex())
       
#         socket.send(b'\x02' + encoded_payload + b'\x03')
        
