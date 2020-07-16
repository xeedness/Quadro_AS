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
        encoded_payload = b85encode(payload, pad=True)
        msg = b'\x02' + encoded_payload + b'\x03'

        try:
            sent = self.send_socket.send(msg)
            if sent:
                print("Sent "+payload.hex()+ "(" + msg.hex() + ")")
            else:
                raise RuntimeError("socket connection broken")
        except: 
            if(not self.listening):
                print("Could not send "+payload.hex()+ "(" + msg.hex() + ")")
                self.start_open_socket_thread()
            
