import socket
from base64 import b85encode
import threading
import MessageTypes
from ClientSocket import ClientSocket

class Sender():
    def __init__(self, client_socket: ClientSocket):
        self.client_socket = client_socket

    def send(self, payload):            
        encoded_payload = b85encode(payload, pad=True)
        msg = b'\x02' + encoded_payload + b'\x03'

        sent = self.client_socket.send(msg)
        # if sent:
        #    print("Sent "+payload.hex()+ "(" + msg.hex() + ")")
        # else:
            # print("Could not send "+payload.hex()+ "(" + msg.hex() + ")")
            
