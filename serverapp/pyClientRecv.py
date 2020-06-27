import socket
import time
from base64 import b85decode
HOST = '192.168.2.116'
PORT = 12346

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    server_address = (HOST, PORT)
    
    s.connect(server_address)
    print('Connected')
    while(1):
        msg = s.recv(100)
        encoded_payload = msg[1:-1]
        payload = b85decode(encoded_payload)
        print("Received " +payload.hex())


    print('Connection aborted.')