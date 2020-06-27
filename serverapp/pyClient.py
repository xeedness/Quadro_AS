import socket
import time
from base64 import b85encode
HOST = '192.168.2.116'
PORT = 12345

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    server_address = (HOST, PORT)
    
    s.connect(server_address)
    print('Connected')
    while(1):
        payload = b"\x00\x00\x00\x00\x01"
        msg = b'\x02' + b85encode(payload) + b'\x02' + b85encode(payload) + b'\x03'
        s.send(msg)
        time.sleep(5)

print('Connection aborted.')