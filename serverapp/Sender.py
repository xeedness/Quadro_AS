import socket
from base64 import b85encode

def sendLoop(socket):
    while(1):
        try:
            x = input()
            values = x.split(" ")
            msg_type = int(values[0])

            if msg_type == 44:
                intensity = float(values[1])
                payload = msg_type.to_bytes(1, "big") + intensity.to_bytes(4, "big")
            else:
                payload = msg_type.to_bytes(1, "big")
        except:
            print("Invalid input")
            continue

        print("Sending bytes: "+payload.hex())
        encoded_payload = b85encode(payload)
        print("Sending b85: "+encoded_payload.hex())
       
        socket.send(b'\x02' + encoded_payload + b'\x03')
        
