import socket
import time
import threading

HOST = '192.168.2.116'
PORT_TX = 12346
PORT_RX = 12345    
maxV = 32768


def recv_thread_func():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        print("Accepting RX\n")
        s.bind((HOST, PORT_RX))
        s.listen()
        conn, addr = s.accept()

        print('RX Connected to ', addr)
        while(1):
            try:
                print("Recv Block")
                response = conn.recv(128)
                value = response.decode("utf-8")
                print("Received: "+value)
            except:
                print("Recv error "+response.hex())

        print('Connection aborted.')
           
if __name__ == "__main__":
    x = threading.Thread(target=recv_thread_func, args=())
    x.start()

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        print("Accepting TX\n")
        s.bind((HOST, PORT_TX))
        s.listen()
        conn, addr = s.accept()

        print('TX Connected to ', addr)

        while(1):
            x = input()
            conn.send(str.encode(x))

    print('Connection aborted.')