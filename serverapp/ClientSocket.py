import threading
import socket

class ClientSocket():
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.send_mutex = threading.Lock()
        self.recv_mutex = threading.Lock()
        self.socket = None
        self.is_connected = False
        self.is_connecting = False

    def start_connect_task(self):
        if not self.is_connecting:
            self.thread = threading.Thread(target=self.connect_task, args=())
            self.thread.start()

    def connect_task(self):
        self.is_connecting = True
        self.connect()
        self.is_connecting = False


    def connect(self):
        self.is_connected = False
        if self.socket:
            # TODO Check if this works without exception
            self.shutdown()

        server_address = (self.host, self.port)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        while not self.is_connected:
            try:
                print("Connecting Socket...")         
                self.socket.connect(server_address)
                # TODO This may timeout
                self.is_connected = True
                print("Socket connected.")
            except TimeoutError:
                print("Timeout")



    def shutdown(self):
        try:
            self.socket.shutdown(socket.SHUT_RDWR)
            self.socket.close()
        except OSError:
            print("Error during socket shutdown. Ignoring")

    def send(self, msg):
        success = False
        self.send_mutex.acquire()
        try:
            if self.is_connected:
                sent = self.socket.send(msg)
                if sent == 0:
                    self.start_connect_task()
                    success = False
                else:
                    success = True
            else:
                self.start_connect_task()
        except OSError:
            self.start_connect_task()
        finally:
            self.send_mutex.release()
        return success

    def recv(self, length):
        received = b''
        self.recv_mutex.acquire()
        try:
            if self.is_connected:
                received = self.socket.recv(length)
                if received == b'':
                    self.start_connect_task()
            else:
                self.start_connect_task()
        except OSError:
            self.start_connect_task()  
        finally:
            self.recv_mutex.release()
        return received