import socket
import time
import threading
from tkinter import Tk
import base64
import Receiver
import Sender
import Plot
import PlotHandler
import Recorder
import argparse

HOST = '192.168.2.116'
PORT_TX = 12346        
PORT_RX = 12345        

START_BYTE = 2
END_BYTE = 3

def recv_thread_func():
    # TODO Exit somehow
    while(1):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            print("Accepting RX on "+str(PORT_RX))
            s.bind((HOST, PORT_RX))
            s.listen()
            conn, addr = s.accept()
            print('RX Connected to ', addr)
            buffer = bytearray()
            while(1):              
                byte = conn.recv(1)
                if(append_byte(byte[0], buffer)):
                    receiver.receive(buffer[1:-1])
                    del buffer[:]
            print('Connection aborted.')
           
def append_byte(byte, buffer):
    if byte == START_BYTE and len(buffer) > 0:
        print("Another start byte received. Throwing away: "+buffer.hex())
        del buffer[:]
    buffer.append(byte)
    return byte == END_BYTE    
    print('Connection aborted.')

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Process some integers.')
    parser.add_argument('--mode', metavar='m', type=int,
                        help='0(default): tcp host, 1: replay data file')
    parser.add_argument('--recordpath', metavar='p', type=str,
                        help='path to recorded data file or directory to put recordings')

    args = parser.parse_args()


    

    tk = Tk()
    tk.geometry("1280x768+300+300")
    plot = Plot.Plot(tk)
    plot_handler = PlotHandler.PlotHandler(plot)
    recorder = Recorder.Recorder(args.recordpath)

    def d(event):
        plot_handler.redraw()
    tk.bind('<Configure>', d)

    if args.mode == 0:
        recorder.start_recording()
        receiver = Receiver.Receiver(plot_handler, recorder)
    elif args.mode == 1:
        receiver = Receiver.Receiver(plot_handler, None)
        recorder.playback_recording(receiver, args.recordpath)
    else:
        print("unrecognized mode")
        
    x = threading.Thread(target=recv_thread_func, args=())
    x.start()
    tk.mainloop()       

    # with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    #     print("Accepting TX on "+str(PORT_TX))
    #     s.bind((HOST, PORT_TX))
    #     while(True):
    #         s.listen()
    #         conn, addr = s.accept()

    #         print('TX Connected to ', addr)
    #         Sender.sendLoop(conn)
            
   

# window = tk.Tk()
    # frlfLabel = tk.Label(window,
    #     text="FRLF",
    #     fg="white",
    #     bg="black",
    # )

    # frrtLabel = tk.Label(window,
    #     text="FRRT",
    #     fg="white",
    #     bg="black",
    # )

    # bklfLabel = tk.Label(window,
    #     text="BKLF",
    #     fg="white",
    #     bg="black",
    # )

    # bkrtLabel = tk.Label(window,
    #     text="BKRT",
    #     fg="white",
    #     bg="black",
    # )

    # tgtAngleXLabel = tk.Label(window,
    #     text="TgtAngleX",
    #     fg="white",
    #     bg="black",
    # )

    # angleXLabel = tk.Label(window,
    #     text="AngleX",
    #     fg="white",
    #     bg="black",
    # )

    # tgtAngleYLabel = tk.Label(window,
    #     text="TgtAngleY",
    #     fg="white",
    #     bg="black",
    # )

    # angleYLabel = tk.Label(window,
    #     text="AngleY",
    #     fg="white",
    #     bg="black",
    # )

    # frlfLabel.grid(column=0, row=0)
    # frrtLabel.grid(column=5, row=0)
    # bklfLabel.grid(column=0, row=5)
    # bkrtLabel.grid(column=5, row=5)

    # tgtAngleXLabel.grid(column=1, row=2)
    # angleXLabel.grid(column=1, row=3)

    # tgtAngleYLabel.grid(column=2, row=1)
    # angleYLabel.grid(column=2, row=2)
    

    # window.mainloop()
    # while True:
    #     print("Destroy yourself")