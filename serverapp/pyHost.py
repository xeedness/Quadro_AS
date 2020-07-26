import time
import threading
import tkinter as tk
import base64
import Receiver
import Sender
import Plot
import PlotHandler
import Recorder
import argparse
import Controller
import Config
import ClientSocket
from tkinter import *
#HOST = '192.168.2.116'
#PORT_TX = 12346        
#PORT_RX = 12345        

HOST = '192.168.4.1'
PORT = 12345     

if __name__ == "__main__":



    parser = argparse.ArgumentParser(description='Process some integers.')
    parser.add_argument('--mode', metavar='m', type=int,
                        help='0(default): tcp host, 1: replay data file')
    parser.add_argument('--recordpath', metavar='p', type=str,
                        help='path to recorded data file or directory to put recordings')

    args = parser.parse_args()

    root = tk.Tk()
    root.geometry("1280x900+0+0")
    client_socket = ClientSocket.ClientSocket(HOST, PORT)    
    sender = Sender.Sender(client_socket)

    plot_frame1 = tk.Frame(root)
    # plot_frame.pack_propagate(0)
    plot_frame1.pack(fill='both', side='top', expand='True')
    plot_frame2 = tk.Frame(root)
    # plot_frame.pack_propagate(0)
    plot_frame2.pack(fill='both', side='top', expand='True')

    x_plot = Plot.Plot(plot_frame1, "both", "left", "True")
    y_plot = Plot.Plot(plot_frame1, "both", "right", "True")
    z_plot = Plot.Plot(plot_frame2, "both", "left", "True")
    throttle_plot = Plot.Plot(plot_frame2, "both", "right", "True")

    config = Config.Config("config.ini")
    controller = Controller.Controller(root, "x", "bottom", "False", sender, config)

    plot_handler = PlotHandler.PlotHandler(x_plot, y_plot, z_plot, throttle_plot)
    recorder = Recorder.Recorder(args.recordpath)

    def d(event):
        plot_handler.redraw()
    root.bind('<Configure>', d)

    if args.mode == 0:
        recorder.start_recording()
        receiver = Receiver.Receiver(client_socket, plot_handler, recorder, controller)
    elif args.mode == 1:
        receiver = Receiver.Receiver(client_socket, plot_handler, None, controller)
        recorder.playback_recording(receiver, args.recordpath)
    else:
        print("unrecognized mode")

    tk.mainloop()           