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
import Controller

HOST = '192.168.2.116'
PORT_TX = 12346        
PORT_RX = 12345        

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Process some integers.')
    parser.add_argument('--mode', metavar='m', type=int,
                        help='0(default): tcp host, 1: replay data file')
    parser.add_argument('--recordpath', metavar='p', type=str,
                        help='path to recorded data file or directory to put recordings')

    args = parser.parse_args()

    tk = Tk()
    tk.geometry("1280x768+0+0")
    plot = Plot.Plot(tk)
    sender = Sender.Sender(HOST, PORT_TX)
    controller = Controller.Controller(tk, sender)

    plot_handler = PlotHandler.PlotHandler(plot)
    recorder = Recorder.Recorder(args.recordpath)

    def d(event):
        plot_handler.redraw()
    tk.bind('<Configure>', d)

    if args.mode == 0:
        recorder.start_recording()
        receiver = Receiver.Receiver(HOST, PORT_RX, plot_handler, recorder)
    elif args.mode == 1:
        receiver = Receiver.Receiver(plot_handler, None)
        recorder.playback_recording(receiver, args.recordpath)
    else:
        print("unrecognized mode")

    tk.mainloop()           
   
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