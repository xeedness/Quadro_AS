import socket
from base64 import b85encode
from tkinter import Tk, Label, Frame, X, RIGHT, LEFT, RAISED, Button, OptionMenu, Entry, StringVar, IntVar
import MessageTypes
import Sender
import struct

class Controller(Frame):
    def __init__(self, root, fill, side, expand, sender: Sender):
        super().__init__(root, relief=RAISED, borderwidth=1, padx = 5, pady = 5)
        self.root = root
        self.sender = sender
        self.side = side
        self.fill = fill
        self.expand = expand
        self.initUI()

    def initUI(self):
        self.pack(fill=self.fill, side=self.side, expand=self.expand)

        self.send_btn = Button(self, text="Send", command=self.send)
        self.send_btn.pack(side=RIGHT, padx=5, pady=5)

        self.arg_encoding = StringVar(self)
        self.arg_encoding.set("int") 
        self.arg_menu = OptionMenu(self, self.arg_encoding, "int", "float", "hex")
        self.arg_menu.pack(side=RIGHT, padx=5, pady=5)
    
        self.args = StringVar(self)
        self.arg_input = Entry(self, textvariable=self.args)
        self.arg_input.pack(side=RIGHT, padx=5, pady=5)
        
        self.arg_input_label = Label(self, text="Arguments")
        self.arg_input_label.pack(side=RIGHT, padx=5, pady=5) 

        self.msg_type = IntVar(self)
        self.msg_type_input = Entry(self, textvariable=self.msg_type)
        self.msg_type_input.pack(side=RIGHT, padx=5, pady=5) 

        self.msg_type_label = Label(self, text="Type")
        self.msg_type_label.pack(side=RIGHT, padx=5, pady=5) 

        self.start_btn = Button(self, text="Start", command=self.start)
        self.start_btn.pack(side=LEFT, padx=5, pady=5)

        self.stop_btn = Button(self, text="Stop", command=self.stop)
        self.stop_btn.pack(side=LEFT, padx=5, pady=5)

    def start(self):
        self.send_msg(MessageTypes.START, None)

    def stop(self):
        self.send_msg(MessageTypes.STOP, None)

    def send(self):
        try:
            arg_list = self.args.get().split(" ")
            payload = bytearray()
            if(self.arg_encoding.get() == "int"):
                for arg in arg_list:
                    payload = payload + int(arg).to_bytes(4, "big")
            elif(self.arg_encoding.get() == "float"):
                for arg in arg_list:
                    payload= payload + struct.pack(">f", float(arg))
            else:
                payload = bytearray.fromhex(self.args.get().strip(" "))

            self.send_msg(self.msg_type.get(), payload)
        except Exception as e:
            print("Couldn't send inputs \""+ self.args.get() +"\". Reason: "+str(e))

    def send_msg(self, msg_type, payload: bytearray):
        if payload:
            self.sender.send(msg_type.to_bytes(1, "big") + payload)
        else:
            self.sender.send(msg_type.to_bytes(1, "big"))
        