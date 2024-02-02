import threading
import io
import socket
from queue import Queue
import json
import numpy as np
import base64
from PIL import Image
import time


class connecter(threading.Thread):
    def __init__(self):
        super().__init__()
        self.serv_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_address = ("0.0.0.0", 4090)
        self.serv_socket.bind(server_address)
        self._senderQueue = Queue()
        self._recvQueue = Queue()
        self._heldVelocity = 0

        self.sendDataEvent = threading.Event()

    def waitConnection(self):
        self.serv_socket.listen(1)
        self.connection, self.client_address = self.serv_socket.accept()
        print("connected with:", self.client_address)
    
    def sendData(self, velocity: int, frame, radar_data):
        data = {"v" : velocity, "f" : frame, "r": radar_data, "t": time.time()}
        self._senderQueue.put(data)
        self.sendDataEvent.set()
    
    def getVelocity(self):
        if not self._recvQueue.empty():
            self._heldVelocity = self._recvQueue.get()
            print("got vel")
        return self._heldVelocity


    def sender(self):
        packet_type = {"v": 0, "f": 1, "r": 2}
        while True:
            self.sendDataEvent.wait()
            data = self._senderQueue.get()
            im = Image.fromarray(data["f"])
            im_stream = io.BytesIO()
            im.save(im_stream, format="JPEG", quality=95)
            data["f"] = base64.b64encode(im_stream.getvalue()).decode("utf-8")
            data["r"] = data["r"].tolist()
            packet = json.dumps(data)
            self.connection.send(len(packet).to_bytes(4, byteorder="big"))
            self.connection.send(packet.encode())
            self.sendDataEvent.clear()

    def receiver(self):
        while True:
            data = self.connection.recv(4)
            self._recvQueue.put(int.from_bytes(data, byteorder="big"))
            time.sleep(0.025)
            
    def run(self):
        self.waitConnection()
        self._senderQueue.queue.clear()
        
        sndr = threading.Thread(target=self.sender)
        rcvr = threading.Thread(target=self.receiver)
        sndr.start()
        rcvr.start()
        sndr.join()
        rcvr.join()
        self.connection.close()
        self.serv_socket.close()