import numpy as np 
import time
import socket
import io 
import base64
import sys
import json
from PIL import Image
import threading
from queue import Queue

class CarlaClient(threading.Thread):
    def __init__(self, server: str, port:int):
        super().__init__()
        self.c_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_addr = (server, port)
        self._senderQueue = Queue()
        self._recvQueue = Queue()
        self.senderEvent = threading.Event()

    def connect(self):
        self.c_sock.connect(self.server_addr)

    def sendVelocity(self, velocity : int):
        self._senderQueue.put(velocity)
        self.senderEvent.set()

    def sender(self):
        while True:
            self.senderEvent.wait()
            data = self._senderQueue.get()
            self.c_sock.send(data.to_bytes(4, byteorder="big"))
            self.senderEvent.clear()
    
    def receiver(self):
        while True:
            to_read = self.c_sock.recv(4)
            data_length = int.from_bytes(to_read, byteorder="big")

            data = b''
            while len(data) < data_length:
                packet = self.c_sock.recv(min(data_length - len(data), 4096))
                if not packet:
                    raise RuntimeError("connection is broken")
                data += packet
            
            decoded = json.loads(data.decode("utf-8"))
            self._recvQueue.put(decoded)

    def getData(self):
        lastData = None
        if not self._recvQueue.empty():
           lastData = self._recvQueue.get()
           im_data = io.BytesIO(base64.b64decode(lastData["f"]))
           image = Image.open(im_data)
           lastData["f"] = np.array(image)
           image.close()
        return lastData

    def run(self):
        self.connect()
        recv = threading.Thread(target=self.receiver)
        sendr = threading.Thread(target=self.sender)
        recv.start()
        sendr.start()
        recv.join()
        sendr.join()

if __name__ == "__main__":
    import cv2
    con = CarlaClient("192.168.1.245", 4090)
    con.start()
    cv2.namedWindow("control view", cv2.WINDOW_AUTOSIZE)
    delays = []
    # main loop acc
    while True:
        if cv2.waitKey(1) == ord('q'):
            break
        data = con.getData()
        if data:
            cv2.imshow("control view", data["f"])
            delays.append(time.time() - data["t"])
            print("mean delay:", f"{int(np.mean(delays)*1e3)} ms", "jitter:", f"{int(np.std(delays)*1e3)} ms", end="\r")
            if len(delays) == 100:
                delays.clear()
            # metti il codice acc qua
            con.sendVelocity(130) # al posto di 130 metti la velocità target che vuoi dare al veicolo (deve essere un intero in km/h)
    con.join() 
