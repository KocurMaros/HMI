import numpy as np
import socket


class UDP_server:
    def __init__(self):
        self.data_to_send = np.zeros((75, 3), dtype=float).tobytes()
        self.UDP_IP = "127.0.0.1"
        self.UDP_PORT = 23432
        self.data_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def set_data(self, data):
        self.data_to_send = data

    def send_message(self):
        self.data_socket.sendto(self.data_to_send, (self.UDP_IP, self.UDP_PORT))