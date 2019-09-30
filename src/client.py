"""
File:       client.py
Author:     Patrick Bertsch
Content:    Implement TCP/IP communication to robot
"""
import socket


HOST = '127.0.0.1'
PORT = 65432
msg = "Hello world"

# Open connection using context manager to omit explicitly calling close()
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    s.sendall(bytes(msg))
    data = s.recv(1024)

print("Received:", repr(data))
