import socket
import serial
import time

# === 글로브 연결 ===
ser = serial.Serial('COM5', 115200, timeout=1)
time.sleep(2)

# === 소켓 클라이언트 세팅 ===
server_ip = '127.0.0.1' #'10.136.96.1'  # ex) 192.168.1.5
server_port = 9999
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((server_ip, server_port))

try:
    while True:
        line = ser.readline().decode().strip()
        if line:  # 받은 줄이 비어있지 않으면
            print(f"Sending: {line}")
            client_socket.sendall(line.encode() + b'\n')  # 줄 단위로 보내기
        time.sleep(0.05)

except KeyboardInterrupt:
    pass

client_socket.close()
ser.close()
