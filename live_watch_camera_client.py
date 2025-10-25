# local_stream_client.py
import cv2, socket, struct, pickle

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(('127.0.0.1', 9998))  # если SSH-туннель, укажи localhost
data = b""
payload_size = struct.calcsize(">L")

while True:
    while len(data) < payload_size:
        data += sock.recv(4096)
    packed_msg_size = data[:payload_size]
    data = data[payload_size:]
    msg_size = struct.unpack(">L", packed_msg_size)[0]
    while len(data) < msg_size:
        data += sock.recv(4096)
    frame_data = data[:msg_size]
    data = data[msg_size:]
    frame = pickle.loads(frame_data)
    cv2.imshow('Stream', frame)
    if cv2.waitKey(1) == 27:
        break

cv2.destroyAllWindows()
