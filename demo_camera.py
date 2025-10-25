import cv2
import socket, struct, pickle

def find_video_device():
    for i in range(5):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            print(f"Камера найдена: /dev/video{i}")
            return cap
        cap.release()
    print("Камера не найдена.")
    return None

cap = find_video_device()
if not cap:
    exit()

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind(('0.0.0.0', 9998))
sock.listen(1)
print("Ожидаю подключение...")

conn, addr = sock.accept()
print("Подключился клиент:", addr)

while True:
    ret, frame = cap.read()
    if not ret:
        break
    data = pickle.dumps(frame)
    conn.sendall(struct.pack(">L", len(data)) + data)

cap.release()
conn.close()
