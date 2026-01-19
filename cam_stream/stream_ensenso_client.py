import cv2
import numpy as np
import zmq

SERVER_IP = "192.168.1.133"        # 換成 Server 那台的 IP
CONNECT_ADDR = f"tcp://{SERVER_IP}:9090"
TOPIC = b"ensenso_rgb"

def main():
    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.SUB)
    sock.connect(CONNECT_ADDR)
    sock.setsockopt(zmq.SUBSCRIBE, TOPIC)

    print(f"[CLIENT] Connected to {CONNECT_ADDR}, subscribing {TOPIC.decode()}")

    last_ts = None
    while True:
        topic, ts_b, jpg_b = sock.recv_multipart()
        ts = int(ts_b.decode())

        # decode JPEG
        jpg = np.frombuffer(jpg_b, dtype=np.uint8)
        frame = cv2.imdecode(jpg, cv2.IMREAD_COLOR)
        if frame is None:
            continue

        if last_ts is not None:
            latency_ms = (ts - last_ts)
        else:
            latency_ms = 0
        last_ts = ts
        cv2.imshow("Ensenso RGB Stream", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
