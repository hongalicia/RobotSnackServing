import cv2
import time
import threading
from flask import Flask, Response
import zmq
import numpy as np
from zed_utils import ZedCamera
import pyzed.sl as sl
app = Flask(__name__)

# =========================
# RTSP 設定
# =========================
RTSP_SOURCES = {
    "cam1": "rtsp://root:admin@169.254.183.33:554/snl/live/1/1",
    "cam2": "rtsp://root:admin@169.254.183.33:554/snl/live/2/1",
    "usb":  None,
    "ensenso": None,
    "cam4": None,
}

# =========================
# 全域狀態
# =========================
latest_frames = {}        # cam_id -> frame
frame_locks = {}          # cam_id -> Lock
running = True


# =========================
# RTSP 背景拉流 Thread
# =========================
def rtsp_worker(cam_id, url):
    print(f"[RTSP] start worker: {cam_id}")

    cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not cap.isOpened():
        print(f"[RTSP] ❌ open failed: {cam_id}")
        return

    while running:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.05)
            continue

        with frame_locks[cam_id]:
            latest_frames[cam_id] = frame

    cap.release()
    print(f"[RTSP] stop worker: {cam_id}")


def zed_worker():
    zed_cam = ZedCamera()
    ctx = zmq.Context.instance()
    pub = ctx.socket(zmq.PUB)
    pub.bind("tcp://*:9091")

    print("[ZED] worker started")

    while running:
        zed_cam.camera.grab()
        zed_cam.camera.retrieve_image(zed_cam.left_image, sl.VIEW.LEFT)

        frame_raw = zed_cam.left_image.get_data()[:, :, :3] 
        # frame_raw = cv2.cvtColor(frame_raw, cv2.COLOR_RGBA2BGR)

        # ===== 給 Flask UI=====
        h, w, _ = frame_raw.shape
        frame_ui = frame_raw[:, :w // 1] 

        with frame_locks["usb"]:
            latest_frames["usb"] = frame_ui

        ok, jpg = cv2.imencode(".jpg", frame_raw)
        if ok:
            pub.send_multipart([
                b"usb_cam",
                str(int(time.time() * 1000)).encode(),
                jpg.tobytes()
            ])

    zed_cam.camera.close()


def usb_worker():
    cap = cv2.VideoCapture(3, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    if not cap.isOpened():
        print("[USB] ❌ open failed")
        return

    ctx = zmq.Context.instance()
    pub = ctx.socket(zmq.PUB)
    pub.bind("tcp://*:9091")

    print("[USB] worker started")

    while running:
        ret, frame_raw = cap.read()
        if not ret:
            time.sleep(0.02)
            continue

        # ===== 給 Flask UI=====
        h, w, _ = frame_raw.shape
        frame_ui = frame_raw[:, :w // 2] 

        with frame_locks["usb"]:
            latest_frames["usb"] = frame_ui

        ok, jpg = cv2.imencode(".jpg", frame_raw)
        if ok:
            pub.send_multipart([
                b"usb_cam",
                str(int(time.time() * 1000)).encode(),
                jpg.tobytes()
            ])

    cap.release()


def ensenso_worker():
    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.SUB)
    sock.connect("tcp://192.168.1.133:9090")
    sock.setsockopt(zmq.SUBSCRIBE, b"ensenso_rgb")

    print("[ENSENSO] worker started")

    while running:
        try:
            topic, ts_b, jpg_b = sock.recv_multipart()
            jpg = np.frombuffer(jpg_b, dtype=np.uint8)
            frame = cv2.imdecode(jpg, cv2.IMREAD_COLOR)
            if frame is None:
                continue

            with frame_locks["ensenso"]:
                latest_frames["ensenso"] = frame

        except Exception as e:
            print("[ENSENSO] error:", e)
            time.sleep(0.1)

def zmq_annotated_worker():
    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.SUB)
    sock.connect("tcp://192.168.1.133:9999")

    # ⭐ 注意：一定要 bytes
    sock.setsockopt(zmq.SUBSCRIBE, b"annotated_frame")

    print("[ZMQ] annotated_frame worker started")

    while running:
        try:
            topic, data = sock.recv_multipart()

            if topic != b"annotated_frame":
                continue

            frame = cv2.imdecode(
                np.frombuffer(data, dtype=np.uint8),
                cv2.IMREAD_COLOR
            )
            if frame is None:
                continue

            with frame_locks["cam4"]:
                latest_frames["cam4"] = frame

        except Exception as e:
            print("[ZMQ] error:", e)
            time.sleep(0.1)

# =========================
# MJPEG Generator（送最新 frame）
# =========================
def gen_mjpeg(cam_id):
    while True:
        frame = None
        with frame_locks[cam_id]:
            frame = latest_frames.get(cam_id)

        if frame is None:
            time.sleep(0.01)
            continue

        ret, buffer = cv2.imencode(".jpg", frame, [
            int(cv2.IMWRITE_JPEG_QUALITY), 80
        ])
        if not ret:
            continue

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" +
            buffer.tobytes() +
            b"\r\n"
        )

        # 控制輸出 FPS（10~15 就很順）
        time.sleep(0.07)


# =========================
# Flask routes
# =========================
@app.route("/video_feed/<cam_id>")
def video_feed(cam_id):
    if cam_id not in RTSP_SOURCES:
        return "Unknown camera", 404
    return Response(
        gen_mjpeg(cam_id),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )


# =========================
# 啟動
# =========================
def start_rtsp_threads():
    for cam_id, url in RTSP_SOURCES.items():
        frame_locks[cam_id] = threading.Lock()
        if cam_id == "usb":
            # t = threading.Thread(target=usb_worker, daemon=True)
            # zed
            t = threading.Thread(target=zed_worker, daemon=True)
        elif cam_id == "cam4":
            t = threading.Thread(target=zmq_annotated_worker, daemon=True)
        else:
            t = threading.Thread(
                target=rtsp_worker,
                args=(cam_id, url),
                daemon=True
            )
        t.start()


if __name__ == "__main__":
    start_rtsp_threads()
    print("[Flask] server start")
    app.run(host="0.0.0.0", port=5000, threaded=True)
