import socket
import threading
import time
from enum import Enum, auto
from dataclasses import dataclass


# ======================
# Enums
# ======================
class LidPos(Enum):
    UNKNOWN = auto()
    CLOSED = auto()
    OPEN = auto()


class WafflePos(Enum):
    UNKNOWN = auto()
    ON_UPPER_LID = auto()
    ON_LOWER_LID = auto()
    NONE = auto()


# ======================
# Data model
# ======================
@dataclass
class VisionResult:
    left_lid: LidPos
    left_waffle: WafflePos
    right_lid: LidPos
    right_waffle: WafflePos
    suggest_ldx: int | None
    suggest_ldy: int | None
    suggest_rdx: int | None
    suggest_rdy: int | None

    def __str__(self):
        return (
            "VisionResult(\n"
            f"  left : {self.left_lid.name}/{self.left_waffle.name}, "
            f"dx={self.suggest_ldx}, dy={self.suggest_ldy}\n"
            f"  right: {self.right_lid.name}/{self.right_waffle.name}, "
            f"dx={self.suggest_rdx}, dy={self.suggest_rdy}\n"
            ")"
        )


# ======================
# Parser
# ======================
def parse_vision_tuple(data):
    lc, ldx, ldy, rc, rdx, rdy = data

    left_dx = left_dy = None
    right_dx = right_dy = None

    # left
    if lc == 1:
        ll, lw = LidPos.CLOSED, WafflePos.UNKNOWN
    elif lc == 2:
        ll, lw = LidPos.OPEN, WafflePos.ON_UPPER_LID
    elif lc == 3:
        ll, lw = LidPos.OPEN, WafflePos.ON_LOWER_LID
        left_dx, left_dy = ldx, ldy
    elif lc == 4:
        ll, lw = LidPos.OPEN, WafflePos.NONE
    else:
        ll, lw = LidPos.UNKNOWN, WafflePos.UNKNOWN

    # right
    if rc == 5:
        rl, rw = LidPos.CLOSED, WafflePos.UNKNOWN
    elif rc == 6:
        rl, rw = LidPos.OPEN, WafflePos.ON_UPPER_LID
    elif rc == 7:
        rl, rw = LidPos.OPEN, WafflePos.ON_LOWER_LID
        right_dx, right_dy = rdx, rdy
    elif rc == 8:
        rl, rw = LidPos.OPEN, WafflePos.NONE
    else:
        rl, rw = LidPos.UNKNOWN, WafflePos.UNKNOWN

    return VisionResult(
        left_lid=ll,
        left_waffle=lw,
        right_lid=rl,
        right_waffle=rw,
        suggest_ldx=left_dx,
        suggest_ldy=left_dy,
        suggest_rdx=right_dx,
        suggest_rdy=right_dy,
    )


# ======================
# Client
# ======================
class LidClient:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.sock = None
        self.running = False
        self.recv_thread = None

        self._buf = ""
        self._lock = threading.Lock()
        self._latest: VisionResult | None = None

    def connect(self):
        """連線到 Server"""
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((self.host, self.port))
        self.running = True
        print(f"[INFO] Connected to {self.host}:{self.port}")

        # 啟動接收執行緒
        self.recv_thread = threading.Thread(
            target=self._recv_loop, daemon=True)
        self.recv_thread.start()

    def _recv_loop(self):
        while self.running:
            try:
                data = self.sock.recv(1024)
                if not data:
                    print("[INFO] Server disconnected")
                    with self._lock:
                        self._latest = None
                    break

                self._buf += data.decode()
                while "\n" in self._buf:
                    line, self._buf = self._buf.split("\n", 1)
                    parts = [float(x) for x in line.split(",")]
                    if len(parts) != 6:
                        continue

                    result = parse_vision_tuple(tuple(parts))
                    with self._lock:
                        self._latest = result

            except Exception as e:
                print(f"[ERROR] recv loop: {e}")
                break

        self.running = False
        self.sock.close()
        print("[INFO] VisionClient disconnected")

    def get_latest(self) -> VisionResult | None:
        with self._lock:
            return self._latest

    def close(self):
        self.running = False
        if self.sock:
            self.sock.close()


if __name__ == "__main__":
    client = LidClient("127.0.0.1", 9000)
    client.connect()

    try:
        while True:
            time.sleep(3.0)
            result = client.get_latest()
            if result:
                print(result, "\n")
                print(result.left_lid)
            else:
                print("vision not ready")
            time.sleep(3.0)
    except KeyboardInterrupt:
        client.close()