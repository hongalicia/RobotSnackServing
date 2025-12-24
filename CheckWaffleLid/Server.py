import socket
import threading
import time
from typing import Optional, Tuple


CMD_LID = 0x66   # 'f'
QUERY   = 0xFF   # query flag


class TCPServer:
    def __init__(self, host: str = "0.0.0.0", port: int = 9000):
        self.host = host
        self.port = port

        # 建立 TCP server socket
        self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_sock.bind((self.host, self.port))
        self.server_sock.listen(5)

        self.server_sock.settimeout(1.0)

        self.is_running = False

        self._current_conn: Optional[socket.socket] = None
        self._current_addr: Optional[Tuple[str, int]] = None
        self._conn_lock = threading.Lock()

        # ===== fake state =====
        self.lid_state = {
            1: False,   # False=open, True=closed
            2: True
        }

        print(f"[INIT] Server listening on {self.host}:{self.port}")

    # ========= 對外 API =========

    def start(self) -> None:
        self.is_running = True
        print("[START] TCPServer is running...")

        try:
            while self.is_running:
                try:
                    conn, addr = self.server_sock.accept()
                except socket.timeout:
                    continue
                except OSError:
                    break

                print(f"[CONNECT] Client connected from {addr}")

                with self._conn_lock:
                    self._current_conn = conn
                    self._current_addr = addr

                t = threading.Thread(
                    target=self.handle_client,
                    args=(conn, addr),
                    daemon=True
                )
                t.start()

        except KeyboardInterrupt:
            print("\n[STOP] KeyboardInterrupt, shutting down...")
        finally:
            self.close()

    def close(self) -> None:
        if self.is_running:
            self.is_running = False

        with self._conn_lock:
            if self._current_conn is not None:
                try:
                    self._current_conn.close()
                except OSError:
                    pass
                self._current_conn = None
                self._current_addr = None

        try:
            self.server_sock.close()
        except OSError:
            pass

        print("[CLOSE] Server socket closed.")

    # ========= client thread =========

    def handle_client(self, conn: socket.socket, addr: Tuple[str, int]) -> None:
        with conn:
            while True:
                try:
                    data = conn.recv(8)
                except (ConnectionResetError, OSError):
                    print(f"[ERROR] Connection lost from {addr}")
                    break

                if not data:
                    print(f"[DISCONNECT] Client {addr} closed connection")
                    break

                print("[RECV]", ' '.join(f"{b:02X}" for b in data))
                self.data_parse(data)

        with self._conn_lock:
            if self._current_conn is conn:
                self._current_conn = None
                self._current_addr = None

        print(f"[END] Handle thread for {addr} exited.")

    # ========= protocol parse =========

    def data_parse(self, data: bytes) -> None:
        if len(data) != 8:
            print("[PARSE] Invalid length")
            return

        if data[0] != 0x88 or data[1] != 0x66:
            print("[PARSE] Invalid header")
            return

        if not self.sum_check(data):
            print("[PARSE] Checksum invalid")
            return

        cmd = data[2]

        # ---------- Lid query ----------
        if cmd == CMD_LID:
            lid_id = data[3]
            state  = data[4]

            if state == QUERY:
                print(f"[QUERY] Lid {lid_id}")
                self.reply_lid_status(lid_id)
            else:
                print(f"[PARSE] Unexpected lid state: {state}")

        else:
            print(f"[PARSE] Unknown command: 0x{cmd:02X}")

    # ========= reply =========

    def reply_lid_status(self, lid_id: int) -> None:
        is_closed = self.lid_state.get(lid_id, False)
        state = 1 if is_closed else 0

        base = bytes([0x88, 0x66, CMD_LID, lid_id, state, 0x00, 0x00])
        self.check_sum_and_send(base)

        print(f"[REPLY] Lid {lid_id} =", "CLOSED" if is_closed else "OPEN")

    # ========= checksum =========

    def sum_check(self, data: bytes) -> bool:
        return (sum(data) & 0xFF) == 0

    def check_sum_and_send(self, data: bytes) -> None:
        if len(data) != 7:
            print("[CHECKSUM] Invalid length")
            return

        checksum = (-sum(data)) & 0xFF
        packet = data + bytes([checksum])
        self.send_data(packet)

    def send_data(self, data: bytes) -> None:
        with self._conn_lock:
            conn = self._current_conn

        if conn is None:
            print("[SEND] No client connected")
            return

        try:
            conn.sendall(data)
            print("[SEND]", ' '.join(f"{b:02X}" for b in data))
        except OSError as e:
            print(f"[SEND ERROR] {e}")


if __name__ == "__main__":
    server = TCPServer(host="0.0.0.0", port=9000)
    server.start()
