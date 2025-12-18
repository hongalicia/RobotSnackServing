import socket
import threading
import time

class MoveClient:
    def __init__(self, host: str, port: int):
        self.host = host
        self.port = port
        self.sock = None
        self.running = False
        self.recv_thread = None
        self.last_msg = "" 

    def connect(self):

        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(5)
            self.sock.connect((self.host, self.port))
            self.running = True
            
            self.recv_thread = threading.Thread(target=self._receive_loop, daemon=True)
            self.recv_thread.start()
            
            print(f"[INFO] Connected to {self.host}:{self.port}")
            return True
        except Exception as e:
            print(f"[ERROR] Connection failed: {e}")
            return False

    def _receive_loop(self):
        while self.running:
            try:
                data = self.sock.recv(1024)
                if not data:
                    break
                hex_data = ' '.join(f"{b:02X}" for b in data)
                self.last_msg = hex_data
                print(f"[RECV] {hex_data}")

            except Exception:
                break
        self.running = False
        print("[INFO] Receiver thread stopped.")

    def send(self, msg: str):
        if not self.running or self.sock is None:
            print("[INFO] Connection lost, trying to reconnect before sending...")
            self.connect() 
            time.sleep(0.2) 

        try:
            formatted_msg = f"{msg.strip()}\n".encode("utf-8")
            self.sock.sendall(formatted_msg)
            print(f"[SEND] {msg.strip()}")
        except Exception as e:
            print(f"[ERROR] Send failed even after reconnect: {e}")

    def close(self):
        self.running = False
        if self.sock:
            self.sock.close()
            self.sock = None
        print("[INFO] Disconnected.")