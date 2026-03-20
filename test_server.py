import socket
import threading

# 根據你 TCP.py 的規則計算 Checksum (前 7 bytes 總和的 2補數)
def calculate_checksum(data_bytes):
    total = sum(data_bytes)
    return (0 - total) & 0xFF

# 建立 8-byte 的訂單封包
def create_order_packet(peanuts, waffles, table_num):
    # 🌟 加上 table_num，並放在 bytearray 的第 5 個位置 (index 5)
    packet = bytearray([136, 102, 79, peanuts, waffles, table_num, 0])
    checksum = calculate_checksum(packet)
    packet.append(checksum)
    return packet

# 負責持續接收機器人回傳的資訊 (例如剩餘時間或結束訊號)
def handle_robot_messages(client_socket):
    while True:
        try:
            data = client_socket.recv(8)
            if not data:
                break
            print(f"\n[收到機器人回傳] {' '.join(f'{b:02X}' for b in data)}")
            print("👉 輸入訂單 (花生 鬆餅): ", end="", flush=True) # 保持輸入提示還在
        except Exception:
            break

def main():
    host = '0.0.0.0' # 監聽本機所有網路介面
    port = 9000      # ⚠️ 這裡的 Port 必須和 MainWindow_ctrl.py 的設定一樣

    # 建立 TCP Server
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((host, port))
    server.listen(1)
    
    print(f"✅ 測試伺服器 (點餐機) 已啟動！正在監聽 Port {port}...")
    print("⏳ 等待機器人 (Client) 連線進來...")

    # 程式會停在這裡，直到你的 MainWindow_ctrl.py 啟動並連線過來
    client_socket, addr = server.accept()
    print(f"\n🎉 機器人已成功連線！來自 IP: {addr[0]}")

    # 啟動背景 Thread 接收機器人的回傳訊息
    recv_thread = threading.Thread(target=handle_robot_messages, args=(client_socket,), daemon=True)
    recv_thread.start()

    print("\n=== 模擬點餐發送系統 ===")
    print("請輸入「桌號 花生數量 鬆餅數量」，用空白隔開 (例如: 5 1 2，代表 5號桌)，按 Enter 發送")
    print("輸入 'q' 離開測試程式\n")

    while True:
        user_input = input("👉 輸入訂單 (桌號 花生 鬆餅): ")
        if user_input.lower() == 'q':
            break
        
        try:
            parts = user_input.split()
            if len(parts) != 3:
                print("⚠️ 格式錯誤！請輸入兩個數字，用空白隔開。")
                continue
            
            table = int(parts[0]) 
            peanuts = int(parts[1])
            waffles = int(parts[2])
            
            # 建立封包並發送給機器人
            packet = create_order_packet(peanuts, waffles, table)
            client_socket.sendall(packet)
            print(f"🚀 已發送訂單: 📍 桌號={table}, 🥜 花生={peanuts}, 🧇 鬆餅={waffles}")
            
        except ValueError:
            print("⚠️ 請輸入有效的數字！")
        except Exception as e:
            print(f"❌ 發送失敗: {e} (機器人可能斷線了)")
            break

    client_socket.close()
    server.close()
    print("伺服器已關閉。")

if __name__ == "__main__":
    main()