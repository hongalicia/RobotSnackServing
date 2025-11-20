import serial
import serial.tools.list_ports
import re

baud_rate = 115200  # 根據實際設備設定波特率
com = None  # 全域變數，用於儲存 COM Port 物件

def find_target_port(TARGET_VID_PID):
    """尋找符合 VID:PID 的 COM Port"""
    ports = serial.tools.list_ports.comports()

    for port in ports:
        match = re.search(
            r'VID:PID=([\dA-F]+:[\dA-F]+)', port.hwid, re.IGNORECASE)
        if match:
            vid_pid = match.group(1)
            if vid_pid == TARGET_VID_PID:
                return port.device  # 回傳符合的 COM Port

    return None  # 沒有找到符合的設備


def com_port_init():
    global com
    # com_port = find_com_port(keyword_1, keyword_2)
    com_port = find_target_port("067B:2303")
    if com_port:
        # print(f"found '{com_port.title}' at {com_port}")
        print(f"found device on {com_port}")
        try:
            com = serial.Serial(com_port, baud_rate, timeout=1)
            if com.is_open:
                print(
                    f"successful open {com_port} and set baudrate {baud_rate}")
                return com  # 回傳com port資訊
                # com.close()
        except serial.SerialException as e:
            print(f"comport error: {e}")
    else:
        # print(f"not found '{device_keyword}' device")
        print(f"not found comport device")


def send_data(data):  # send to motion
    com.write(data)
    print(f"data sent: {data}")


def com_port():  # share com port
    return com


if __name__ == '__main__':
    com_port_init()
