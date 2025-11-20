import threading
import time
import UART


class Wok(threading.Thread):
    def __init__(self):
        super().__init__()
        self._stop_event = threading.Event()  # 用於停止執行緒的旗標
        self._init_ = UART.com_port_init()

    def run(self):
        while not self._stop_event.is_set():
            if UART.com_port().in_waiting > 0:
                recv_data = UART.com_port().read(UART.com_port().in_waiting)
                data_array = list(recv_data)
                if len(data_array) < 8:
                    if data_array[0] == 136:  # 0x88
                        new_array = data_array
                    else:
                        new_array.extend(data_array)
                        self.data_parse(new_array)
                else:
                    self.data_parse(data_array)

            time.sleep(0.01)

    def flip(self):
        data = b'\x88\x66\x57\x00\x00\x00\x00'
        self.check_sum(data)

    def home(self):
        data = b'\x88\x66\x57\x01\x00\x00\x00'
        self.check_sum(data)

    def down(self):
        data = b'\x88\x66\x57\x02\x00\x00\x00'
        self.check_sum(data)

    def AC(self, on_off):
        data = b'\x88\x66\x41' + bytes([on_off]) + b'\x00\x00\x00'
        self.check_sum(data)

    def check_sum(self, data):
        data_array = list(data)
        sum = 0
        for i in range(7):
            sum = sum+data_array[i]
        check_sum = (0-sum) & 0xff
        data += bytes([check_sum])
        UART.send_data(data)

    def data_parse(self, data):
        print(f"recv_data:{data}")
        if data[0] == 136 and data[1] == 102:  # 0x88 & 0x66
            match data[2]:
                case 77:  # 'w'
                    match data[3]:
                        case 1:
                            print("wok home done")
                        case 2:
                            print("wok down done")


if __name__ == "__main__":
    wok = Wok()
    wok.flip()
    time.sleep(5)
    wok.home()
    time.sleep(3)
    wok.down()
    time.sleep(1)
