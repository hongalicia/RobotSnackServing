from ROS.socket_communication import (
    NonBlockingJSONSender,
    BlockingJSONReceiver,
)

class ROSCommunication:
    def __init__(self, port_sender=9893, port_receiver=9894):
        self.sender = NonBlockingJSONSender(port=port_sender)
        self.receiever = BlockingJSONReceiver(port=port_receiver)

    def send_data(self, data):
        try:
            self.sender.send_data(data)
            return self.receiever.capture_data()
        except Exception as e:
            raise e

    def quit(self):
        self.sender.disconnect()
        self.receiever.disconnect()