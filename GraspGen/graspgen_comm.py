from GraspGen.common_utils.socket_communication import (
    NonBlockingJSONSender,
    BlockingJSONReceiver,
)

class GraspGenCommunication:
    def __init__(self, port_sender=9878, port_receiver=9879):
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