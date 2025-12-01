from GraspGen.common_utils.socket_communication import (
    NonBlockingJSONSender,
    BlockingJSONReceiver,
)

class GraspGenCommunication:
    def __init__(self, port_sender=9890, port_receiver=9891):
        self.sender = NonBlockingJSONSender(port=port_sender)
        self.receiever = BlockingJSONReceiver(port=port_receiver)
        
    def send_data(self, data):
        try:
            self.sender.send_data(data)
            response = self.receiever.capture_data()
            if response.get("message") == "Success":
                return response
            else:
                raise ValueError("GraspGen server failed.")
        except Exception as e:
            raise e

    def quit(self):
        self.sender.disconnect()
        self.receiever.disconnect()