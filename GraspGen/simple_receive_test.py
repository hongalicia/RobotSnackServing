
import time
import sys
import os
current_file_dir = os.path.dirname(os.path.abspath(__file__))
project_root_dir = os.path.dirname(current_file_dir)
sys.path.insert(0, project_root_dir) 
from GraspGen.common_utils.socket_communication import (
    NonBlockingJSONSender,
    NonBlockingJSONReceiver,
)

sender = NonBlockingJSONSender(port=9879)
receiever = NonBlockingJSONReceiver(port=9878)

try:
    while True:
        time.sleep(0.1)
        data = receiever.capture_data()
        if data is None:
            continue
        print(data)
        sender.send_data({"message": "Success!!!"})
finally:
    sender.disconnect()
    receiever.disconnect()