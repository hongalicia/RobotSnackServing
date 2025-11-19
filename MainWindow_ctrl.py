from PySide6.QtWidgets import QMainWindow
from PySide6.QtGui import QCloseEvent, QPixmap, QImage
from PySide6.QtCore import Qt

from MainWindow_ui import Ui_MainWindow
from Camera.camera import *
from GraspGen.graspgen_comm import *
from PeanutNumberClassification.PeanutNumClassification import *
from ROS.trajectory_parser import *
from ROS.ros_comm import *
import cv2

import json
import numpy as np

class main_window_ctrl(QMainWindow):
    def __init__(self):
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.ui_connect()

        self.init()

    # ---------------- UI 綁定 ----------------
    def ui_connect(self):
        # functions for peanuts
        self.ui.pushButton_CheckPeanuts.clicked.connect(self.pushButton_CheckPeanuts_clicked)
        
        self.ui.pushButton_GrabNDumpPeanuts.clicked.connect(self.pushButton_GrabNDumpPeanuts_clicked)

        self.ui.pushButton_PressButton.clicked.connect(self.press_button)
        self.ui.pushButton_GetSpoon.clicked.connect(self.get_spoon)
        self.ui.pushButton_SpoonPeanuts.clicked.connect(self.spoon_peanuts)
        self.ui.pushButton_DropSpoon.clicked.connect(self.drop_spoon)

        # functions for waffle
        self.ui.pushButton_GrabBatterNPour.clicked.connect(self.pushButton_GrabBatterNPour_clicked)
        self.ui.pushButton_OpenLid.clicked.connect(self.pushButton_OpenLid_clicked)
        self.ui.pushButton_CloseLid.clicked.connect(self.pushButton_CloseLid_clicked)
        self.ui.pushButton_GetCookedWaffle.clicked.connect(self.pushButton_GetCookedWaffle_clicked)        

    def init(self):
        if 'self.PeanutNumClassifier' not in globals():
            try:
                self.PeanutNumClassification_init()
            except Exception as e:
                self.ui.textEdit_status.append(f"PeanutNumClassification_init error: {e}\n")
                return
            
        if 'self.GraspGenCommunication' not in globals():
            try:
                self.GraspGenCommunication_init()
            except Exception as e:
                self.ui.textEdit_status.append(f"GraspGenCommunication_init error: {e}\n")
                return
        
        if 'self.rosCommunication' not in globals():
            try:
                self.ros_init()
            except Exception as e:
                self.ui.textEdit_status.append(f"ros_init error: {e}\n")
                return
            
        if 'self.cam' not in globals():
            try:
                self.cam_init()
                # frame = self.cam.capture_single(2)
                # cv2.imwrite('test.png', frame)
            except Exception as e:
                self.ui.textEdit_status.append(f"cam_init error: {e}\n")
                return                

    def cam_init(self):
        self.cam = Camera()
        self.cam.cam_init([2])

    def PeanutNumClassification_init(self):
        try:
            self.peanutNumClassifier = PeanutNumClassification()  # Initialize the classifier
        except Exception as e:
            self.ui.textEdit_status.append(f"PeanutNumClassification_init error: {e}\n")

    def GraspGenCommunication_init(self):
        try:
            self.graspGenCommunication = GraspGenCommunication(port_sender=9890, port_receiver=9891)
        except Exception as e:
            self.ui.textEdit_status.append(f"GraspGenCommunication_init error: {e}\n")

    def GraspGenCommunication_destroy(self):
        try:
            self.graspGenCommunication.quit()
        except Exception as e:
            self.ui.textEdit_status.append(f"GraspGenCommunication_destroy error: {e}\n")

    def ros_init(self):        
        try:
            self.rosCommunication = ROSCommunication()
        except Exception as e:
            self.ui.textEdit_status.append(f"ros_init error: {e}\n")

    def ros_destroy(self):
        try:
            self.rosCommunication.quit()
        except Exception as e:
            self.ui.textEdit_status.append(f"ros_destroy error: {e}\n")
        
    def pushButton_CheckPeanuts_clicked(self):
        # output = self.check_peanuts(save_image=True)
        output = self.check_peanuts()
        # output = self.check_peanuts('PeanutNumberClassification/operating-sub.png') only use when debugging
        self.ui.textEdit_status.append(f"check_peanuts output: {output}\n")

    def check_peanuts(self, image_path=None, save_image = False):       
        try:
            if image_path is not None:
                image = cv2.imread(image_path)
            else:
                image = self.cam.capture_single(2)

            file = open('PeanutNumberClassification/roi.json', 'r')
            self.roi = json.loads(file.read())["roi"] # x, y, w, h
            if self.roi[2] != 0 and self.roi[3] != 0: # with & height
                right = (self.roi[0] + self.roi[2])
                bottom = (self.roi[1] + self.roi[3])
                image = image[self.roi[1]:bottom, self.roi[0]:right]

            # show image
            image_showed = np.ascontiguousarray(image)
            if save_image:
                cv2.imwrite(f'PeanutNumberClassification/dataset/peanuts_image{int(time.time())}.png', image_showed)
            peanuts_image = QImage(image_showed, image_showed.shape[1], image_showed.shape[0], 3 * image_showed.shape[1], QImage.Format.Format_BGR888)
            peanuts_pixmap = QPixmap.fromImage(peanuts_image)    
            peanuts_pixmap_scaled = peanuts_pixmap.scaled(self.ui.label_image_peanuts.width(), self.ui.label_image_peanuts.height(), aspectMode=Qt.AspectRatioMode.KeepAspectRatio)
            self.ui.label_image_peanuts.setPixmap(peanuts_pixmap_scaled)

            output = self.peanutNumClassifier.classify(image)
            return output
        except Exception as e:
            self.ui.textEdit_status.append(f"check_peanuts error: {e}\n")

    def pushButton_GrabNDumpPeanuts_clicked(self):
         try:
            data = {"SAY": "HELLO"}
            message = self.graspGenCommunication.send_data(data)
            self.ui.textEdit_status.append(f"graspGenCommunication return message: {message}\n")
         except Exception as e:
            self.ui.textEdit_status.append(f"pushButton_GrabNDumpPeanuts_clicked error: {e}\n")

    def run_trajectory(self, filename):
        try:
            # load nodes
            nodes = load_trajectory_from_csv(filename)
            parsed_nodes = []
            print("Number of nodes:", len(nodes))
            last_is_move = False
            for i, node in enumerate(nodes):
                if node.mode == Mode.MOVE:
                    if last_is_move:
                        parsed_nodes[-1].joints_values.append(list(np.deg2rad(node.joint_value)))
                    else:
                        parsed_nodes.append(node)
                        parsed_nodes[-1].joints_values.append(list(np.deg2rad(node.joint_value)))
                        last_is_move = True
                else:
                    last_is_move = False
                    parsed_nodes.append(node)
            nodes = parsed_nodes
            # append positions
            for node in nodes:
                print("mode:", node.mode, "joint_value:", node.joint_value)
                if node.mode == Mode.OPEN:
                    # self.rosCommunication.open_gripper() 
                    self.rosCommunication.send_data({"type": "gripper", "grip_type": "open", "wait_time": 3.0})
                elif node.mode == Mode.CLOSE:
                    # self.rosCommunication.close_gripper() 
                    self.rosCommunication.send_data({"type": "gripper", "grip_type": "close", "wait_time": 1.5})
                elif node.mode == Mode.HALF_OPEN:
                    self.rosCommunication.send_data({"type": "gripper", "grip_type": "half_open", "wait_time": 1.5})
                elif node.mode == Mode.CLOSE_TIGHT:
                    self.rosCommunication.send_data({"type": "gripper", "grip_type": "close_tight", "wait_time": 1.5})
                elif node.mode == Mode.MOVE:
                    # self.rosCommunication.append_joints(node.joint_value, block=False)     
                    self.rosCommunication.send_data({"type": "arm", "joints_values": node.joints_values, "wait_time": 0.0})

        except Exception as e:
            raise e

    def press_button(self):
        try:
            self.run_trajectory("ROS/trajectories/press_button.csv")
        except Exception as e:
            self.ui.textEdit_status.append(f"press_button error: {e}\n")

    def get_spoon(self):
        try:
            # status_peanuts = self.check_peanuts()
            # self.ui.textEdit_status.append(f"check_peanuts: {status_peanuts}\n")
            # if status_peanuts == 'insufficient' or status_peanuts == 'operating':
            #     return

            self.run_trajectory("ROS/trajectories/get_spoon.csv")
        except Exception as e:
            self.ui.textEdit_status.append(f"get_spoon error: {e}\n")

    def spoon_peanuts(self):
        try:
            # status_peanuts = self.check_peanuts()
            # self.ui.textEdit_status.append(f"check_peanuts: {status_peanuts}\n")
            # if status_peanuts == 'insufficient' or status_peanuts == 'operating':
            #     return

            self.run_trajectory("ROS/trajectories/spoon_peanuts.csv")
        except Exception as e:
            self.ui.textEdit_status.append(f"spoon_peanuts error: {e}\n")

    def drop_spoon(self):
        try:
            self.run_trajectory("ROS/trajectories/drop_spoon.csv")
        except Exception as e:
            self.ui.textEdit_status.append(f"drop_spoon error: {e}\n")

    def pushButton_GrabBatterNPour_clicked(self):
        try:
            data = {}
            message = self.graspGenCommunication.send_data(data)
            self.ui.textEdit_status.append(f"graspGenCommunication return message: {message}\n")
        except Exception as e:
            self.ui.textEdit_status.append(f"pushButton_GrabNDumpPeanuts_clicked error: {e}\n")

    def pushButton_OpenLid_clicked(self):
        try:
            self.run_trajectory("ROS/trajectories/open_lid.csv")
        except Exception as e:
            self.ui.textEdit_status.append(f"open_lid error: {e}\n")

    def pushButton_CloseLid_clicked(self):
        try:
            self.run_trajectory("ROS/trajectories/close_lid.csv")
        except Exception as e:
            self.ui.textEdit_status.append(f"close_lid error: {e}\n")

    def pushButton_GetCookedWaffle_clicked(self):
        try:
            self.run_trajectory("ROS/trajectories/get_cooked_waffle.csv")
        except Exception as e:
            self.ui.textEdit_status.append(f"get_cooked_waffle error: {e}\n")

    def closeEvent(self, event: QCloseEvent):
        if self.cam:
            self.cam.quit()

        if self.graspGenCommunication:
            self.GraspGenCommunication_destroy()

        if self.rosCommunication:
            self.ros_destroy()

