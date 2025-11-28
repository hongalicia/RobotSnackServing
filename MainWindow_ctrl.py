from PySide6.QtWidgets import QMainWindow
from PySide6.QtGui import QCloseEvent, QPixmap, QImage
from PySide6.QtCore import Qt

from MainWindow_ui import Ui_MainWindow
from Camera.camera import *
from GraspGen.graspgen_comm import *
from PeanutNumberClassification.PeanutNumClassification import *
from ROS.trajectory_parser import *
from ROS.ros_comm import *
from Uart.Wok import *
from TCP.TCP import *

import cv2
import json
import numpy as np
from enum import Enum
import queue
import threading

class PAN_POS(Enum):
    HOME = 1
    DOWN = 2
    UNKNOWN = 3

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
        self.ui.pushButton_SpoonPeanuts.clicked.connect(self.pushButton_SpoonPeanuts_clicked)
        self.ui.pushButton_DropSpoon.clicked.connect(self.drop_spoon)
        self.ui.pushButton_ServePeanuts.clicked.connect(self.pushButton_ServePeanuts_clicked)

        self.ui.pushButton_WokHome.clicked.connect(self.pan_home)
        self.ui.pushButton_WokDown.clicked.connect(self.pan_down)
        self.ui.pushButton_WokFlip.clicked.connect(self.pan_flip)
        self.ui.pushButton_WokAC.clicked.connect(self.AC)

        # functions for waffle
        self.ui.pushButton_GrabBatterNPour.clicked.connect(self.pushButton_GrabBatterNPour_clicked)
        self.ui.pushButton_Open1stLid.clicked.connect(self.pushButton_Open1stLid_clicked)
        self.ui.pushButton_Grab1stBatter.clicked.connect(self.pushButton_Grab1stBatter_clicked)
        self.ui.pushButton_Pour1stBatter.clicked.connect(self.pushButton_Pour1stBatter_clicked)
        self.ui.pushButton_Drop1stBatter.clicked.connect(self.pushButton_Drop1stBatter_clicked)
        self.ui.pushButton_Close1stLid.clicked.connect(self.pushButton_Close1stLid_clicked)
        self.ui.pushButton_Get1stWaffle.clicked.connect(self.pushButton_Get1stWaffle_clicked)  
        self.ui.pushButton_Open2ndLid.clicked.connect(self.pushButton_Open2ndLid_clicked)
        self.ui.pushButton_Grab2ndBatter.clicked.connect(self.pushButton_Grab2ndBatter_clicked)
        self.ui.pushButton_Pour2ndBatter.clicked.connect(self.pushButton_Pour2ndBatter_clicked)
        self.ui.pushButton_Drop2ndBatter.clicked.connect(self.pushButton_Drop2ndBatter_clicked)
        self.ui.pushButton_Close2ndLid.clicked.connect(self.pushButton_Close2ndLid_clicked)
        self.ui.pushButton_Get2ndWaffle.clicked.connect(self.pushButton_Get2ndWaffle_clicked)    
        self.ui.pushButton_GrabFork.clicked.connect(self.pushButton_GrabFork_clicked)
        self.ui.pushButton_DropFork.clicked.connect(self.pushButton_DropFork_clicked)  
        self.ui.pushButton_DropWaffle.clicked.connect(self.pushButton_DropWaffle_clicked)
        self.ui.pushButton_ServeWaffle.clicked.connect(self.pushButton_ServeWaffle_clicked)
        self.ui.pushButton_GoToDefault.clicked.connect(self.pushButton_GoToDefault_clicked)

    def init(self):
        self.serving_orders = True
        self.left_seconds = 0

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

        if 'self.wok' not in globals():
            try:
                self.wok_init()
            except Exception as e:
                self.ui.textEdit_status.append(f"wok_init error: {e}\n")
                return  

        if 'self.tcp' not in globals():
            try:
                self.tcp_init()
            except Exception as e:
                self.ui.textEdit_status.append(f"tcp_init error: {e}\n")
                return         

        self.cooked_waffle = 0
        self.grabbing_spoon = False
        self.grabbing_fork = False
        self.on_off = False          

        self.time_peanut_spoon = 20
        self.time_peanut_drop = 35
        self.time_peanut_heat = 150
        self.time_peanut_flip = 10
        self.time_peanut_grab_spoon = 10
        self.time_peanut_drop_spoon = 5

        self.time_waffle_pour = 30
        self.time_waffle_heat = 120
        self.time_waffle_serve = 30
        self.time_waffle_2nd_stove = 30

    def tcp_init(self):
        self.tcp = TcpClient()
        self.tcp.connect()
        self.thread_processing_orders = threading.Thread(target=self.serve_orders)
        self.thread_processing_orders.start() 

        self.thread_counting_left_time = threading.Thread(target=self.count_left_time)
        self.thread_counting_left_time.start()

    def count_left_time(self):
        while self.tcp.running:
            if self.serve_orders == False:
                break

            self.tcp.left_time(self.left_seconds // 60, self.left_seconds % 60)
            if self.left_seconds > 0:
                self.left_seconds -= 1

            time.sleep(1)

    def wok_init(self):
        self.wok = Wok()
        self.wok.down()
        self.pan_position = PAN_POS.DOWN
        #self.wok.pan_position.connect(self.pan_position_received)
        self.thread_pan_position_check = threading.Thread(target=self.receive_pan_position)
        self.thread_pan_position_check.start()            

    def receive_pan_position(self):
        while self.serving_orders == True:
            if self.wok.received_status.empty() == False:                
                position = self.wok.received_status.get()
                self.pan_position_received(position)
            time.sleep(0.01)

    def pan_position_received(self, position):
        print("receive_pan_position: ", position)
        if position == "home":            
            self.pan_position = PAN_POS.HOME
        elif position == "down":
            self.pan_position = PAN_POS.DOWN

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
            self.graspGenCommunication = GraspGenCommunication()
        except Exception as e:
            self.ui.textEdit_status.append(f"GraspGenCommunication_init error: {e}\n")

    def GraspGenCommunication_destroy(self):
        try:
            self.graspGenCommunication.quit()
            print("GraspGenCommunication_destroy done.")
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
            print("ros_destroy done.")
        except Exception as e:
            self.ui.textEdit_status.append(f"ros_destroy error: {e}\n")
        
    def check_pan_pos(self, position):
        if position == PAN_POS.HOME:
            if self.pan_position != PAN_POS.HOME:                
                self.wok.home()
                print("Wait for home.")
                st_time = time.time()
                while self.pan_position != PAN_POS.HOME:
                    time.sleep(0.1)
                    if time.time() - st_time > 5:
                        raise Exception("Error: Pan returning home failed.")
        elif position == PAN_POS.DOWN:
            if self.pan_position != PAN_POS.DOWN:                
                self.wok.down()
                print("Wait for down.")
                st_time = time.time()
                while self.pan_position != PAN_POS.DOWN:
                    time.sleep(0.1)
                    if time.time() - st_time > 5:
                        raise Exception("Error: Pan returning down failed.")
                    
    def pushButton_CheckPeanuts_clicked(self):
        try:
            # output = self.check_peanuts(save_image=True)
            output = self.check_peanuts()
            # output = self.check_peanuts('PeanutNumberClassification/operating-sub.png') only use when debugging
            self.ui.textEdit_status.append(f"check_peanuts output: {output}\n")
        except Exception as e:
            self.ui.textEdit_status.append(f"check_peanuts error: {e}\n")

    def check_peanuts(self, image_path=None, save_image = False):       
        try:
            self.check_pan_pos(PAN_POS.HOME)

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
            self.check_pan_pos(PAN_POS.DOWN)

            if self.grabbing_spoon == True:
                self.drop_spoon()

            data = {"actions": "Grasp_and_Dump"}
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
            self.run_trajectory("ROS/trajectories/press_button1.csv")
            self.run_trajectory("ROS/trajectories/press_button2.csv")
        except Exception as e:
            self.ui.textEdit_status.append(f"press_button error: {e}\n")

    def get_spoon(self):
        try:
            # status_peanuts = self.check_peanuts()
            # self.ui.textEdit_status.append(f"check_peanuts: {status_peanuts}\n")
            # if status_peanuts == 'insufficient' or status_peanuts == 'operating':
            #     return

            self.run_trajectory("ROS/trajectories/get_spoon.csv")
            self.grabbing_spoon = True
        except Exception as e:
            self.ui.textEdit_status.append(f"get_spoon error: {e}\n")

    def pushButton_SpoonPeanuts_clicked(self):
        try:
            self.check_pan_pos(PAN_POS.HOME)
            #peanuts_status = self.check_peanuts()
            #if peanuts_status == 'sufficient':
            self.spoon_single_peanuts()
            #else:
            #    self.ui.textEdit_status.append(f"spoon_peanuts insufficient.")
        except Exception as e:
            self.ui.textEdit_status.append(f"spoon_peanuts error: {e}\n")

    def spoon_single_peanuts(self):
        try:
            self.run_trajectory("ROS/trajectories/spoon_peanuts.csv")
        except Exception as e:
            raise e

    def spoon_peanuts(self):
        # check amount of peanuts, dump peanuts if insufficient
        wait_for_flip_done = False
        status_peanuts = self.check_peanuts()
        self.ui.textEdit_status.append(f"Check Peanuts Amount: {status_peanuts}\n")
        while status_peanuts == 'operating':
            time.sleep(0.01)
            status_peanuts = self.check_peanuts()
        while status_peanuts != 'sufficient':
            self.ui.textEdit_status.append(f"Refilling Peanuts...\n")
            self.left_seconds += self.time_peanut_drop
            #self.pushButton_GrabNDumpPeanuts_clicked(self)
            status_peanuts = self.check_peanuts()
            if status_peanuts == 'sufficient':
                wait_for_flip_done = True   
                self.ui.textEdit_status.append(f"Peanuts refilled.\n")
                break
            time.sleep(1)
            self.left_seconds += 1

        # press button
        if wait_for_flip_done == True:
            if self.grabbing_spoon == True:
                self.ui.textEdit_status.append(f"Dropping spoon...\n")
                self.drop_spoon()
                self.left_seconds += self.time_peanut_drop_spoon
                self.ui.textEdit_status.append(f"Spoon dropped.\n")
            self.pan_position = PAN_POS.DOWN
            self.ui.textEdit_status.append(f"Pressing button...\n")
            self.press_button()
            self.left_seconds += self.time_peanut_heat
            self.ui.textEdit_status.append(f"Button pressed.\n")

        # get spoon
        if self.grabbing_spoon == False:
            self.ui.textEdit_status.append(f"Grabbing spoon...\n")
            self.get_spoon()
            self.ui.textEdit_status.append(f"Grab spoon done.\n")

        # check pan position
        while self.pan_position != PAN_POS.HOME:
            time.sleep(0.01)

        # spoon peanuts              
        self.spoon_single_peanuts()

    def drop_spoon(self):
        try:
            self.run_trajectory("ROS/trajectories/drop_spoon.csv")
            self.grabbing_spoon = False
        except Exception as e:
            self.ui.textEdit_status.append(f"drop_spoon error: {e}\n")

    def pushButton_ServePeanuts_clicked(self):
        try:
            if self.ui.lineEdit_NumOfPeanuts.text():
                new_order = order(int(self.ui.lineEdit_NumOfPeanuts.text()), 0)
                self.tcp.received_orders.put(new_order)
            else:
                self.ui.textEdit_status.append(f"Num of Peanuts is empty.\n")      
        except Exception as e:
            self.ui.textEdit_status.append(f"Serve Peanuts error: {e}\n")

    def serve_peanuts(self, num_peanuts):
        try:
            count = 1
            while count <= num_peanuts:                
                self.spoon_peanuts()
                self.ui.textEdit_status.append(f"Spoon peanuts: {count}.\n")             
                count += 1         
            
            self.ui.textEdit_status.append(f"Serve Peanuts done.\n")
        except Exception as e:
            self.ui.textEdit_status.append(f"Spoon peanuts error: {e}.\n") 

    def pushButton_GrabBatterNPour_clicked(self):
        try:
            if self.grabbing_spoon == True:
                self.drop_spoon()

            data = {}
            message = self.graspGenCommunication.send_data(data)
            self.ui.textEdit_status.append(f"graspGenCommunication return message: {message}\n")
        except Exception as e:
            self.ui.textEdit_status.append(f"pushButton_GrabNDumpPeanuts_clicked error: {e}\n")

    def pushButton_Open1stLid_clicked(self):
        try:
            if self.grabbing_spoon == True:
                self.drop_spoon()

            self.run_trajectory("ROS/trajectories/open_1st_lid.csv")
        except Exception as e:
            self.ui.textEdit_status.append(f"open_1st_lid error: {e}\n")

    def pushButton_Open2ndLid_clicked(self):
        try:
            if self.grabbing_spoon == True:
                self.drop_spoon()

            self.run_trajectory("ROS/trajectories/open_2nd_lid.csv")
        except Exception as e:
            self.ui.textEdit_status.append(f"open_2nd_lid error: {e}\n")

    def pushButton_Grab1stBatter_clicked(self):
        try:
            if self.grabbing_spoon == True:
                self.drop_spoon()

            self.run_trajectory("ROS/trajectories/grab_1st_batter.csv")
        except Exception as e:
            self.ui.textEdit_status.append(f"grab_1st_batter error: {e}\n")

    def pushButton_Grab2ndBatter_clicked(self):
        try:
            if self.grabbing_spoon == True:
                self.drop_spoon()

            self.run_trajectory("ROS/trajectories/grab_2nd_batter.csv")
        except Exception as e:
            self.ui.textEdit_status.append(f"grab_2nd_batter error: {e}\n")

    def pushButton_Pour1stBatter_clicked(self):
        try:
            if self.grabbing_spoon == True:
                self.drop_spoon()

            self.run_trajectory("ROS/trajectories/pour_1st_batter.csv")
        except Exception as e:
            self.ui.textEdit_status.append(f"pour_1st_batter error: {e}\n")

    def pushButton_Pour2ndBatter_clicked(self):
        try:
            if self.grabbing_spoon == True:
                self.drop_spoon()

            self.run_trajectory("ROS/trajectories/pour_2nd_batter.csv")
        except Exception as e:
            self.ui.textEdit_status.append(f"pour_2nd_batter error: {e}\n")

    def pushButton_Drop1stBatter_clicked(self):
        try:
            if self.grabbing_spoon == True:
                self.drop_spoon()

            self.run_trajectory("ROS/trajectories/drop_1st_batter.csv")
        except Exception as e:
            self.ui.textEdit_status.append(f"drop_1st_batter error: {e}\n")    

    def pushButton_Drop2ndBatter_clicked(self):
        try:
            if self.grabbing_spoon == True:
                self.drop_spoon()

            self.run_trajectory("ROS/trajectories/drop_2nd_batter.csv")
        except Exception as e:
            self.ui.textEdit_status.append(f"drop_2nd_batter error: {e}\n")    

    def pushButton_Close1stLid_clicked(self):
        try:
            if self.grabbing_spoon == True:
                self.drop_spoon()

            self.run_trajectory("ROS/trajectories/close_1st_lid.csv")
        except Exception as e:
            self.ui.textEdit_status.append(f"close_1st_lid error: {e}\n")

    def pushButton_Close2ndLid_clicked(self):
        try:
            if self.grabbing_spoon == True:
                self.drop_spoon()

            self.run_trajectory("ROS/trajectories/close_2nd_lid.csv")
        except Exception as e:
            self.ui.textEdit_status.append(f"close_2nd_lid error: {e}\n")

    def pushButton_GrabFork_clicked(self):
        try:
            if self.grabbing_spoon == True:
                self.drop_spoon()

            self.run_trajectory("ROS/trajectories/grab_fork.csv")
            self.grabbing_fork = True
        except Exception as e:
            self.ui.textEdit_status.append(f"grab_fork error: {e}\n")

    def pushButton_DropFork_clicked(self):
        try:
            if self.grabbing_spoon == True:
                self.drop_spoon()

            self.run_trajectory("ROS/trajectories/drop_fork.csv")
            self.grabbing_fork = False
        except Exception as e:
            self.ui.textEdit_status.append(f"drop_fork error: {e}\n")

    def pushButton_Get1stWaffle_clicked(self):
        try:
            if self.grabbing_spoon == True:
                self.drop_spoon()

            self.run_trajectory("ROS/trajectories/get_1st_waffle.csv")
        except Exception as e:
            self.ui.textEdit_status.append(f"get_1st_waffle error: {e}\n")

    def pushButton_Get2ndWaffle_clicked(self):
        try:
            if self.grabbing_spoon == True:
                self.drop_spoon()

            self.run_trajectory("ROS/trajectories/get_2nd_waffle.csv")
        except Exception as e:
            self.ui.textEdit_status.append(f"get_2nd_waffle error: {e}\n")

    def pushButton_DropWaffle_clicked(self):
        try:
            if self.grabbing_spoon == True:
                self.drop_spoon()

            self.run_trajectory("ROS/trajectories/drop_waffle.csv")
        except Exception as e:
            self.ui.textEdit_status.append(f"drop_waffle error: {e}\n")

    def pushButton_GoToDefault_clicked(self):
        try:
            if self.grabbing_spoon == True:
                self.drop_spoon()

            self.run_trajectory("ROS/trajectories/go_to_default.csv")
        except Exception as e:
            self.ui.textEdit_status.append(f"go_to_default error: {e}\n")

    def pushButton_ServeWaffle_clicked(self):
        try:
            if self.ui.lineEdit_NumOfWaffle.text():
                new_order = order(0, int(self.ui.lineEdit_NumOfWaffle.text()))
                self.tcp.received_orders.put(new_order)
            else:
                self.ui.textEdit_status.append(f"Num of Waffle is empty.\n")      
        except Exception as e:
            self.ui.textEdit_status.append(f"Serve Waffle error: {e}\n")

    def serve_waffle(self, num_waffle):
        self.cooked_waffle = num_waffle
        if num_waffle <= self.cooked_waffle:
            self.cooked_waffle -= num_waffle
            return
        
        if num_waffle - self.cooked_waffle > 4:
            use_2nd_stove = True
        else:
            use_2nd_stove = False

        self.wok.AC(1)
        
        self.pushButton_Grab1stBatter_clicked()
        self.pushButton_Pour1stBatter_clicked()
        self.pushButton_Drop1stBatter_clicked()
        self.pushButton_Close1stLid_clicked()
        first_start_time = time.time()

        heating_time = 10
        if use_2nd_stove == True:
            self.pushButton_Grab2ndBatter_clicked()
            self.pushButton_Pour2ndBatter_clicked()
            self.pushButton_Drop2ndBatter_clicked
            self.pushButton_Close2ndLid_clicked()
            second_start_time = time.time()

        while time.time() - first_start_time < heating_time:
            time.sleep(0.01)
        
        if use_2nd_stove == False:
            self.wok.AC(0)

        self.pushButton_Open1stLid_clicked()
        self.pushButton_GrabFork_clicked()
        self.pushButton_Get1stWaffle_clicked()
        self.pushButton_DropWaffle_clicked()
        self.pushButton_DropFork_clicked()

        if use_2nd_stove == True:
            while time.time() - second_start_time < heating_time:
                time.sleep(0.01)
            self.wok.AC(0)       

        if use_2nd_stove == True:
            self.pushButton_Open2ndLid_clicked()
            self.pushButton_GrabFork_clicked()
            self.pushButton_Get2ndWaffle_clicked()
            self.pushButton_DropWaffle_clicked()
            self.pushButton_DropFork_clicked()

        if use_2nd_stove == False:
            self.cooked_waffle = self.cooked_waffle + 4 - num_waffle
        else:
            self.cooked_waffle = self.cooked_waffle + 8 - num_waffle

    def serve_orders(self):
        while self.serving_orders == True:

            if self.tcp.received_orders.empty() == True:
                continue

            try:
                order = self.tcp.received_orders.get()
                self.left_seconds = self.get_order_time(order)
                self.ui.textEdit_status.append(f"left_seconds: {self.left_seconds}\n")

                if self.serving_orders == False:
                    break

                if order.peanuts_num > 0:
                    self.serve_peanuts(order.peanuts_num)

                if self.serving_orders == False:
                    break

                if order.waffle_num > 0:
                    self.serve_waffle(order.waffle_num)

                self.left_seconds = 0
                self.tcp.send_end()
                self.ui.textEdit_status.append(f"Serve order done: {order.peanuts_num} + {order.waffle_num}.\n")
            except Exception as e:
                self.ui.textEdit_status.append(f"serve_orders error: {e}\n")

    def get_order_time(self, order):
        seconds = 0

        if order.peanuts_num > 0:
            seconds += self.time_peanut_spoon * order.peanuts_num

        if order.waffle_num > self.cooked_waffle:
            if order.waffle_num - self.cooked_waffle <= 4:
                seconds += self.time_waffle_pour + self.time_waffle_heat + self.time_waffle_serve
            else:
                seconds += self.time_waffle_pour + self.time_waffle_heat + self.time_waffle_serve + self.time_waffle_2nd_stove

        return seconds

    def pan_home(self):
        self.wok.home()

    def pan_down(self):
        self.wok.down()

    def pan_flip(self):
        self.wok.flip()

    def AC(self):
        self.on_off = not self.on_off
        if self.on_off == True:
            self.wok.AC(1)
        else:
            self.wok.AC(0)

    def closeEvent(self, event: QCloseEvent):
        try:
            if self.cam:
                self.cam.quit()
        except Exception as e:
            self.ui.textEdit_status.append(f"cam.quit error: {e}\n")

        try:
            if self.graspGenCommunication:
                self.GraspGenCommunication_destroy()
        except Exception as e:
            self.ui.textEdit_status.append(f"GraspGenCommunication_destroy error: {e}\n")

        try:
            if self.rosCommunication:
                self.ros_destroy()
        except Exception as e:
            self.ui.textEdit_status.append(f"ros_destroy error: {e}\n")

        try:
            if self.tcp:
                self.tcp.close()
        except Exception as e:
            self.ui.textEdit_status.append(f"tcp.close error: {e}\n")

        self.serving_orders = False

        try:
            if self.thread_pan_position_check:
                self.thread_pan_position_check.join()
        except Exception as e:
            self.ui.textEdit_status.append(f"thread_pan_position_check join error: {e}\n")

        try:
            if self.thread_processing_orders:    
                self.thread_processing_orders.join()    
        except Exception as e:
            self.ui.textEdit_status.append(f"thread_processing_orders join error: {e}\n")        

        try:
            if self.thread_counting_left_time:
                self.thread_counting_left_time.join()  
        except Exception as e:
            self.ui.textEdit_status.append(f"thread_counting_left_time join error: {e}\n")  

