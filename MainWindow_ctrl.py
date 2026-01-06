from PySide6.QtWidgets import QMainWindow
from PySide6.QtGui import QCloseEvent, QPixmap, QImage
from PySide6.QtCore import Qt, Signal, QTimer

from MainWindow_ui import Ui_MainWindow
from Camera.camera import *
from GraspGen.graspgen_comm import *
from PeanutNumberClassification.PeanutNumClassification import *
from ROS.trajectory_parser import *
from ROS.ros_comm import *
from Uart.Wok import *
from TCP.TCP import *
from Thermal.thermal_TCP import *
from CheckEmptyCup.cup_TCP import *
from CheckWaffleLid.lid_TCP import *
from web_panel import WebPanel
from move_TCP import *  

import cv2
import json
import numpy as np
from enum import Enum
import threading
import os

class PAN_POS(Enum):
    HOME = 1
    DOWN = 2
    UNKNOWN = 3

class main_window_ctrl(QMainWindow):
    statusChanged = Signal(str)
    tempChanged = Signal(float)
    grabbingSpoonChanged = Signal(bool)
    peanutStatusChanged = Signal(str)
    def __init__(self):
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.ui_connect()
        self.init()
        self.open_web_panel()
        
        self.statusChanged.connect(self.on_status_changed)
        self.temp_timer = QTimer(self)
        self.temp_timer.timeout.connect(self._emit_temp)
        self.temp_timer.start(1000)  # 每 1 秒更新一次

    # ---------------- UI 綁定 ----------------
    def ui_connect(self):
        # functions for peanuts
        self.ui.pushButton_CheckPeanuts.clicked.connect(self.pushButton_CheckPeanuts_clicked)  
        self.ui.pushButton_CheckTemperature.clicked.connect(self.pushButton_CheckTemperature_clicked)  

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
        self.ui.pushButton_WokHeat.clicked.connect(self.pan_heat)

        self.ui.lineEdit_ThermalThreshold.textChanged.connect(self.lineEdit_ThermalThreshold_textChanged)
        self.ui.lineEdit_PeanutsHeatingTime.textChanged.connect(self.lineEdit_PeanutsHeatingTime_textChanged)

        # functions for waffle
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

        self.ui.pushButton_CheckWaffleLid.clicked.connect(self.pushButton_CheckWaffleLid_clicked)

        self.ui.lineEdit_WaffleHeatingTime.textChanged.connect(self.lineEdit_WaffleHeatingTime_textChanged)

        self.ui.pushButton_ServeBoth.clicked.connect(self.pushButton_ServeBoth_clicked)

        self.ui.pushButton_SaveParameters.clicked.connect(self.pushButton_SaveParameters_clicked)

    def init(self):
        try:
            self.load_parameters()
            # self.preload_all_trajectories("ROS/trajectories")

            self.serving_orders = True
            self.peanuts_wait_for_pan_home = False
            self.waffle_first_stove_start_time = 0
            self.current_order_left_seconds = 0
            self.pressing_button_needed = False
            
            self.num_left_waffle = 0
            self.grabbing_spoon = False
            self.grabbingSpoonChanged.emit(False)
            self.grabbing_fork = False
            self.waffle_machine_on_off = False          
            self.right_cartesian_pose = [377.204,-450.270,344.386,-97.57,-43.60,-81.72]
            self.suggest_1st_lid_x = 1.4
            self.suggest_1st_lid_y = 3.5
            self.suggest_2nd_lid_x = 0
            self.suggest_2nd_lid_y = 0

            self.time_peanut_heat = (int)(self.ui.lineEdit_PeanutsHeatingTime.text())
            self.time_waffle_heat = (int)(self.ui.lineEdit_WaffleHeatingTime.text())
            self.thermal_threshold = (int)(self.ui.lineEdit_ThermalThreshold.text())

            if 'self.PeanutNumClassifier' not in globals():
                try:
                    self.PeanutNumClassification_init()
                except Exception as e:
                    self.ui.textEdit_status.append(f"[ERROR]PeanutNumClassification_init error: {e}\n")
                    return
                
            if 'self.GraspGenCommunication' not in globals():
                try:
                    self.GraspGenCommunication_init()
                except Exception as e:
                    self.ui.textEdit_status.append(f"[ERROR]GraspGenCommunication_init error: {e}\n")
                    return
            
            if 'self.rosCommunication' not in globals():
                try:
                    self.ros_init()
                except Exception as e:
                    self.ui.textEdit_status.append(f"[ERROR]ros_init error: {e}\n")
                    return                     

            if 'self.wok' not in globals():
                try:
                    self.wok_init()
                except Exception as e:
                    self.ui.textEdit_status.append(f"[ERROR]wok_init error: {e}\n")
                    return  

            if 'self.tcp' not in globals():
                try:
                    self.tcp_init()
                except Exception as e:
                    self.ui.textEdit_status.append(f"[ERROR]tcp_init error: {e}\n")
                    return  

            if 'self.tcp_thermal' not in globals():
                try:
                    self.tcp_thermal_init()
                except Exception as e:
                    self.ui.textEdit_status.append(f"[ERROR]tcp_thermal_init error: {e}\n")
                    return
                 
            if 'self.tcp_check_empty_cup' not in globals():
                try:
                    self.tcp_check_empty_cup_init()
                except Exception as e:
                    self.ui.textEdit_status.append(f"[ERROR]tcp_check_empty_cup_init error: {e}\n")
                    return          

            if 'self.cam' not in globals():
                try:
                    self.cam_init()
                except Exception as e:
                    self.ui.textEdit_status.append(f"[ERROR]cam_init error: {e}\n")
                    return

            if 'self.tcp_check_waffle_lid' not in globals():
                try:
                    self.tcp_check_waffle_lid_init()
                except Exception as e:
                    self.ui.textEdit_status.append(f"[ERROR]tcp_check_empty_cup_init error: {e}\n")
                    return        
                            
        except Exception as e:
            self.ui.textEdit_status.append(f"[ERROR]init error: {e}\n")

    #region init
    def tcp_init(self):
        self.tcp = TcpClient("192.168.1.111", 9000)
        # self.tcp.connect()
        self.thread_processing_orders = threading.Thread(target=self.serve_orders)
        self.thread_processing_orders.start() 

        self.thread_counting_left_time = threading.Thread(target=self.count_left_time)
        self.thread_counting_left_time.start()

    def tcp_thermal_init(self):
        self.tcp_thermal = ThermalClient("192.168.1.133", 9000)
        # self.tcp_thermal.connect()
        # print("tcp_thermal_init connect")
        



    def tcp_check_empty_cup_init(self):
        self.tcp_check_empty_cup = CupClient("192.168.1.133", 9999)
        # print("tcp_check_empty_cup_init start")
        # self.tcp_check_empty_cup.connect()
        # print("tcp_check_empty_cup_init connect")

    def wok_init(self):
        self.wok = Wok()
        self.wok.down()
        self.pan_position = PAN_POS.DOWN
        self.thread_pan_position_check = threading.Thread(target=self.receive_pan_position)
        self.thread_pan_position_check.start()    

    def cam_init(self):
        self.cam = Camera()
        self.cam.cam_init([2])

    def tcp_check_waffle_lid_init(self):
        self.tcp_check_waffle_lid = LidClient("192.168.1.133", 9000)
        print("tcp_check_waffle_lid_init start")
        self.tcp_check_waffle_lid.connect()
        print("tcp_check_waffle_lid_init connect")

    def PeanutNumClassification_init(self):
        try:
            self.peanutNumClassifier = PeanutNumClassification()  # Initialize the classifier
        except Exception as e:
            self.ui.textEdit_status.append(f"[ERROR]PeanutNumClassification_init error: {e}\n")

    def GraspGenCommunication_init(self):
        try:
            self.graspGenCommunication = GraspGenCommunication()
        except Exception as e:
            self.ui.textEdit_status.append(f"[ERROR]GraspGenCommunication_init error: {e}\n")

    def GraspGenCommunication_destroy(self):
        try:
            self.graspGenCommunication.quit()
            print("GraspGenCommunication_destroy done.")
        except Exception as e:
            self.ui.textEdit_status.append(f"[ERROR]GraspGenCommunication_destroy error: {e}\n")

    def ros_init(self):        
        try:
            self.rosCommunication = ROSCommunication()
        except Exception as e:
            self.ui.textEdit_status.append(f"[ERROR]ros_init error: {e}\n")

    def ros_destroy(self):
        try:
            self.rosCommunication.quit()
            print("ros_destroy done.")
        except Exception as e:
            self.ui.textEdit_status.append(f"[ERROR]ros_destroy error: {e}\n") 
    #endregion

    #region threading
    def count_left_time(self):
        while self.serving_orders == True:

            self.tcp.left_time(self.current_order_left_seconds // 60, self.current_order_left_seconds % 60)
            self.ui.lineEdit_LeftTime.setText(str(int(self.current_order_left_seconds))) 
            if self.current_order_left_seconds > 0:
                self.current_order_left_seconds -= 1

            time.sleep(1)

    def receive_pan_position(self):
        while self.serving_orders == True:
            if self.wok.received_status.empty() == False:                
                position = self.wok.received_status.get()
                self.pan_position_received(position)
            time.sleep(0.01)    
    #endregion

    #region check pan position
    def pan_position_received(self, position):
        print("receive_pan_position: ", position)
        if position == "home":            
            self.pan_position = PAN_POS.HOME
        elif position == "down":
            self.pan_position = PAN_POS.DOWN

    def check_pan_pos(self, position):
        if position == PAN_POS.HOME:
            if self.pan_position != PAN_POS.HOME:                
                self.wok.home()
                print("Wait for home.")
                st_time = time.time()
                while self.pan_position != PAN_POS.HOME:
                    time.sleep(0.1)
                    #if time.time() - st_time > 5:
                    #    raise Exception("Error: Pan returning home failed.")
        elif position == PAN_POS.DOWN:
            if self.pan_position != PAN_POS.DOWN:                
                self.wok.down()
                print("Wait for down.")
                st_time = time.time()
                while self.pan_position != PAN_POS.DOWN:
                    time.sleep(0.1)
                    #if time.time() - st_time > 5:
                    #    raise Exception("Error: Pan returning down failed.")
    #endregion

    #region check peanuts amount
    def pushButton_CheckPeanuts_clicked(self):
        try:
            # output = self.check_peanuts(save_image=True)
            output = self.check_peanuts_amount()
            # output = self.check_peanuts('PeanutNumberClassification/operating-sub.png') only use when debugging
            self.ui.textEdit_status.append(f"[INFO]check_peanuts output: {output}\n")
        except Exception as e:
            self.ui.textEdit_status.append(f"[ERROR]check_peanuts error: {e}\n")

    def check_peanuts_amount(self, image_path=None, save_image = False):       
        try:
            self.check_pan_pos(PAN_POS.HOME)

            print("Get peanuts amount image.")
            if image_path is not None:
                image = cv2.imread(image_path)
            else:
                image = self.cam.capture_single(2)

            print("Load peanuts amount roi.")
            file = open('PeanutNumberClassification/roi.json', 'r')
            self.roi = json.loads(file.read())["roi"] # x, y, w, h
            if self.roi[2] != 0 and self.roi[3] != 0: # with & height
                right = (self.roi[0] + self.roi[2])
                bottom = (self.roi[1] + self.roi[3])
                image = image[self.roi[1]:bottom, self.roi[0]:right]

            # show image
            print("Show peanuts amount image.")
            image_showed = np.ascontiguousarray(image)
            if save_image:
                cv2.imwrite(f'PeanutNumberClassification/dataset/peanuts_image{int(time.time())}.png', image_showed)
            peanuts_image = QImage(image_showed, image_showed.shape[1], image_showed.shape[0], 3 * image_showed.shape[1], QImage.Format.Format_BGR888)
            peanuts_pixmap = QPixmap.fromImage(peanuts_image)    
            peanuts_pixmap_scaled = peanuts_pixmap.scaled(self.ui.label_image_peanuts.width(), self.ui.label_image_peanuts.height(), aspectMode=Qt.AspectRatioMode.KeepAspectRatio)
            self.ui.label_image_peanuts.setPixmap(peanuts_pixmap_scaled)

            print("Checking peanuts amount...")
            output = self.peanutNumClassifier.classify(image)
            self.peanutStatusChanged.emit(output)
            return output
        except Exception as e:
            raise e
    #endregion           

    #region graspgen
    def grasp_and_dump_peanuts_flow(self):
         try:
            self.check_pan_pos(PAN_POS.DOWN)

            if self.grabbing_spoon == True:
                self.drop_spoon_flow()
            if self.current_order_left_seconds > 0:
                self.current_order_left_seconds += 65
            data = {"actions": "Grasp_and_Dump"}
            message = self.graspGenCommunication.send_data(data)
            self.statusChanged.emit(f"[INFO]graspGen {message}")
         except Exception as e:
            self.statusChanged.emit(f"[ERROR]pushButton_GrabNDumpPeanuts_clicked error: {e}\n")

    def pushButton_GrabNDumpPeanuts_clicked(self):
        self.grasp_and_dump_peanuts_flow()
    #endregion

    #region peanuts related
    def press_button(self):
        try:
            press_button_time = (int)(self.ui.lineEdit_PressButtonTime.text())
            self.press_button_flow(press_button_time)
        except Exception as e:
            self.ui.textEdit_status.append(f"[ERROR]press_button error: {e}\n")

    def press_button_flow(self, press_button_time: int | None = None):
        try:
            if press_button_time is None:
                press_button_time = self.parameters["PressButtonTime"]
            self.statusChanged.emit("[INFO] 🔧 start Pressing Button.")
            self.run_trajectory("ROS/trajectories/press_button1.csv")
            self.run_trajectory("ROS/trajectories/press_button2.csv")
            
            if self.current_order_left_seconds >0:
                self.current_order_left_seconds += press_button_time
            self.statusChanged.emit("[INFO] ✅ Pressing Button Done.")
        except Exception as e:
            self.statusChanged.emit("[ERROR] ❌ Pressing Button Failed")
            self.statusChanged.emit(f"[ERROR]press button error: {e}\n")



    def get_spoon(self):
        try:
            get_spoon_time = int(self.ui.lineEdit_GetSpoonTime.text())
            self.get_spoon_flow(get_spoon_time)
        except Exception as e:
            self.ui.textEdit_status.append(f"[ERROR]get_spoon{e}\n")
    
    def get_spoon_flow(self, get_spoon_time: int | None = None):
        try:
            if get_spoon_time is None:
                get_spoon_time = self.parameters["GetSpoonTime"]
            self.statusChanged.emit("[INFO] 🔧 開始拿取湯匙")
            self.run_trajectory("ROS/trajectories/get_spoon.csv")
            self.grabbing_spoon = True
            self.grabbingSpoonChanged.emit(True)
            if self.current_order_left_seconds >0:
                self.current_order_left_seconds += get_spoon_time
            self.statusChanged.emit("[INFO] ✅ 拿取湯匙完成")
        except Exception as e:
            self.statusChanged.emit("[ERROR] ❌ 拿取湯匙失敗")
            self.statusChanged.emit(f"[ERROR]get_spoon error: {e}\n")




    def pushButton_SpoonPeanuts_clicked(self):
        try:
            self.check_pan_pos(PAN_POS.HOME)
            #peanuts_status = self.check_peanuts()
            #if peanuts_status == 'sufficient':
            self.spoon_single_peanuts()
            #else:
            #    self.ui.textEdit_status.append(f"spoon_peanuts insufficient.")
        except Exception as e:
            self.ui.textEdit_status.append(f"[ERROR]spoon_peanuts error: {e}\n")

    def spoon_single_peanuts(self):
        try:
            self.run_trajectory("ROS/trajectories/spoon_peanuts.csv", vel=60, acc=500)
        except Exception as e:
            raise e

    def spoon_peanuts(self):
        try:
            # check if reheat or flip is done
            if self.peanuts_wait_for_pan_home == True:
                while self.pan_position != PAN_POS.HOME:
                    time.sleep(0.01)
                self.peanuts_wait_for_pan_home = False

            # check amount of peanuts, refill peanuts if insufficient
            peanuts_amount = self.check_peanuts_amount()
            self.statusChanged.emit(f"[INFO] 🥜 檢查花生數量: {peanuts_amount}")
            last_t = time.monotonic()

            while peanuts_amount == 'operating': 
                time.sleep(0.01)
                # self.current_order_left_seconds += 0.01  #will make the current order time become large infinitely or small infinitely
                now = time.monotonic()
                if int(now) != int(last_t):
                    self.current_order_left_seconds += 1
                last_t = now
                self.statusChanged.emit(f"[INFO]Operating. Please remove everything.") 
                peanuts_amount = self.check_peanuts_amount()

            while peanuts_amount != 'sufficient':      
                # check if there's any empty cup     
                while self.tcp_check_empty_cup.get_isEmpty() == True:
                    self.statusChanged.emit("[WARNING]Please refill the peanut cup.")               
                    time.sleep(0.01)
                    self.current_order_left_seconds += 0.01

                # no empty cup, run GraspGen
                self.statusChanged.emit("[INFO]GraspGen Refilling Peanuts...")
                self.pushButton_GrabNDumpPeanuts_clicked()

                # check peanuts amount before spooning
                peanuts_amount = self.check_peanuts_amount()
                if peanuts_amount == 'sufficient':
                    self.pressing_button_needed = True   
                    self.statusChanged.emit("[INFO]Peanuts refilled.")
                    break
                time.sleep(1)
                self.current_order_left_seconds += 1

            # press button if needed
            if self.pressing_button_needed == True:
                if self.grabbing_spoon == True:
                    self.statusChanged.emit("[INFO]Dropping spoon...")
                    self.drop_spoon_flow()
                    self.statusChanged.emit("[INFO]Spoon drop...")
                    
                self.pan_position = PAN_POS.DOWN
                self.statusChanged.emit("[INFO]Pressing button...")
                self.press_button_flow()
                self.statusChanged.emit("[INFO]Button pressed.")
                self.peanuts_wait_for_pan_home = True            
                self.pressing_button_needed = False

                # get back to waffle is waffle is cooking
                if self.waffle_first_stove_start_time != 0:
                    self.statusChanged.emit("[INFO]Return to waffle.")
                    return 0
                else:
                    PeanutsHeatFlipTime = self.parameters["PeanutsHeatFlipTime"]
                    self.current_order_left_seconds += PeanutsHeatFlipTime
            
            # reheat peanuts if needed
            elif self.tcp_thermal.get_cur_temp() < self.thermal_threshold:
                self.statusChanged.emit(f"[INFO]Current Temperature: {self.tcp_thermal.get_cur_temp()}")
                self.pan_position = PAN_POS.DOWN
                self.statusChanged.emit("[INFO]Re-heating.")
                self.wok.heat()                
                self.peanuts_wait_for_pan_home = True

                # get back to waffle is waffle is cooking
                if self.waffle_first_stove_start_time != 0:
                    self.statusChanged.emit("[INFO]Return to waffle.")
                    return 0
                else:
                    self.current_order_left_seconds += self.parameters["PeanutsHeatFlipTime"]
            elif self.tcp_thermal.get_cur_temp() >= self.thermal_threshold:
                self.statusChanged.emit(f"[INFO]Current Temperature: {self.tcp_thermal.get_cur_temp()}")
                self.statusChanged.emit(f"[INFO]Don't need to re-heat.")

            # get spoon
            if self.grabbing_spoon == False:
                self.statusChanged.emit(f"[INFO]Grabbing spoon...")
                self.get_spoon_flow()
                self.statusChanged.emit(f"[INFO]Grab spoon done.")

            # check pan position
            self.statusChanged.emit(f"[INFO]Wait for HOME.")
            while self.pan_position != PAN_POS.HOME:
                time.sleep(0.01)
            self.peanuts_wait_for_pan_home = False

            # spoon peanuts              
            self.statusChanged.emit(f"[INFO]Spooning...")
            self.spoon_single_peanuts()
            
            self.pan_position = PAN_POS.DOWN
            self.statusChanged.emit(f"[INFO]Flipping...")
            self.pan_flip()            

            self.statusChanged.emit(f"[INFO]Wait for HOME.")
            while self.pan_position != PAN_POS.HOME:
                time.sleep(0.01)

            return 1
        except Exception as e:
            raise e

    def spoon_peanuts_flow(self):
        self.statusChanged.emit("[INFO] 🥄 開始執行 spoon peanuts 流程")
        result = self.spoon_peanuts()
        self.statusChanged.emit("[INFO] 🥄 spoon peanuts 流程結束")
        return result

    def drop_spoon(self):
        try:
            drop_spoon_time = int(self.ui.lineEdit_DropSpoonTime.text())
            self.drop_spoon_flow(drop_spoon_time)
        except Exception as e:
            self.ui.textEdit_status.append(f"[ERROR]drop_spoon error: {e}\n")

    def drop_spoon_flow(self,drop_spoon_time: int | None = None):
        try:
            if drop_spoon_time is None:
                drop_spoon_time = self.parameters["DropSpoonTime"]
            self.statusChanged.emit("[INFO] 🔧 開始放湯匙")
            self.run_trajectory("ROS/trajectories/drop_spoon.csv")
            self.grabbing_spoon = False
            self.grabbingSpoonChanged.emit(False)
            if self.current_order_left_seconds > 0:
                self.current_order_left_seconds += drop_spoon_time
            self.statusChanged.emit("[INFO] ✅ 放湯匙完成")
        except Exception as e:
            self.statusChanged.emit("[ERROR] ❌ 放取湯匙失敗")
            self.statusChanged.emit(f"[ERROR]drop_spoon error: {e}\n")

        

    def pushButton_ServePeanuts_clicked(self):
        try:
            if self.ui.lineEdit_NumOfPeanuts.text():
                new_order = order(int(self.ui.lineEdit_NumOfPeanuts.text()), 0)
                self.tcp.received_orders.put(new_order)
            else:
                self.ui.textEdit_status.append(f"[WARNING]Num of Peanuts is empty.\n")      
        except Exception as e:
            self.ui.textEdit_status.append(f"[ERROR]Serve Peanuts error: {e}\n")    
    #endregion

    #region waffle related
    def pushButton_CheckWaffleLid_clicked(self):
        state = self.tcp_check_waffle_lid.get_latest()
        self.statusChanged.emit(f"Lid State: {state}")
    
    def pushButton_Open1stLid_clicked(self):
        try:
            if self.grabbing_spoon == True:
                self.drop_spoon()

            state = self.tcp_check_waffle_lid.get_latest()

            if state.right_lid == LidPos.OPEN:
                self.ui.textEdit_status.append(f"[INFO]1st Lid already opened.\n")
                return

            self.run_trajectory("ROS/trajectories/open_1st_lid.csv", vel=100, acc=500)
        except Exception as e:
            self.ui.textEdit_status.append(f"[ERROR]open_1st_lid error: {e}\n")

    def pushButton_Open2ndLid_clicked(self):
        try:
            if self.grabbing_spoon == True:
                self.drop_spoon()

            state = self.tcp_check_waffle_lid.get_latest()
            if state.left_lid == LidPos.OPEN:
                self.ui.textEdit_status.append(f"[INFO]2nd Lid already opened.\n")
                return 
            self.run_trajectory("ROS/trajectories/open_2nd_lid.csv", vel=100, acc=500)
        except Exception as e:
            self.ui.textEdit_status.append(f"[ERROR]open_2nd_lid error: {e}\n")

    def pushButton_Grab1stBatter_clicked(self):
        try:
            if self.grabbing_spoon == True:
                self.drop_spoon()

            state= self.tcp_check_waffle_lid.get_latest()
            if state.right_lid == LidPos.CLOSED:
                self.run_trajectory("ROS/trajectories/open_1st_lid.csv", vel=100, acc=500)
    
            self.run_trajectory("ROS/trajectories/grab_1st_batter.csv", vel=100, acc=500)
        except Exception as e:
            self.ui.textEdit_status.append(f"[ERROR]grab_1st_batter error: {e}\n")

    def pushButton_Grab2ndBatter_clicked(self):
        try:
            if self.grabbing_spoon == True:
                self.drop_spoon()
            state = self.tcp_check_waffle_lid.get_latest()
            if state.left_lid == LidPos.CLOSED:
                self.run_trajectory("ROS/trajectories/open_2nd_lid.csv", vel=100, acc=500)
            self.run_trajectory("ROS/trajectories/grab_2nd_batter.csv", vel=100, acc=500)
        except Exception as e:
            self.ui.textEdit_status.append(f"[ERROR]grab_2nd_batter error: {e}\n")
    def apply_offset(self,v):
        if v > 0:
            return 5 if v <= 1 else 10
        elif v < 0:
            return -5 if v >= -1 else -10
        else:
            return 0
    def pushButton_Pour1stBatter_clicked(self):
        try:
            if self.grabbing_spoon == True:
                self.drop_spoon()

            state = self.tcp_check_waffle_lid.get_latest()
            if state.right_lid == LidPos.CLOSED:
                self.run_trajectory("ROS/trajectories/drop_1st_batter.csv", vel=50, acc=500)
                self.run_trajectory("ROS/trajectories/open_1st_lid.csv", vel=100, acc=500)
                self.run_trajectory("ROS/trajectories/grab_1st_batter.csv", vel=100, acc=500)

            self.run_trajectory("ROS/trajectories/pour_1st_batter.csv", vel=100, acc=500)
            time.sleep(3)
            
            cartesian_pose = [377.204,-450.270,344.386,-97.57,-43.60,-81.72]
            cartesian_pose[0] += self.apply_offset(self.suggest_1st_lid_x)
            cartesian_pose[1] += self.apply_offset(self.suggest_1st_lid_y)
            
            self.ui.textEdit_status.append(f"certesian_pose:{cartesian_pose}\n")
            self.rosCommunication.send_data({"type": "PTP", "cartesian_poses": [cartesian_pose], "wait_time": 0.0})
        except Exception as e:
            self.ui.textEdit_status.append(f"[ERROR]pour_1st_batter error: {e}\n")

    def pushButton_Pour2ndBatter_clicked(self):
        try:
            if self.grabbing_spoon == True:
                self.drop_spoon()
            
            state  = self.tcp_check_waffle_lid.get_latest()
            if state.left_lid == LidPos.CLOSED:
                self.run_trajectory("ROS/trajectories/drop_2nd_batter.csv", vel=50, acc=500)
                self.run_trajectory("ROS/trajectories/open_2nd_lid.csv", vel=100, acc=500)
                self.run_trajectory("ROS/trajectories/grab_2nd_batter.csv", vel=100, acc=500)

            self.run_trajectory("ROS/trajectories/pour_2nd_batter.csv", vel=100, acc=500)
            cartesian_pose = [372.229,-200.201,344.319,-97.58,-43.59,-81.71]
            time.sleep(3)
            cartesian_pose[0] += self.apply_offset(self.suggest_2nd_lid_x)
            cartesian_pose[1] += self.apply_offset(self.suggest_2nd_lid_y)
            self.ui.textEdit_status.append(f"certesian_pose:{cartesian_pose}\n")
            self.rosCommunication.send_data({"type": "PTP", "cartesian_poses": [cartesian_pose], "wait_time": 0.0})
        except Exception as e:
            self.ui.textEdit_status.append(f"[ERROR]pour_2nd_batter error: {e}\n")

    def pushButton_Drop1stBatter_clicked(self):
        try:
            if self.grabbing_spoon == True:
                self.drop_spoon()
            self.run_trajectory("ROS/trajectories/drop_1st_batter.csv", vel=50, acc=500)
        except Exception as e:
            self.ui.textEdit_status.append(f"[ERROR]drop_1st_batter error: {e}\n")    

    def pushButton_Drop2ndBatter_clicked(self):
        try:
            if self.grabbing_spoon == True:
                self.drop_spoon()
            self.run_trajectory("ROS/trajectories/drop_2nd_batter.csv", vel=100, acc=500)
        except Exception as e:
            self.ui.textEdit_status.append(f"[ERROR]drop_2nd_batter error: {e}\n")    

    def pushButton_Close1stLid_clicked(self):
        try:
            if self.grabbing_spoon == True:
                self.drop_spoon()

            state = self.tcp_check_waffle_lid.get_latest()
            self.ui.textEdit_status.append(f"suggest_1st_lid_x: {state.suggest_rdx}, suggest_1st_lid_y: {state.suggest_rdy}\n")

            if state.right_lid == LidPos.CLOSED:
                self.ui.textEdit_status.append(f"[INFO]1st Lid already closed.\n")
                return
            
            self.run_trajectory("ROS/trajectories/close_1st_lid.csv", vel=100, acc=500)
        except Exception as e:
            self.ui.textEdit_status.append(f"[ERROR]close_1st_lid error: {e}\n")

    def pushButton_Close2ndLid_clicked(self):
        try:
            if self.grabbing_spoon == True:
                self.drop_spoon()

            state = self.tcp_check_waffle_lid.get_latest()
            self.ui.textEdit_status.append(f"suggest_2nd_lid_x: {state.suggest_ldx}, suggest_2nd_lid_y: {state.suggest_ldy}\n")
            if state.left_lid == LidPos.CLOSED:
                self.ui.textEdit_status.append(f"[INFO]2nd Lid already closed.\n")
                return

            self.run_trajectory("ROS/trajectories/close_2nd_lid.csv", vel=100, acc=500)
        except Exception as e:
            self.ui.textEdit_status.append(f"[ERROR]close_2nd_lid error: {e}\n")

    def pushButton_GrabFork_clicked(self)-> bool:
        try:
            if self.grabbing_spoon == True:
                self.drop_spoon()

            state = self.tcp_check_waffle_lid.get_latest()
            if state.right_waffle == WafflePos.ON_UPPER_LID:
                self.ui.textEdit_status.append("[ERROR] Waffle on upper lid, abort opening.\n")
                return False
            
            self.run_trajectory("ROS/trajectories/grab_fork.csv", vel=100, acc=500)
            self.grabbing_fork = True
            return True
        except Exception as e:
            self.ui.textEdit_status.append(f"[ERROR]grab_fork error: {e}\n")

    def wait_for_waffle_done(self):
        state = self.tcp_check_waffle_lid.get_latest()
        self.ui.textEdit_status.append(f"suggest_1st_lid_x: {state.suggest_rdx}, suggest_1st_lid_y: {state.suggest_rdy}\n")
        if state.suggest_rdx is not None and state.suggest_rdy is not None:
            self.suggest_1st_lid_x = state.suggest_rdx
            self.suggest_1st_lid_y = state.suggest_rdy
        if state.suggest_ldx is not None and state.suggest_ldy is not None:
            self.suggest_2nd_lid_x = state.suggest_ldx
            self.suggest_2nd_lid_y = state.suggest_ldy
        self.ui.textEdit_status.append(f"suggest_2nd_lid_x: {state.suggest_ldx}, suggest_2nd_lid_y: {state.suggest_ldy}\n")
        wait_time = (int)(self.ui.lineEdit_WaitForWaffleTime.text())
        time.sleep(wait_time)

    def wait_for_waffle_pour(self):
        wait_time = (int)(self.ui.lineEdit_WaitForWafflePourTime.text())
        time.sleep(wait_time)

    def pushButton_DropFork_clicked(self):
        try:
            if self.grabbing_spoon == True:
                self.drop_spoon()

            state = self.tcp_check_waffle_lid.get_latest()
            if state.left_lid == LidPos.OPEN:
                if state.left_waffle == WafflePos.ON_UPPER_LID or state.left_waffle == WafflePos.ON_LOWER_LID:
                    self.ui.textEdit_status.append(f"[INFO]2nd lid already open and have waffle. Don't need to drop the fork.\n")
                    return
                else:
                    self.run_trajectory("ROS/trajectories/drop_fork.csv", vel=100, acc=500)
                    self.grabbing_fork = False
            else:
                self.run_trajectory("ROS/trajectories/drop_fork.csv", vel=100, acc=500)
                self.grabbing_fork = False
        except Exception as e:
            self.ui.textEdit_status.append(f"[ERROR]drop_fork error: {e}\n")

    def pushButton_Get1stWaffle_clicked(self):
        try:
            if self.grabbing_spoon == True:
                self.drop_spoon()

            self.run_trajectory("ROS/trajectories/get_1st_waffle.csv", vel=50, acc=500, blend=80)
        except Exception as e:
            self.ui.textEdit_status.append(f"[ERROR]get_1st_waffle error: {e}\n")

    def pushButton_Get2ndWaffle_clicked(self):
        try:
            if self.grabbing_spoon == True:
                self.drop_spoon()

            state = self.tcp_check_waffle_lid.get_latest()
            if state.left_waffle == WafflePos.ON_UPPER_LID:
                self.ui.textEdit_status.append(f"[INFO]2nd waffle on upper lid. RUN Get 2nd Waffle on Top Lid.\n")
                self.run_trajectory("ROS/trajectories/get_2nd_waffle_top_lid.csv", vel=35, acc=500, blend=100)
            else:
                self.run_trajectory("ROS/trajectories/get_2nd_waffle.csv", vel=35, acc=500, blend=80)

        except Exception as e:
            self.ui.textEdit_status.append(f"[ERROR]get_2nd_waffle error: {e}\n")

    def pushButton_DropWaffle_clicked(self):
        try:
            if self.grabbing_spoon == True:
                self.drop_spoon()

            self.run_trajectory("ROS/trajectories/drop_waffle.csv", vel=40, acc=500)
        except Exception as e:
            self.ui.textEdit_status.append(f"[ERROR]drop_waffle error: {e}\n")

    def pushButton_GoToDefault_clicked(self):
        try:
            if self.grabbing_spoon == True:
                self.drop_spoon()

            self.run_trajectory("ROS/trajectories/go_to_default.csv", vel=100, acc=500)
        except Exception as e:
            self.ui.textEdit_status.append(f"[ERROR]go_to_default error: {e}\n")

    def pushButton_ServeWaffle_clicked(self):
        try:
            if self.ui.lineEdit_NumOfWaffle.text():
                new_order = order(0, int(self.ui.lineEdit_NumOfWaffle.text()))
                self.tcp.received_orders.put(new_order)
            else:
                self.ui.textEdit_status.append(f"[WARNING]Num of Waffle is empty.\n")      
        except Exception as e:
            self.ui.textEdit_status.append(f"[ERROR]Serve Waffle error: {e}\n")

    def cook_1st_stove(self):
        self.pushButton_Open1stLid_clicked()
        self.pushButton_Grab1stBatter_clicked()
        self.pushButton_Pour1stBatter_clicked()
        self.wait_for_waffle_pour()
        self.pushButton_Drop1stBatter_clicked()
        self.pushButton_Close1stLid_clicked()

    def cook_2nd_stove(self):
        self.pushButton_Open2ndLid_clicked()
        self.pushButton_Grab2ndBatter_clicked()
        self.pushButton_Pour2ndBatter_clicked()
        self.wait_for_waffle_pour()
        self.pushButton_Drop2ndBatter_clicked()
        self.pushButton_Close2ndLid_clicked()

    def serve_1st_stove(self):
        self.pushButton_Open1stLid_clicked()
        if not self.pushButton_GrabFork_clicked():
            self.ui.textEdit_status.append("[INFO] Abort at grab fork, cause waffle is on the upper lid.\n")
            return
        self.wait_for_waffle_done()
        self.pushButton_Get1stWaffle_clicked()
        self.pushButton_DropWaffle_clicked()
        self.pushButton_DropFork_clicked()

    def serve_2nd_stove(self):
        self.pushButton_Open2ndLid_clicked()
        if self.grabbing_fork == False:
            self.pushButton_GrabFork_clicked()
        self.wait_for_waffle_done()
        self.pushButton_Get2ndWaffle_clicked()
        self.pushButton_DropWaffle_clicked()
        self.pushButton_DropFork_clicked()
        

    def pushButton_ServeBoth_clicked(self):
        try:
            if self.ui.lineEdit_NumOfWaffle.text():
                num_waffle = int(self.ui.lineEdit_NumOfWaffle.text())
            else:
                self.ui.textEdit_status.append(f"[WARNING]Num of Waffle is empty.\n")   
                return

            if self.ui.lineEdit_NumOfPeanuts.text():
                num_peanuts = int(self.ui.lineEdit_NumOfPeanuts.text())
            else:
                self.ui.textEdit_status.append(f"[WARNING]Num of Peanuts is empty.\n")  
                return

            new_order = order(num_peanuts, num_waffle)
            self.tcp.received_orders.put(new_order)   
        except Exception as e:
            self.ui.textEdit_status.append(f"[ERROR]Serve Waffle error: {e}\n")
    #endregion

    #region serving orders
    def serve_orders(self):
        while self.serving_orders == True:
            if self.tcp.received_orders.empty() == True:
                continue

            try:
                order = self.tcp.received_orders.get()
                print(f"{order}, {order.peanuts_num}, {order.waffle_num}")
                self.ui.textEdit_status.append(f"[INFO]Serve order received: peanuts: {order.peanuts_num} + waffle: {order.waffle_num}.\n")
                print("log appended")
                if order.waffle_num <= self.num_left_waffle:
                    self.num_left_waffle -= order.waffle_num
                    order.waffle_num = 0
                else:
                    order.waffle_num -= self.num_left_waffle
                    self.num_left_waffle = 0

                self.current_order_left_seconds = self.get_order_time(order)

                try:
                    self.tcp.send_time(self.current_order_left_seconds // 60, self.current_order_left_seconds % 60)
                except Exception as e:
                    self.ui.textEdit_status.append(f"[ERROR]tcp send_time failed.\n")

                if self.serving_orders == False:
                    break

                self.serve_both(order.waffle_num, order.peanuts_num)

                self.ui.textEdit_status.append(f"[INFO]Serve order done.\n")
                self.ui.textEdit_status.append(f"[INFO]Number of left waffle: {self.num_left_waffle}.\n")
                self.current_order_left_seconds = 0   

                try:              
                    self.tcp.send_end()
                except Exception as e:
                    self.ui.textEdit_status.append(f"[ERROR]tcp send_end failed.\n")

            except ValueError as e:
                print(e)
                self.ui.textEdit_status.append(f"[ERROR]Clearing all remaining tasks.\n")
                while not self.tcp.received_orders.empty():
                    self.tcp.received_orders.get()
                continue
            except Exception as e:
                self.ui.textEdit_status.append(f"[ERROR]serve_orders error: {e}\n")

    def serve_both(self, num_waffle, num_peanuts):
        try:
            # cook waffle first if needed
            cook_waffle = False
            if num_waffle <= self.num_left_waffle:
                self.num_left_waffle -= num_waffle
            else:
                cook_waffle = True

            if cook_waffle == True:
                self.ui.textEdit_status.append(f"[INFO]Cooking waffle...\n") 
                if num_waffle - self.num_left_waffle > 4:
                    use_2nd_stove = True                
                else:
                    use_2nd_stove = False                

                # turn on waffle machine
                self.wok.AC(1)
                
                self.ui.textEdit_status.append(f"[INFO]Use 1st stove.\n")
                self.cook_1st_stove()
                self.waffle_first_stove_start_time = time.time()

                if use_2nd_stove == True:
                    self.ui.textEdit_status.append(f"[INFO]Use 2nd stove.\n")
                    self.cook_2nd_stove()
                    second_start_time = time.time()

            # then spoon peanuts
            peanuts_spooned_count = 1
            if num_peanuts > 0:            
                while peanuts_spooned_count <= num_peanuts:
                    done = self.spoon_peanuts()
                    if done == 1:
                        self.ui.textEdit_status.append(f"[INFO]Spoon peanuts: {peanuts_spooned_count}.\n")
                        peanuts_spooned_count += 1                
                    else: # if peanuts is reprocessing or reheating, back to waffle first
                        break

                    # check if waffle is ready before next spoon
                    if cook_waffle == True and time.time() - self.waffle_first_stove_start_time >= self.time_waffle_heat:
                        break

            if cook_waffle == True:
                # do waffle first if waffle is ready
                while time.time() - self.waffle_first_stove_start_time < self.time_waffle_heat:
                    time.sleep(0.01)

                # turn off waffle machine if 2nd stove is not in use
                if use_2nd_stove == False:
                    self.wok.AC(1)

                if self.grabbing_spoon == True:
                    self.drop_spoon()

                self.ui.textEdit_status.append(f"[INFO]Serving first stove...\n")
                self.serve_1st_stove()

                if use_2nd_stove == True:
                    while time.time() - second_start_time < self.time_waffle_heat:
                        time.sleep(0.01)
                    self.wok.AC(1)       
                    self.ui.textEdit_status.append(f"[INFO]Serving second stove...\n")
                    self.serve_2nd_stove()
                    self.num_left_waffle = self.num_left_waffle + 8 - num_waffle
                else:
                    if self.grabbing_fork == True:
                        self.run_trajectory("ROS/trajectories/drop_fork.csv", vel=100, acc=500)
                        self.grabbing_fork = False
                    self.num_left_waffle = self.num_left_waffle + 4 - num_waffle                

                self.waffle_first_stove_start_time = 0

            # back to peanuts when waffle is done
            while peanuts_spooned_count <= num_peanuts:
                self.spoon_peanuts()
                self.ui.textEdit_status.append(f"[INFO]Spoon peanuts: {peanuts_spooned_count}.\n")
                peanuts_spooned_count += 1
        except Exception as e:
            raise e

    def serve_waffle(self, num_waffle):
        if num_waffle <= self.num_left_waffle:
            self.num_left_waffle -= num_waffle
            return
        
        if num_waffle - self.num_left_waffle > 4:
            use_2nd_stove = True
        else:
            use_2nd_stove = False

        self.wok.AC(1)
        
        self.cook_1st_stove()
        first_start_time = time.time()

        if use_2nd_stove == True:
            self.cook_2nd_stove()
            second_start_time = time.time()

        while time.time() - first_start_time < self.time_waffle_heat:
            time.sleep(0.01)
        
        if use_2nd_stove == False:
            self.wok.AC(1)

        self.serve_1st_stove()

        if use_2nd_stove == True:
            while time.time() - second_start_time < self.time_waffle_heat:
                time.sleep(0.01)
            self.wok.AC(1)       

        if use_2nd_stove == True:
            self.serve_2nd_stove()

        if use_2nd_stove == False:
            self.num_left_waffle = self.num_left_waffle + 4 - num_waffle
        else:
            self.num_left_waffle = self.num_left_waffle + 8 - num_waffle

    def serve_peanuts(self, num_peanuts):
        try:
            count = 1
            while count <= num_peanuts:                
                self.spoon_peanuts_flow()
                self.statusChanged.emit(f"[INFO]🥜 Spoon peanuts: {count}")             
                count += 1         
            self.statusChanged.emit(f"[INFO]🥜 Serve Peanuts done.")
        except Exception as e:
            self.statusChanged.emit(f"[ERROR]🥜 Serve Peanuts error: {e}.")
    #endregion
    
        
    def run_trajectory(self, filename, vel=40, acc=20, blend=100):
        try:
            # load nodes
            # waffle_type = False
            # if filename in ["ROS/trajectories/close_1st_lid.csv","ROS/trajectories/close_2nd_lid.csv",
            #                 "ROS/trajectories/drop_1st_batter.csv","ROS/trajectories/drop_2nd_batter.csv",
            #                 "ROS/trajectories/drop_fork.csv","ROS/trajectories/drop_waffle.csv",
            #                 "ROS/trajectories/get_1st_waffle.csv","ROS/trajectories/get_2nd_waffle.csv",
            #                 "ROS/trajectories/grab_1st_batter.csv","ROS/trajectories/grab_2nd_batter.csv",
            #                 "ROS/trajectories/grab_fork.csv",
            #                 "ROS/trajectories/open_1st_lid.csv","ROS/trajectories/open_2nd_lid.csv",
            #                 "ROS/trajectories/pour_1st_batter.csv","ROS/trajectories/pour_2nd_batter.csv",
            #                 "ROS/trajectories/spoon_peanuts.csv"]:
            #     waffle_type = True
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
                    response = self.rosCommunication.send_data({"type": "gripper", "grip_type": "open", "wait_time": 1.5})
                elif node.mode == Mode.CLOSE:
                    # self.rosCommunication.close_gripper() 
                    response = self.rosCommunication.send_data({"type": "gripper", "grip_type": "close", "wait_time": 0.9})
                elif node.mode == Mode.HALF_OPEN:
                    response = self.rosCommunication.send_data({"type": "gripper", "grip_type": "half_open", "wait_time": 0.5})
                elif node.mode == Mode.CLOSE_TIGHT:
                    response = self.rosCommunication.send_data({"type": "gripper", "grip_type": "close_tight", "wait_time": 0.9})
                elif node.mode == Mode.MOVE:
                    response = self.rosCommunication.send_data({"type": "arm", "joints_values": node.joints_values, "wait_time": 0.0, "custom_vel": vel, "custom_acc": acc, "custom_blend": blend})
                print(response)
        except Exception as e:
            raise e   
    
    #region get order time
    def get_order_time(self, order):
        seconds = 0

        if order.waffle_num > self.num_left_waffle:
            if order.waffle_num - self.num_left_waffle <= 4:
                seconds = self.get_waffle_time(False)
            else:
                seconds = self.get_waffle_time(True)
            seconds += self.time_waffle_heat
                
        if order.peanuts_num > 0:
            seconds += (int)(self.ui.lineEdit_SpoonPeanutsTime.text()) * order.peanuts_num

        return seconds

    def get_waffle_time(self, use_2nd_stove):
        seconds = 0
        
        seconds += (int)(self.ui.lineEdit_Grab1stBatterTime.text())
        seconds += (int)(self.ui.lineEdit_Pour1stBatterTime.text())
        seconds += (int)(self.ui.lineEdit_Drop1stBatterTime.text())
        seconds += (int)(self.ui.lineEdit_Close1stLidTime.text())

        seconds += (int)(self.ui.lineEdit_Open1stLidTime.text())
        seconds += (int)(self.ui.lineEdit_GrabForkTime.text())
        seconds += (int)(self.ui.lineEdit_WaitForWaffleTime.text())
        seconds += (int)(self.ui.lineEdit_Get1stWaffleTime.text())
        seconds += (int)(self.ui.lineEdit_DropWaffleTime.text())
        seconds += (int)(self.ui.lineEdit_DropForkTime.text())

        if use_2nd_stove == True:
            seconds += (int)(self.ui.lineEdit_Grab2ndBatterTime.text())
            seconds += (int)(self.ui.lineEdit_Pour2ndBatterTime.text())
            seconds += (int)(self.ui.lineEdit_Drop2ndBatterTime.text())
            seconds += (int)(self.ui.lineEdit_Close2ndLidTime.text())

            seconds += (int)(self.ui.lineEdit_Open2ndLidTime.text())
            seconds += (int)(self.ui.lineEdit_GrabForkTime.text())
            seconds += (int)(self.ui.lineEdit_WaitForWaffleTime.text())
            seconds += (int)(self.ui.lineEdit_Get2ndWaffleTime.text())
            seconds += (int)(self.ui.lineEdit_DropWaffleTime.text())
            seconds += (int)(self.ui.lineEdit_DropForkTime.text())

        return seconds
    #endregion

    #region wok
    def pan_home(self):
        self.wok.home()

    def pan_down(self):
        self.wok.down()

    def pan_flip(self):
        if self.current_order_left_seconds > 0:
            # self.current_order_left_seconds += (int)(self.ui.lineEdit_PeanutsFlipTime.text())
            self.current_order_left_seconds += self.parameters["PeanutsHeatFlipTime"]

        self.wok.flip()        
    
    def pan_heat(self):
        if self.current_order_left_seconds > 0:
            self.current_order_left_seconds += (int)(self.ui.lineEdit_PeanutsHeatingTime.text())
            
        self.wok.heat()        

    def AC(self):
        self.waffle_machine_on_off = not self.waffle_machine_on_off
        if self.waffle_machine_on_off == True:
            self.wok.AC(1)
        else:
            self.wok.AC(1)
    #endregion

    #region parameters
    def load_parameters(self):
        try:
            file = open('parameters.json', 'r')
            self.parameters = json.loads(file.read())
            print(self.parameters)
            self.ui.lineEdit_ThermalThreshold.setText(str(self.parameters["ThermalThreshold"]))
            self.ui.lineEdit_PressButtonTime.setText(str(self.parameters["PressButtonTime"]))
            self.ui.lineEdit_GetSpoonTime.setText(str(self.parameters["GetSpoonTime"]))
            self.ui.lineEdit_SpoonPeanutsTime.setText(str(self.parameters["SpoonPeanutsTime"]))
            self.ui.lineEdit_DropSpoonTime.setText(str(self.parameters["DropSpoonTime"]))
            self.ui.lineEdit_PeanutsHeatFlipTime.setText(str(self.parameters["PeanutsHeatFlipTime"]))
            self.ui.lineEdit_PeanutsHeatingTime.setText(str(self.parameters["PeanutsHeatingTime"]))
            self.ui.lineEdit_PeanutsFlipTime.setText(str(self.parameters["PeanutsFlipTime"]))
            self.ui.lineEdit_Open1stLidTime.setText(str(self.parameters["Open1stLidTime"]))
            self.ui.lineEdit_Open2ndLidTime.setText(str(self.parameters["Open2ndLidTime"]))
            self.ui.lineEdit_Grab1stBatterTime.setText(str(self.parameters["Grab1stBatterTime"]))
            self.ui.lineEdit_Grab2ndBatterTime.setText(str(self.parameters["Grab2ndBatterTime"]))
            self.ui.lineEdit_Pour1stBatterTime.setText(str(self.parameters["Pour1stBatterTime"]))
            self.ui.lineEdit_Pour2ndBatterTime.setText(str(self.parameters["Pour2ndBatterTime"]))
            self.ui.lineEdit_Drop1stBatterTime.setText(str(self.parameters["Drop1stBatterTime"]))
            self.ui.lineEdit_Drop2ndBatterTime.setText(str(self.parameters["Drop2ndBatterTime"]))
            self.ui.lineEdit_Close1stLidTime.setText(str(self.parameters["Close1stLidTime"]))
            self.ui.lineEdit_Close2ndLidTime.setText(str(self.parameters["Close2ndLidTime"]))
            self.ui.lineEdit_GrabForkTime.setText(str(self.parameters["GrabForkTime"]))
            self.ui.lineEdit_Get1stWaffleTime.setText(str(self.parameters["Get1stWaffleTime"]))
            self.ui.lineEdit_Get2ndWaffleTime.setText(str(self.parameters["Get2ndWaffleTime"]))
            self.ui.lineEdit_DropWaffleTime.setText(str(self.parameters["DropWaffleTime"]))
            self.ui.lineEdit_WaffleHeatingTime.setText(str(self.parameters["WaffleHeatingTime"]))
            self.ui.lineEdit_WaitForWaffleTime.setText(str(self.parameters["WaitForWaffleTime"]))
            self.ui.lineEdit_WaitForWafflePourTime.setText(str(self.parameters["WaitForWafflePourTime"]))
            file.close()
        except Exception as e:
            raise e
        
    def pushButton_CheckTemperature_clicked(self):
        if self.tcp_thermal:
            self.ui.textEdit_status.append(f"[INFO]Current Temperature: {self.tcp_thermal.get_cur_temp()}\n")             

    def lineEdit_PeanutsHeatingTime_textChanged(self):
        try:
            self.time_peanut_heat = (int)(self.ui.lineEdit_PeanutsHeatingTime.text())
        except Exception as e:
            self.ui.textEdit_status.append(f"[ERROR]lineEdit_PeanutsHeatingTime_textChanged error: {e}.\n") 

    def lineEdit_ThermalThreshold_textChanged(self):
        try:
            self.thermal_threshold = (int)(self.ui.lineEdit_ThermalThreshold.text())
        except Exception as e:
            self.ui.textEdit_status.append(f"[ERROR]lineEdit_ThermalThreshold_textChanged error: {e}.\n") 

    def lineEdit_WaffleHeatingTime_textChanged(self):
        try:
            self.time_waffle_heat = (int)(self.ui.lineEdit_WaffleHeatingTime.text())
        except Exception as e:
            self.ui.textEdit_status.append(f"[ERROR]lineEdit_WaffleHeatingTime_textChanged error: {e}.\n") 

    def pushButton_SaveParameters_clicked(self):
        try:            
            parameters = {
                "ThermalThreshold": (int)(self.ui.lineEdit_ThermalThreshold.text()),
                "PressButtonTime": (int)(self.ui.lineEdit_PressButtonTime.text()),
                "GetSpoonTime": (int)(self.ui.lineEdit_GetSpoonTime.text()),
                "SpoonPeanutsTime": (int)(self.ui.lineEdit_SpoonPeanutsTime.text()),
                "DropSpoonTime": (int)(self.ui.lineEdit_DropSpoonTime.text()),
                "PeanutsHeatFlipTime": (int)(self.ui.lineEdit_PeanutsHeatFlipTime.text()),
                "PeanutsHeatingTime": (int)(self.ui.lineEdit_PeanutsHeatingTime.text()),
                "PeanutsFlipTime": (int)(self.ui.lineEdit_PeanutsFlipTime.text()),
                "Open1stLidTime": (int)(self.ui.lineEdit_Open1stLidTime.text()),
                "Open2ndLidTime": (int)(self.ui.lineEdit_Open2ndLidTime.text()),
                "Grab1stBatterTime": (int)(self.ui.lineEdit_Grab1stBatterTime.text()),
                "Grab2ndBatterTime": (int)(self.ui.lineEdit_Grab2ndBatterTime.text()),
                "Pour1stBatterTime": (int)(self.ui.lineEdit_Pour1stBatterTime.text()),
                "Pour2ndBatterTime": (int)(self.ui.lineEdit_Pour2ndBatterTime.text()),
                "Drop1stBatterTime": (int)(self.ui.lineEdit_Drop1stBatterTime.text()),
                "Drop2ndBatterTime": (int)(self.ui.lineEdit_Drop2ndBatterTime.text()),
                "Close1stLidTime": (int)(self.ui.lineEdit_Close1stLidTime.text()),
                "Close2ndLidTime": (int)(self.ui.lineEdit_Close2ndLidTime.text()),
                "GrabForkTime": (int)(self.ui.lineEdit_GrabForkTime.text()),
                "Get1stWaffleTime": (int)(self.ui.lineEdit_Get1stWaffleTime.text()),
                "Get2ndWaffleTime": (int)(self.ui.lineEdit_Get2ndWaffleTime.text()),
                "DropWaffleTime": (int)(self.ui.lineEdit_DropWaffleTime.text()),
                "WaffleHeatingTime": (int)(self.ui.lineEdit_WaffleHeatingTime.text()),
                "WaitForWaffleTime": (int)(self.ui.lineEdit_WaitForWaffleTime.text()),
                "WaitForWafflePourTime": (int)(self.ui.lineEdit_WaitForWafflePourTime.text())
            }

            with open('parameters.json', 'w') as file:
                json.dump(parameters, file, indent=4)

        except Exception as e:
            self.ui.textEdit_status.append(f"[ERROR]pushButton_SaveParameters_clicked error: {e}\n")
    #endregion

    def closeEvent(self, event: QCloseEvent):
        try:
            if self.cam:
                self.cam.quit()
        except Exception as e:
            self.ui.textEdit_status.append(f"[ERROR]cam.quit error: {e}\n")

        try:
            if self.graspGenCommunication:
                self.GraspGenCommunication_destroy()
        except Exception as e:
            self.ui.textEdit_status.append(f"[ERROR]GraspGenCommunication_destroy error: {e}\n")

        try:
            if self.rosCommunication:
                self.ros_destroy()
        except Exception as e:
            self.ui.textEdit_status.append(f"[ERROR]ros_destroy error: {e}\n")

        try:
            if self.tcp:
                self.tcp.close()
        except Exception as e:
            self.ui.textEdit_status.append(f"[ERROR]tcp.close error: {e}\n")

        try:
            if self.tcp_thermal:
                self.tcp_thermal.close()
        except Exception as e:
            self.ui.textEdit_status.append(f"[ERROR]tcp_thermal.close error: {e}\n")

        try:
            if self.tcp_check_empty_cup:
                self.tcp_check_empty_cup.close()
        except Exception as e:
            self.ui.textEdit_status.append(f"[ERROR]tcp_check_empty_cup.close error: {e}\n")

        try:
            if self.tcp_check_waffle_lid:
                self.tcp_check_waffle_lid.close()
        except Exception as e:
            self.ui.textEdit_status.append(f"[ERROR]tcp_check_waffle_lid.close error: {e}\n")

        self.serving_orders = False

        try:
            if self.thread_pan_position_check:
                self.thread_pan_position_check.join()
        except Exception as e:
            self.ui.textEdit_status.append(f"[ERROR]thread_pan_position_check join error: {e}\n")

        try:
            if self.thread_processing_orders:    
                self.thread_processing_orders.join()    
        except Exception as e:
            self.ui.textEdit_status.append(f"[ERROR]thread_processing_orders join error: {e}\n")        

        try:
            if self.thread_counting_left_time:
                self.thread_counting_left_time.join()  
        except Exception as e:
            self.ui.textEdit_status.append(f"[ERROR]thread_counting_left_time join error: {e}\n")  
    def open_web_panel(self):
        if not hasattr(self, "_web_panel"):
            self._web_panel = WebPanel(self)
        self._web_panel.show()
        self._web_panel.raise_()

    def on_status_changed(self,msg: str):
        print("QT UI received:", msg)  # ← 你一定會看到
        self.ui.textEdit_status.append(msg)

    def _emit_temp(self):
        temp = self.tcp_thermal.get_cur_temp()
        self.tempChanged.emit(temp)