from PySide6.QtCore import QObject, Slot, QThread, Signal
from action_worker import ActionWorker
import json
class WebBridge(QObject):
    statusChanged = Signal(str)  
    tempChanged = Signal(float)
    grabbingSpoonChanged = Signal(bool)
    peanutStatusChanged = Signal(str)
    receiveOrder = Signal(str)
    def __init__(self, main_ctrl):
        super().__init__()
        self.ctrl = main_ctrl
        self._thread = None
        self.ctrl.statusChanged.connect(lambda msg: self.statusChanged.emit(str(msg)))
        self.ctrl.statusChanged.connect(self.statusChanged.emit)
        self.ctrl.tempChanged.connect(self.tempChanged.emit)
        self.ctrl.grabbingSpoonChanged.connect(self.grabbingSpoonChanged.emit)
        self.ctrl.peanutStatusChanged.connect(self.peanutStatusChanged.emit)
        self.ctrl.receiveOrder.connect(lambda msg: self.receiveOrder.emit(str(msg)))


    @Slot(str)
    def sendAction(self, action: str):
        # 檢查系統是否忙碌
        if self._thread is not None:
            self.statusChanged.emit("[WARNING] 系統忙碌中，請稍候")
            return

        # 定義一個內部函式，用來根據字串回傳對應的執行 function
        def get_target_func(act_name):
            if act_name == "get_spoon":
                return self.ctrl.get_spoon_flow
            elif act_name == "drop_spoon":
                return self.ctrl.drop_spoon_flow
            elif act_name == "spoon_peanuts":
                return self.ctrl.spoon_peanuts_flow
            elif act_name == "serve_peanuts":
                return lambda: self.ctrl.serve_peanuts(1)
            elif act_name == "serve_waffle":
                return lambda: self.ctrl.serve_waffle(4)
            elif act_name == "refill_peanut":
                return self.ctrl.grasp_and_dump_peanuts_flow
            elif act_name == "pan_heat":
                return self.ctrl.pan_heat
            elif act_name == "Grab1stBatter":
                return self.ctrl.Grab1stBatter_flow
            elif act_name == "Pour1stBatter":
                return self.ctrl.Pour1stBatter_flow
            elif act_name == "Drop1stBatter":
                return self.ctrl.Drop1stBatter_flow
            elif act_name == "Grab2ndBatter":
                return self.ctrl.Grab2ndBatter_flow
            elif act_name == "Pour2ndBatter":
                return self.ctrl.Pour2ndBatter_flow
            elif act_name == "Drop2ndBatter":
                return self.ctrl.Drop2ndBatter_flow
            elif act_name == "GrabFork":
                return self.ctrl.GrabFork_flow
            elif act_name == "Open1stLid":
                return self.ctrl.Open1stLid_flow
            elif act_name == "Close1stLid":
                return self.ctrl.Close1stLid_flow
            elif act_name == "Open2ndLid":
                return self.ctrl.Open2ndLid_flow
            elif act_name == "Close2ndLid":
                return self.ctrl.Close2ndLid_flow
            elif act_name == "WaitWaffle":
                return self.ctrl.wait_for_waffle_pour
            elif act_name == "serve_1st_stove":
                return self.ctrl.serve_1st_stove_flow
            elif act_name == "serve_2nd_stove":
                return self.ctrl.serve_2nd_stove_flow
            else:
                return None


        # 判定是「單一動作」還是「自定義流程」
        if action.startswith("run_workflow:"):
            try:
                # 解析 JSON 陣列，例如 ["get_spoon", "spoon_peanuts"]
                action_list = json.loads(action.replace("run_workflow:", ""))
                
                # 定義一個組合函式，依序執行所有動作
                def workflow_executor():
                    self.statusChanged.emit("系統訊息：啟動自定義程序")

                    for step in action_list:

                        # -------- 一般 step（字串）--------
                        if isinstance(step, str):
                            func_to_run = get_target_func(step)
                            if func_to_run:
                                print(f"[WebBridge] 執行流程步驟: {step}")
                                func_to_run()
                            else:
                                print(f"[WebBridge] 未知流程步驟: {step}")

                        # -------- 參數 step（dict）--------
                        elif isinstance(step, dict):
                            step_name = step.get("step")

                            if step_name == "WaitSeconds":
                                seconds = step.get("seconds", 0)
                                self.statusChanged.emit(f"[INFO] ⏱ 等待 {seconds} 秒")
                                self.ctrl.wait_time(seconds)
                            else:
                                print(f"[WebBridge] 未知參數流程: {step}")

                    self.statusChanged.emit("系統訊息：完成自定義程序")
                
                final_func = workflow_executor
            except Exception as e:
                print(f"[WebBridge] JSON 解析錯誤: {e}")
                return
        else:
            # 單一動作處理
            final_func = get_target_func(action)

        # 如果找不到對應函式則退出
        if final_func is None:
            print(f"[WebBridge] 未知動作: {action}")
            return

        # --- 執行 Thread 邏輯 (保持你原本的寫法) ---
        thread = QThread()
        worker = ActionWorker(final_func)
        worker.moveToThread(thread)

        thread.started.connect(worker.run)
        worker.finished.connect(thread.quit)
        worker.error.connect(lambda e: print("[WebBridge error]", e))

        def cleanup():
            worker.deleteLater()
            thread.deleteLater()
            self._thread = None

        thread.finished.connect(cleanup)
        self._thread = thread
        thread.start()
