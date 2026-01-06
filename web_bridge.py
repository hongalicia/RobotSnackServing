from PySide6.QtCore import QObject, Slot, QThread, Signal
from action_worker import ActionWorker
import json
class WebBridge(QObject):
    statusChanged = Signal(str)  
    tempChanged = Signal(float)
    grabbingSpoonChanged = Signal(bool)
    peanutStatusChanged = Signal(str)
    def __init__(self, main_ctrl):
        super().__init__()
        self.ctrl = main_ctrl
        self._thread = None
        self.ctrl.statusChanged.connect(lambda msg: self.statusChanged.emit(str(msg)))
        self.ctrl.statusChanged.connect(self.statusChanged.emit)
        self.ctrl.tempChanged.connect(self.tempChanged.emit)
        self.ctrl.grabbingSpoonChanged.connect(self.grabbingSpoonChanged.emit)
        self.ctrl.peanutStatusChanged.connect(self.peanutStatusChanged.emit)



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
            elif act_name == "refill_peanut":
                return self.ctrl.grasp_and_dump_peanuts_flow
            return None

        # 判定是「單一動作」還是「自定義流程」
        if action.startswith("run_workflow:"):
            try:
                # 解析 JSON 陣列，例如 ["get_spoon", "spoon_peanuts"]
                action_list = json.loads(action.replace("run_workflow:", ""))
                
                # 定義一個組合函式，依序執行所有動作
                def workflow_executor():
                    for act in action_list:
                        func_to_run = get_target_func(act)
                        if func_to_run:
                            print(f"[WebBridge] 執行流程步驟: {act}")
                            func_to_run() # 執行該步驟 (需確保此動作是阻塞的，否則會同時執行)
                        else:
                            print(f"[WebBridge] 流程中包含未知動作: {act}")
                
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
