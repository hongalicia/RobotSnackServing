# order_panel.py
from PySide6.QtWidgets import QWidget, QHBoxLayout, QVBoxLayout, QLabel, QListWidget, QFrame
from PySide6.QtGui import QFont
from PySide6.QtCore import Qt

class OrderListPanel(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("訂單監控面板 (KDS)")
        self.resize(1000, 600)
        self.setup_ui()

    def setup_ui(self):
        main_layout = QHBoxLayout(self)
        
        # ===== 左側：正在處理的訂單 =====
        left_frame = QFrame()
        left_frame.setStyleSheet("background-color: #f8f9fa; border-radius: 10px;")
        left_layout = QVBoxLayout(left_frame)
        
        title_current = QLabel("🧑‍🍳 正在處理的訂單")
        title_current.setFont(QFont("Microsoft JhengHei", 24, QFont.Bold))
        title_current.setAlignment(Qt.AlignCenter)
        title_current.setStyleSheet("color: #333333; margin-top: 20px;")
        
        self.label_current_order = QLabel("目前無訂單，等待中...")
        self.label_current_order.setFont(QFont("Microsoft JhengHei", 40, QFont.Bold))
        self.label_current_order.setAlignment(Qt.AlignCenter)
        self.label_current_order.setStyleSheet("color: #7f8c8d;")
        
        left_layout.addWidget(title_current)
        left_layout.addStretch()
        left_layout.addWidget(self.label_current_order)
        left_layout.addStretch()

        # ===== 右側：排隊中的訂單 (Queue) =====
        right_frame = QFrame()
        right_layout = QVBoxLayout(right_frame)
        
        title_queue = QLabel("📋 排隊中的訂單")
        title_queue.setFont(QFont("Microsoft JhengHei", 18, QFont.Bold))
        
        self.list_queue = QListWidget()
        self.list_queue.setFont(QFont("Microsoft JhengHei", 16))
        self.list_queue.setStyleSheet("""
            QListWidget { background-color: #ffffff; border: 2px solid #cccccc; border-radius: 5px; }
            QListWidget::item { padding: 15px; border-bottom: 1px solid #eeeeee; }
        """)
        
        right_layout.addWidget(title_queue)
        right_layout.addWidget(self.list_queue)
        
        # 將左右加入主畫面，設定比例 6 : 4
        main_layout.addWidget(left_frame, 6)
        main_layout.addWidget(right_frame, 4)

    # --- 接收主程式訊號的 Slot 方法 ---
    def update_current(self, table, peanuts, waffle):
        if table == -1:
            self.label_current_order.setText("目前無訂單，等待中...")
            self.label_current_order.setStyleSheet("color: #7f8c8d;")
        else:
            self.label_current_order.setText(f"桌號: {table}\n\n🥜 花生: {peanuts} 份\n🧇 鬆餅: {waffle} 份")
            self.label_current_order.setStyleSheet("color: #27ae60;") 

    def update_queue(self, order_strings):
        self.list_queue.clear()
        self.list_queue.addItems(order_strings)