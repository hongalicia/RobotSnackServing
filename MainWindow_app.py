from MainWindow_ctrl import *
from PySide6.QtWidgets import QApplication

if __name__ == '__main__':
    import sys
    app = QApplication(sys.argv)
    window = main_window_ctrl()
    window.show()
    sys.exit(app.exec())