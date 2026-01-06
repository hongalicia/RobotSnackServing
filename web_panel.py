import os
from PySide6.QtWidgets import QWidget, QVBoxLayout
from PySide6.QtWebEngineWidgets import QWebEngineView
from PySide6.QtWebChannel import QWebChannel
from PySide6.QtCore import QUrl

from web_bridge import WebBridge

class WebPanel(QWidget):
    def __init__(self, main_ctrl):
        super().__init__()
        self.setWindowTitle("Web Control Panel")
        self.resize(420, 720)

        layout = QVBoxLayout(self)
        self.view = QWebEngineView()
        layout.addWidget(self.view)

        # WebChannel
        self.channel = QWebChannel()
        self.bridge = WebBridge(main_ctrl)
        self.channel.registerObject("bridge", self.bridge)
        self.view.page().setWebChannel(self.channel)

        html_path = os.path.abspath("ui_demo.html")
        self.view.load(QUrl.fromLocalFile(html_path))
