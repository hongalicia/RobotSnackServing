from PySide6.QtCore import QObject, Signal, Slot


class ActionWorker(QObject):
    finished = Signal()
    error = Signal(str)

    def __init__(self, func):
        super().__init__()
        self.func = func

    @Slot()
    def run(self):
        try:
            self.func()
            self.finished.emit()
        except Exception as e:
            self.error.emit(str(e))
