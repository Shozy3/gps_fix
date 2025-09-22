from PySide6.QtWidgets import QTextBrowser
from PySide6.QtCore import QObject, Signal, QTimer


class Bridge(QObject):
    lastData = Signal(object)

    def __init__(self, queue):
        super().__init__()
        self.queue = queue
        self.timer = QTimer()
        self.timer.timeout.connect(self.poll)
        self.timer.start(100)  # adjust poll rate (ms) as needed

    def poll(self):
        while not self.queue.empty():
            data = self.queue.get()
            self.lastData.emit(data)


class PrintStream(QObject):
    message_signal = Signal(str, str)

    def __init__(self, text_browser: QTextBrowser):
        super().__init__()
        self.text_browser = text_browser
        self.message_signal.connect(self.append_text)

    def write(self, message):
        if message.strip():
            self.message_signal.emit(message, "black")

    def flush(self):
        pass

    def print(self, message, color="black"):
        self.message_signal.emit(str(message), color)

    def append_text(self, message, color):
        # Wrap text in HTML span with color
        html = f'<span style="color:{color};">{message}</span><br>'
        self.text_browser.insertHtml(html)

        # Autoscroll to the bottom
        self.text_browser.verticalScrollBar().setValue(self.text_browser.verticalScrollBar().maximum())
