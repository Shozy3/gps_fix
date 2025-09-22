import sys
from PySide6.QtWidgets import QApplication, QMainWindow
from PySide6.QtLocation import QGeoServiceProvider
from PySide6.QtPositioning import QGeoPositionInfoSource
from PySide6.QtQuick import QQuickWindow
from src.mainwindow import MainWindow
import logging

import src.ui.qrc_resources  # <--- Load embedded QML

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

if __name__ == "__main__":
    try:
        app = QApplication(sys.argv)
        window = MainWindow()
        window.show()
        logger.debug("MainWindow launched successfully.")
        sys.exit(app.exec())
    except Exception as e:
        import traceback
        logger.error("Exception occurred during startup:\n" +
                     traceback.format_exc())
