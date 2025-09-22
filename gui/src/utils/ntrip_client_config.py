from threading import Event
from PySide6.QtWidgets import QDialog
from pygnssutils import GNSSNTRIPClient
from src.ui.ui_ntrip_dialog import Ui_NtripDialog

from pygnssutils.globals import (
    NOGGA,
    NTRIP2,
)


class NtripClientConfig(QDialog):
    def __init__(self, parent=None, settings=None):
        super().__init__(parent)
        self.ui = Ui_NtripDialog()
        self.ui.setupUi(self)

        self.settings = settings  # Store settings here

        # Connect actions
        self.ui.buttonBox.accepted.disconnect()
        self.ui.buttonBox.accepted.connect(self.save_settings)
        self.ui.buttonBox.rejected.connect(self.reject)

        self.ntrip_client = GNSSNTRIPClient()
        self._stopevent = Event()

        if settings != {}:
            self.load_settings(settings)

    def save_settings(self):
        # Fetch data from UI
        host = self.ui.server.toPlainText()
        port = self.ui.port.toPlainText()
        username = self.ui.user.toPlainText()
        password = self.ui.password.toPlainText()
        https = self.ui.https.isChecked()
        datatype = self.ui.datatype.currentText()
        version = self.ui.version.currentText()
        ggainterval = self.ui.ggainterval.toPlainText()
        if version == "2.0":
            version = NTRIP2

        sourcetable = self.ui.sourcetable.currentText()
        self.settings = {"server": host, "port": port, "ntripuser": username,
                         "ntrippassword": password, "https": https, "datatype": datatype,
                         "version": version, "ggainterval": ggainterval, "mountpoint": sourcetable}

        # You could do basic validation here
        if not host or not sourcetable:
            self.settings['ggainterval'] = NOGGA
            self.ntrip_client._read_thread(
                self.settings, self._stopevent, None)
            result = self.ntrip_client.settings
            sources = result.get("sourcetable")

            for mountpoint in sources:
                self.ui.sourcetable.addItem(mountpoint[0])
                print("Mountpoint: ", mountpoint)
            # Return without closing the dialog
            return

        self.accept()  # Close the dialog and mark it as accepted

    def load_settings(self, settings):
        # Load settings into the UI
        self.ui.server.setPlainText(settings.get("server", ""))
        self.ui.port.setPlainText(settings.get("port", ""))
        self.ui.user.setPlainText(settings.get("ntripuser", ""))
        self.ui.password.setPlainText(settings.get("ntrippassword", ""))
        self.ui.https.setChecked(settings.get("https", False))
        self.ui.datatype.setCurrentText(settings.get("datatype", ""))
        self.ui.version.setCurrentText(settings.get("version", ""))
        self.ui.ggainterval.setPlainText(settings.get("ggainterval", ""))
        self.ui.sourcetable.setCurrentText(settings.get("mountpoint", ""))

    def get_settings(self):
        return self.settings
