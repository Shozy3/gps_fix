import os
import cv2
import math
import datetime
import numpy as np
from multiprocessing import Process, Queue
from PySide6.QtSerialPort import QSerialPortInfo
from PySide6.QtGui import QDesktopServices, QPixmap, Qt, QImage
from PySide6.QtCore import QUrl, Slot, QTimer, QSettings, Signal, QObject, QThread, Qt
from PySide6.QtWidgets import QMessageBox, QMainWindow, QFileDialog, QInputDialog, QMessageBox, QLineEdit, QDialog

from src.utils.ublox import Ublox
from src.utils.map import MapInterface
from src.utils.camera_zed import CameraZed
from src.ui.ui_mainwindow import Ui_mainWindow
from src.utils.helpers import Bridge, PrintStream
from src.utils.camera_depthai import CameraDepthai
from src.utils.ntrip_client_config import NtripClientConfig


RAD_TO_DEG = 180.0 / np.pi


class FrameProcessor(QObject):
    frame_processed_signal = Signal(QPixmap)

    @Slot(object)  # Slot to receive the raw cv2.Mat frame
    def process_frame(self, cv_img):
        if cv_img is not None:
            q_image = self.convert_cv_qt(cv_img)
            self.frame_processed_signal.emit(q_image)

    def convert_cv_qt(self, cv_img):
        """Convert from an opencv image to QPixmap"""
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        convert_to_Qt_format = QImage(
            rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
        p = convert_to_Qt_format.scaled(
            1239, 629, Qt.KeepAspectRatio)
        return QPixmap.fromImage(p)


class MainWindow(QMainWindow):
    recording_path = "/home/Desktop/AT"
    password = ""
    ntrip_details = {}

    def __init__(self, parent=None):
        super().__init__(parent)
        self.ui = Ui_mainWindow()
        self.ui.setupUi(self)
        self.current_index = 0

        self.load_settings()

        # Set initial values
        self.ui.gpsSerial.addItem("None")

        # Populate available serial ports
        for serial_port in QSerialPortInfo.availablePorts():
            self.ui.gpsSerial.addItem(
                f"{serial_port.portName()} - {serial_port.manufacturer()}")

        self.gps_error_queue = Queue()
        self.video_depthai_error_queue = Queue()
        self.video_zed_error_queue = Queue()  # New error queue for ZED

        self.error_timer = QTimer()
        self.error_timer.timeout.connect(self.check_error_queues)
        self.error_timer.start(1000)  # check every 1 second

        self.printer = PrintStream(self.ui.outputScreen)

        # Camera
        self.processing_thread_depthai = QThread()
        self.frame_processor_depthai = FrameProcessor()
        self.frame_processor_depthai.moveToThread(self.processing_thread_depthai)
        self.processing_thread_depthai.start()  # Start the worker thread

        self.processing_thread_zed = QThread()
        self.frame_processor_zed = FrameProcessor()
        self.frame_processor_zed.moveToThread(self.processing_thread_zed)
        self.processing_thread_zed.start() 

        # Initialize CameraDepthai and its thread
        self.camera_depthai_thread = QThread()
        self.camera_depthai = CameraDepthai(
            f"{MainWindow.recording_path}/{datetime.date.today().strftime('%Y-%m-%d')}/depthai", self.video_depthai_error_queue)
        self.camera_depthai.moveToThread(self.camera_depthai_thread)

        # Connect the control command signal from CameraDepthai to its own slot
        self.camera_depthai.control_command_signal.connect(
            self.camera_depthai._receive_control_command, Qt.QueuedConnection)

        # Connect the raw frame signal from camera to the processor's slot
        self.camera_depthai.previewframesignal.connect(
            self.frame_processor_depthai.process_frame)
        # Connect the processed QPixmap signal from processor to the display slot in MainWindow
        self.frame_processor_depthai.frame_processed_signal.connect(
            self.display_depthai_frame)

        # Initialize CameraZed and its thread
        self.camera_zed_thread = QThread()
        self.camera_zed = CameraZed(
            f"{MainWindow.recording_path}/{datetime.date.today().strftime('%Y-%m-%d')}/zed", self.video_zed_error_queue)
        self.camera_zed.moveToThread(self.camera_zed_thread)

        # NEW: Connect ZED preview frames to processor
        self.camera_zed.previewframesignal.connect(
            self.frame_processor_zed.process_frame)
        self.frame_processor_zed.frame_processed_signal.connect(
            self.display_zed_frame)

        # Connect actions
        self.ui.actionQuit.triggered.connect(self.close)
        self.ui.actionAbout.triggered.connect(self.show_about)
        self.ui.actionRecording_Path.triggered.connect(self.set_recording_path)
        self.ui.actionSet_Password.triggered.connect(self.set_password)
        self.ui.actionNTRIP_Configuration.triggered.connect(
            self.set_ntrip_details)
        self.ui.next_values.clicked.connect(self.on_next_clicked)
        self.ui.next_depthai.clicked.connect(self.on_next_clicked)
        self.ui.next_zed.clicked.connect(self.on_next_clicked)
        self.ui.prev_values.clicked.connect(self.on_prev_clicked)
        self.ui.prev_zed.clicked.connect(self.on_prev_clicked)
        self.ui.prev_depthai.clicked.connect(self.on_prev_clicked)
        self.ui.btnGPSStatus.clicked.connect(self.on_btnGPSStatus_clicked)
        self.ui.initializeDepthai.clicked.connect(
            self.on_initializeDepthai_clicked)
        self.ui.initializeZed.clicked.connect(self.on_initializeZed_clicked)
        self.ui.startrecDepthai.clicked.connect(
            self.on_startrecDepthai_clicked)
        self.ui.stoprecDepthai.clicked.connect(self.on_stoprecDepthai_clicked)
        # NEW: Connect ZED recording buttons
        self.ui.startrecZed.clicked.connect(self.on_startrecZed_clicked)
        self.ui.stoprecZed.clicked.connect(self.on_stoprecZed_clicked)

        # Setup Map
        self.map_iface = MapInterface()
        ctx = self.ui.mapwidget.rootContext()
        ctx.setContextProperty("mapInterface", self.map_iface)
        self.ui.mapwidget.setSource(QUrl("qrc:/src/ui/map.qml"))
        self.qml_obj = self.ui.mapwidget.rootObject()

        # Connect signal from map_iface to QML function
        self.map_iface.marker1.connect(
            lambda lat, lon: self.qml_obj.setLocationMarker_1(lat, lon)
        )

    def closeEvent(self, event):
        self.printer.print(
            "Application closing. Attempting graceful shutdown...", "blue")

        # Stop DepthAI camera
        if self.camera_depthai_thread.isRunning():
            self.camera_depthai.stop_camera()
            self.camera_depthai_thread.quit()
            self.camera_depthai_thread.wait(3000)
            if self.camera_depthai_thread.isRunning():
                self.printer.print(
                    "Warning: Camera Depthai QThread did not terminate cleanly.", "orange")
            else:
                self.printer.print(
                    "Camera Depthai QThread terminated.", "green")

        # NEW: Stop ZED camera
        if self.camera_zed_thread.isRunning():
            self.camera_zed.stop_camera()
            self.camera_zed_thread.quit()
            self.camera_zed_thread.wait(3000)
            if self.camera_zed_thread.isRunning():
                self.printer.print(
                    "Error: Camera ZED QThread did not terminate cleanly.", "orange")
            else:
                self.printer.print("Success: Camera ZED QThread terminated.", "green")

        # Stop frame processing thread
        if self.processing_thread_depthai.isRunning():
            self.processing_thread_depthai.quit()
            self.processing_thread_depthai.wait(1000)
            if self.processing_thread_depthai.isRunning():
                self.printer.print(
                    "Error: Frame Processor QThread did not terminate cleanly.", "orange")
            else:
                self.printer.print(
                    "Success: Frame Processor QThread terminated.", "green")
                
        # Stop frame processing thread
        if self.processing_thread_zed.isRunning():
            self.processing_thread_zed.quit()
            self.processing_thread_zed.wait(1000)
            if self.processing_thread_zed.isRunning():
                self.printer.print(
                    "Error: Frame Processor QThread did not terminate cleanly.", "orange")
            else:
                self.printer.print(
                    "Success: Frame Processor QThread terminated.", "green")

        # Terminate GPS process if running
        if hasattr(self, "gps_process") and self.gps_process.is_alive():
            self.printer.print("Stopping GPS process...", "blue")
            self.gps_process.terminate()
            self.gps_process.join(timeout=2)
            if self.gps_process.is_alive():
                self.printer.print(
                    "Error: GPS process did not terminate cleanly.", "orange")
            else:
                self.printer.print("GPS process stopped.", "green")

        # Clean up GPS queues/bridges if they exist
        if hasattr(self, "gps_queue") and self.gps_queue:
            self.gps_queue.close()

        if hasattr(self, "gps_bridge") and self.gps_bridge:
            try:
                self.gps_bridge.lastData.disconnect(self.displayGPSData)
            except RuntimeError:
                pass
            self.gps_bridge.deleteLater()

        self.printer.print("Application shutdown complete.", "blue")
        super().closeEvent(event)

    def load_settings(self):
        settings = QSettings("CoeRPP", "CoeRPP GUI")
        MainWindow.recording_path = settings.value(
            "recording_path", MainWindow.recording_path)
        MainWindow.password = settings.value("password", MainWindow.password)
        MainWindow.ntrip_details = settings.value(
            "ntrip_details", MainWindow.ntrip_details)

    def save_settings(self):
        settings = QSettings("CoeRPP", "CoeRPP GUI")
        settings.setValue("recording_path", MainWindow.recording_path)
        settings.setValue("password", MainWindow.password)
        settings.setValue("ntrip_details", MainWindow.ntrip_details)

    def show_about(self):
        QMessageBox.about(self, "Residential Parking Project Recording Program",
                          "This is a GUI Program to record GPS and Video data with ease.\n"
                          "Currently running version 1.0.0.\n"
                          "Credits: Krupal Shah")

    def set_recording_path(self):
        directory = QFileDialog.getExistingDirectory(
            self, "Select Recording Directory")
        if directory:
            MainWindow.recording_path = directory
            print("Recording path set to:", MainWindow.recording_path)
            self.save_settings()

    def set_password(self):
        password, ok = QInputDialog.getText(self, "Authentication", "Enter your password:",
                                            QLineEdit.Password)
        if ok and password:
            print("Password entered:", password)
            MainWindow.password = password
            self.save_settings()
        else:
            print("Password input cancelled or empty.")

    def set_ntrip_details(self):
        dialog = NtripClientConfig(settings=MainWindow.ntrip_details)
        if dialog.exec_() == QDialog.Accepted:
            settings = dialog.get_settings()
            MainWindow.ntrip_details = settings
            self.save_settings()

    @Slot()
    def on_next_clicked(self):
        self.current_index += 1
        if self.current_index >= self.ui.stackedDisplay.count():
            self.current_index = 0
        self.ui.stackedDisplay.setCurrentIndex(self.current_index)

    @Slot()
    def on_prev_clicked(self):
        self.current_index -= 1
        if self.current_index < 0:
            self.current_index = self.ui.stackedDisplay.count() - 1
        self.ui.stackedDisplay.setCurrentIndex(self.current_index)

    @Slot()
    def on_btnGPSStatus_clicked(self):
        QDesktopServices.openUrl(QUrl("http://192.168.3.1"))

    @Slot()
    def on_recordingFolderbtn_clicked(self):
        QDesktopServices.openUrl(QUrl.fromLocalFile(MainWindow.recording_path))

    def format_float(value, precision=6):
        try:
            return f"{float(value):.{precision}f}"
        except (ValueError, TypeError):
            return ""

    @Slot(dict)
    def displayGPSData(self, data):
        def format_float(value, precision=6, radians_to_degrees=False):
            try:
                val = float(value)
                if radians_to_degrees:
                    val = math.degrees(val)
                return f"{val:.{precision}f}"
            except (ValueError, TypeError):
                return ""

        self.map_iface.add_marker(data.get("lat"), data.get("lon"))

        self.ui.systemTimeGPS.setPlainText(data.get("systemtime", ""))
        self.ui.GPSTime.setPlainText(data.get("gpstime", ""))
        self.ui.latitude.setPlainText(format_float(data.get("lat")))
        self.ui.longitude.setPlainText(format_float(data.get("lon")))
        self.ui.altitude.setPlainText(format_float(data.get("alt")))
        self.ui.heading.setPlainText(format_float(data.get("azimuth")))
        self.ui.fix.setPlainText(data.get("fix", ""))

        self.ui.diffAge.setPlainText(data.get("diffage", ""))
        self.ui.diffStation.setPlainText(data.get("diffstation", ""))

        self.ui.hAccu.setPlainText(format_float(data.get("2D hAcc")))
        self.ui.vAccu.setPlainText(format_float(data.get("2D vAcc")))
        self.ui.acc3D.setPlainText(format_float(data.get("3D Acc")))
        self.ui.GPSFix.setPlainText(data.get("gpsFix", ""))
        self.ui.numSat.setPlainText(data.get("numSV", ""))
        self.ui.HDOP.setPlainText(format_float(data.get("HDOP")))
        self.ui.VDOP.setPlainText(format_float(data.get("VDOP")))
        self.ui.PDOP.setPlainText(format_float(data.get("PDOP")))
        self.ui.fusionMode.setPlainText(data.get("fusionMode", ""))

        self.ui.calibGyroX.setPlainText(data.get("gyroX_calib", ""))
        self.ui.calibGyroY.setPlainText(data.get("gyroY_calib", ""))
        self.ui.calibGyroZ.setPlainText(data.get("gyroZ_calib", ""))
        self.ui.calibAccX.setPlainText(data.get("accZ_calib", ""))
        self.ui.calibAccY.setPlainText(data.get("accY_calib", ""))
        self.ui.calibAccZ.setPlainText(data.get("accZ_calib", ""))

    @Slot()
    def on_serialConnectionButton_clicked(self):
        gpsport = self.ui.gpsSerial.currentText().split('-')[0].strip()
        gpsbaud = int(self.ui.baudGPS.currentText())

        save = self.ui.saveButton.isChecked()
        gps = False
        ntrip_details = MainWindow.ntrip_details
        ntrip_details["start"] = self.ui.ntripConnection.isChecked()

        if gpsport != "None" and gpsbaud != 0:
            gps = True
            gpsport = f"/dev/{gpsport}"
            self.gps_queue = Queue()
            self.gps_bridge = Bridge(self.gps_queue)
            self.gps_bridge.lastData.connect(self.displayGPSData)

            try:
                recording_path_gps = f"{MainWindow.recording_path}/{datetime.date.today().strftime('%Y-%m-%d')}/gps" if save else None
                if recording_path_gps:
                    os.makedirs(recording_path_gps, exist_ok=True)

                self.gps_process = Process(
                    target=Ublox,
                    kwargs={
                        "gps_port": gpsport,
                        "baud_rate": gpsbaud,
                        "fusion": True,
                        "save_data": save,
                        "save_path": recording_path_gps,
                        "ntrip_details": MainWindow.ntrip_details,
                        "gps_queue": self.gps_queue,
                        "gps_error_queue": self.gps_error_queue,
                        "display_timer": 0.1,
                    }
                )
                self.gps_process.start()
                self.printer.print(
                    "Success: GPS process started successfully.", "green")

            except Exception as e:
                self.printer .print(
                    f"Error: GPS Connection not found or error: {e}", "red")
                gps = False

        if gps:
            self.ui.serialConnectionButton.setEnabled(False)
            self.ui.serialTerminationButton.setEnabled(True)
        else:
            self.printer.print(
                "Error: GPS not initialized. Check port/baud rate.", "orange")

    @Slot()
    def on_serialTerminationButton_clicked(self):
        # Terminate GPS process
        if hasattr(self, "gps_process") and self.gps_process.is_alive():
            self.printer.print(
                "Process: Stopping GPS process...", "black")
            self.gps_process.terminate()
            self.gps_process.join()
            self.printer.print(
                "Success: GPS process stopped.", "green")

        # Clean up queues and bridges
        if hasattr(self, "gps_queue"):
            self.gps_queue.close()

        if hasattr(self, "gps_bridge"):
            try:
                self.gps_bridge.lastData.disconnect(self.displayGPSData)
            except RuntimeError:
                pass
            self.gps_bridge.deleteLater()

        # UI buttons
        self.ui.serialConnectionButton.setEnabled(True)
        self.ui.serialTerminationButton.setEnabled(False)

    def check_error_queues(self):
        while not self.gps_error_queue.empty():
            err = self.gps_error_queue.get()
            if "Serial port error" in err:
                self.printer.print(err, "red")
            else:
                self.printer.print(err, "orange")

        while not self.video_depthai_error_queue.empty():
            err = self.video_depthai_error_queue.get()
            if "Error" in err:
                self.printer.print(err, "red")
            elif "Success" in err:
                self.printer.print(err, "green")
            elif "Comment" in err:
                self.printer.print(err, "blue")
            elif "Process" in err:
                self.printer.print(err, "black")

        while not self.video_zed_error_queue.empty():
            err = self.video_zed_error_queue.get()
            if "Error" in err:
                self.printer.print(err, "red")
            elif "Success" in err:
                self.printer.print(err, "green")
            elif "Comment" in err:
                self.printer.print(err, "blue")
            elif "Process" in err:
                self.printer.print(err, "black")

    @Slot()
    def on_initializeDepthai_clicked(self):
        if not self.camera_depthai_thread.isRunning():
            resolution = self.ui.resolutionDepthai.currentText()
            fps = int(self.ui.fpsDepthai.currentText())

            # Start the QThread first.
            self.camera_depthai_thread.start()

            # Now, call the setup method on the CameraDepthai instance.
            self.camera_depthai.start_camera_setup(resolution, fps)

            # Connect camera control buttons to CameraDepthai methods
            self.ui.exposureDepthaiAdd.clicked.connect(
                self.camera_depthai.add_exposure)
            self.ui.exposureDepthaiSub.clicked.connect(
                self.camera_depthai.subtract_exposure)
            self.ui.ISODepthaiAdd.clicked.connect(self.camera_depthai.add_iso)
            self.ui.ISODepthaiSub.clicked.connect(
                self.camera_depthai.subtract_iso)
            self.ui.focusDepthaiAdd.clicked.connect(
                self.camera_depthai.add_focus)
            self.ui.focusDepthaiSub.clicked.connect(
                self.camera_depthai.subtract_focus)
            self.ui.brightnessDepthaiAdd.clicked.connect(
                self.camera_depthai.add_brightness)
            self.ui.brightnessDepthaiSub.clicked.connect(
                self.camera_depthai.subtract_brightness)
            self.ui.contrastDepthaiAdd.clicked.connect(
                self.camera_depthai.add_contrast)
            self.ui.contrastDepthaiSub.clicked.connect(
                self.camera_depthai.subtract_contrast)
            self.ui.saturationDepthaiAdd.clicked.connect(
                self.camera_depthai.add_saturation)
            self.ui.saturationDepthaiSub.clicked.connect(
                self.camera_depthai.subtract_saturation)
            self.ui.sharpnessDepthaiAdd.clicked.connect(
                self.camera_depthai.add_sharpness)
            self.ui.sharpnessDepthaiSub.clicked.connect(
                self.camera_depthai.subtract_sharpness)

            # Connect Auto buttons
            self.ui.autoexposureDepthai.clicked.connect(
                self.camera_depthai.enable_auto_exposure)
            self.ui.autofocusDepthai.clicked.connect(
                self.camera_depthai.enable_continuous_af)
            self.ui.autowbDepthai.clicked.connect(
                self.camera_depthai.enable_auto_wb)

            # Connect WASD/ROI buttons
            self.ui.rosDepthaiUp.clicked.connect(
                self.camera_depthai.move_roi_up)
            self.ui.rosDepthaiDown.clicked.connect(
                self.camera_depthai.move_roi_down)
            self.ui.rosDepthaiLeft.clicked.connect(
                self.camera_depthai.move_roi_left)
            self.ui.rosDepthaiRight.clicked.connect(
                self.camera_depthai.move_roi_right)

            # Update the UI states for recording buttons
            self.ui.startrecDepthai.setEnabled(True)
            self.ui.stoprecDepthai.setEnabled(False)

    @Slot()
    def on_startrecDepthai_clicked(self):
        if self.camera_depthai.device:
            self.camera_depthai.start_recording()
            self.ui.startrecDepthai.setEnabled(False)
            self.ui.stoprecDepthai.setEnabled(True)

    @Slot()
    def on_stoprecDepthai_clicked(self):
        if self.camera_depthai.recording:
            self.camera_depthai.stop_recording()
            self.ui.startrecDepthai.setEnabled(True)
            self.ui.stoprecDepthai.setEnabled(False)

    @Slot()
    def on_initializeZed_clicked(self):
        if not self.camera_zed_thread.isRunning():
            # Assuming you have a resolutionZed QComboBox
            resolution = self.ui.resolutionZed.currentText()
            # Assuming you have a fpsZed QComboBox
            fps = int(self.ui.fpsZed.currentText())

            # Start the QThread for ZED camera
            self.camera_zed_thread.start()

            # Call the setup method on the CameraZed instance (which is now in its own thread)
            self.camera_zed.start_camera_setup(resolution, fps)

            self.ui.brightnessZedAdd.clicked.connect(
                self.camera_zed.add_brightness)
            self.ui.brightnessZedSub.clicked.connect(
                self.camera_zed.subtract_brightness)
            self.ui.contrastZedAdd.clicked.connect(
                self.camera_zed.add_contrast)
            self.ui.contrastZedSub.clicked.connect(
                self.camera_zed.subtract_contrast)
            self.ui.saturationZedAdd.clicked.connect(
                self.camera_zed.add_saturation)
            self.ui.saturationZedSub.clicked.connect(
                self.camera_zed.subtract_saturation)
            self.ui.sharpnessZedAdd.clicked.connect(
                self.camera_zed.add_sharpness)
            self.ui.sharpnessZedSub.clicked.connect(
                self.camera_zed.subtract_sharpness)
            self.ui.gainZedAdd.clicked.connect(self.camera_zed.add_gain)
            self.ui.gainZedSub.clicked.connect(self.camera_zed.subtract_gain)
            self.ui.exposureZedAdd.clicked.connect(
                self.camera_zed.add_exposure)
            self.ui.exposureZedSub.clicked.connect(
                self.camera_zed.subtract_exposure)
            self.ui.hueZedAdd.clicked.connect(
                self.camera_zed.add_hue)
            self.ui.hueZedSub.clicked.connect(
                self.camera_zed.subtract_hue)

            # Connect ZED Auto/Reset/LED buttons
            self.ui.autobrightnessZed.clicked.connect(
                self.camera_zed.enable_auto_brightness)
            self.ui.autoexposureZed.clicked.connect(
                self.camera_zed.enable_auto_exposure)
            self.ui.resetZed.clicked.connect(
                self.camera_zed.reset_all_settings)

            # Update the UI states for recording buttons
            self.ui.startrecZed.setEnabled(True)
            self.ui.stoprecZed.setEnabled(False)

    @Slot()
    def on_startrecZed_clicked(self):
        if self.camera_zed.device and self.camera_zed.device.is_opened():
            self.camera_zed.start_recording()
            self.ui.startrecZed.setEnabled(False)
            self.ui.stoprecZed.setEnabled(True)

    @Slot()
    def on_stoprecZed_clicked(self):
        if self.camera_zed.recording:
            self.camera_zed.stop_recording()
            self.ui.startrecZed.setEnabled(True)
            self.ui.stoprecZed.setEnabled(False)

    @Slot(QPixmap)
    def display_depthai_frame(self, q_image):
        if q_image is not None:
            self.ui.videoDepthai.setPixmap(q_image)
            self.ui.videoDepthai.setScaledContents(True)
        else:
            print("No frame received from Depthai camera.")

    @Slot(QPixmap)
    def display_zed_frame(self, q_image):
        if q_image is not None:
            self.ui.videoZed.setPixmap(q_image)
            self.ui.videoZed.setScaledContents(True)
        else:
            print("No frame received from Depthai camera.")
