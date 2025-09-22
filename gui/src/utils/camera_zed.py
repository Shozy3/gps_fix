import os
import cv2
import csv
import time
import queue
import threading
import pyzed.sl as sl
from datetime import datetime, timezone
from PySide6.QtCore import QObject, Signal, Slot  # Corrected import for PyQt5

# Helper function to clamp values within a range


def clamp(value, min_val, max_val):
    return max(min_val, min(value, max_val))


RESOLUTION = {
    '720p': sl.RESOLUTION.HD720,
    '1080p': sl.RESOLUTION.HD1080,
    '2K': sl.RESOLUTION.HD2K,
}


class TimestampHandler:
    def __init__(self):
        self.t_imu = sl.Timestamp()
        self.t_baro = sl.Timestamp()
        self.t_mag = sl.Timestamp()

    def is_new(self, sensor):
        if isinstance(sensor, sl.IMUData):
            new_ = (sensor.timestamp.get_microseconds()
                    > self.t_imu.get_microseconds())
            if new_:
                self.t_imu = sensor.timestamp
            return new_
        elif isinstance(sensor, sl.MagnetometerData):
            new_ = (sensor.timestamp.get_microseconds()
                    > self.t_mag.get_microseconds())
            if new_:
                self.t_mag = sensor.timestamp
            return new_
        elif isinstance(sensor, sl.BarometerData):
            new_ = (sensor.timestamp.get_microseconds()
                    > self.t_baro.get_microseconds())
            if new_:
                self.t_baro = sensor.timestamp
            return new_
        return False


class CameraZed(QObject):
    previewframesignal = Signal(cv2.Mat)
    imu_data_signal = Signal(dict)
    brightness_updated_signal = Signal(int)
    contrast_updated_signal = Signal(int)
    hue_updated_signal = Signal(int)
    saturation_updated_signal = Signal(int)
    sharpness_updated_signal = Signal(int)
    gain_updated_signal = Signal(int)
    exposure_updated_signal = Signal(int)
    wb_temperature_updated_signal = Signal(int)
    led_status_updated_signal = Signal(bool)
    roi_aec_agc_updated_signal = Signal(int, int, int, int)

    def __init__(self, folder_name, error_queue):
        super().__init__()
        self.device = None
        self.sensor = sl.SensorsData()
        self.ts_handler = TimestampHandler()
        self.folder_name = folder_name
        self.camera_error_queue = error_queue

        self.csv_file = None
        self.csv_writer = None
        self.video_writer = None

        self.running = False
        self.recording = False

        self.video_capture_thread = None
        self.imu_capture_thread = None
        self.imu_writer_thread = None

        self.imu_queue = queue.Queue()

        # ZED Camera settings (current values)
        self.brightness = 4  # Default ZED brightness
        self.contrast = 4   # Default ZED contrast
        self.hue = 0        # Default ZED hue
        self.saturation = 4  # Default ZED saturation
        self.sharpness = 4  # Default ZED sharpness
        self.gain = 100     # Default ZED gain (auto by default)
        self.exposure = 100  # Default ZED exposure (auto by default)
        self.wb_temperature = 4200  # Default ZED WB (auto by default)
        self.led_on = True

        # ROI selection variables - these will be managed by the UI and passed here
        self.selection_rect = sl.Rect(0, 0, 0, 0)  # Current ROI
        self.origin_rect = (-1, -1)  # Start point of a new ROI selection
        self.select_in_progress = False

    @Slot(str, int)
    def start_camera_setup(self, resolution_str, fps):
        """
        Initializes the ZED camera. This method runs in the CameraZed's QThread.
        """
        if self.device and self.device.is_opened():
            self.camera_error_queue.put("Error: ZED Camera already initialized.")
            return

        self.camera_error_queue.put(
            f"Process: Initializing ZED camera with resolution {resolution_str} and {fps} FPS...")
        self.device = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = RESOLUTION.get(
            resolution_str, sl.RESOLUTION.HD1080)
        init_params.camera_fps = fps
        init_params.coordinate_units = sl.UNIT.METER
        init_params.sdk_verbose = 1
        init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD

        status = self.device.open(init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            self.camera_error_queue.put("Error: ZED Camera Open Error:", status)
            self.device = None  # Ensure device is None if opening fails
            return

        self.running = True
        self.camera_error_queue.put("Sucess: ZED Camera opened successfully.")

        # Start internal capture threads
        self.video_capture_thread = threading.Thread(
            target=self._run_video_capture, daemon=True)
        self.imu_capture_thread = threading.Thread(
            target=self._run_imu_capture, daemon=True)
        self.imu_writer_thread = threading.Thread(
            target=self._run_imu_writer, daemon=True)

        self.video_capture_thread.start()
        self.imu_capture_thread.start()
        self.imu_writer_thread.start()

        self.camera_error_queue.put("Process: ZED internal capture threads started.")

        # Emit initial settings to update UI
        self._emit_all_current_settings()

    def _emit_all_current_settings(self):
        status, val = self.device.get_camera_settings(
            sl.VIDEO_SETTINGS.BRIGHTNESS)
        if status == sl.ERROR_CODE.SUCCESS:
            self.brightness_updated_signal.emit(val)
        status, val = self.device.get_camera_settings(
            sl.VIDEO_SETTINGS.CONTRAST)
        if status == sl.ERROR_CODE.SUCCESS:
            self.contrast_updated_signal.emit(val)
        status, val = self.device.get_camera_settings(sl.VIDEO_SETTINGS.HUE)
        if status == sl.ERROR_CODE.SUCCESS:
            self.hue_updated_signal.emit(val)
        status, val = self.device.get_camera_settings(
            sl.VIDEO_SETTINGS.SATURATION)
        if status == sl.ERROR_CODE.SUCCESS:
            self.saturation_updated_signal.emit(val)
        status, val = self.device.get_camera_settings(
            sl.VIDEO_SETTINGS.SHARPNESS)
        if status == sl.ERROR_CODE.SUCCESS:
            self.sharpness_updated_signal.emit(val)
        status, val = self.device.get_camera_settings(sl.VIDEO_SETTINGS.GAIN)
        if status == sl.ERROR_CODE.SUCCESS:
            self.gain_updated_signal.emit(val)
        status, val = self.device.get_camera_settings(
            sl.VIDEO_SETTINGS.EXPOSURE)
        if status == sl.ERROR_CODE.SUCCESS:
            self.exposure_updated_signal.emit(val)
        status, val = self.device.get_camera_settings(
            sl.VIDEO_SETTINGS.WHITEBALANCE_TEMPERATURE)
        if status == sl.ERROR_CODE.SUCCESS:
            self.wb_temperature_updated_signal.emit(val)
        status, val = self.device.get_camera_settings(
            sl.VIDEO_SETTINGS.LED_STATUS)
        if status == sl.ERROR_CODE.SUCCESS:
            self.led_status_updated_signal.emit(
                bool(val))  # LED status is bool

        # ROI is more complex to get current state, so we'll just emit the stored selection_rect if any
        self.roi_aec_agc_updated_signal.emit(self.selection_rect.x, self.selection_rect.y,
                                             self.selection_rect.width, self.selection_rect.height)

    def _get_frame(self):
        image = sl.Mat()
        runtime_parameters = sl.RuntimeParameters()
        if self.device and self.device.is_opened():
            if self.device.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
                self.device.retrieve_image(image, sl.VIEW.LEFT)
                left_img = image.get_data()  # This converts sl.Mat to cv2.Mat (numpy array)
                if left_img is not None and left_img.size > 0:
                    return cv2.cvtColor(left_img, cv2.COLOR_BGRA2BGR)
        return None

    def _get_sensor_data(self):
        if self.device and self.device.is_opened():
            # Changed to Image, Current to get latest data.
            if self.device.get_sensors_data(self.sensor, sl.TIME_REFERENCE.IMAGE) == sl.ERROR_CODE.SUCCESS:
                return self.sensor
        return None

    def _run_video_capture(self):
        self.camera_error_queue.put("Process: ZED video capture thread started.")
        while self.running:
            try:
                frame = self._get_frame()
                if frame is not None:
                    self.previewframesignal.emit(frame)
                    if self.recording and self.video_writer:
                        self.video_writer.write(frame)
            except Exception as e:
                self.camera_error_queue.put(f"Error: ZED video capture thread: {e}")
                self.running = False  # Stop if error occurs
        self.camera_error_queue.put("Success: ZED video capture thread stopped.")

    def _run_imu_capture(self):
        """Internal thread for ZED IMU data acquisition and saving."""
        self.camera_error_queue.put("Process: ZED IMU capture thread started.")
        while self.running:
            try:
                sensors_data = self._get_sensor_data()
                if sensors_data:
                    imu_data = sensors_data.get_imu_data()
                    if self.ts_handler.is_new(imu_data):
                        imu_dict = {
                            "timestamp": datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f'),
                            "quat_x": float(imu_data.get_pose().get_orientation().get()[0]),
                            "quat_y": float(imu_data.get_pose().get_orientation().get()[1]),
                            "quat_z": float(imu_data.get_pose().get_orientation().get()[2]),
                            "quat_w": float(imu_data.get_pose().get_orientation().get()[3]),
                            "accelerometer_x": float(imu_data.get_linear_acceleration()[0]),
                            "accelerometer_y": float(imu_data.get_linear_acceleration()[1]),
                            "accelerometer_z": float(imu_data.get_linear_acceleration()[2]),
                            "gyroscope_x": float(imu_data.get_angular_velocity()[0]),
                            "gyroscope_y": float(imu_data.get_angular_velocity()[1]),
                            "gyroscope_z": float(imu_data.get_angular_velocity()[2]),
                        }
                        
                        # Add magnetometer and barometer if new
                        mag_data = sensors_data.get_magnetometer_data()
                        if self.ts_handler.is_new(mag_data):
                            imu_dict["magnetometer_x"] = float(mag_data.get_magnetic_field_calibrated()[0])
                            imu_dict["magnetometer_y"] = float(mag_data.get_magnetic_field_calibrated()[1])
                            imu_dict["magnetometer_z"] = float(mag_data.get_magnetic_field_calibrated()[2])
                        else:
                            imu_dict["magnetometer_x"], imu_dict["magnetometer_y"], imu_dict["magnetometer_z"] = 0.0, 0.0, 0.0

                        baro_data = sensors_data.get_barometer_data()
                        if self.ts_handler.is_new(baro_data):
                            imu_dict["barometer_pressure"] = float(baro_data.pressure)
                        else:
                            imu_dict["barometer_pressure"] = 0.0

                        self.imu_data_signal.emit(imu_dict)  # Emit IMU data

                        if self.recording and self.csv_writer:
                            # Create a list with timestamp and float values
                            imu_values = [
                                imu_dict["timestamp"],
                                imu_dict["quat_x"],
                                imu_dict["quat_y"],
                                imu_dict["quat_z"],
                                imu_dict["quat_w"],
                                imu_dict["accelerometer_x"],
                                imu_dict["accelerometer_y"],
                                imu_dict["accelerometer_z"],
                                imu_dict["gyroscope_x"],
                                imu_dict["gyroscope_y"],
                                imu_dict["gyroscope_z"],
                                imu_dict["magnetometer_x"],
                                imu_dict["magnetometer_y"],
                                imu_dict["magnetometer_z"],
                                imu_dict["barometer_pressure"]
                            ]
                            self.imu_queue.put(imu_values)


                time.sleep(1 / 400.0)  # ZED IMU typically 400 Hz
            except Exception as e:
                self.camera_error_queue.put(f"Error in ZED IMU capture thread: {e}")
                self.running = False  # Stop if error occurs
        self.camera_error_queue.put("Success: ZED IMU capture thread stopped.")

    def _run_imu_writer(self):
        """Internal thread for writing IMU data to CSV."""
        self.camera_error_queue.put("Process: ZED IMU writer thread started.")
        imu_batch = []
        batch_size = 50  # Adjust batch size as needed
        while self.running:
            if not self.recording:
                # Clear the queue when not recording
                while not self.imu_queue.empty():
                    try:
                        self.imu_queue.get_nowait()
                    except queue.Empty:
                        break
                time.sleep(0.1)
                continue

            try:
                imu_data = self.imu_queue.get(timeout=0.1)
                if imu_data is not None:
                    imu_batch.append(imu_data)

                    if len(imu_batch) >= batch_size:
                        if self.csv_writer and self.recording:
                            self.csv_writer.writerows(imu_batch)
                            self.csv_file.flush()
                        imu_batch.clear()

            except queue.Empty:
                if imu_batch and self.csv_writer and self.recording:
                    self.csv_writer.writerows(imu_batch)
                    self.csv_file.flush()
                    imu_batch.clear()
                continue
            except Exception as e:
                self.camera_error_queue.put(f"Error: IMU writer error: {e}")

        # Write any remaining data on shutdown
        if imu_batch and self.csv_writer and self.recording:
            self.csv_writer.writerows(imu_batch)
            self.csv_file.flush()

    def start_recording(self):
        """Starts video and IMU data recording."""
        if not self.device or not self.device.is_opened():
            self.camera_error_queue.put("Error: Cannot start recording: ZED camera not initialized.")
            return
        if self.recording:
            self.camera_error_queue.put("Comment: ZED recording already in progress.")
            return

        os.makedirs(self.folder_name, exist_ok=True)

        # Initialize CSV file for IMU data
        self.csv_file = open(os.path.join(
            self.folder_name, f'{datetime.now().strftime("%H-%M-%S")}.csv'), 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'timestamp',
            'quat_x', 'quat_y', 'quat_z', 'quat_w',
            'accelerometer_x', 'accelerometer_y', 'accelerometer_z',
            'gyroscope_x', 'gyroscope_y', 'gyroscope_z',
            'magnetometer_x', 'magnetometer_y', 'magnetometer_z',
            'barometer_pressure'
        ])

        # Initialize VideoWriter
        res = self.device.get_camera_information().camera_configuration.resolution
        w = res.width
        h = res.height
        fps = self.device.get_camera_information().camera_configuration.fps
        video_path = os.path.join(
            self.folder_name, f'{datetime.now().strftime("%H-%M-%S")}.avi')
        self.video_writer = cv2.VideoWriter(
            video_path, cv2.VideoWriter_fourcc(*"MJPG"), fps, (w, h))

        if not self.video_writer.isOpened():
            self.camera_error_queue.put(f"Error: Could not open video writer for {video_path}")
            if self.csv_file:
                self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None
            return

        self.recording = True
        self.camera_error_queue.put(f"Process: ZED recording started to {self.folder_name}")

    def stop_recording(self):
        """Stops video and IMU data recording."""
        if not self.recording:
            self.camera_error_queue.put("Error: ZED camera is not currently recording.")
            return

        self.recording = False
        self.camera_error_queue.put("Process: Stopping ZED recording...")

        # Give a small moment for any pending writes
        time.sleep(0.1)

        if self.video_writer:
            self.video_writer.release()
            self.video_writer = None
            self.camera_error_queue.put("Success: ZED video writer released.")
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None
            self.camera_error_queue.put("Success: ZED IMU CSV file closed.")

        self.camera_error_queue.put("Success: ZED recording stopped.")

    def stop_camera(self):
        """Stops all ZED camera related threads and closes the device."""
        if not self.running and not (self.device and self.device.is_opened()):
            self.camera_error_queue.put("Error: ZED Camera is already stopped or not initialized.")
            return

        self.camera_error_queue.put("Process: Signaling ZED camera threads to stop...")
        self.running = False  # Signal all internal threads to stop

        # Wait for internal threads to finish their loops
        if self.video_capture_thread and self.video_capture_thread.is_alive():
            self.video_capture_thread.join(timeout=1)
            if self.video_capture_thread.is_alive():
                self.camera_error_queue.put("Warning: ZED video capture thread did not terminate cleanly.")
        if self.imu_capture_thread and self.imu_capture_thread.is_alive():
            self.imu_capture_thread.join(timeout=1)
            if self.imu_capture_thread.is_alive():
                self.camera_error_queue.put("Warning: ZED IMU capture thread did not terminate cleanly.")
        if self.imu_writer_thread and self.imu_writer_thread.is_alive():
            self.imu_writer_thread.join(timeout=1)
            if self.imu_writer_thread.is_alive():
                self.camera_error_queue.put("Warning: ZED IMU writer thread did not terminate cleanly.")

        if self.recording:
            self.stop_recording()

        if self.device and self.device.is_opened():
            try:
                self.device.close()
                self.camera_error_queue.put("Success: ZED device closed.")
            except Exception as e:
                self.camera_error_queue.put(f"Error closing ZED device: {e}")
            self.device = None

        self.camera_error_queue.put("Success: CameraZed instance stopped and cleaned up.")

    # --- ZED Camera Control Methods ---
    def _set_zed_setting(self, setting_type, value):
        """Helper to set a ZED camera setting and emit update signal."""
        if not self.device or not self.device.is_opened():
            self.camera_error_queue.put("Error: ZED Camera not initialized. Cannot set setting.")
            return

        # Get current value to ensure we're within bounds if needed, or just for logging
        status, current_value = self.device.get_camera_settings(setting_type)
        if status != sl.ERROR_CODE.SUCCESS:
            self.camera_error_queue.put(f"Error: Failed to get current value for {setting_type}: {status}")
            return

        if value == 0:  # Special value for auto/default
            self.device.set_camera_settings(setting_type, value)
            self.camera_error_queue.put(f"Comment: [Zed] Set {setting_type.name} to AUTO/Default")
        else:
            new_value = clamp(current_value + value, 0, 100)
            if setting_type == sl.VIDEO_SETTINGS.WHITEBALANCE_TEMPERATURE:
                new_value = clamp(current_value + value, 2800, 6500)
            elif setting_type == sl.VIDEO_SETTINGS.EXPOSURE or setting_type == sl.VIDEO_SETTINGS.GAIN:
                new_value = clamp(current_value + value, 0, 100)
            else:
                new_value = clamp(current_value + value, 0, 8)

            self.device.set_camera_settings(setting_type, new_value)
            self.camera_error_queue.put(f"Comment: [Zed] Set {setting_type.name}: {new_value}")

            # Update internal state and emit signal
            if setting_type == sl.VIDEO_SETTINGS.BRIGHTNESS:
                self.brightness = new_value
                self.brightness_updated_signal.emit(self.brightness)
            elif setting_type == sl.VIDEO_SETTINGS.CONTRAST:
                self.contrast = new_value
                self.contrast_updated_signal.emit(self.contrast)
            elif setting_type == sl.VIDEO_SETTINGS.HUE:
                self.hue = new_value
                self.hue_updated_signal.emit(self.hue)
            elif setting_type == sl.VIDEO_SETTINGS.SATURATION:
                self.saturation = new_value
                self.saturation_updated_signal.emit(self.saturation)
            elif setting_type == sl.VIDEO_SETTINGS.SHARPNESS:
                self.sharpness = new_value
                self.sharpness_updated_signal.emit(self.sharpness)
            elif setting_type == sl.VIDEO_SETTINGS.GAIN:
                self.gain = new_value
                self.gain_updated_signal.emit(self.gain)
            elif setting_type == sl.VIDEO_SETTINGS.EXPOSURE:
                self.exposure = new_value
                self.exposure_updated_signal.emit(self.exposure)
            elif setting_type == sl.VIDEO_SETTINGS.WHITEBALANCE_TEMPERATURE:
                self.wb_temperature = new_value
                self.wb_temperature_updated_signal.emit(self.wb_temperature)

    # --- Brightness ---
    @Slot()
    def add_brightness(self): self._set_zed_setting(
        sl.VIDEO_SETTINGS.BRIGHTNESS, 1)

    @Slot()
    def subtract_brightness(self): self._set_zed_setting(
        sl.VIDEO_SETTINGS.BRIGHTNESS, -1)

    # --- Contrast ---
    @Slot()
    def add_contrast(self): self._set_zed_setting(
        sl.VIDEO_SETTINGS.CONTRAST, 1)

    @Slot()
    def subtract_contrast(self): self._set_zed_setting(
        sl.VIDEO_SETTINGS.CONTRAST, -1)

    # --- Hue ---
    @Slot()
    def add_hue(self): self._set_zed_setting(sl.VIDEO_SETTINGS.HUE, 1)
    @Slot()
    def subtract_hue(self): self._set_zed_setting(sl.VIDEO_SETTINGS.HUE, -1)

    # --- Saturation ---
    @Slot()
    def add_saturation(self): self._set_zed_setting(
        sl.VIDEO_SETTINGS.SATURATION, 1)

    @Slot()
    def subtract_saturation(self): self._set_zed_setting(
        sl.VIDEO_SETTINGS.SATURATION, -1)

    # --- Sharpness ---
    @Slot()
    def add_sharpness(self): self._set_zed_setting(
        sl.VIDEO_SETTINGS.SHARPNESS, 1)

    @Slot()
    def subtract_sharpness(self): self._set_zed_setting(
        sl.VIDEO_SETTINGS.SHARPNESS, -1)

    # --- Gain ---
    @Slot()
    def add_gain(self): self._set_zed_setting(
        sl.VIDEO_SETTINGS.GAIN, 10)  # Larger step for gain

    @Slot()
    def subtract_gain(self): self._set_zed_setting(sl.VIDEO_SETTINGS.GAIN, -10)

    # --- Exposure ---
    @Slot()
    def add_exposure(self): self._set_zed_setting(
        sl.VIDEO_SETTINGS.EXPOSURE, 10)  # Larger step for exposure

    @Slot()
    def subtract_exposure(self): self._set_zed_setting(
        sl.VIDEO_SETTINGS.EXPOSURE, -10)

    # --- White Balance Temperature ---
    @Slot()
    def add_wb_temperature(self): self._set_zed_setting(
        sl.VIDEO_SETTINGS.WHITEBALANCE_TEMPERATURE, 100)  # Step in 100K

    @Slot()
    def subtract_wb_temperature(self): self._set_zed_setting(
        sl.VIDEO_SETTINGS.WHITEBALANCE_TEMPERATURE, -100)

    # --- Auto Modes / Reset ---
    @Slot()
    def enable_auto_brightness(self): self._set_zed_setting(
        sl.VIDEO_SETTINGS.BRIGHTNESS, 0)

    @Slot()
    def enable_auto_contrast(self): self._set_zed_setting(
        sl.VIDEO_SETTINGS.CONTRAST, 0)

    @Slot()
    def enable_auto_hue(self): self._set_zed_setting(sl.VIDEO_SETTINGS.HUE, 0)

    @Slot()
    def enable_auto_saturation(self): self._set_zed_setting(
        sl.VIDEO_SETTINGS.SATURATION, 0)

    @Slot()
    def enable_auto_sharpness(self): self._set_zed_setting(
        sl.VIDEO_SETTINGS.SHARPNESS, 0)

    @Slot()
    def enable_auto_gain(self): self._set_zed_setting(
        sl.VIDEO_SETTINGS.GAIN, 0)

    @Slot()
    def enable_auto_exposure(self): self._set_zed_setting(
        sl.VIDEO_SETTINGS.EXPOSURE, 0)

    @Slot()
    def enable_auto_wb_temperature(self): self._set_zed_setting(
        sl.VIDEO_SETTINGS.WHITEBALANCE_TEMPERATURE, 0)

    @Slot()
    def reset_all_settings(self):
        """Resets all ZED camera settings to default (-1)."""
        if not self.device or not self.device.is_opened():
            self.camera_error_queue.put("Error: ZED Camera not initialized. Cannot reset settings.")
            return

        settings_to_reset = [
            sl.VIDEO_SETTINGS.BRIGHTNESS, sl.VIDEO_SETTINGS.CONTRAST, sl.VIDEO_SETTINGS.HUE,
            sl.VIDEO_SETTINGS.SATURATION, sl.VIDEO_SETTINGS.SHARPNESS, sl.VIDEO_SETTINGS.GAIN,
            sl.VIDEO_SETTINGS.EXPOSURE, sl.VIDEO_SETTINGS.WHITEBALANCE_TEMPERATURE
        ]
        for setting in settings_to_reset:
            self.device.set_camera_settings(setting, -1)
        self.camera_error_queue.put("Comment: [ZED] Reset all settings to default")
        self._emit_all_current_settings()  # Update UI after reset

    @Slot()
    def toggle_led_status(self):
        """Toggles the ZED camera LED status."""
        if not self.device or not self.device.is_opened():
            self.camera_error_queue.put("Error: ZED Camera not initialized. Cannot toggle LED.")
            return
        self.led_on = not self.led_on
        self.device.set_camera_settings(
            sl.VIDEO_SETTINGS.LED_STATUS, int(self.led_on))
        self.camera_error_queue.put(f"Comment: [ZED] LED Status: {'ON' if self.led_on else 'OFF'}")
        self.led_status_updated_signal.emit(self.led_on)

    # --- ROI Control ---
    @Slot(int, int, int, int)
    def set_aec_agc_roi(self, x, y, width, height):
        """Sets the AEC/AGC ROI on the ZED camera."""
        if not self.device or not self.device.is_opened():
            self.camera_error_queue.put("Error: ZED Camera not initialized. Cannot set ROI.")
            return

        self.selection_rect = sl.Rect(x, y, width, height)
        status = self.device.set_camera_settings_roi(
            sl.VIDEO_SETTINGS.AEC_AGC_ROI, self.selection_rect, sl.SIDE.BOTH)
        if status == sl.ERROR_CODE.SUCCESS:
            self.camera_error_queue.put(
                f"[ZED] Set AEC/AGC ROI on target [{x},{y},{width},{height}]")
            self.roi_aec_agc_updated_signal.emit(x, y, width, height)
        else:
            self.camera_error_queue.put(f"Error: [ZED] Failed to set AEC/AGC ROI: {status}")

    @Slot()
    def reset_aec_agc_roi(self):
        """Resets the AEC/AGC ROI to full resolution on the ZED camera."""
        if not self.device or not self.device.is_opened():
            self.camera_error_queue.put("Error: ZED Camera not initialized. Cannot reset ROI.")
            return

        # Passing an empty rect with reset=True resets it to full resolution
        self.selection_rect = sl.Rect(0, 0, 0, 0)  # Clear internal state
        status = self.device.set_camera_settings_roi(
            sl.VIDEO_SETTINGS.AEC_AGC_ROI, sl.Rect(), sl.SIDE.BOTH, True)
        if status == sl.ERROR_CODE.SUCCESS:
            self.camera_error_queue.put("Comment: [ZED] Reset AEC/AGC ROI to full resolution")
            self.roi_aec_agc_updated_signal.emit(
                0, 0, 0, 0)  # Emit default values
        else:
            self.camera_error_queue.put(f"Error: [ZED] Failed to reset AEC/AGC ROI: {status}")
