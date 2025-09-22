import os
import csv
import cv2
import time
import queue
import threading
import subprocess
import depthai as dai
from itertools import cycle
from datetime import datetime, timedelta
from PySide6.QtCore import QObject, Signal, Slot


STEP_SIZE = 8
EXP_STEP = 500  # us
ISO_STEP = 50
LENS_STEP = 3
WB_STEP = 200
RESOLUTION = {
    '720p': dai.ColorCameraProperties.SensorResolution.THE_720_P,
    '1080p': dai.ColorCameraProperties.SensorResolution.THE_1080_P,
    '2K': dai.ColorCameraProperties.SensorResolution.THE_2024X1520,
    '4K': dai.ColorCameraProperties.SensorResolution.THE_4_K,
}


def clamp(num, v0, v1):
    return max(v0, min(num, v1))


class CameraDepthai(QObject):
    previewframesignal = Signal(cv2.Mat)
    # Add signals for control updates (these will be emitted by _receive_control_command)
    exposure_updated_signal = Signal(int)
    iso_updated_signal = Signal(int)
    focus_updated_signal = Signal(int)
    brightness_updated_signal = Signal(int)
    contrast_updated_signal = Signal(int)
    saturation_updated_signal = Signal(int)
    sharpness_updated_signal = Signal(int)
    wb_updated_signal = Signal(int)
    control_command_signal = Signal(object)

    def __init__(self, folder_name, camera_error_queue):
        super().__init__()
        self.device = None
        self.pipeline = None
        self.camRgb = None
        self.recording = False
        self.folder_name = folder_name
        self.camera_error_queue = camera_error_queue

        # IMU related attributes
        self.csv_file = None
        self.csv_writer = None
        self.imu_queue = queue.Queue(maxsize=2000)
        self.running = False  # Control for internal threads, initially False

        # Camera Related
        self.previewframe = None

        # Add thread objects here
        self.video_thread = None
        self.imu_writer_thread = None
        self.imu_capture_thread = None
        self.preview_capture_thread = None

        # Queues
        self.controlQueue = None
        self.configQueue = None
        self.previewQueue = None
        self.encodedQueue = None
        self.q_imu = None
        self.syncQueue = None

        # Control states
        self.cropX, self.cropY = 0, 0
        self.sendCamConfig = True
        self.lensPos, self.expTime, self.sensIso = 50, 5000, 800
        self.wbManual = 4000
        self.control = 'none'
        self.ae_comp = 0
        self.show = False
        self.ae_lock, self.awb_lock = False, False
        self.saturation, self.contrast, self.brightness, self.sharpness = 0, 0, 0, 0
        self.luma_denoise, self.chroma_denoise = 0, 0

        self.awb_mode = cycle([m for name, m in vars(
            dai.CameraControl.AutoWhiteBalanceMode).items() if name.isupper()])
        self.anti_banding_mode = cycle([m for name, m in vars(
            dai.CameraControl.AntiBandingMode).items() if name.isupper()])
        self.effect_mode = cycle([m for name, m in vars(
            dai.CameraControl.EffectMode).items() if name.isupper()])

        self.maxCropX = 0.0
        self.maxCropY = 0.0

    def start_camera_setup(self, resolution, fps):
        """
        Initializes the DepthAI pipeline and device.
        This method is designed to be called in the CameraDepthai's QThread context.
        """
        if self.device:
            print("Depthai camera already initialized.")
            self.camera_error_queue.put("Success: Depthai Camera Intialized\n")
            return

        self.camera_error_queue.put(
            f"Process: Setting up Depthai camera with resolution {resolution} and {fps} FPS...")
        try:
            self.pipeline = dai.Pipeline()

            self.camRgb = self.pipeline.create(dai.node.ColorCamera)
            self.camRgb.setResolution(
                RESOLUTION.get(resolution, dai.ColorCameraProperties.SensorResolution.THE_4_K))
            self.camRgb.setFps(fps)

            self.camRgb.setPreviewSize(1280, 720)
            self.camRgb.setInterleaved(False)
            self.camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

            # Video encoder for full-res MJPEG saving
            self.videoEncoder = self.pipeline.create(dai.node.VideoEncoder)
            self.videoEncoder.setDefaultProfilePreset(
                30, dai.VideoEncoderProperties.Profile.H265_MAIN)
            self.videoEncoder.setQuality(100)
            self.videoEncoder.setLossless(True)    # Changed to True

            # XLink outputs
            self.previewOut = self.pipeline.create(dai.node.XLinkOut)
            self.previewOut.setStreamName("preview")

            self.encodedOut = self.pipeline.create(dai.node.XLinkOut)
            self.encodedOut.setStreamName("encoded")

            self.controlIn = self.pipeline.create(dai.node.XLinkIn)
            self.controlIn.setStreamName("control")

            self.configIn = self.pipeline.create(dai.node.XLinkIn)
            self.configIn.setStreamName("config")

            # IMU setup
            self.imu = self.pipeline.create(dai.node.IMU)
            self.imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, 400)
            self.imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, 400)
            self.imu.enableIMUSensor(dai.IMUSensor.ROTATION_VECTOR, 400)
            # Changed Both to 20 for better performance
            self.imu.setBatchReportThreshold(20)
            self.imu.setMaxBatchReports(20)

            # Sync node for synchronizing video and IMU
            # self.sync = self.pipeline.create(dai.node.Sync)
            # self.sync.setSyncThreshold(timedelta(milliseconds=10))
            # self.sync.setSyncAttempts(-1)

            # IMU output
            self.xout_imu = self.pipeline.create(dai.node.XLinkOut)
            self.xout_imu.setStreamName("imu")

            # Sync output
            # self.xoutGrp = self.pipeline.create(dai.node.XLinkOut)
            # self.xoutGrp.setStreamName("sync")

            # Linking
            self.camRgb.preview.link(
                self.previewOut.input)
            self.camRgb.video.link(self.videoEncoder.input)
            self.videoEncoder.bitstream.link(
                self.encodedOut.input)     # Encoded MJPEG stream
            self.controlIn.out.link(self.camRgb.inputControl)
            self.configIn.out.link(self.camRgb.inputConfig)

            # Sync connections
            # self.camRgb.video.link(self.sync.inputs["video"])
            # self.imu.out.link(self.sync.inputs["imu"])
            # self.sync.out.link(self.xoutGrp.input)

            # Also keep direct IMU output for compatibility
            self.imu.out.link(self.xout_imu.input)

            self.device = dai.Device(self.pipeline)
            imuType = self.device.getConnectedIMU()
            imuFirmwareVersion = self.device.getIMUFirmwareVersion()
            self.camera_error_queue.put(
                f"Comment: DepthAI IMU type: {imuType}, firmware version: {imuFirmwareVersion}")

            self._init_queues()
            self._init_controls()

            # Start the internal threads
            self.running = True  # Set running to True before starting threads
            self.video_thread = threading.Thread(
                target=self._run_video_capture, daemon=True)
            self.imu_writer_thread = threading.Thread(
                target=self._run_imu_writer, daemon=True)
            self.imu_capture_thread = threading.Thread(
                target=self._run_imu_capture, daemon=True)
            self.preview_capture_thread = threading.Thread(
                target=self._run_preview_capture, daemon=True)

            self.video_thread.start()
            self.imu_writer_thread.start()
            self.imu_capture_thread.start()
            self.preview_capture_thread.start()

            self.camera_error_queue.put("Success: Depthai camera initialized successfully.")
        except RuntimeError as e:
            self.camera_error_queue.put(f"Error: Failed to initialize Depthai camera: {e}")
            self.device = None  # Ensure device is None if initialization fails
            self.running = False  # Ensure internal threads don't try to run
        except Exception as e:
            self.camera_error_queue.put(f"Error: Depthai An unexpected error occurred during camera setup: {e}")
            self.device = None
            self.running = False

    def _init_queues(self):
        # Ensure device is not None before getting queues
        if self.device:
            self.controlQueue = self.device.getInputQueue('control')
            self.configQueue = self.device.getInputQueue('config')
            self.previewQueue = self.device.getOutputQueue('preview')
            self.encodedQueue = self.device.getOutputQueue('encoded')
            self.q_imu = self.device.getOutputQueue(
                name="imu", maxSize=1000, blocking=False)
            # self.syncQueue = self.device.getOutputQueue(
            #     name="sync", maxSize=500, blocking=True)
        else:
            self.camera_error_queue.put("Error: Depthai Device not initialized, cannot get queues.")

    def _init_controls(self):
        # Check if camRgb is initialized before accessing its properties
        if self.camRgb:
            self.maxCropX = (self.camRgb.getIspWidth() -
                             self.camRgb.getVideoWidth()) / self.camRgb.getIspWidth()
            self.maxCropY = (self.camRgb.getIspHeight(
            ) - self.camRgb.getVideoHeight()) / self.camRgb.getIspHeight()
        else:
            self.maxCropX = 0.0
            self.maxCropY = 0.0

    def start_recording(self):
        if not self.device:
            self.camera_error_queue.put("Error: Cannot start recording: Depthai device not initialized.")
            return

        os.makedirs(self.folder_name, exist_ok=True)
        self.outFile = open(
            f'{self.folder_name}/{datetime.now().strftime("%H-%M-%S")}.h265', 'wb')

        # Initialize CSV file for IMU data
        self.csv_file = open(os.path.join(
            self.folder_name, f'{datetime.now().strftime("%H-%M-%S")}.csv'), 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'system_timestamp',
            'imu_time', 
            # 'video_time',
            'accelerometer_x', 'accelerometer_y', 'accelerometer_z',
            'gyroscope_x', 'gyroscope_y', 'gyroscope_z',
            'quat_x', 'quat_y', 'quat_z', 'quat_w'
        ])

        # Clear the IMU queue before starting recording
        while not self.imu_queue.empty():
            try:
                self.imu_queue.get_nowait()
            except queue.Empty:
                break

        self.recording = True
        self.camera_error_queue.put(f"Process: Depthai Started recording to {self.folder_name}")

    def stop_recording(self):
        if not self.recording:
            self.camera_error_queue.put("Error: Depthai Not currently recording.")
            return

        self.recording = False
        self.camera_error_queue.put("Process: Depthai Stopping recording...")

        # Give a small moment for any pending writes
        time.sleep(0.1)

        if self.outFile:
            self.outFile.close()
            self.outFile = None
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None

        print("Recording stopped.")

    @staticmethod
    def convert_to_mp4(file):
        if os.path.exists(file):
            print(f"Converting {file} to MP4 format...")
            try:
                output_file = file.replace('.mjpeg', '.mp4')
                command = [
                    'ffmpeg',
                    '-i', file,
                    '-c:v', 'libx264',
                    '-preset', 'medium',
                    '-crf', '23',
                    '-pix_fmt', 'yuv420p',
                    output_file
                ]
                subprocess.run(command, check=True,
                               capture_output=True, text=True)
                print("Conversion successful!")
                # os.remove(file) # Uncomment to remove original MJPEG
            except FileNotFoundError:
                print(
                    "Error: ffmpeg not found. Please install ffmpeg to convert videos.")
            except subprocess.CalledProcessError as e:
                print(f"Error during ffmpeg conversion: {e.stderr}")
            except Exception as e:
                print(
                    f"An unexpected error occurred during video conversion: {e}")

    # def _run_video_capture(self):
    #     """Internal thread for video encoding capture."""
    #     while self.running:
    #         if not self.encodedQueue:
    #             time.sleep(0.01)  # Wait if queue not ready
    #             continue
    #         try:
    #             for bitstream in self.encodedQueue.tryGetAll():
    #                 if self.recording and self.outFile:
    #                     self.outFile.write(bytearray(bitstream.getData()))
    #             time.sleep(0.001)  # Small sleep to yield CPU
    #         except RuntimeError as e:
    #             print(f"Video capture thread RuntimeError: {e}")
    #             self.running = False  # Signal to stop if device error
    #         except Exception as e:
    #             print(f"An error occurred in video capture thread: {e}")
    #             time.sleep(0.01)  # Sleep on other errors

    def _run_video_capture(self):
        """Video capture and encoding thread"""
        while self.running:
            if self.recording:
                for bitstream in self.encodedQueue.tryGetAll():
                    if self.recording and self.outFile:
                        self.outFile.write(bytearray(bitstream.getData()))
            else:
                # Clear the queue when not recording to prevent buildup
                self.encodedQueue.tryGetAll()
            time.sleep(0.001)


    def _run_imu_writer(self):
        """Dedicated thread for writing IMU data to CSV with batch processing."""
        batch_size = 50
        imu_batch = []

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
                self.camera_error_queue.put(f"Error: Depthai IMU writer error: {e}")

        # Write any remaining data on shutdown
        if imu_batch and self.csv_writer and self.recording:
            self.csv_writer.writerows(imu_batch)
            self.csv_file.flush()

    def _run_imu_capture(self):
        """Internal thread for IMU data capture and synchronization."""
        while self.running:
            if not self.device or not self.q_imu:
                time.sleep(0.01)
                continue

            try:
                syncMessage = self.q_imu.get()
                if syncMessage is not None:
                    # imuMessage = syncMessage["imu"]
                    # colorMessage = syncMessage["video"]
                    # imu_device_timestamp = imuMessage.getTimestampDevice()
                    # color_device_timestamp = colorMessage.getTimestampDevice()


                    accel = syncMessage.packets[-1].acceleroMeter
                    gyro = syncMessage.packets[-1].gyroscope
                    rotation = syncMessage.packets[-1].rotationVector
                    timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')

                    imu_row = [
                        timestamp,
                        accel.getTimestampDevice(),
                        # color_device_timestamp,
                        accel.x, accel.y, accel.z,
                        gyro.x, gyro.y, gyro.z,
                        rotation.i, rotation.j, rotation.k, rotation.real
                    ]

                    if self.recording:
                        try:
                            self.imu_queue.put_nowait(imu_row)
                        except queue.Full:
                            self.camera_error_queue.put("Error: Depthai IMU queue is full, dropping data.")
            except RuntimeError as e:
                self.camera_error_queue.put(f"Error: Depthai IMU capture thread RuntimeError: {e}")
                # self.running = False  # Stop the thread if device error occurs
            except Exception as e:
                self.camera_error_queue.put(f"Error: Depthai An error occurred in IMU capture thread: {e}")
                time.sleep(0.01)

    def _run_preview_capture(self):
        """Main camera preview capture loop, run in a separate QThread."""
        self.camera_error_queue.put("Process: Starting camera preview capture loop...")
        while self.running:
            if not self.device or not self.previewQueue:  # Check if device and queue are initialized
                time.sleep(0.01)
                continue

            try:
                inPreview = self.previewQueue.tryGet()  # No timeout argument for tryGet()
                if inPreview is not None:
                    self.previewframe = inPreview.getCvFrame()
                    if self.sendCamConfig:
                        cfg = dai.ImageManipConfig()
                        cfg.setCropRect(self.cropX, self.cropY,
                                        1.0 - self.cropX, 1.0 - self.cropY)
                        self.configQueue.send(cfg)
                        self.sendCamConfig = False
                        print(
                            f"New crop set: x={self.cropX:.3f}, y={self.cropY:.3f}")

                    if self.show:
                        print(
                            f"[{inPreview.getSequenceNum()}] Exposure: {self.expTime} us | ISO: {self.sensIso} | WB: {self.wbManual} | Lens: {self.lensPos}")
                    self.previewframesignal.emit(self.previewframe)
                else:
                    time.sleep(0.005)
            except RuntimeError as e:
                self.camera_error_queue.put(f"Error: Depthai Preview capture thread RuntimeError: {e}")
                # self.running = False  # Stop the thread if device error occurs
            except Exception as e:
                self.camera_error_queue.put(f"Error: Depthai An error occurred in preview capture thread: {e}")
                time.sleep(0.01)
        self.camera_error_queue.put("Success: Depthai Camera preview capture loop stopped.")

    def stop_camera(self):
        """Stops all camera related threads and cleans up the DepthAI device."""
        if not self.running and not self.device:
            self.camera_error_queue.put("Error: Depthai Camera is already stopped or not initialized.")
            return

        self.camera_error_queue.put("Process: Depthai Signaling camera threads to stop...")
        self.running = False  # Signal all internal threads to stop

        # Wait for internal threads to finish their loops
        if self.video_thread and self.video_thread.is_alive():
            self.video_thread.join(timeout=1)
            if self.video_thread.is_alive():
                self.camera_error_queue.put("Warning: Depthai Video thread did not terminate cleanly.")
        if self.imu_writer_thread and self.imu_writer_thread.is_alive():
            self.imu_writer_thread.join(timeout=1)
            if self.imu_writer_thread.is_alive():
                self.camera_error_queue.put("Warning: Depthai IMU writer thread did not terminate cleanly.")
        if self.imu_capture_thread and self.imu_capture_thread.is_alive():
            self.imu_capture_thread.join(timeout=1)
            if self.imu_capture_thread.is_alive():
                self.camera_error_queue.put("Warning: Depthai IMU capture thread did not terminate cleanly.")
        if self.preview_capture_thread and self.preview_capture_thread.is_alive():
            self.preview_capture_thread.join(timeout=1)
            if self.preview_capture_thread.is_alive():
                self.camera_error_queue.put("Warning: Depthai Preview capture thread did not terminate cleanly.")

        # Ensure recording is stopped and files are closed
        if self.recording:
            self.stop_recording()

        # Close the device only after all dependent threads have stopped
        if self.device:
            try:
                self.device.close()
                self.camera_error_queue.put("Sucess: Depthai device closed.")
            except RuntimeError as e:
                self.camera_error_queue.put(f"Error: Could not close Depthai device: {e}")
            self.device = None
            # Clear queues as they are now invalid
            self.controlQueue = None
            self.configQueue = None
            self.previewQueue = None
            self.encodedQueue = None
            self.q_imu = None
            self.syncQueue = None

        self.camera_error_queue.put("Sucess: CameraDepthai instance stopped and cleaned up.")

    @Slot(object)
    def _receive_control_command(self, ctrl):
        """Receives a dai.CameraControl object and sends it to the device."""
        if self.device and self.controlQueue:
            try:
                self.controlQueue.send(ctrl)
                # print(f"Sent control command: {ctrl}") # For debugging
            except RuntimeError as e:
                self.camera_error_queue.put(f"Error: Depthai RuntimeError sending control command: {e}")
        else:
            self.camera_error_queue.put("Error: Depthai Cannot send control command: Device or control queue not ready.")

    @Slot()
    def add_exposure(self):
        ctrl = dai.CameraControl()
        self.expTime = clamp(self.expTime + EXP_STEP, 1, 33000)
        ctrl.setManualExposure(self.expTime, self.sensIso)
        self.control_command_signal.emit(ctrl)  # Emit signal
        self.exposure_updated_signal.emit(self.expTime)
        self.camera_error_queue.put(f"Comment: Depthai Exposure time: {self.expTime} us")

    @Slot()
    def subtract_exposure(self):
        ctrl = dai.CameraControl()
        self.expTime = clamp(self.expTime - EXP_STEP, 1, 33000)
        ctrl.setManualExposure(self.expTime, self.sensIso)
        self.control_command_signal.emit(ctrl)  # Emit signal
        self.exposure_updated_signal.emit(self.expTime)
        self.camera_error_queue.put(f"Comment: Depthai Exposure time: {self.expTime} us")

    @Slot()
    def add_iso(self):
        ctrl = dai.CameraControl()
        self.sensIso = clamp(self.sensIso + ISO_STEP, 100, 1600)
        ctrl.setManualExposure(self.expTime, self.sensIso)
        self.control_command_signal.emit(ctrl)  # Emit signal
        self.iso_updated_signal.emit(self.sensIso)
        self.camera_error_queue.put(f"Comment: Depthai ISO: {self.sensIso}")

    @Slot()
    def subtract_iso(self):
        ctrl = dai.CameraControl()
        self.sensIso = clamp(self.sensIso - ISO_STEP, 100, 1600)
        ctrl.setManualExposure(self.expTime, self.sensIso)
        self.control_command_signal.emit(ctrl)  # Emit signal
        self.iso_updated_signal.emit(self.sensIso)
        self.camera_error_queue.put(f"Comment: Depthai ISO: {self.sensIso}")

    @Slot()
    def add_focus(self):
        ctrl = dai.CameraControl()
        self.lensPos = clamp(self.lensPos + LENS_STEP, 0, 255)
        ctrl.setManualFocus(self.lensPos)
        self.control_command_signal.emit(ctrl)  # Emit signal
        self.focus_updated_signal.emit(self.lensPos)
        self.camera_error_queue.put(f"Comment: Depthai Lens position: {self.lensPos}")

    @Slot()
    def subtract_focus(self):
        ctrl = dai.CameraControl()
        self.lensPos = clamp(self.lensPos - LENS_STEP, 0, 255)
        ctrl.setManualFocus(self.lensPos)
        self.control_command_signal.emit(ctrl)  # Emit signal
        self.focus_updated_signal.emit(self.lensPos)
        self.camera_error_queue.put(f"Comment: Depthai Lens position: {self.lensPos}")

    @Slot()
    def add_brightness(self):
        self.control = 'brightness'
        self.update_advanced_control(1)

    @Slot()
    def subtract_brightness(self):
        self.control = 'brightness'
        self.update_advanced_control(-1)

    @Slot()
    def add_contrast(self):
        self.control = 'contrast'
        self.update_advanced_control(1)

    @Slot()
    def subtract_contrast(self):
        self.control = 'contrast'
        self.update_advanced_control(-1)

    @Slot()
    def add_saturation(self):
        self.control = 'saturation'
        self.update_advanced_control(1)

    @Slot()
    def subtract_saturation(self):
        self.control = 'saturation'
        self.update_advanced_control(-1)

    @Slot()
    def add_sharpness(self):
        self.control = 'sharpness'
        self.update_advanced_control(1)

    @Slot()
    def subtract_sharpness(self):
        self.control = 'sharpness'
        self.update_advanced_control(-1)

    @Slot()
    def enable_auto_exposure(self):
        ctrl = dai.CameraControl()
        ctrl.setAutoExposureEnable()
        self.control_command_signal.emit(ctrl)  # Emit signal
        self.camera_error_queue.put(f"Comment: Depthai Autoexposure enabled.")
        self.expTime = 20000  # Reset manual values
        self.sensIso = 800
        self.exposure_updated_signal.emit(self.expTime)
        self.iso_updated_signal.emit(self.sensIso)

    @Slot()
    def enable_auto_focus(self):
        ctrl = dai.CameraControl()
        ctrl.setAutoFocusMode(dai.CameraControl.AutoFocusMode.AUTO)
        ctrl.setAutoFocusTrigger()
        self.control_command_signal.emit(ctrl)  # Emit signal
        self.camera_error_queue.put(f"Comment: Depthai Autofocus triggered (one-shot).")

    @Slot()
    def enable_continuous_af(self):
        ctrl = dai.CameraControl()
        ctrl.setAutoFocusMode(dai.CameraControl.AutoFocusMode.CONTINUOUS_VIDEO)
        self.control_command_signal.emit(ctrl)  # Emit signal
        self.camera_error_queue.put(f"Comment: Depthai Continuous autofocus enabled.")

    @Slot()
    def enable_auto_wb(self):
        ctrl = dai.CameraControl()
        ctrl.setAutoWhiteBalanceMode(
            dai.CameraControl.AutoWhiteBalanceMode.AUTO)
        self.control_command_signal.emit(ctrl)  # Emit signal
        self.camera_error_queue.put(f"Comment: Depthai Auto white-balance enabled.")
        self.wbManual = 4000  # Reset manual WB value
        self.wb_updated_signal.emit(self.wbManual)

    @Slot()
    def move_roi_up(self):
        self.cropY = clamp(self.cropY - STEP_SIZE, 0.0, self.maxCropY)
        self.sendCamConfig = True
        self.camera_error_queue.put(f"Comment: Depthai ROI moved up to Y: {self.cropY:.3f}")

    @Slot()
    def move_roi_down(self):
        self.cropY = clamp(self.cropY + STEP_SIZE, 0.0, self.maxCropY)
        self.sendCamConfig = True
        self.camera_error_queue.put(f"Comment: Depthai ROI moved down to Y: {self.cropY:.3f}")

    @Slot()
    def move_roi_left(self):
        self.cropX = clamp(self.cropX - STEP_SIZE, 0.0, self.maxCropX)
        self.sendCamConfig = True
        self.camera_error_queue.put(f"Comment: Depthai ROI moved left to X: {self.cropX:.3f}")

    @Slot()
    def move_roi_right(self):
        self.cropX = clamp(self.cropX + STEP_SIZE, 0.0, self.maxCropX)
        self.sendCamConfig = True
        self.camera_error_queue.put(f"Comment: DepthaiROI moved right to X: {self.cropX:.3f}")

    @Slot()
    def add_wb(self):
        ctrl = dai.CameraControl()
        self.wbManual = clamp(self.wbManual + WB_STEP, 1000, 12000)
        ctrl.setManualWhiteBalance(self.wbManual)
        self.control_command_signal.emit(ctrl)  # Emit signal
        self.wb_updated_signal.emit(self.wbManual)
        self.camera_error_queue.put(f"Comment: Depthai White Balance: {self.wbManual} K")

    @Slot()
    def subtract_wb(self):
        ctrl = dai.CameraControl()
        self.wbManual = clamp(self.wbManual - WB_STEP, 1000, 12000)
        ctrl.setManualWhiteBalance(self.wbManual)
        self.control_command_signal.emit(ctrl)  # Emit signal
        self.wb_updated_signal.emit(self.wbManual)
        self.camera_error_queue.put(f"Comment: Depthai White Balance: {self.wbManual} K")

    def update_advanced_control(self, change=1):
        ctrl = dai.CameraControl()
        val = None
        if self.control == 'ae_comp':
            self.ae_comp = clamp(self.ae_comp + change, -9, 9)
            ctrl.setAutoExposureCompensation(self.ae_comp)
            val = self.ae_comp
        elif self.control == 'awb_mode':
            val = next(self.awb_mode)
            ctrl.setAutoWhiteBalanceMode(val)
        elif self.control == 'anti_banding_mode':
            val = next(self.anti_banding_mode)
            ctrl.setAntiBandingMode(val)
        elif self.control == 'effect_mode':
            val = next(self.effect_mode)
            ctrl.setEffectMode(val)
        elif self.control == 'brightness':
            self.brightness = clamp(self.brightness + change, -10, 10)
            ctrl.setBrightness(self.brightness)
            val = self.brightness
            self.brightness_updated_signal.emit(self.brightness)
        elif self.control == 'contrast':
            self.contrast = clamp(self.contrast + change, -10, 10)
            ctrl.setContrast(self.contrast)
            val = self.contrast
            self.contrast_updated_signal.emit(self.contrast)
        elif self.control == 'saturation':
            self.saturation = clamp(self.saturation + change, -10, 10)
            ctrl.setSaturation(self.saturation)
            val = self.saturation
            self.saturation_updated_signal.emit(self.saturation)
        elif self.control == 'sharpness':
            self.sharpness = clamp(self.sharpness + change, 0, 4)
            ctrl.setSharpness(self.sharpness)
            val = self.sharpness
            self.sharpness_updated_signal.emit(self.sharpness)
        elif self.control == 'luma_denoise':
            self.luma_denoise = clamp(self.luma_denoise + change, 0, 4)
            ctrl.setLumaDenoise(self.luma_denoise)
            val = self.luma_denoise
        elif self.control == 'chroma_denoise':
            self.chroma_denoise = clamp(self.chroma_denoise + change, 0, 4)
            ctrl.setChromaDenoise(self.chroma_denoise)
            val = self.chroma_denoise

        self.camera_error_queue.put(f"Comment: Depthai {self.control} updated: {val}")
        self.control_command_signal.emit(ctrl)  # Emit signal
