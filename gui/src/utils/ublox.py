import os
import time
import math
import serial
import threading
import requests
import numpy as np
import src.utils.datatypes as dt

from pysbf2 import SBFReader
from queue import Queue, Empty
from pygnssutils import GNSSNTRIPClient
from PySide6.QtCore import QObject, QThread
from datetime import datetime, timezone, date

GPS_EPOCH = datetime(1980, 1, 6)
GPS_UTC_OFFSET = 18
DEG_TO_RAD = np.pi / 180
FIX_FLAGS = {
    "0": "No Fix",
    "1": "2D/3D GNSS fix",
    "2": "Differential GNSS fix",
    "4": "RTK Fixed",
    "5": "RTK Float",
    "6": "GNSS Dead Reckoning",
}
GNSS_FIX_FLAGS = {
    0: "No Fix",
    1: "Dead Reckoning",
    2: "2D GNSS fix",
    3: "3D GNSS fix",
    4: "GNSS + Dead Reckoning",
    5: "Time only",
}

# Weather
# https://api.weather.gc.ca/collections/climate-hourly/items?limit=10&offset=0&LOCAL_YEAR=2025&LOCAL_MONTH=07&LOCAL_DAY=20
# https://acis.alberta.ca/weather-conditions-map.jsp#


def weather_data(lat, lon):
    try:
        response = requests.get(
            "https://511.alberta.ca/api/v2/get/weatherstations"
        )
        if response.status_code == 200:
            data = response.json()
            for station in data:
                station_lat = station.get("Latitude")
                station_lon = station.get("Longitude")
                if station_lat and station_lon:
                    distance = math.sqrt(
                        (station_lat - lat) ** 2 + (station_lon - lon) ** 2)
                    if distance < 0.1:
                        return {
                            'id': station.get("ID"),
                            'stationlat': station_lat,
                            'stationlon': station_lon,
                            'AirTemperature': station.get("AirTemperature"),
                            'PavementTemperature': station.get("PavementTemperature"),
                            'RelativeHumidity': station.get("RelativeHumidity"),
                            'WindSpeed': station.get("WindSpeed"),
                            'WindDirection': station.get("WindDirection"),
                        }
    except requests.RequestException as e:
        print(f"Weather data request error: {e}")
        return None


class Ublox(QObject):
    def __init__(self, **kwargs):
        super().__init__()

        self._serial = None
        self.running = False
        self.gps_port = kwargs.get("gps_port", "/dev/ttyACM0")
        self.baud_rate = kwargs.get("baud_rate", 9600)
        self.save_data = kwargs.get("save_data", False)
        self.save_path = kwargs.get("save_path", None)
        self.ntrip_details = kwargs.get("ntrip_details", {"start": False})
        self.gps_queue = kwargs.get("gps_queue", None)
        self.gps_error_queue = kwargs.get("gps_error_queue", None)
        self.display_timer = kwargs.get("display_timer", 1)

        self.template = {**dt.time_template, **dt.gps_template}

        self._current_data = self.template.copy()
        self._last_data = self.template.copy()
        self._status = dt.status_template.copy()
        self._calib_status = dt.calib_status_template.copy()

        if self.save_data:
            base_dir = self.save_path or "."
            filename = f'{datetime.now().strftime("%H-%M-%S")}.csv'

            while True:
                full_path = os.path.join(base_dir, filename)
                if not os.path.exists(full_path):
                    break

            self.save_path = full_path

        self._rawbuffer = Queue()
        self._filebuffer = Queue()  # Use a queue for thread-safe data transfer
        self._ntripbuffer = Queue()
        self._raw_data_thread = None
        self._parse_thread = None
        self._save_thread = None
        self._ntrip_thread = None
        self._ntrip_client = None

        if self.save_data and self.save_path:
            try:
                os.makedirs(base_dir, exist_ok=True)
            except Exception as e:
                print(f"Error opening file for writing: {e}")
                self.save_data = False

        self.start()

    def start(self):
        try:
            self._serial = serial.Serial(
                self.gps_port, self.baud_rate, timeout=1)
            self._sbf_reader = SBFReader(
                self._serial, protfilter=7, quitonerror=0)

            self.running = True

            self._raw_data_thread = threading.Thread(target=self._read_raw)
            self._raw_data_thread.start()

            self._parse_thread = threading.Thread(
                target=self._parse_sensor_data)
            self._parse_thread.start()

            if self.save_data:
                self._save_thread = threading.Thread(
                    target=self._save_data_thread)
                self._save_thread.start()

            if self.gps_queue is not None:
                while True:
                    temp = {**self._last_data, **
                            self._status, **self._calib_status}
                    temp['fix'] = FIX_FLAGS.get(
                        temp['fix'], "Unknown")
                    temp = {k: str(v) if isinstance(
                        v, (int, float)) else v for k, v in temp.items()}

                    self.gps_queue.put(temp)
                    time.sleep(self.display_timer)

        except Exception as e:
            print(f"UBlox port error: {e}")
            if self.gps_error_queue:
                self.gps_error_queue.put(f"Serial port error: {e}")

    def _start_ntrip_thread(self):
        self._ntrip_client = GNSSNTRIPClient(app=self)
        self._ntrip_client.run(
            server=self.ntrip_details.get('server'),
            port=self.ntrip_details.get('port'),
            mountpoint=self.ntrip_details.get('mountpoint'),
            datatype=self.ntrip_details.get('datatype'),
            ntripuser=self.ntrip_details.get('ntripuser'),
            ntrippassword=self.ntrip_details.get('ntrippassword'),
            version=self.ntrip_details.get('version'),
            ggainterval=self.ntrip_details.get('ggainterval'),
            ggamode=0,
            output=self._ntripbuffer,
        )

        self.ntrip_thread = threading.Thread(target=self._read_ntrip)
        self.ntrip_thread.start()

    def stop(self):  # Ensure any remaining data is saved
        self.running = False

        if self.ntrip_details['start']:
            self._stop_ntrip()

        if isinstance(self._raw_data_thread, threading.Thread) and self._raw_data_thread.is_alive():
            self._raw_data_thread.join()

        if isinstance(self._parse_thread, threading.Thread) and self._parse_thread.is_alive():
            self._parse_thread.join()

        if isinstance(self._save_thread, threading.Thread) and self._save_thread.is_alive():
            self._save_thread.join()

        if self._serial and self._serial.is_open:
            self._serial.close()

    def _stop_ntrip(self):
        self.ntrip_details['start'] = False

        if isinstance(self._ntrip_client, GNSSNTRIPClient) and self._ntrip_client._connected:
            self._ntrip_client.stop()
            self._ntrip_client = None

        if isinstance(self._ntrip_thread, threading.Thread) and self._ntrip_thread.is_alive():
            self._ntrip_thread.join()
            self._ntrip_thread = None

    def _read_raw(self):
        while self.running:
            try:
                if self._serial.in_waiting:
                    _, parsed_data = self._sbf_reader.read()
                    self._rawbuffer.put(parsed_data)
            except Exception as e:
                print(f"GPS Read Error: {e}")
                if self.gps_error_queue:
                    self.gps_error_queue.put(f"GPS Read Error: {e}")

    def _read_ntrip(self):
        while self.running:
            try:
                # Block for a short time waiting for data (non-busy loop)
                raw_data, parsed_data = self._ntripbuffer.get(
                    timeout=1)  # Blocking read, 1s timeout
                self._serial.write(raw_data)
            except Empty:
                # print("Empty Data NTRIP")
                continue  # No data this second, just keep looping
            except Exception as e:
                print(f"NTRIP Read Error: {e}")
                if self.gps_error_queue:
                    self.gps_error_queue.put(f"NTRIP Read Error: {e}")

    def _parse_sensor_data(self):
        while self.running:
            try:
                parsed_data = self._rawbuffer.get(timeout=1)
                # print(parsed_data)
                if hasattr(parsed_data, "identity"):
                    msg_type = parsed_data.identity

                    if msg_type.endswith("GSA"):
                        self._status.update({
                            "HDOP": parsed_data.HDOP,
                            "VDOP": parsed_data.VDOP,
                            "PDOP": parsed_data.PDOP,
                        })

                    elif msg_type.endswith("GGA"):
                        time_str = str(parsed_data.time)
                        if time_str.find(".") == -1:
                            time_str += ".000000"
                        date_str = date.today()
                        iso_time = f"{date_str}T{time_str}Z"
                        epoch_time = datetime.strptime(iso_time, "%Y-%m-%dT%H:%M:%S.%fZ").replace(
                            tzinfo=timezone.utc
                        )
                        epoch_time = epoch_time.timestamp()

                        system_time = datetime.now()
                        system_time_str = system_time.strftime(
                            "%Y-%m-%dT%H:%M:%S.%fZ")
                        system_epoch_time = datetime.strptime(system_time_str, "%Y-%m-%dT%H:%M:%S.%fZ").replace(
                            tzinfo=timezone.utc
                        )

                        self._current_data.update({
                            "systemtime": system_time_str,
                            "systemepoch": f"{system_epoch_time.timestamp():.3f}",
                            "gpstime": iso_time,
                            "gpsepoch": f"{epoch_time:.3f}",
                            "lat": parsed_data.lat,
                            "lon": parsed_data.lon,
                            "alt": parsed_data.alt,
                            "sep": parsed_data.sep,
                            "sip": parsed_data.numSV,
                            "fix": parsed_data.quality,
                            "hdop": parsed_data.HDOP,
                            "diffage": parsed_data.diffAge,
                            "diffstation": parsed_data.diffStation
                        })

                        self._status.update({
                            "HDOP": parsed_data.HDOP,
                            "diffage": parsed_data.diffAge,
                            "diffstation": parsed_data.diffStation
                        })

                    elif msg_type.endswith("VTG"):
                        self._current_data["azimuth"] = parsed_data.cogt

                    elif msg_type in ["PosCovCartesian"]:
                        #     # print(parsed_data)
                        #     cov_latlat = parsed_data.Cov_latlat
                        #     cov_lonlon = parsed_data.Cov_lonlon
                        #     cov_altalt = parsed_data.Cov_hgthgt
                        #     d2acc = 2 * math.sqrt(cov_latlat + cov_lonlon)
                        #     d3acc = 2 * \
                        #         math.sqrt(cov_latlat + cov_lonlon + cov_altalt)
                        #     self._status.update({
                        #         "2D hAcc": d2acc,  # m
                        #         # "2D vAcc": parsed_data.VAccuracy / 100,  # m
                        #         "3D Acc": d3acc,  # m
                        #     })
                        cov_xx = parsed_data.Cov_xx
                        cov_yy = parsed_data.Cov_yy
                        cov_zz = parsed_data.Cov_zz
                        # print(f"Covariance: {cov_xx}, {cov_yy}, {cov_zz}")
                        # # Check for valid variances
                        if any(cov < 0 for cov in [cov_xx, cov_yy, cov_zz]):
                            hacc_2d, vacc_2d, acc_3d = 0.0, 0.0, 0.0
                        else:
                            hacc_2d = 2 * math.sqrt(cov_xx + cov_yy)
                            vacc_2d = 2 * math.sqrt(cov_zz)
                            acc_3d = 2 * math.sqrt(cov_xx + cov_yy + cov_zz)

                        self._status.update({
                            "2D hAcc": hacc_2d,  # m
                            "2D vAcc": vacc_2d,  # m
                            "3D Acc": acc_3d,  # m
                        })

                    # else:
                    #     print(f"Unknown message type: {msg_type}")
                    #     print(f"Data: {parsed_data}")

                required_keys = ["systemtime", "gpstime",
                                 "lat", "lon", "alt", "fix"]
                if all(self._current_data.get(k) is not None for k in required_keys):
                    self._last_data = {
                        **self._current_data.copy(), **self._status.copy(), **self._calib_status.copy()}
                    self._current_data = self.template.copy()
                    # print(self._last_data)
                    if (self._ntrip_client is None and
                            self.ntrip_details['start'] and
                            not self._last_data['lat'] == '' and
                            not self._last_data['lon'] == '' and
                            self._last_data['fix'] > 0
                        ):
                        print('STARTING NTRIP client')
                        self._start_ntrip_thread()

                    self._last_data = {k: str(v) if isinstance(
                        v, (int, float)) else v for k, v in self._last_data.items()}

                    if self.save_data:
                        self._filebuffer.put(self._last_data)

            except Exception as e:
                print(f"Parsing Error: {e}")
                if self.gps_error_queue:
                    self.gps_error_queue.put(f"Parsing Error: {e}")

    def _save_data_thread(self):
        with open(self.save_path, "a", buffering=1) as f:
            data_batch = []  # List to collect data packets
            # Write temperature information
            if self._last_data.get('lat') and self._last_data.get('lon'):
                weather = weather_data(
                    self._last_data['lat'], self._last_data['lon'])
                if weather:
                    f.write(
                        f"# Weather data for lat: {self._last_data['lat']}, lon: {self._last_data['lon']}\n")
                    f.write(f"# Station ID: {weather['id']}\n")
                    f.write(
                        f"# Station Lat: {weather['stationlat']}, Lon: {weather['stationlon']}\n")
                    f.write(
                        f"# Air Temperature: {weather['AirTemperature']} °C\n")
                    f.write(
                        f"# Pavement Temperature: {weather['PavementTemperature']} °C\n")
                    f.write(
                        f"# Relative Humidity: {weather['RelativeHumidity']} %\n")
                    f.write(f"# Wind Speed: {weather['WindSpeed']} m/s\n")
                    f.write(
                        f"# Wind Direction: {weather['WindDirection']} °\n")
            else:
                f.write("# No weather data available\n")

            # Write the header only once
            f.write(",".join(
                {**self._current_data, **self._status, **self._calib_status}.keys()) + "\n")
            while self.running:
                try:
                    data = self._filebuffer.get(timeout=1)
                    # Collect data in the batch
                    data_batch.append(",".join(data.values()))

                    # Check if we have collected 100 data packets
                    if len(data_batch) >= 5:
                        # Write the batch to the file
                        f.write("\n".join(data_batch) + "\n")
                        data_batch.clear()  # Clear the batch after writing
                except Empty:
                    continue
                except Exception as e:
                    print(f"Error writing to file: {e}")
                    if self.gps_error_queue:
                        self.gps_error_queue.put(f"Error writing to file: {e}")

            # Write any remaining data in the batch when the thread stops
            if data_batch:
                f.write("\n".join(data_batch) + "\n")

    def get_coordinates(self):
        return self._last_data

    def get_calib_status(self):
        return self._calib_status

    def get_status(self):
        return self._status

    def clear_status(self):
        self._status = dt.status_template.copy()
        self._calib_status = dt.calib_status_template.copy()

    def __del__(self):
        self.stop()


if __name__ == "__main__":
    gps_thread = QThread()
    gps = Ublox(gps_port="/dev/ttyACM0",
                save_data=False, save_path="test")
    gps.moveToThread(gps_thread)
    gps_thread.started.connect(gps.start)
    gps_thread.start()
    try:
        while True:
            print(gps.get_coordinates())
            time.sleep(1)
    except KeyboardInterrupt:
        gps.stop()
    except Exception as e:
        print(f"Unexpected error: {e}")
        gps.stop()
