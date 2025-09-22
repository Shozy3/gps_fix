time_template = {
    "systemtime": None,
    "systemepoch": None,
}

gps_template = {
    "gpstime": None,
    "gpsepoch": None,
    "lat": None,
    "lon": None,
    "alt": None,
    "azimuth": None,
    "sep": None,    # Geoid separation: difference between ellipsoid and mean sea level
    "fix": None,
    "sip": None,    # number of satellites in view
}

status_template = {
    "diffage": None,
    "diffstation": None,
    "2D hAcc": None,
    "2D vAcc": None,
    "3D Acc": None,
    "fusionMode": 0,
    "imuStatus": 0,
    "gpsFix": 0,
    "numSV": 0,
    "HDOP": 0,
    "VDOP": 0,
    "PDOP": 0,
    "rollAcc": 0,
    "pitchAcc": 0,
    "yawAcc": 0,
    "rtcm_msg": 0,
    "rtcm_crc": 0,
}

calib_status_template = {
    "gyroX_calib": "Not Calibrated",
    "gyroY_calib": "Not Calibrated",
    "gyroZ_calib": "Not Calibrated",
    "accX_calib": "Not Calibrated",
    "accY_calib": "Not Calibrated",
    "accZ_calib": "Not Calibrated",
}
