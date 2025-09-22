from PySide6.QtCore import Signal, QObject


class MapInterface(QObject):
    marker1 = Signal(float, float)

    def __init__(self):
        super().__init__()

    def add_marker(self, lat, lon):
        if (lat != 0 and lon != 0) and (lat is not None and lon is not None):
            self.marker1.emit(float(lat), float(lon))
