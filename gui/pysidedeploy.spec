[app]
title = coerpp
project_dir = .
input_file = /home/cst/Downloads/coe_rpp/gui/main.py
project_file = gui.pyproject
exec_directory = releases/coerpp/usr/bin
icon = assets/icon.png

[python]
python_path = /home/cst/Downloads/coe_rpp/gui/venv/bin/python3
packages = Nuitka

[qt]
qml_files = ['src/ui/map.qml']
excluded_qml_plugins = []
modules = Gui,DBus,Network,Positioning,Qml,SerialPort,PositioningQuick,QuickWidgets,QmlMeta,QuickShapes,Location,QmlModels,OpenGL,QmlWorkerScript,Quick,Widgets,Core
plugins = styles,iconengines,egldeviceintegrations,accessiblebridge,imageformats,platformthemes,platforminputcontexts,generic,xcbglintegrations,platforms,position,geoservices

[android]
wheel_pyside = 
wheel_shiboken = 
plugins = []

[nuitka]
macos.permissions = []
extra_args = --quiet --noinclude-qt-translations --include-package=PySide6.QtLocation --include-package=PySide6.QtQuick

