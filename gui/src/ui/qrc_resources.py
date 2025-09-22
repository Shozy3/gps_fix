# Resource object code (Python 3)
# Created by: object code
# Created by: The Resource Compiler for Qt version 6.9.1
# WARNING! All changes made in this file will be lost!

from PySide6 import QtCore

qt_resource_data = b"\
\x00\x00\x0c\xd2\
i\
mport QtQuick\x0aim\
port QtQuick.Win\
dow\x0aimport QtLoc\
ation\x0aimport QtP\
ositioning\x0a\x0aRect\
angle {\x0a    prop\
erty list<MapQui\
ckItem> markerBu\
ffer\x0a\x0a    Plugin\
 {\x0a        id: m\
apPlugin\x0a       \
 name: \x22osm\x22\x0a   \
 }\x0a\x0a    Map {\x0a  \
      id: map\x0a  \
      anchors.fi\
ll: parent\x0a     \
   plugin: mapPl\
ugin\x0a        cop\
yrightsVisible: \
false\x0a        ce\
nter: QtPosition\
ing.coordinate(5\
3.52560193189683\
, -113.526401104\
09622) // Edmont\
on\x0a        zoomL\
evel: 14\x0a       \
 property geoCoo\
rdinate startCen\
troid\x0a\x0a        P\
inchHandler {\x0a  \
          id: pi\
nch\x0a            \
target: null\x0a   \
         onActiv\
eChanged: if (ac\
tive) {\x0a        \
        map.star\
tCentroid = map.\
toCoordinate(pin\
ch.centroid.posi\
tion, false)\x0a   \
         }\x0a     \
       onScaleCh\
anged: (delta) =\
> {\x0a            \
    map.zoomLeve\
l += Math.log2(d\
elta)\x0a          \
      map.alignC\
oordinateToPoint\
(map.startCentro\
id, pinch.centro\
id.position)\x0a   \
         }\x0a     \
       onRotatio\
nChanged: (delta\
) => {\x0a         \
       map.beari\
ng -= delta\x0a    \
            map.\
alignCoordinateT\
oPoint(map.start\
Centroid, pinch.\
centroid.positio\
n)\x0a            }\
\x0a            gra\
bPermissions: Po\
interHandler.Tak\
eOverForbidden\x0a \
       }\x0a       \
 WheelHandler {\x0a\
            id: \
wheel\x0a          \
  // workaround \
for QTBUG-87646 \
/ QTBUG-112394 /\
 QTBUG-112432:\x0a \
           // Ma\
gic Mouse preten\
ds to be a track\
pad but doesn't \
work with PinchH\
andler\x0a         \
   // and we don\
't yet distingui\
sh mice and trac\
kpads on Wayland\
 either\x0a        \
    acceptedDevi\
ces: Qt.platform\
.pluginName === \
\x22cocoa\x22 || Qt.pl\
atform.pluginNam\
e === \x22wayland\x22\x0a\
                \
             ? P\
ointerDevice.Mou\
se | PointerDevi\
ce.TouchPad\x0a    \
                \
         : Point\
erDevice.Mouse\x0a \
           rotat\
ionScale: 1/120\x0a\
            prop\
erty: \x22zoomLevel\
\x22\x0a        }\x0a    \
    DragHandler \
{\x0a            id\
: drag\x0a         \
   target: null\x0a\
            onTr\
anslationChanged\
: (delta) => map\
.pan(-delta.x, -\
delta.y)\x0a       \
 }\x0a        Short\
cut {\x0a          \
  enabled: map.z\
oomLevel < map.m\
aximumZoomLevel\x0a\
            sequ\
ence: StandardKe\
y.ZoomIn\x0a       \
     onActivated\
: map.zoomLevel \
= Math.round(map\
.zoomLevel + 1)\x0a\
        }\x0a      \
  Shortcut {\x0a   \
         enabled\
: map.zoomLevel \
> map.minimumZoo\
mLevel\x0a         \
   sequence: Sta\
ndardKey.ZoomOut\
\x0a            onA\
ctivated: map.zo\
omLevel = Math.r\
ound(map.zoomLev\
el - 1)\x0a        \
}\x0a    }\x0a\x0a    // \
Main point-plott\
ing function\x0a   \
 function plotPo\
int(lat, lon) {\x0a\
        const co\
ord = QtPosition\
ing.coordinate(l\
at, lon)\x0a       \
 const marker = \
locmarker.create\
Object(map, {\x0a  \
          coordi\
nate: coord\x0a    \
    })\x0a\x0a        \
map.addMapItem(m\
arker)\x0a        m\
arkerBuffer.push\
(marker)\x0a       \
 map.center = co\
ord \x0a\x0a        if\
 (markerBuffer.l\
ength > 1000) {\x0a\
            cons\
t oldest = marke\
rBuffer.shift()\x0a\
            map.\
removeMapItem(ol\
dest)\x0a        }\x0a\
    }\x0a\x0a    funct\
ion setLocationM\
arker_1(lat, lon\
) {\x0a        cons\
ole.log(\x22Setting\
 location marker\
 at:\x22, lat, lon)\
;\x0a        plotPo\
int(lat, lon)\x0a  \
  }\x0a\x0a    Compone\
nt {\x0a        id:\
 locmarker\x0a     \
   MapQuickItem \
{\x0a            co\
ordinate: QtPosi\
tioning.coordina\
te(0, 0)\x0a       \
     anchorPoint\
.x: dot.width / \
2\x0a            an\
chorPoint.y: dot\
.height / 2\x0a    \
        sourceIt\
em: Rectangle {\x0a\
                \
id: dot\x0a        \
        width: 6\
\x0a               \
 height: 6\x0a     \
           radiu\
s: 3\x0a           \
     color: \x22red\
\x22\x0a            }\x0a\
        }\x0a    }\x0a\
}\
"

qt_resource_name = b"\
\x00\x03\
\x00\x00z\x83\
\x00s\
\x00r\x00c\
\x00\x02\
\x00\x00\x07\xb9\
\x00u\
\x00i\
\x00\x07\
\x03\x83X\xdc\
\x00m\
\x00a\x00p\x00.\x00q\x00m\x00l\
"

qt_resource_struct = b"\
\x00\x00\x00\x00\x00\x02\x00\x00\x00\x01\x00\x00\x00\x01\
\x00\x00\x00\x00\x00\x00\x00\x00\
\x00\x00\x00\x00\x00\x02\x00\x00\x00\x01\x00\x00\x00\x02\
\x00\x00\x00\x00\x00\x00\x00\x00\
\x00\x00\x00\x0c\x00\x02\x00\x00\x00\x01\x00\x00\x00\x03\
\x00\x00\x00\x00\x00\x00\x00\x00\
\x00\x00\x00\x16\x00\x00\x00\x00\x00\x01\x00\x00\x00\x00\
\x00\x00\x01\x98C\xd5+\x1a\
"

def qInitResources():
    QtCore.qRegisterResourceData(0x03, qt_resource_struct, qt_resource_name, qt_resource_data)

def qCleanupResources():
    QtCore.qUnregisterResourceData(0x03, qt_resource_struct, qt_resource_name, qt_resource_data)

qInitResources()
