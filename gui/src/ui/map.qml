import QtQuick
import QtQuick.Window
import QtLocation
import QtPositioning

Rectangle {
    property list<MapQuickItem> markerBuffer

    Plugin {
        id: mapPlugin
        name: "osm"
    }

    Map {
        id: map
        anchors.fill: parent
        plugin: mapPlugin
        copyrightsVisible: false
        center: QtPositioning.coordinate(53.52560193189683, -113.52640110409622) // Edmonton
        zoomLevel: 14
        property geoCoordinate startCentroid

        PinchHandler {
            id: pinch
            target: null
            onActiveChanged: if (active) {
                map.startCentroid = map.toCoordinate(pinch.centroid.position, false)
            }
            onScaleChanged: (delta) => {
                map.zoomLevel += Math.log2(delta)
                map.alignCoordinateToPoint(map.startCentroid, pinch.centroid.position)
            }
            onRotationChanged: (delta) => {
                map.bearing -= delta
                map.alignCoordinateToPoint(map.startCentroid, pinch.centroid.position)
            }
            grabPermissions: PointerHandler.TakeOverForbidden
        }
        WheelHandler {
            id: wheel
            // workaround for QTBUG-87646 / QTBUG-112394 / QTBUG-112432:
            // Magic Mouse pretends to be a trackpad but doesn't work with PinchHandler
            // and we don't yet distinguish mice and trackpads on Wayland either
            acceptedDevices: Qt.platform.pluginName === "cocoa" || Qt.platform.pluginName === "wayland"
                             ? PointerDevice.Mouse | PointerDevice.TouchPad
                             : PointerDevice.Mouse
            rotationScale: 1/120
            property: "zoomLevel"
        }
        DragHandler {
            id: drag
            target: null
            onTranslationChanged: (delta) => map.pan(-delta.x, -delta.y)
        }
        Shortcut {
            enabled: map.zoomLevel < map.maximumZoomLevel
            sequence: StandardKey.ZoomIn
            onActivated: map.zoomLevel = Math.round(map.zoomLevel + 1)
        }
        Shortcut {
            enabled: map.zoomLevel > map.minimumZoomLevel
            sequence: StandardKey.ZoomOut
            onActivated: map.zoomLevel = Math.round(map.zoomLevel - 1)
        }
    }

    // Main point-plotting function
    function plotPoint(lat, lon) {
        const coord = QtPositioning.coordinate(lat, lon)
        const marker = locmarker.createObject(map, {
            coordinate: coord
        })

        map.addMapItem(marker)
        markerBuffer.push(marker)
        map.center = coord 

        if (markerBuffer.length > 1000) {
            const oldest = markerBuffer.shift()
            map.removeMapItem(oldest)
        }
    }

    function setLocationMarker_1(lat, lon) {
        console.log("Setting location marker at:", lat, lon);
        plotPoint(lat, lon)
    }

    Component {
        id: locmarker
        MapQuickItem {
            coordinate: QtPositioning.coordinate(0, 0)
            anchorPoint.x: dot.width / 2
            anchorPoint.y: dot.height / 2
            sourceItem: Rectangle {
                id: dot
                width: 6
                height: 6
                radius: 3
                color: "red"
            }
        }
    }
}