import QtQuick 2.15
import QtQuick.Controls 2.15
import QtLocation 5.15
import QtPositioning 5.15

Item {
    id: root
    width: 600
    height: 381

    property real gpsLat: Number.NaN
    property real gpsLon: Number.NaN
    property bool followGps: true
    property bool firstFixReceived: false
    property var initialCenter: [0, 0]
    property real initialZoom: 8
    property int minimumZoomLevel: 0
    property int maximumZoomLevel: 15
    property string tileDirectory: mapTileDirectory

    function clampZoom(value) {
        if (maximumZoomLevel < minimumZoomLevel) {
            return value;
        }
        return Math.min(Math.max(value, minimumZoomLevel), maximumZoomLevel);
    }

    function applyInitialView() {
        if (initialCenter.length === 2) {
            map.center = QtPositioning.coordinate(initialCenter[0], initialCenter[1]);
        }
        map.minimumZoomLevel = minimumZoomLevel;
        map.maximumZoomLevel = maximumZoomLevel;
        map.zoomLevel = clampZoom(initialZoom);
    }

    function updateGpsMarker() {
        if (!isFinite(root.gpsLat) || !isFinite(root.gpsLon)) {
            return;
        }
        var coordinate = QtPositioning.coordinate(root.gpsLat, root.gpsLon);
        gpsMarker.coordinate = coordinate;
        gpsMarker.visible = true;
        if (!root.firstFixReceived) {
            root.firstFixReceived = true;
            map.center = coordinate;
        } else if (root.followGps) {
            map.center = coordinate;
        }
    }

    function setZoomLevel(value) {
        map.zoomLevel = clampZoom(value);
    }

    function setInitialCenter(lat, lon) {
        initialCenter = [lat, lon];
        if (!root.firstFixReceived) {
            applyInitialView();
        }
    }

    Plugin {
        id: offlinePlugin
        name: "osm"
        PluginParameter {
            name: "osm.mapping.offline.directory"
            value: root.tileDirectory
        }
        PluginParameter {
            name: "osm.mapping.highdpi_tiles"
            value: true
        }
    }

    Map {
        id: map
        anchors.fill: parent
        plugin: offlinePlugin
        gesture: MapGestureArea {
            enabled: true
        }
        copyrightsVisible: false
        activeMapType: supportedMapTypes.length > 0 ? supportedMapTypes[0] : null

        MapQuickItem {
            id: gpsMarker
            anchorPoint.x: markerVisual.width / 2
            anchorPoint.y: markerVisual.height
            visible: false
            sourceItem: Item {
                id: markerVisual
                width: 24
                height: 24

                Rectangle {
                    anchors.centerIn: parent
                    width: 20
                    height: 20
                    radius: width / 2
                    color: "#ff3b30"
                    border.width: 2
                    border.color: "white"
                }

                Rectangle {
                    anchors.horizontalCenter: parent.horizontalCenter
                    anchors.bottom: parent.bottom
                    width: 4
                    height: 8
                    radius: 2
                    color: "#ff3b30"
                }
            }
        }

        onZoomLevelChanged: {
            if (map.zoomLevel > root.maximumZoomLevel) {
                map.zoomLevel = root.maximumZoomLevel;
            } else if (map.zoomLevel < root.minimumZoomLevel) {
                map.zoomLevel = root.minimumZoomLevel;
            }
        }

        Component.onCompleted: {
            applyInitialView();
        }
    }

    onGpsLatChanged: updateGpsMarker()
    onGpsLonChanged: updateGpsMarker()

    onFollowGpsChanged: {
        if (root.followGps && root.firstFixReceived && isFinite(root.gpsLat) && isFinite(root.gpsLon)) {
            var coord = QtPositioning.coordinate(root.gpsLat, root.gpsLon);
            map.center = coord;
        }
    }
}
