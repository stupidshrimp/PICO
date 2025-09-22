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
    property string tileDirectoryUrl: (typeof mapTileDirectoryUrl !== "undefined") ? mapTileDirectoryUrl : ""
    property bool hasOfflineTiles: (typeof mapHasOfflineTiles !== "undefined") ? mapHasOfflineTiles : false
    property string offlineStatus: (typeof mapOfflineStatus !== "undefined") ? mapOfflineStatus : ""
    property var mapObject: null

    function clampZoom(value) {
        if (maximumZoomLevel < minimumZoomLevel) {
            return value;
        }
        return Math.min(Math.max(value, minimumZoomLevel), maximumZoomLevel);
    }

    function applyInitialView() {
        var map = mapLoader.item;
        if (!map) {
            return;
        }
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
        var map = mapLoader.item;
        if (!map || !map.gpsMarkerItem) {
            return;
        }
        var coordinate = QtPositioning.coordinate(root.gpsLat, root.gpsLon);
        map.gpsMarkerItem.coordinate = coordinate;
        map.gpsMarkerItem.visible = true;
        if (!root.firstFixReceived) {
            root.firstFixReceived = true;
            map.center = coordinate;
        } else if (root.followGps) {
            map.center = coordinate;
        }
    }

    function setZoomLevel(value) {
        var map = mapLoader.item;
        if (!map) {
            return;
        }
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
            name: "osm.mapping.providersrepository.disabled"
            value: true
        }
        PluginParameter {
            name: "osm.mapping.providersrepository.enabled"
            value: false
        }
        PluginParameter {
            name: "osm.mapping.custom.host"
            value: root.hasOfflineTiles ? root.tileDirectoryUrl : ""
        }
        PluginParameter {
            name: "osm.mapping.highdpi_tiles"
            value: true
        }
        PluginParameter {
            name: "osm.mapping.offline.directory"
            value: ""
        }
    }

    Loader {
        id: mapLoader
        anchors.fill: parent
        active: root.hasOfflineTiles
        sourceComponent: Component {
            Map {
                id: map
                property alias gpsMarkerItem: gpsMarker
                anchors.fill: parent
                plugin: offlinePlugin
                copyrightsVisible: false

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
                    root.applyInitialView();
                    for (var i = 0; i < supportedMapTypes.length; ++i) {
                        var type = supportedMapTypes[i];
                        if (type.name && type.name.toLowerCase().indexOf("custom") !== -1) {
                            activeMapType = type;
                            break;
                        }
                    }
                    if (!activeMapType && supportedMapTypes.length > 0) {
                        activeMapType = supportedMapTypes[0];
                    }
                }
            }
        }

        onStatusChanged: {
            if (status === Loader.Ready && item) {
                root.mapObject = item;
                root.applyInitialView();
                root.updateGpsMarker();
            } else if (status === Loader.Error || status === Loader.Null) {
                root.mapObject = null;
            }
        }
    }

    Rectangle {
        anchors.fill: parent
        visible: !root.hasOfflineTiles
        color: "#101820"
        border.color: "#2c3e50"
        border.width: 1

        Text {
            anchors.centerIn: parent
            width: parent.width * 0.8
            text: root.offlineStatus && root.offlineStatus.length > 0 ? root.offlineStatus : "Offline map tiles unavailable."
            color: "#f0f0f0"
            wrapMode: Text.WordWrap
            horizontalAlignment: Text.AlignHCenter
        }
    }

    onGpsLatChanged: updateGpsMarker()
    onGpsLonChanged: updateGpsMarker()

    onFollowGpsChanged: {
        if (root.followGps && root.firstFixReceived && isFinite(root.gpsLat) && isFinite(root.gpsLon)) {
            var map = mapLoader.item;
            if (!map) {
                return;
            }
            var coord = QtPositioning.coordinate(root.gpsLat, root.gpsLon);
            map.center = coord;
        }
    }

    onMinimumZoomLevelChanged: {
        var map = mapLoader.item;
        if (!map) {
            return;
        }
        map.minimumZoomLevel = minimumZoomLevel;
        if (map.zoomLevel < minimumZoomLevel) {
            map.zoomLevel = minimumZoomLevel;
        }
    }

    onMaximumZoomLevelChanged: {
        var map = mapLoader.item;
        if (!map) {
            return;
        }
        map.maximumZoomLevel = maximumZoomLevel;
        if (map.zoomLevel > maximumZoomLevel) {
            map.zoomLevel = maximumZoomLevel;
        }
    }

    onHasOfflineTilesChanged: {
        if (!hasOfflineTiles) {
            mapObject = null;
        }
    }
}
