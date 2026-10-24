import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3

Rectangle {
  property var control: null
  color: "transparent"
  anchors.fill: parent
  Layout.minimumWidth: 320
  Layout.minimumHeight: 145

  ColumnLayout {
    anchors.fill: parent
    anchors.margins: 10
    spacing: 6

    Button {
      objectName: "replayButton"
      text: qsTr("Replay landing")
      highlighted: true
      enabled: control !== null && !control.busy
      Layout.fillWidth: true
      onClicked: control.Replay()
    }
    Label {
      text: qsTr("Reset the scene and start a fresh landing.")
      Layout.fillWidth: true
      wrapMode: Text.WordWrap
    }
    Label {
      objectName: "replayStatus"
      text: control !== null ? control.statusText : qsTr("Connecting to landing controller...")
      Layout.fillWidth: true
      wrapMode: Text.WordWrap
      opacity: 0.8
      font.pixelSize: 12
    }
    Item { Layout.fillHeight: true }
  }
}
