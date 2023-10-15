import QtQuick 2.13
import QtQuick.Window 2.13
import QtQuick.Controls 2.13
import QtCharts 2.13

ApplicationWindow
{
    id: root
    visible: true
    width: 640
    height: 480
    title: qsTr("USV GUI")
    property int timeStep: 0
    property double point1_x: 0.0
    property double point1_y: 0.0
    property double point2_x: 0.0
    property double point2_y: 0.0

    Button{
        id: start_moving
        text: "Start Moving"
        anchors.left : parent.horizontalCenter
        anchors.leftMargin: 10
        anchors.bottom: parent.bottom
        anchors.bottomMargin: 10
        onClicked: console.log("Start Route Button")
    }

    Button{
        id: start_planning
        text: "Set Route"
        anchors.right : parent.horizontalCenter
        anchors.rightMargin: 10
        anchors.bottom: parent.bottom
        anchors.bottomMargin: 10
        onClicked: console.log("Start Route Planning")
    }

    Rectangle{
        id: for_buttons
        color: "lightgrey"
        width: 150
        anchors.right : parent.right
        anchors.rightMargin: 25
        anchors.top: parent.top
        anchors.topMargin: 25
        anchors.bottom: parent.bottom
        anchors.bottomMargin: 25
        radius: 8

        Text{
            id: velocity_value
            anchors.horizontalCenter: for_buttons.horizontalCenter
            anchors.top: for_buttons.top
            anchors.topMargin: 30
            text: "Velocity 0.0 m/s"
        }

        Text{
            id: yaw_value
            anchors.horizontalCenter: for_buttons.horizontalCenter
            anchors.top: velocity_value.top
            anchors.topMargin: 30
            text: "Yaw rad"
        }

        Text{
            id: pitch_value
            anchors.horizontalCenter: for_buttons.horizontalCenter
            anchors.top: yaw_value.top
            anchors.topMargin: 30
            text: "Yaw rad"
        }

        Text{
            id: roll_value
            anchors.horizontalCenter: for_buttons.horizontalCenter
            anchors.top: pitch_value.top
            anchors.topMargin: 30
            text: "Roll rad"
        }

        Text{
            id: state_value
            anchors.horizontalCenter: for_buttons.horizontalCenter
            anchors.top: roll_value.top
            anchors.topMargin: 30
            text: "State: Standing"
        }

        TextField{
            id: point1_x
            anchors.top: state_value.top
            anchors.topMargin: 30
            anchors.left: for_buttons.left
            anchors.leftMargin: 10
            anchors.right: for_buttons.right
            anchors.rightMargin: 10
            text: "Point1 X: "

        }

        TextField{
            id: point1_y
            anchors.top: point1_x.top
            anchors.topMargin: 30
            anchors.left: for_buttons.left
            anchors.leftMargin: 10
            anchors.right: for_buttons.right
            anchors.rightMargin: 10
            text: "Point1 Y: "
        }

        TextField{
            id: point2_x
            anchors.top: point1_y.top
            anchors.topMargin: 30
            anchors.left: for_buttons.left
            anchors.leftMargin: 10
            anchors.right: for_buttons.right
            anchors.rightMargin: 10
            text: "Point2 X: "
        }

        TextField{
            id: point2_y
            anchors.top: point2_x.top
            anchors.topMargin: 30
            anchors.left: for_buttons.left
            anchors.leftMargin: 10
            anchors.right: for_buttons.right
            anchors.rightMargin: 10
            text: "Point2: Y"
        }
    }


    ChartView {
        id: chart
        anchors.top: parent.top
        anchors.topMargin: 10
        anchors.right: for_buttons.left
        anchors.rightMargin: 10
        anchors.left: parent.left
        anchors.leftMargin: 10
        anchors.bottom: start_planning.bottom
        anchors.bottomMargin: 40

        ValueAxis {
            id: axisX
            min: -400
            max: 400
        }

        ValueAxis {
            id: axisY
            min: -400
            max: 400
        }

        LineSeries {
            id: path
            axisX: axisX
            axisY: axisY
            name: "Global Path"
        }

        LineSeries {
            id: traj
            axisX: axisX
            axisY: axisY
            name: "Trajectory"
        }
    }

}
