#include "mainwindow.h"
#include <QApplication>
#include <memory>
#include "rviz_common/ros_integration/ros_node_abstraction.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    QApplication a(argc, argv);
    a.setWindowIcon(QIcon("/home/tomas/ros2_ws/src/PneuTrunk/pneutrunk_gui/Icons/icon.png"));

    auto ros_node_abs =
    std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>("pneutrunk_gui");
    auto w = std::make_shared<MainWindow>(&a, ros_node_abs);
    w->show();

    while (rclcpp::ok()) {
        a.processEvents();
    }
    return 0;
}
