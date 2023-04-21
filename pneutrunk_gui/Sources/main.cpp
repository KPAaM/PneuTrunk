#include "mainwindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    QApplication a(argc, argv);
    a.setWindowIcon(QIcon("/home/tomas/ros2_ws/src/PneuTrunk/pneutrunk_gui/Icons/icon.png"));
    
    MainWindow w;
    w.show();
    
    while (rclcpp::ok()) {
        a.processEvents();
    }
}
