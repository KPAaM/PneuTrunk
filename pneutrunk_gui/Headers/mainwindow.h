#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QApplication>
#include <QMainWindow>

#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <pneutrunk_msgs/msg/pneutrunk_joint_state.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace Ui {
class MainWindow;
}


class MainWindow : public QMainWindow
{
    Q_OBJECT

    public:
        explicit MainWindow(QWidget *parent = nullptr);
        ~MainWindow();

        // QWidget * getParentWindow() override;

    private:
        Ui::MainWindow *ui;

        // ROS utils
        rclcpp::Node::SharedPtr _node;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _publisher;
        rclcpp::Subscription<pneutrunk_msgs::msg::PneutrunkJointState>::SharedPtr _state_subscriber;

        // Kinematics (yaw, pitch)*6 + 1x Translation
        const uint _NUM_DOF = 7;
        std::vector<std::pair<double, double>> _joints_actual = std::vector<std::pair<double, double>>(_NUM_DOF);
        std::vector<std::pair<double, double>> _joints_cmd = std::vector<std::pair<double, double>>(_NUM_DOF);


        void JointStateCallback(const pneutrunk_msgs::msg::PneutrunkJointState &msg);

    private slots:
        // void on_Button_Page_1_clicked();
        // void on_Button_Page_2_clicked();

    public slots:
        void on_Button();
};

#endif // MAINWINDOW_H