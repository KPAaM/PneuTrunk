#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/display.hpp>
// #include <rviz_common/visualization_manager.hpp>
#include <rviz_common/display_context.hpp>
#include "rviz_common/ros_integration/ros_node_abstraction.hpp"
#include "rviz_common/window_manager_interface.hpp"
namespace Ui {
class MainWindow;
}


class MainWindow : public QMainWindow, public rviz_common::WindowManagerInterface
{
    Q_OBJECT

    public:
        explicit MainWindow(QApplication *app, rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node, QWidget *parent = nullptr);
        ~MainWindow();

    private:
        Ui::MainWindow *ui;

        // RViz
        rviz_common::VisualizationManager* _manager;                         
        rviz_common::RenderPanel* _render_panel;                             
        rviz_common::Display* _grid;      

    private slots:
        // void on_Button_Page_1_clicked();
        // void on_Button_Page_2_clicked();

};

#endif // MAINWINDOW_H