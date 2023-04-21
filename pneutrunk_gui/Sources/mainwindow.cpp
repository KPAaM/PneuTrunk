#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QApplication * app, rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    app_(app), rviz_ros_node_(rviz_ros_node)
{
    ui->setupUi(this);


    // RViz
    // _render_panel = new rviz_common::RenderPanel;  
    // _manager = _render_panel->getManager();                          
    // _manager = new rviz_common::VisualizationManager(_render_panel);                         
    // _render_panel->initialize(_manager->getSceneManager(),_manager);
    // ui->Page_Visualize->AddWidget(_render_panel);

    // Menu buttons
    connect(ui->Button_Page_1, &QPushButton::clicked, this, [this] { ui->Pages_Widget->setCurrentWidget(ui->Page_Main); });
    connect(ui->Button_Page_2, &QPushButton::clicked, this, [this] { ui->Pages_Widget->setCurrentWidget(ui->Page_Manual); });
    connect(ui->Button_Page_3, &QPushButton::clicked, this, [this] { ui->Pages_Widget->setCurrentWidget(ui->Page_Visualize); });

}

MainWindow::~MainWindow()
{
    delete ui;
}

// void MainWindow::on_Button_Page_1_clicked()
// {
//     ui->Pages_Widget->setCurrentWidget(ui->Page_Main);

// //    ui->Button_Page_1->setStyleSheet(ui->Button_Page_1->styleSheet()+"QPushButton{border-right: 5px solid rgb(44, 49, 60);}");
// //    ui->Button_Page_2->setStyleSheet(ui->Button_Page_2->styleSheet()+"QPushButton{border-right: none");
// //    ui->Button_Page_3->setStyleSheet(ui->Button_Page_3->styleSheet()+"QPushButton{border-right: none");
// }