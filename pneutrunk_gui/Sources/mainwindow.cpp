#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <memory>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
  
    _node = std::make_shared<rclcpp::Node>("pneutrunk_gui");
    _publisher = _node->create_publisher<std_msgs::msg::String>("/gui_control", 10);
    _state_subscriber = _node->create_subscription<pneutrunk_msgs::msg::PneutrunkJointState>("/continuum_robot/state", 
                        1, std::bind(&MainWindow::JointStateCallback, this, _1));

    // Menu buttons
    connect(ui->Button_Page_1, &QPushButton::clicked, this, [this] { ui->Pages_Widget->setCurrentWidget(ui->Page_Main); });
    connect(ui->Button_Page_2, &QPushButton::clicked, this, [this] { ui->Pages_Widget->setCurrentWidget(ui->Page_Manual_Joints); });
    connect(ui->Button_Page_3, &QPushButton::clicked, this, [this] { ui->Pages_Widget->setCurrentWidget(ui->Page_Manual_EEF); });

    // connect(ui->rvizbutton,  &QPushButton::clicked, this, &MainWindow::on_Button);

}

MainWindow::~MainWindow()
{
    delete ui;
    rclcpp::shutdown();
}


void 
MainWindow::on_Button()
{
    std_msgs::msg::String msg;
    msg.data = "asda";
    _publisher->publish(msg);
}

void
MainWindow::JointStateCallback(const pneutrunk_msgs::msg::PneutrunkJointState &msg)
{
    std::cout << "received\n";
    // Copy rotation joints
    for (uint i=0; i< _NUM_DOF-1; i+=2)
    {
        _joints_actual[i].first = msg.segment_state[i];
        _joints_actual[i].second = msg.segment_state[i+1];
    }
    _joints_actual[_NUM_DOF-1].first = msg.translation;

    // Compute forwards kinematics

    // --- Visualize ---
    // Segment 1
    ui->Label_Seg1_ActRoll->setText(QString::number(_joints_actual[0].first));
    ui->Label_Seg1_ActPitch->setText(QString::number(_joints_actual[0].second));
    // Segment 2
    ui->Label_Seg2_ActRoll->setText(QString::number(_joints_actual[1].first));
    ui->Label_Seg2_ActPitch->setText(QString::number(_joints_actual[1].second));
    // Segment 3
    ui->Label_Seg3_ActRoll->setText(QString::number(_joints_actual[2].first));
    ui->Label_Seg3_ActPitch->setText(QString::number(_joints_actual[2].second));
    // Segment 4
    ui->Label_Seg4_ActRoll->setText(QString::number(_joints_actual[3].first));
    ui->Label_Seg4_ActPitch->setText(QString::number(_joints_actual[3].second));
    // Segment 5
    ui->Label_Seg5_ActRoll->setText(QString::number(_joints_actual[4].first));
    ui->Label_Seg5_ActPitch->setText(QString::number(_joints_actual[4].second));
    // Segment 6
    ui->Label_Seg6_ActRoll->setText(QString::number(_joints_actual[5].first));
    ui->Label_Seg6_ActPitch->setText(QString::number(_joints_actual[5].second));
    // Segment 7
    ui->Label_Seg6_ActRoll->setText(QString::number(_joints_actual[6].first));
    //ui->Label_x_actual->setText(QString::number())
}
