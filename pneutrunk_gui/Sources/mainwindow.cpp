#include "mainwindow.h"
#include "ui_mainwindow.h"


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
  
    _node = std::make_shared<rclcpp::Node>("pneutrunk_gui");
    _joint_cmd_publisher = _node->create_publisher<pneutrunk_msgs::msg::PneutrunkJointState>("/pneutrunk/joints/cmd", 1);

    _T.resize(7);
    _Seg_T.resize(7);

    // Menu buttons
    connect(ui->Button_Page_1, &QPushButton::clicked, this, [this] { ui->Pages_Widget->setCurrentWidget(ui->Page_Main); });
    connect(ui->Button_Page_2, &QPushButton::clicked, this, [this] { ui->Pages_Widget->setCurrentWidget(ui->Page_Manual_Joints); });
    connect(ui->Button_Page_3, &QPushButton::clicked, this, [this] { ui->Pages_Widget->setCurrentWidget(ui->Page_Manual_EEF); });
    connect(ui->Button_Page_4, &QPushButton::clicked, this, [this] { ui->Pages_Widget->setCurrentWidget(ui->Page_Cameras); });

    // Main page buttons
    connect(ui->Button_Manual_Joint, &QPushButton::clicked, this, [this] { ui->Pages_Widget->setCurrentWidget(ui->Page_Manual_Joints); });
    connect(ui->Button_Manual_EEF, &QPushButton::clicked, this, [this] { ui->Pages_Widget->setCurrentWidget(ui->Page_Manual_EEF); });
    connect(ui->Button_Cameras, &QPushButton::clicked, this, [this] { ui->Pages_Widget->setCurrentWidget(ui->Page_Cameras); });

    // connect(ui->rvizbutton,  &QPushButton::clicked, this, &MainWindow::on_Button);
    // Sliders buttons
    connect(ui->Slider_seg1_setRoll, SIGNAL(sliderReleased()), this, SLOT(on_Slider_seg1_setRoll_Released()));
    connect(ui->Slider_seg1_setPitch, &QSlider::sliderReleased, this, &MainWindow::on_Slider_seg1_setPitch_Released);

    connect(ui->Slider_seg2_setRoll, &QSlider::sliderReleased, this, &MainWindow::on_Slider_seg2_setRoll_Released);
    connect(ui->Slider_seg2_setPitch, &QSlider::sliderReleased, this, &MainWindow::on_Slider_seg2_setPitch_Released);

    connect(ui->Slider_seg3_setRoll, &QSlider::sliderReleased, this, &MainWindow::on_Slider_seg3_setRoll_Released);
    connect(ui->Slider_seg3_setPitch, &QSlider::sliderReleased, this, &MainWindow::on_Slider_seg3_setPitch_Released);

    connect(ui->Slider_seg4_setRoll, &QSlider::sliderReleased, this, &MainWindow::on_Slider_seg4_setRoll_Released);
    connect(ui->Slider_seg4_setPitch, &QSlider::sliderReleased, this, &MainWindow::on_Slider_seg4_setPitch_Released);

    connect(ui->Slider_seg5_setRoll, &QSlider::sliderReleased, this, &MainWindow::on_Slider_seg5_setRoll_Released);
    connect(ui->Slider_seg5_setPitch, &QSlider::sliderReleased, this, &MainWindow::on_Slider_seg5_setPitch_Released);

    connect(ui->Slider_seg6_setRoll, &QSlider::sliderReleased, this, &MainWindow::on_Slider_seg6_setRoll_Released);
    connect(ui->Slider_seg6_setPitch, &QSlider::sliderReleased, this, &MainWindow::on_Slider_seg6_setPitch_Released);

    connect(ui->Slider_seg7_setRoll, &QSlider::sliderReleased, this, &MainWindow::on_Slider_seg7_setRoll_Released);


    ROS_thread = new rosModule(this);
    connect(ROS_thread, SIGNAL(rosUpdate()), this, SLOT(JointStateCallback()));
    connect(ROS_thread, SIGNAL(rosGestureUpdate()), this, SLOT(GestureCameraCallback()));
    connect(ROS_thread, SIGNAL(rosObjectDetectionUpdate()), this, SLOT(ObjectDetectionCameraCallback()));
    ROS_thread->start(); // This invokes WorkerThread::run in a new thread
}

MainWindow::~MainWindow()
{
    delete ui;
    rclcpp::shutdown();
}


void 
MainWindow::on_Button()
{
   
}

// const pneutrunk_msgs::msg::PneutrunkJointState &msg
void
MainWindow::JointStateCallback()
{
    //TODO: Compute forwards kinematics
    Eigen::VectorXd q(13);
    q << ROS_thread->joints_actual[0].first, ROS_thread->joints_actual[0].second,
         ROS_thread->joints_actual[1].first, ROS_thread->joints_actual[1].second,
         ROS_thread->joints_actual[2].first, ROS_thread->joints_actual[2].second,
         ROS_thread->joints_actual[3].first, ROS_thread->joints_actual[3].second,
         ROS_thread->joints_actual[4].first, ROS_thread->joints_actual[4].second,
         ROS_thread->joints_actual[5].first, ROS_thread->joints_actual[5].second,
         ROS_thread->joints_actual[6].first;
    ForwardKinematics(q);


    // --- Write actual joints ---
    // Segment 1
    ui->Label_Seg1_ActRoll->setText(QString::number(ROS_thread->joints_actual[0].first));
    ui->Label_Seg1_ActPitch->setText(QString::number(ROS_thread->joints_actual[0].second));
    // Segment 2
    ui->Label_Seg2_ActRoll->setText(QString::number(ROS_thread->joints_actual[1].first));
    ui->Label_Seg2_ActPitch->setText(QString::number(ROS_thread->joints_actual[1].second));
    // Segment 3
    ui->Label_Seg3_ActRoll->setText(QString::number(ROS_thread->joints_actual[2].first));
    ui->Label_Seg3_ActPitch->setText(QString::number(ROS_thread->joints_actual[2].second));
    // Segment 4
    ui->Label_Seg4_ActRoll->setText(QString::number(ROS_thread->joints_actual[3].first));
    ui->Label_Seg4_ActPitch->setText(QString::number(ROS_thread->joints_actual[3].second));
    // Segment 5
    ui->Label_Seg5_ActRoll->setText(QString::number(ROS_thread->joints_actual[4].first));
    ui->Label_Seg5_ActPitch->setText(QString::number(ROS_thread->joints_actual[4].second));
    // Segment 6
    ui->Label_Seg6_ActRoll->setText(QString::number(ROS_thread->joints_actual[5].first));
    ui->Label_Seg6_ActPitch->setText(QString::number(ROS_thread->joints_actual[5].second));
    // Segment 7
    ui->Label_Seg7_ActRoll->setText(QString::number(ROS_thread->joints_actual[6].first));

    // --- Write actual EEF pose ---
    ui->Label_x_actual->setText(QString::number(_Seg_T[6](0,3)));
    ui->Label_y_actual->setText(QString::number(_Seg_T[6](1,3)));
    ui->Label_z_actual->setText(QString::number(_Seg_T[6](2,3)));

}



///////////////////////////////////////////////////////////////////////
/// @brief Computes transformation matrix for one segment
///////////////////////////////////////////////////////////////////////
Eigen::Matrix4d 
MainWindow::SegmentTransform(const double &roll, const double &pitch)
{
    double roll_rad = roll*M_PI/180.0;
    double pitch_rad = pitch*M_PI/180.0;
    // Transform from prev plate to middle (potentiometer level)
    Eigen::MatrixXd T_base_to_poten(4,4);   
    T_base_to_poten << 1, 0, 0, 0,
                       0, 1, 0, 0,
                       0, 0, 1, _r,
                       0, 0, 0, 1;
    // Rotation around roll axis
    Eigen::MatrixXd T_rot_roll(4,4);  
    T_rot_roll << cos(roll_rad), 0, sin(roll_rad), 0,
                  0, 1, 0, 0,
                  -sin(roll_rad), 0, cos(roll_rad), 0,
                  0, 0, 0, 1;
    // Rotation around pitch axis
    Eigen::MatrixXd T_rot_pitch(4,4);
    T_rot_pitch << 1, 0, 0, 0,
               0, cos(pitch_rad), -sin(pitch_rad), 0,
               0, sin(pitch_rad), cos(pitch_rad), 0,
               0, 0, 0, 1;
    // Translation from middle to next plate
    Eigen::MatrixXd T_poten_to_base2(4,4);
    T_poten_to_base2 << 1, 0, 0, 0,
                        0, 1, 0, 0,
                        0, 0, 1, _r,
                        0, 0, 0, 1;
    return T_base_to_poten*T_rot_roll*T_rot_pitch*T_poten_to_base2;
}

///////////////////////////////////////////////////////////////////////
/// @brief Computes Forward kinematics 
/// @param q -> vector of joint angles
///////////////////////////////////////////////////////////////////////
void
MainWindow::ForwardKinematics(const Eigen::VectorXd &q)
{
    _T[0] = SegmentTransform(q(0),q(1));
    _T[1] = SegmentTransform(q(2),q(3));
    _T[2] = SegmentTransform(q(4),q(5));
    _T[3] = SegmentTransform(q(6),q(7));
    _T[4] = SegmentTransform(q(8),q(9));
    _T[5] = SegmentTransform(q(10),q(11));
    _T[6] << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 2*_r+q(12),
            0, 0, 0, 1;
    

    _Seg_T[0] = _T[0];
    _Seg_T[1] = _Seg_T[0]*_T[1];
    _Seg_T[2] = _Seg_T[1]*_T[2];
    _Seg_T[3] = _Seg_T[2]*_T[3];
    _Seg_T[4] = _Seg_T[3]*_T[4];
    _Seg_T[5] = _Seg_T[4]*_T[5];
    _Seg_T[6] = _Seg_T[5]*_T[6];
}    



void MainWindow::GestureCameraCallback()
{
    if (ui->Pages_Widget->currentIndex() == 3)
    {
        QImage qimage = Mat2QImage(ROS_thread->camera_gesture_matrix);
        ui->Label_Camera_gesture->setPixmap(QPixmap::fromImage(qimage));
    }
    
}

void MainWindow::ObjectDetectionCameraCallback()
{
    if (ui->Pages_Widget->currentIndex() == 3)
    {
        QImage qimage = Mat2QImage(ROS_thread->camera_object_detection_matrix);
        ui->Label_Camera_object->setPixmap(QPixmap::fromImage(qimage));
    }
}


QImage Mat2QImage(cv::Mat const& src)
{
     cv::Mat temp; // make the same cv::Mat
     cvtColor(src, temp,CV_BGR2RGB); // cvtColor Makes a copt, that what i need
     QImage dest((const uchar *) temp.data, temp.cols, temp.rows, temp.step, QImage::Format_RGB888);
     dest.bits(); // enforce deep copy, see documentation 
     // of QImage::QImage ( const uchar * data, int width, int height, Format format )
     return dest;
}

void MainWindow::on_Slider_seg1_setRoll_Released()
{
    std::cout << "roll1\n";
}

void MainWindow::on_Slider_seg1_setPitch_Released()
{
    std::cout << "pitch1\n";
}

void MainWindow::on_Slider_seg2_setRoll_Released()
{
    std::cout << "roll2\n";
}

void MainWindow::on_Slider_seg2_setPitch_Released()
{
    std::cout << "pitch2\n";
}

void MainWindow::on_Slider_seg3_setRoll_Released()
{
    std::cout << "roll3\n";
}

void MainWindow::on_Slider_seg3_setPitch_Released()
{
    std::cout << "pitch3\n";
}

void MainWindow::on_Slider_seg4_setRoll_Released()
{
    std::cout << "roll4\n";
}

void MainWindow::on_Slider_seg4_setPitch_Released()
{
    std::cout << "pitch4\n";
}

void MainWindow::on_Slider_seg5_setRoll_Released()
{
    std::cout << "roll5\n";
}

void MainWindow::on_Slider_seg5_setPitch_Released()
{
    std::cout << "pitch5\n";
}

void MainWindow::on_Slider_seg6_setRoll_Released()
{
    std::cout << "roll6\n";
}

void MainWindow::on_Slider_seg6_setPitch_Released()
{
    std::cout << "pitch6\n";
}

void MainWindow::on_Slider_seg7_setRoll_Released()
{
    std::cout << "roll7\n";
}