#include "pneutrunk_rviz_plugin/pneutrunk_plugin.h"

namespace pneutrunk_rviz_plugin
{

ManualTab::ManualTab(QWidget *parent)
    : QWidget(parent)
{
    QLabel *fileNameLabel = new QLabel("Epoch");
    epoch_value = new QLineEdit();

    QFormLayout *mainLayout = new QFormLayout;
    mainLayout->addRow(fileNameLabel, epoch_value);

    setLayout(mainLayout);
}

PneutrunkPlugin::PneutrunkPlugin(QWidget* parent)
    : rviz_common::Panel( parent ),
     file("/home/km/ros2_ws/src/PneuTrunk/pneutrunk_rviz_plugin/stylesheet.qss")
{
    file.open(QFile::ReadOnly);
    styleSheetSliders = QLatin1String(file.readAll());

    grid = new QGridLayout(this);
    this->setLayout(grid);
    // box = new QGroupBox("Joints" ,this);
    // grid->addWidget(box, 0, 0);




    // Create tabs
    tabWidget = new QTabWidget(this);
    manual_tab_widget = new QWidget;
    automat_tab_widget = new QWidget;
    InitManualTab();
    InitAutomatTab();
    tabWidget->addTab(manual_tab_widget, "Manual");
    tabWidget->addTab(automat_tab_widget, "Automat");

    // auto manual_tab_layout = new QGridLayout;
    // auto txt = new QLabel("Epoch");
    // manual_tab_layout->addWidget(txt, 0, 0);
    // manual_tab_widget->setLayout(manual_tab_layout);

    grid->addWidget(tabWidget,0,0);
}


void
PneutrunkPlugin::InitManualTab()
{
    manual_tab_layout = new QGridLayout;

    // Create labels for segments
    segments_labels.resize(7);
    segments_joints_cmd_sliders.resize(7);

    for (uint i=0; i<7; ++i)
    {
        // Labels
        std::string segment_label = "Segment ";
        segment_label += std::to_string(i+1);
        segments_labels[i] = new QLabel(QString::fromStdString(segment_label));
        manual_tab_layout->addWidget(segments_labels[i], 3*i, 0);

        // Sliders
        segments_joints_cmd_sliders[i].first = new QSlider;
        segments_joints_cmd_sliders[i].second = new QSlider;
        // Roll slider
        auto roll_label = new QLabel("Roll");
        segments_joints_cmd_sliders[i].first->setOrientation(Qt::Horizontal);
        segments_joints_cmd_sliders[i].first->setMinimum(-25);
        segments_joints_cmd_sliders[i].first->setMaximum(25);
        segments_joints_cmd_sliders[i].first->setSingleStep(1);
        segments_joints_cmd_sliders[i].first->setStyleSheet(styleSheetSliders);
        manual_tab_layout->addWidget(roll_label, 1+3*i, 0);
        manual_tab_layout->addWidget(segments_joints_cmd_sliders[i].first, 1+3*i, 1);

        // Pitch slider
        auto pitch_label = new QLabel("Pitch");
        segments_joints_cmd_sliders[i].second->setOrientation(Qt::Horizontal);
        segments_joints_cmd_sliders[i].second->setMinimum(-25);
        segments_joints_cmd_sliders[i].second->setMaximum(25);
        segments_joints_cmd_sliders[i].first->setSingleStep(1);
        manual_tab_layout->addWidget(pitch_label, 1+3*i, 2);
        manual_tab_layout->addWidget(segments_joints_cmd_sliders[i].second, 1+3*i, 3);

    }
   



    manual_tab_widget->setLayout(manual_tab_layout);
}

void
PneutrunkPlugin::InitAutomatTab()
{
    automat_tab_layout = new QGridLayout;
}

// void 
// PneutrunkPlugin::save(rviz_common::Config config) const
// {
//     Panel::save(config);
// }

// void 
// PneutrunkPlugin::load(const rviz_common::Config &conf)
// {
//     Panel::load(conf);
// }

// void 
// PneutrunkPlugin::onInitialize()
// {
//     auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
// }


}//namespace

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(pneutrunk_rviz_plugin::PneutrunkPlugin,rviz_common::Panel)