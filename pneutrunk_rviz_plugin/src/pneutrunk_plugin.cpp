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
    : rviz_common::Panel( parent )
{
    grid = new QGridLayout(this);
    this->setLayout(grid);
    // box = new QGroupBox("Joints" ,this);
    // grid->addWidget(box, 0, 0);

    tabWidget = new QTabWidget(this);
    tabWidget->addTab(new ManualTab(), QString("Manual"));
    tabWidget->addTab(new QWidget(), QString("Automat"));
    grid->addWidget(tabWidget,0,0);

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