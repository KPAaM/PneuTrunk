#pragma once

#include <rviz_common/panel.hpp>
#include <memory>
#include <vector>
#include <utility>
#include <rviz_common/display_context.hpp>

#include <QLabel>
#include <QGridLayout>
#include <QLineEdit>
#include <QString>
#include <QGroupBox>
#include <QFormLayout>
#include <QTabWidget>
#include <QSlider>
#include <QFile>



namespace pneutrunk_rviz_plugin
{

class ManualTab : public QWidget
{
    Q_OBJECT
    public:
        explicit ManualTab(QWidget *parent = nullptr);
    private:
        QLineEdit *epoch_value;
};



class PneutrunkPlugin : public rviz_common::Panel
{
    Q_OBJECT
    public:
        PneutrunkPlugin( QWidget* parent = 0 );
        // void onInitialize() override;
        // void save(rviz_common::Config config) const override;
        // void load(const rviz_common::Config &conf) override;
    private:
        void InitManualTab();
        void InitAutomatTab();

    protected:
        QFile file;
        QString styleSheetSliders;

        QGridLayout *grid;
        QGroupBox *box;
        QFormLayout *layoutWidget;
        QTabWidget *tabWidget;
        QVBoxLayout *mainLayout = new QVBoxLayout;

        QWidget *manual_tab_widget;
        QWidget *automat_tab_widget;

        // manual tab
        QGridLayout *manual_tab_layout;
        std::vector<QLabel *> segments_labels;
        std::vector<std::pair<QSlider *, QSlider *>> segments_joints_cmd_sliders;

        // automat tab
        QGridLayout *automat_tab_layout;





    public Q_SLOTS:

    Q_SIGNALS:

};



}