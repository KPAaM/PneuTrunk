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
        /* data */

    protected:
        QGridLayout *grid;
        QGroupBox *box;
        QFormLayout *layoutWidget;
        QTabWidget *tabWidget;
        QVBoxLayout *mainLayout = new QVBoxLayout;

    public Q_SLOTS:

    Q_SIGNALS:

};



}