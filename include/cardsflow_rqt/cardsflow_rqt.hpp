#pragma once

#ifndef Q_MOC_RUN

#include <ros/ros.h>
#include <rqt_gui_cpp/plugin.h>
#include <cardsflow_rqt/ui_cardsflow_rqt.h>
#include <roboy_middleware_msgs/ControlMode.h>
#include <roboy_middleware_msgs/MotorCommand.h>
#include <roboy_middleware_msgs/MotorConfigService.h>
#include <roboy_simulation_msgs/CardsflowStatus.h>
#include <QWidget>
#include <QtQuick/QQuickView>
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <QSlider>
#include <QLineEdit>
#include <QScrollArea>
#include <QRadioButton>
#include <QVBoxLayout>
#include <QLabel>
#include <map>
#include <common_utilities/CommonDefinitions.h>
#include <common_utilities/MotorConfig.hpp>
#include <std_srvs/SetBool.h>

#endif

//TODO check in common utilities
#define NUMBER_OF_MOTORS 8

using namespace std;

class CardsflowRqt
        : public rqt_gui_cpp::Plugin, MotorConfig {
    Q_OBJECT
public:
    CardsflowRqt();

    virtual void initPlugin(qt_gui_cpp::PluginContext &context);

    virtual void shutdownPlugin();

    virtual void saveSettings(qt_gui_cpp::Settings &plugin_settings,
                              qt_gui_cpp::Settings &instance_settings) const;

    virtual void restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                                 const qt_gui_cpp::Settings &instance_settings);
public Q_SLOTS:
    void setPointChanged();
    void setPointChangedSlider();

    void plotData();
    void rescale();
    void plotMotorChanged();
    void toggleAll();
    void fpgaChanged(int fpga);

    Q_SIGNALS:
    void newData();

private:
    int counter = 0;
    bool plotMotor[NUMBER_OF_MOTORS];
    Ui::CardsflowRqt ui;
    QWidget *widget_;
    ros::NodeHandlePtr nh;
    vector<ros::Publisher> jointCommand;
    boost::shared_ptr<ros::AsyncSpinner> spinner;
    void CardslfowStatusCallback(const roboy_simulation_msgs::CardsflowStatus::ConstPtr &msg);

    bool stopButton;
    vector<double> setpoint;
    vector<int> control_mode;
    vector<string> endeffectors;
    vector<string> joint_names;
    vector<QSlider*> setpoint_slider_widget;
    vector<QLineEdit*> setpoint_widget;

    QVector<double> time;
    int samples_per_plot = 300;
    QVector<double> motorData[NUMBER_OF_MOTORS][2];
    QColor color_pallette[16] = {Qt::blue, Qt::red, Qt::green, Qt::cyan, Qt::magenta, Qt::darkGray, Qt::darkRed, Qt::darkGreen,
                                 Qt::darkBlue, Qt::darkCyan, Qt::darkMagenta, Qt::darkYellow, Qt::black, Qt::gray, Qt::green, Qt::cyan};
    ros::Subscriber cardsflowStatus;
    ros::Time start_time;
};
