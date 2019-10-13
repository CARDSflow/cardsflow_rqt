#pragma once

#ifndef Q_MOC_RUN

#include <ros/ros.h>
#include <rqt_gui_cpp/plugin.h>
#include <cardsflow_rqt/ui_cardsflow_rqt.h>
#include <roboy_middleware_msgs/ControlMode.h>
#include <roboy_middleware_msgs/MotorCommand.h>
#include <roboy_middleware_msgs/MotorConfigService.h>
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
#include <sensor_msgs/JointState.h>

#endif

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
private:
    Ui::CardsflowRqt ui;
    QWidget *widget_;
    ros::NodeHandlePtr nh;
    vector<ros::Publisher> jointCommand;
    boost::shared_ptr<ros::AsyncSpinner> spinner;
    bool vr_puppet = false;
private:
    bool stopButton;
    vector<double> setpoint;
    vector<int> control_mode;
    vector<string> endeffectors;
    vector<string> joint_names;
    vector<QSlider*> setpoint_slider_widget;
    vector<QLineEdit*> setpoint_widget;
};
