#include <std_msgs/Float32.h>
#include "cardsflow_rqt/cardsflow_rqt.hpp"

CardsflowRqt::CardsflowRqt()
        : rqt_gui_cpp::Plugin(), widget_(0) {
    setObjectName("CardsflowRqt");
}

void CardsflowRqt::initPlugin(qt_gui_cpp::PluginContext &context) {
    // access standalone command line arguments
    QStringList argv = context.argv();
    // create QWidget
    widget_ = new QWidget();
    // extend the widget with all attributes and children from UI file
    ui.setupUi(widget_);
    // add widget to the user interface
    context.addWidget(widget_);

    QScrollArea* scrollArea = widget_->findChild<QScrollArea *>("joint_command");
    scrollArea->setBackgroundRole(QPalette::Window);
    scrollArea->setFrameShadow(QFrame::Plain);
    scrollArea->setFrameShape(QFrame::NoFrame);
    scrollArea->setWidgetResizable(true);

    //vertical box that contains all the checkboxes for the filters
    QWidget* motor_command_scrollarea = new QWidget(widget_);
    motor_command_scrollarea->setObjectName("joint_command_scrollarea");
    motor_command_scrollarea->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
    motor_command_scrollarea->setLayout(new QVBoxLayout(motor_command_scrollarea));
    scrollArea->setWidget(motor_command_scrollarea);

    nh = ros::NodeHandlePtr(new ros::NodeHandle);
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "cardsflow_rqt_plugin");
    }

    spinner.reset(new ros::AsyncSpinner(2));
    spinner->start();

    nh->getParam("/endeffectors", endeffectors);
    nh->getParam("/vr_puppet", vr_puppet);

    int joint = 0;
    for(string ef:endeffectors) {
        vector<string> names;
        nh->getParam(ef+"/joints", names);
        for (string joint_name:names) {
            joint_names.push_back(joint_name);
            QWidget *widget = new QWidget(motor_command_scrollarea);
            widget->setObjectName(joint_name.c_str());
            widget->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
            widget->setLayout(new QHBoxLayout(widget));

            QLabel *label = new QLabel(widget);
            char str[100];
            sprintf(str, "%s/%s", ef.c_str(), joint_name.c_str());
            label->setFixedSize(200,30);
            label->setText(str);
            widget->layout()->addWidget(label);

            QLineEdit *line = new QLineEdit(widget);
            line->setFixedSize(150,30);
            widget->layout()->addWidget(line);
            setpoint_widget.push_back(line);
            QObject::connect(line, SIGNAL(editingFinished()), this, SLOT(setPointChanged()));
            setpoint_widget.back()->setText(QString::number(0));

            QSlider *slider = new QSlider(Qt::Orientation::Horizontal,widget);
            slider->setFixedSize(150,30);
            slider->setValue(50);
            widget->layout()->addWidget(slider);
            setpoint_slider_widget.push_back(slider);

            QObject::connect(slider, SIGNAL(valueChanged(int)), this, SLOT(setPointChangedSlider()));

            motor_command_scrollarea->layout()->addWidget(widget);
            if(!vr_puppet) {
                ros::Publisher command = nh->advertise<std_msgs::Float32>(
                        (joint_name + "/" + joint_name + "/target").c_str(), 1);
                jointCommand.push_back(command);
            }

            joint++;
        }
    }
    if(vr_puppet){
        ros::Publisher command = nh->advertise<sensor_msgs::JointState>(
                "/joint_targets", 1);
        jointCommand.push_back(command);
    }
}

void CardsflowRqt::shutdownPlugin() {
    // unregister all publishers here
}

void CardsflowRqt::saveSettings(qt_gui_cpp::Settings &plugin_settings,
                                    qt_gui_cpp::Settings &instance_settings) const {
    // instance_settings.setValue(k, v)
}

void CardsflowRqt::restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                                       const qt_gui_cpp::Settings &instance_settings) {
    // v = instance_settings.value(k)
}

void CardsflowRqt::setPointChanged(){
    std_msgs::Float32 msg;
    int joint = 0;
    bool ok;
    sensor_msgs::JointState msg2;
    msg2.name = joint_names;
    for (auto slider:setpoint_widget) {
        msg.data = slider->text().toDouble(&ok)*M_PI/180.0;
        if(ok) {
            if(!vr_puppet)
                jointCommand[joint].publish(msg);
            else{
                msg2.position.push_back(slider->text().toDouble(&ok)*M_PI/180.0);
                msg2.velocity.push_back(0);
                msg2.effort.push_back(0);
            }
        }
        joint++;
    }
    if(vr_puppet){
        jointCommand[0].publish(msg2);
    }
}

void CardsflowRqt::setPointChangedSlider(){
    std_msgs::Float32 msg;
    sensor_msgs::JointState msg2;
    msg2.name = joint_names;
    int joint = 0;
    for (auto slider:setpoint_slider_widget) {
        double setpoint = (slider->value()-50.0)/100.0 * 360.0;
        setpoint_widget[joint]->setText(QString::number(setpoint)+"/"+QString::number(setpoint*M_PI/180.0));
        msg.data = setpoint*M_PI/180.0;
        if(!vr_puppet)
            jointCommand[joint].publish(msg);
        else{
            msg2.position.push_back(setpoint*M_PI/180.0);
            msg2.velocity.push_back(0);
            msg2.effort.push_back(0);
        }
        joint++;
    }
    if(vr_puppet){
        jointCommand[0].publish(msg2);
    }
}

#if ROS_VERSION_MINOR == 14 // ros melodic
    PLUGINLIB_EXPORT_CLASS(CardsflowRqt, rqt_gui_cpp::Plugin)
#else // earlier
    PLUGINLIB_DECLARE_CLASS(cardsflow_rqt, CardsflowRqt, CardsflowRqt, rqt_gui_cpp::Plugin)
#endif