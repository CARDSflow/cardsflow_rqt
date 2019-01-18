#include <std_msgs/Float32.h>
#include <QtWidgets/QCheckBox>
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

    for (uint motor = 0; motor < NUMBER_OF_MOTORS; motor++) {
        ui.current_plot->addGraph();
        QPen pen(color_pallette[motor]);
        ui.current_plot->graph(motor)->setPen(pen);

        // TODO rename to force plot
        ui.target_plot->addGraph();
        ui.target_plot->graph(motor)->setPen(pen);

        char str[20];
        sprintf(str,"motor_%d",motor);
        QCheckBox *box = widget_->findChild<QCheckBox*>(str);
        QObject::connect(box, SIGNAL(stateChanged(int)), this, SLOT(plotMotorChanged()));
        plotMotor[motor] = true;
    }

    for (uint motor = 0; motor < NUMBER_OF_MOTORS; motor++) {
        QPen pen(color_pallette[motor]);
        pen.setStyle(Qt::DotLine);
        ui.current_plot->addGraph();
        ui.current_plot->graph(NUMBER_OF_MOTORS + motor)->setPen(pen);
    }

    ui.target_plot->xAxis->setLabel("time[s]");
    ui.target_plot->yAxis->setLabel("ticks");
    ui.target_plot->replot();

    ui.current_plot->xAxis->setLabel("time[s]");
    ui.current_plot->yAxis->setLabel("ticks");
    ui.current_plot->replot();

    nh = ros::NodeHandlePtr(new ros::NodeHandle);
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "cardsflow_rqt_plugin");
    }

    cardsflowStatus = nh->subscribe("/cardsflow/status", 1, &CardsflowRqt::CardslfowStatusCallback, this);
    tendonStates = nh->subscribe("/tendon_states", 1, &CardsflowRqt::TendonStatesCallback, this);
    QObject::connect(this, SIGNAL(newData()), this, SLOT(plotData()));
    QObject::connect(ui.toggle_all, SIGNAL(clicked()), this, SLOT(toggleAll()));

    spinner.reset(new ros::AsyncSpinner(2));
    spinner->start();

    nh->getParam("/endeffectors", endeffectors);

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
            line->setFixedSize(50,30);
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

            ros::Publisher command = nh->advertise<std_msgs::Float32>((joint_name+"/"+joint_name+"/target").c_str(),1);
            jointCommand.push_back(command);

            joint++;
        }
    }

    start_time = ros::Time::now();


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
    for (auto slider:setpoint_widget) {
        msg.data = slider->text().toDouble(&ok)*M_PI/180.0;
        if(ok)
            jointCommand[joint].publish(msg);
        joint++;
    }
}

void CardsflowRqt::setPointChangedSlider(){
    std_msgs::Float32 msg;
    int joint = 0;
    for (auto slider:setpoint_slider_widget) {
        double setpoint = (slider->value()-50.0)/100.0 * 180.0;
        setpoint_widget[joint]->setText(QString::number(setpoint));
        msg.data = setpoint*2*M_PI/180.0;
        jointCommand[joint].publish(msg);
        joint++;
    }
}

void CardsflowRqt::CardslfowStatusCallback(const roboy_simulation_msgs::CardsflowStatus::ConstPtr &msg) {
    lock_guard<mutex> lock(mux);
//    ROS_INFO_STREAM_THROTTLE(5, "receiving CARDSflow status");
    ros::Duration delta = (ros::Time::now() - start_time);
    time.push_back(delta.toSec());
    for (uint motor=0; motor<msg->current.size(); motor++) {
        if (plotMotor[motor]) {
            motorData[motor][0].push_back(msg->current[motor]);
            motorData[motor][1].push_back(msg->target[motor]);
            if (motorData[motor][0].size() > samples_per_plot) {
                motorData[motor][0].pop_front();
                motorData[motor][1].pop_front();
            }

        } else {
            motorData[motor][0].push_back(std::numeric_limits<double>::quiet_NaN());
            motorData[motor][1].push_back(std::numeric_limits<double>::quiet_NaN());
            if (motorData[motor][0].size() > samples_per_plot) {
                motorData[motor][0].pop_front();
                motorData[motor][1].pop_front();
            }
        }
    }

    if (time.size() > samples_per_plot)
        time.pop_front();

    if ((counter++) % 20 == 0) {
        Q_EMIT newData();
    }

    if (counter % 100 == 0) {
        rescale();
    }
}

void CardsflowRqt::TendonStatesCallback(const roboy_simulation_msgs::Tendon::ConstPtr &msg) {
    lock_guard<mutex> lock(mux);
    ros::Duration delta = (ros::Time::now() - start_time);
    time2.push_back(delta.toSec());
    for (uint motor=0; motor<msg->force.size(); motor++) {
        if (plotMotor[motor]) {
            motorData[motor][2].push_back(msg->force[motor]);
            if (motorData[motor][2].size() > samples_per_plot) {
                motorData[motor][2].pop_front();
            }

        } else {
            motorData[motor][2].push_back(std::numeric_limits<double>::quiet_NaN());
            if (motorData[motor][2].size() > samples_per_plot) {
                motorData[motor][2].pop_front();
            }
        }
    }
//
    if (time2.size() > samples_per_plot/10)
        time2.pop_front();

    if (counter2++ % 10 == 0) {
        Q_EMIT newData();
    }

    if (counter2 % 50 == 0) {
        rescale();
    }
}

void CardsflowRqt::plotData() {
    for (uint motor = 0; motor < NUMBER_OF_MOTORS; motor++) {
        ui.current_plot->graph(motor)->setData(time, motorData[motor][0]);
        ui.current_plot->graph(NUMBER_OF_MOTORS + motor)->setData(time, motorData[motor][1]);
        ui.target_plot->graph(motor)->setData(time2, motorData[motor][2]);
    }

    ui.current_plot->xAxis->rescale();
    ui.target_plot->xAxis->rescale();

    ui.current_plot->replot();
    ui.target_plot->replot();
}

void CardsflowRqt::rescale(){
    double minima[NUMBER_OF_MOTORS][3], maxima[NUMBER_OF_MOTORS][3];
    uint minimal_motor[3] = {0,0,0}, maximal_motor[3] = {0,0,0};
    for(uint type=0;type<3;type++) {
        for (uint motor = 0; motor < NUMBER_OF_MOTORS; motor++) {
            minima[motor][type] = 0;
            maxima[motor][type] = 0;
            for (auto val:motorData[motor][type]) {
                if (val < minima[motor][type])
                    minima[motor][type] = val;
                if (val > maxima[motor][type])
                    maxima[motor][type] = val;
            }
        }

        for (uint motor = 0; motor < NUMBER_OF_MOTORS; motor++) {
            if (minima[motor][type] <= minima[minimal_motor[type]][type] && plotMotor[motor])
                minimal_motor[type] = motor;
            if (maxima[motor][type] <= maxima[maximal_motor[type]][type] && plotMotor[motor])
                maximal_motor[type] = motor;
        }
    }

    for (uint motor = 0; motor < NUMBER_OF_MOTORS; motor++) {
        if (minimal_motor[0] == motor||maximal_motor[0] == motor || minimal_motor[1] == motor||maximal_motor[1] == motor) {
            ui.current_plot->graph(motor)->rescaleAxes();
            ui.current_plot->graph(NUMBER_OF_MOTORS + motor)->rescaleAxes();
        }
//        if (minimal_motor[1] == motor||maximal_motor[1] == motor)
//            ui.current_plot->graph(NUMBER_OF_MOTORS + motor)->rescaleAxes();
        if (minimal_motor[2] == motor||maximal_motor[2] == motor)
            ui.target_plot->graph(motor)->rescaleAxes();
    }

    for (uint motor = 0; motor < NUMBER_OF_MOTORS; motor++) {
        if (minimal_motor[0] != motor||maximal_motor[0] != motor || minimal_motor[1] != motor||maximal_motor[1] != motor) {
            ui.current_plot->graph(motor)->rescaleAxes(true);
            ui.current_plot->graph(NUMBER_OF_MOTORS + motor)->rescaleAxes(true);
        }
//        if (minimal_motor[1] != motor||maximal_motor[1] != motor)
//            ui.current_plot->graph(NUMBER_OF_MOTORS + motor)->rescaleAxes(true);

        if (minimal_motor[2] != motor||maximal_motor[2] != motor)
            ui.target_plot->graph(motor)->rescaleAxes(true);
    }
}

void CardsflowRqt::plotMotorChanged(){
    for(int i=0;i<NUMBER_OF_MOTORS;i++) {
        char str[20];
        sprintf(str,"motor_%d",i);
        QCheckBox *box = widget_->findChild<QCheckBox*>(str);
        if(box!=nullptr)
            plotMotor[i] = box->isChecked();
    }
}

void CardsflowRqt::toggleAll(){
    for(int i=0;i<NUMBER_OF_MOTORS;i++) {
        char str[20];
        sprintf(str,"motor_%d",i);
        QCheckBox *box = widget_->findChild<QCheckBox*>(str);
        if(box!=nullptr) {
            plotMotor[i] = !box->isChecked();
            box->setChecked(plotMotor[i]);
        }
    }
}


PLUGINLIB_DECLARE_CLASS(cardsflow_rqt, CardsflowRqt, CardsflowRqt, rqt_gui_cpp::Plugin)
