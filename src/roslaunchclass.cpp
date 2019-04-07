#include "roslaunchclass.h"

RosLaunchClass::RosLaunchClass()
{
    Roslaunch = new QProcess();
}

void RosLaunchClass::LaunchLidar()
{
    //QString Launch = "gnome-terminal roslaunch sick_scan sick_lms_1xx.launch";

    QProcessEnvironment env = QProcessEnvironment::systemEnvironment();

    Roslaunch->setProcessEnvironment(env);
    Roslaunch->start("gnome-terminal", QStringList() << "source ~/catkin_ws/devel/setup.bash; roslaunch sick_scan sick_lms_1xx.launch");
    Roslaunch->waitForStarted(-1);
}
