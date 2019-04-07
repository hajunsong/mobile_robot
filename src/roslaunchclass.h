#ifndef ROSLAUNCHCLASS_H
#define ROSLAUNCHCLASS_H

#include <QStringList>
#include <QProcess>

class RosLaunchClass
{
public:
    RosLaunchClass();

    void        LaunchLidar();

private:
    QProcess    *Roslaunch;
};

#endif // ROSLAUNCHCLASS_H
