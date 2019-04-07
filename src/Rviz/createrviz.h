#ifndef CREATERVIZ_H
#define CREATERVIZ_H

//#include "include/MobileRobot/qnode.hpp"

#include <QDebug>

#include <ros/ros.h>


#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>

#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>
#include <rviz/tool_manager.h>
#include <rviz/default_plugin/tools/initial_pose_tool.h>
#include <rviz/default_plugin/tools/goal_tool.h>


class CreateRviz
{
public:
    CreateRviz();

    void    Rviz_Init(void *_ui);
    void    SetInitialPose(double Px, double Py, double Th);
    void    SetGoalPose(double Px, double Py, double Th);


private:
    rviz::VisualizationManager  *m_RvizManager;
    rviz::RenderPanel           *m_RvizRenderPanel;

    ros::NodeHandle             _n;
    ros::Publisher              _PubInitialPose;
    ros::Publisher              _PubGoalPose;

    rviz::Tool                  *m_RvizTool;

    rviz::StringProperty        *m_RvizStringProperty;

    rviz::Display               *m_RvizGrid;
    rviz::Display               *m_RvizScan;
    rviz::Display               *m_RvizSetFixedFrame;
    rviz::Display               *m_RvizRobotModel;
    rviz::Display               *m_RvizMap;
    rviz::Display               *m_RvizAmclParticles;

    rviz::InitialPoseTool       *m_RvizInitialPose;
    rviz::GoalTool              *m_RvizGoalPose;
    

    //QNode                       *_RosInfo;

private:
    void    SetTopice();
};

#endif // CREATERVIZ_H
