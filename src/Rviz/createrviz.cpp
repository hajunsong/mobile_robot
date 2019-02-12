#include "createrviz.h"

#include "include/mobile_robot/main_window.hpp"
#include "ui_main_window.h"

#include <qstandarditemmodel.h>


CreateRviz::CreateRviz()
{

}

void CreateRviz::Rviz_Init(void *_ui)
{
    Ui::MainWindowDesign* ui = (Ui::MainWindowDesign*)_ui;

    m_RvizRenderPanel=new rviz::RenderPanel();

    ui->vlRvizPanel->addWidget(m_RvizRenderPanel);

    m_RvizManager = new rviz::VisualizationManager(m_RvizRenderPanel);

    m_RvizRenderPanel->initialize(m_RvizManager->getSceneManager(),m_RvizManager);
    m_RvizManager->initialize();
    m_RvizManager->startUpdate();

    //m_RvizInitialPose->onInitialize();

    SetTopice();
}

void CreateRviz::SetTopice()
{
    //Set Fixed Frame
    m_RvizManager->setFixedFrame("map");

    //Set Grount Grid
    m_RvizGrid = m_RvizManager->createDisplay( "rviz/Grid", "adjustable grid", true );
    m_RvizGrid->subProp( "Line Style" )->setValue( "Billboards" );


    //Set LaserScanner Topic
    m_RvizScan = m_RvizManager->createDisplay( "rviz/LaserScan", "adjustable scan", true );
    m_RvizScan->subProp("Topic")->setValue("/scan");
    m_RvizScan->subProp("Size (m)")->setValue("0.01");

    m_RvizRobotModel = m_RvizManager->createDisplay("rviz/RobotModel", "adjustable robotmodel", true);
    m_RvizRobotModel->subProp("Robot Description")->setValue("robot_description");

    m_RvizMap = m_RvizManager->createDisplay("rviz/Map","adjustable map",true);
    m_RvizMap ->subProp("Topic")->setValue("/map");

    m_RvizAmclParticles = m_RvizManager->createDisplay("rviz/PoseArray", "adjustable particlecloud", true);
    m_RvizAmclParticles->subProp("Topic")->setValue("/particlecloud");
}

void CreateRviz::SetInitialPose(double Px, double Py, double Th)
/*
 *position:
      x: -0.676681313421
      y: -0.234239122796
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.274648161662
      w: 0.961544792142


 pose:
    position:
      x: 0.204144161725
      y: -0.0591819499943
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.21720005391
      w: 0.976127110873

 pose:
    position:
      x: 1.00314873185
      y: -0.07236798109
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: -0.120681040127
      w: 0.992691334985


pose:
    position:
      x: 1.93100414919
      y: -0.811078638227
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: -0.373427062257
      w: 0.927659543784

*/
{
    _PubInitialPose = _n.advertise<geometry_msgs::PoseWithCovarianceStamped>
            ("/initialpose", 10);

    std::string fixed_frame = m_RvizManager->getFixedFrame().toStdString();

    geometry_msgs::PoseWithCovarianceStamped pose;

    pose.header.frame_id = fixed_frame;
    pose.header.stamp = ros::Time::now();
    pose.pose.pose.position.x = Px;
    pose.pose.pose.position.y = Py;

    tf::Quaternion quat;

    quat.setRPY(0.0, 0.0, Th);

    tf::quaternionTFToMsg(quat, pose.pose.pose.orientation);

    pose.pose.covariance[6*0+0] = 0.5 * 0.5;
    pose.pose.covariance[6*1+1] = 0.5 * 0.5;
    pose.pose.covariance[6*5+5] = M_PI/12.0 * M_PI/12.0;

    ROS_INFO("Setting Pose: %.3f %.3f %.3f [frame=%s]", Px , Py, Th, fixed_frame.c_str());

    _PubInitialPose.publish(pose);

    //void* f1 = reinterpret_cast<void*>(m_RvizInitialPose->onPoseSet(Px, Py, Th));

    //m_RvizInitialPose->onPoseSet(Px, Py, Th);
}

void CreateRviz::SetGoalPose(double Px, double Py, double Th)
{
    _PubGoalPose = _n.advertise<geometry_msgs::PoseStamped>
            ("/move_base_simple/goal", 10);

    std::string fixed_frame = m_RvizManager->getFixedFrame().toStdString();

    tf::Quaternion quat;

    quat.setRPY(0.0, 0.0, Th);

    tf::Stamped<tf::Pose> p = tf::Stamped<tf::Pose>(tf::Pose(quat, tf::Point(Px, Py, 0.0)), ros::Time::now(), fixed_frame);

    geometry_msgs::PoseStamped goal;

    tf::poseStampedTFToMsg(p, goal);

    ROS_INFO("Setting goal: Frame:%s, Position(%.3f, %.3f, %.3f), Orientation(%.3f, %.3f, %.3f, %.3f) = Angle: %.3f\n", fixed_frame.c_str(),
             goal.pose.position.x, goal.pose.position.y, goal.pose.position.z,
             goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w, Th);

    _PubGoalPose.publish(goal);
}

