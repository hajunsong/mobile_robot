/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/mobile_robot/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace MobileRobot {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
    init_argc(argc), init_argv(argv)
{
    _SystemMsg.clear();

    ros::init(init_argc,init_argv,"mobile_robot");
}

QNode::~QNode() {
    if(ros::isStarted()) {
        ros::shutdown(); // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }
    wait();
}

bool QNode::init()
{
    ros::init(init_argc,init_argv,"mobile_robot");

    if ( ! ros::master::check() )
    {
        return false;
    }
    else
    {
        ros::start();

        ros::NodeHandle n;

        start();
    }

    return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url)
{
    std::map<std::string,std::string> remappings;

    remappings["__master"] = master_url;
    remappings["__hostname"] = host_url;

    ros::init(remappings,"mobile_robot");

    ros::NodeHandle n;


    if ( ! ros::master::check() )
    {
        return false;
    }
    else
    {
        ros::start();

        _SubScanTopic = n.subscribe("scan", 1000, &QNode::ScanCallback, this);
        _SubOdom = n.subscribe("odom", 1000, &QNode::OdomCallback, this);
        _SubAmclPoseTopic = n.subscribe("amcl_pose", 1000, &QNode::PoseCallback, this);

        cmd_vel_publisher = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);
        imu_subscriber = n.subscribe("mobile_base/sensors/imu_data", 3000, &QNode::ImuCallback, this);
        dock_subscriber = n.subscribe("mobile_base/sensors/dock_ir", 3000, &QNode::DockCallback, this);

        start();
    }

    return true;
}

void QNode::run() {
    ros::Rate loop_rate(1);

    while ( ros::ok() )
    {
        ros::spinOnce();

        loop_rate.sleep();
    }
    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    Q_EMIT rosShutdown();
}


void QNode::log( const LogLevel &level, const std::string &msg) {
    logging_model.insertRows(logging_model.rowCount(),1);
    std::stringstream logging_model_msg;
    switch ( level ) {
    case(Debug) : {
        ROS_DEBUG_STREAM(msg);
        logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
        break;
    }
    case(Info) : {
        ROS_INFO_STREAM(msg);
        logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
        break;
    }
    case(Warn) : {
        ROS_WARN_STREAM(msg);
        logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
        break;
    }
    case(Error) : {
        ROS_ERROR_STREAM(msg);
        logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
        break;
    }
    case(Fatal) : {
        ROS_FATAL_STREAM(msg);
        logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
        break;
    }
    }
    QVariant new_row(QString(logging_model_msg.str().c_str()));
    logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}



void QNode::ScanCallback(const sensor_msgs::LaserScan_<std::allocator<void> >::ConstPtr& scan)
{
    m_TopicPacket.m_ScanTime = scan->scan_time;
}

void QNode::PoseCallback(const geometry_msgs::PoseWithCovarianceStamped_<std::allocator<void> >::ConstPtr &amcl_pose)
{
    m_TopicPacket.m_AmclPx = amcl_pose->pose.pose.position.x;
    m_TopicPacket.m_AmclPy = amcl_pose->pose.pose.position.y;
    //    m_TopicPacket.m_AmclTheta = amcl_pose->pose.pose.orientation.w;
    m_TopicPacket.m_AmclRz = amcl_pose->pose.pose.orientation.z;
    m_TopicPacket.m_AmclRw = amcl_pose->pose.pose.orientation.w;


    // ROS_INFO("position =: [%f]", amcl_pose->pose.pose.position.x);
}

void QNode::OdomCallback(const nav_msgs::Odometry_<std::allocator<void> >::ConstPtr &Odom)
{
    //ROS_INFO("position =: [%f]", Odom->pose.pose.position.x);

    m_TopicPacket.m_OdomPx = Odom->pose.pose.position.x;
    m_TopicPacket.m_OdomPy = Odom->pose.pose.position.y;
    m_TopicPacket.m_OdomTheta = Odom->pose.pose.orientation.z;
    m_TopicPacket.m_OdomRz = Odom->pose.pose.orientation.z;
    m_TopicPacket.m_OdomRw = Odom->pose.pose.orientation.w;
}

void QNode::KobukiMove(VelCmd linear, VelCmd angular) {
    geometry_msgs::Twist cmd;

    cmd.linear.x = linear.x;
    cmd.linear.y = linear.y;
    cmd.linear.z = linear.z;

    cmd.angular.x = angular.x;
    cmd.angular.y = angular.y;
    cmd.angular.z = angular.z;

    //publish the assembled command
    cmd_vel_publisher.publish(cmd);
}

void QNode::ImuCallback(const sensor_msgs::Imu_<std::allocator<void> >::ConstPtr &imu){
    // ROS_INFO("imu_data =: [%f]", imu->orientation.z);
    m_TopicPacket.m_ImuRx = imu->orientation.x;
    m_TopicPacket.m_ImuRy = imu->orientation.y;
    m_TopicPacket.m_ImuRz = imu->orientation.z;
    m_TopicPacket.m_ImuRw = imu->orientation.w;

    m_TopicPacket.m_ImuYaw = atan2(2.0 * (m_TopicPacket.m_ImuRw * m_TopicPacket.m_ImuRz), 1.0 - 2.0 * (m_TopicPacket.m_ImuRz * m_TopicPacket.m_ImuRz));
}

void QNode::DockCallback(const kobuki_msgs::DockInfraRed_<std::allocator<void> >::ConstPtr &dock){
    // ROS_INFO("dock_ir data : R:[%d], C:[%d], L:[%d]", dock->data[0], dock->data[1], dock->data[2]);
    m_TopicPacket.m_IrRight = dock->data[0];
    m_TopicPacket.m_IrCenter = dock->data[1];
    m_TopicPacket.m_IrLeft = dock->data[2];
}

CTopicPacket* QNode::GetTopicPacket()
{
    return &m_TopicPacket;
}

}  // namespace robomap