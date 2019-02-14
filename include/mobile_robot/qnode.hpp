/**
 * @file /include/robomap/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef robomap_QNODE_HPP_
#define robomap_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include <QString>

#include "sensor_msgs/LaserScan.h"

#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "kobuki_msgs/DockInfraRed.h"
#include "actionlib_msgs/GoalStatusArray.h"

#include <iostream>
using namespace std;

struct CTopicPacket
{
	double m_ScanTime;

	double m_OdomPx;
	double m_OdomPy;
	double m_OdomTheta;
	double m_OdomRz;
	double m_OdomRw;

	double m_AmclPx;
	double m_AmclPy;
	double m_AmclRz;
	double m_AmclRw;

	double m_ImuRx;
	double m_ImuRy;
	double m_ImuRz;
	double m_ImuRw;
	double m_ImuYaw;

	double m_IrLeft;
	double m_IrCenter;
	double m_IrRight;

	bool m_GoalReached;
};

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace MobileRobot
{

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread
{
	Q_OBJECT
public:
	QNode(int argc, char **argv);
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();

	/*********************
	** Logging
	**********************/
	enum LogLevel
	{
		Debug,
		Info,
		Warn,
		Error,
		Fatal
	};

	QStringListModel *loggingModel() { return &logging_model; }
	void log(const LogLevel &level, const std::string &msg);

	void ScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan);
	void PoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &amcl_pose);

	void OdomCallback(const nav_msgs::Odometry::ConstPtr &Odom);
	void KobukiMove(double vx, double vy, double vz, double wx, double wy, double wz);
	void ImuCallback(const sensor_msgs::Imu::ConstPtr &imu);
	void DockCallback(const kobuki_msgs::DockInfraRed::ConstPtr &dock);
	void GoalCallback(const actionlib_msgs::GoalStatusArray::ConstPtr &goal);

public:
	CTopicPacket *GetTopicPacket();

	//ros::NodeHandle GetNodeHandle();

Q_SIGNALS:
	void loggingUpdated();
	void rosShutdown();

private:
	int init_argc;

	char **init_argv;

	ros::Publisher chatter_publisher;

	ros::Subscriber _SubScanTopic;
	ros::Subscriber _SubAmclPoseTopic;
	ros::Subscriber _SubOdom;

	QStringListModel logging_model;

	ros::Publisher cmd_vel_publisher;
	ros::Subscriber imu_subscriber;
	ros::Subscriber dock_subscriber;
	ros::Subscriber goal_subscriber;

private:
	QString _SystemMsg;

public:
	CTopicPacket m_TopicPacket;
};

} // namespace MobileRobot

#endif /* robomap_QNODE_HPP_ */
