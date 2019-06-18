/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/mobile_robot/main_window.hpp"

double init_pose[4] = {0.031, -0.008, 0.000, 1.000};

const int wp_size1 = 1;
double path1[wp_size1][4] = {
    // -0.433, -0.356, 0.932, -0.364,
    // -0.561, -0.450, 0.921, -0.389
    -0.765, -0.369, 0.973, -0.232};
uint wp_indx1 = 0;

const int wp_size2 = 1;
double path2[wp_size2][4] = {
    // -0.744, -0.608, 0.890, -0.455,
    // -0.945, -0.884, 0.887, -0.462
    -1.240, -0.723, 0.982, -0.190};
uint wp_indx2 = 0;

const uint wp_size3 = 1;
double path3[wp_size3][4] = {
    // -0.515, -0.411, 0.371, 0.929,
    // 0.035, -0.002, 0.151, 0.989,
    // -0.074, -0.005, 0.021, 1.000
    // 0.062, 0.082, -0.022, 1.000,
    -0.304, -0.013, 0.000, 1.000

};
uint wp_indx3 = 0;

const double D2R = M_PI / 180.0;
const double des_ang1 = -160 * D2R;
const double des_ang2 = 90 * D2R;
const double des_ang3 = -160 * D2R;
const double des_ang4 = 30 * D2R;

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace MobileRobot
{

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char **argv, QWidget *parent) : QMainWindow(parent), qnode(argc, argv), ui(new Ui::MainWindowDesign)
{
    ui->setupUi(this);

    m_Rviz = new CreateRviz();
    if (DEBUG_ENABLED)
        qDebug() << "m_Rviz Class";

    m_RosLaunch = new RosLaunchClass();
    if (DEBUG_ENABLED)
        qDebug() << "m_RosLaunch Class";

    m_TopicUpdate = new TopicGuiUpdateClass();
    if (DEBUG_ENABLED)
        qDebug() << "m_TopicUpdate Class";
    modelMain = new QStandardItemModel(5, 5, this);
    m_TopicUpdate->Initialize(ui, modelMain);

    QObject::connect(ui->actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    ReadSettings();

    setWindowIcon(QIcon(":/images/icon.png"));
    ui->tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

    /*********************
    ** Logging
    **********************/
    ui->view_logging->setModel(qnode.loggingModel());
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

    /*********************
    ** Auto Start
    **********************/
    if (ui->checkbox_remember_settings->isChecked())
    {
        on_button_connect_clicked(true);
    }

    timerUIUpdate = new QTimer(this);
    connect(timerUIUpdate, SIGNAL(timeout()), this, SLOT(UiUpdate()));
    timerUIUpdate->start(100);

    connect(ui->btnConnectServer, SIGNAL(clicked()), this, SLOT(btnConnectServerClicked()));

    connect(ui->btnGuest, SIGNAL(clicked()), this, SLOT(btnGuestClicked()));
    connect(ui->btnHome, SIGNAL(clicked()), this, SLOT(btnHomeClicked()));

    timerScenario = new QTimer(this);
    connect(timerScenario, SIGNAL(timeout()), this, SLOT(scenarioUpdate()));

    guestCome = false;
    event_flag = 0;
    connectServer = false;

    m_Client = new TcpClient(this);

    connect(m_Client->socket, SIGNAL(connected()), this, SLOT(onConnectServer()));
    connect(m_Client->socket, SIGNAL(readyRead()), this, SLOT(readMessage()));

    enableDxl = false;
    connect(ui->cbEnableDxl, SIGNAL(stateChanged(int)), this, SLOT(cbEnableDxlStateChanged(int)));
    connect(ui->btnRunDxl, SIGNAL(clicked()), this, SLOT(btnRunDxlClicked()));

    dxlControl = new DxlControl();

    connect(ui->btnDocking, SIGNAL(clicked()), this, SLOT(btnDockingClicked()));

    init_flag = false;

    timerTurning = new QTimer(this);
    connect(timerTurning, SIGNAL(timeout()), this, SLOT(turning()));
    timerTurning->setInterval(50);

    timerDocking = new QTimer(this);
    connect(timerDocking, SIGNAL(timeout()), this, SLOT(docking()));
    timerDocking->setInterval(500);
    countDocking = 0;

    backgroundWidget = new QWidget(this);
    backgroundWidget->setFixedSize(1280, 800);
    backgroundWidget->setGeometry(0, 0, 1280, 800);
    backgroundWidget->hide();

    serviceImage = new QLabel(backgroundWidget);
    serviceImage->setGeometry(backgroundWidget->rect());

    flag = false;

    // AutoDockingROS autoDocking("mobile_robot");
    autoDocking = new AutoDockingROS("mobile_robot");

    ros::NodeHandle n;
    autoDocking->init(n);

    timerTcpWait = new QTimer(this);
    timerTcpWait->setInterval(2*60000);
    connect(timerTcpWait, SIGNAL(timeout()), this, SLOT(tcpWaitTimeout()));
    timerTcpWait->start();

    AutoDriveFlag = false;
    // bumper_subscriber = n.subscribe("mobile_base/events/bumper", 1000, &QNode::BumperCallback, this);

    timerBumper = new QTimer(this);
    timerBumper->setInterval(100);
    connect(timerBumper, SIGNAL(timeout()), this, SLOT(bumperCheck()));
    qnode.m_TopicPacket.m_BumperState = false;

    on_button_connect_clicked(true);
    btnConnectServerClicked();
}

MainWindow::~MainWindow()
{
    timerUIUpdate->stop();
    timerScenario->stop();
    timerTurning->stop();
    delete m_Client;
    if (enableDxl)
        delete dxlControl;
    delete timerUIUpdate;
    delete timerScenario;
    delete timerTurning;
    delete timerDocking;
    delete serviceImage;
    delete backgroundWidget;
    delete autoDocking;
    delete m_Rviz;
    delete m_RosLaunch;
    delete m_TopicUpdate;
    delete ui;
}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::UiUpdate()
{
    switch (ui->tab_manager->currentIndex())
    {
    case 0:
        m_TopicUpdate->UIUpdate(ui, modelMain, &qnode);
        break;
    }
}

void MainWindow::showNoMasterMessage()
{
    QMessageBox msgBox;
    msgBox.setText("Couldn't find the ros master.");
    msgBox.exec();
    close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_connect_clicked(bool check)
{
    if (ui->checkbox_use_environment->isChecked())
    {
        if (!qnode.init())
        {
            showNoMasterMessage();
        }
        else
        {
            ui->button_connect->setEnabled(false);
        }
    }
    else
    {
        if (!qnode.init(ui->line_edit_master->text().toStdString(),
                        ui->line_edit_host->text().toStdString()))
        {
            showNoMasterMessage();
        }
        else
        {
            ui->button_connect->setEnabled(false);
            ui->line_edit_master->setReadOnly(true);
            ui->line_edit_host->setReadOnly(true);
            ui->line_edit_topic->setReadOnly(true);
        }
    }

    m_Rviz->Rviz_Init(ui);
}

void MainWindow::on_checkbox_use_environment_stateChanged(int state)
{
    bool enabled;
    if (state == 0)
    {
        enabled = true;
    }
    else
    {
        enabled = false;
    }
    ui->line_edit_master->setEnabled(enabled);
    ui->line_edit_host->setEnabled(enabled);
    //ui->line_edit_topic->setEnabled(enabled);
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView()
{
    ui->view_logging->scrollToBottom();
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered()
{
    QMessageBox::about(this, tr("About ..."), tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings()
{
    // QSettings settings("Qt-Ros Package", "robomap");
    // restoreGeometry(settings.value("geometry").toByteArray());
    // restoreState(settings.value("windowState").toByteArray());
    // QString master_url = settings.value("master_url", QString("http://192.168.1.2:11311/")).toString();
    // QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    // //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    // ui->line_edit_master->setText(master_url);
    // ui->line_edit_host->setText(host_url);
    // //ui->line_edit_topic->setText(topic_name);
    // bool remember = settings.value("remember_settings", false).toBool();
    // ui->checkbox_remember_settings->setChecked(remember);
    // bool checked = settings.value("use_environment_variables", false).toBool();
    // ui->checkbox_use_environment->setChecked(checked);
    // if (checked)
    // {
    //     ui->line_edit_master->setEnabled(false);
    //     ui->line_edit_host->setEnabled(false);
    //     //ui->line_edit_topic->setEnabled(false);
    // }
}

void MainWindow::WriteSettings()
{
    // QSettings settings("Qt-Ros Package", "robomap");
    // settings.setValue("master_url", ui->line_edit_master->text());
    // settings.setValue("host_url", ui->line_edit_host->text());
    // //settings.setValue("topic_name",ui->line_edit_topic->text());
    // settings.setValue("use_environment_variables", QVariant(ui->checkbox_use_environment->isChecked()));
    // settings.setValue("geometry", saveGeometry());
    // settings.setValue("windowState", saveState());
    // settings.setValue("remember_settings", QVariant(ui->checkbox_remember_settings->isChecked()));
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    WriteSettings();
    QMainWindow::closeEvent(event);
}

void MainWindow::btnConnectServerClicked()
{
    qDebug() << "Try connect server";
    m_Client->setIpAddress(ui->ipAddress->text());
    m_Client->setPortNum(ui->portNum->text());
    emit m_Client->connectToServer();
}

void MainWindow::onConnectServer()
{
    m_Client->socket->write("Server-MobileRobot Connection success");
    ui->btnConnectServer->setDisabled(true);
    ui->ipAddress->setDisabled(true);
    ui->portNum->setDisabled(true);

    // enableDxl = true;
    // dxlControl->dxl_init();
    // ui->cbEnableDxl->setChecked(enableDxl);

    connectServer = true;

    on_btnSetInitialPose_clicked();
}

void MainWindow::readMessage()
{
    timerTcpWait->stop();
    QByteArray rxData = m_Client->socket->readAll();
    if (rxData.length() > 0)
    {
        qDebug() << "Receive Data(Frome Server) : " + rxData;
        char rxHead = 0x02, rxHeadSub = 0x05;
        QList<QByteArray> rxDataList = rxData.split(rxHead);
        int len = rxDataList.length();
        for (int i = 0; i < len; i++)
        {
            QList<QByteArray> rxDataListSub = rxDataList[i].split(rxHeadSub);
            int len_sub = rxDataListSub.length();
            if (len_sub > 1)
            {
                for (int j = 0; j < len_sub; j++)
                {
                    if (rxDataListSub[j].length() > 0)
                    {
                        int data = rxDataListSub[j].at(0);
                        switch (data)
                        {
                        case 0:
                            ROS_INFO("Guest leaved");
                            viewImageWink();
                            btnHomeClicked();
                            break;
                        case 1:
                            if (countDocking > 0)
                            {
                                timerDocking->stop();
                                docking_ac->cancelGoal();
                                countDocking = 0;
                                delete docking_ac;
                                on_btnSetInitialPose_clicked();
                            }
                            btnGuestClicked();
                            break;
                        case 3:
                            viewImageWait();
                            break;
                        case 6:
                            timerTcpWait->start();
                            break;
                        case 7:
                            ROS_INFO("Guest leaved");
                            viewImageSmile();
                            btnHomeClicked();
                            break;
                        case 8:
                            // ROS_INFO("No Guest");
                            // event_flag = 9;
                            // scenarioCnt = 0;
                            // timerScenario->start(50);
                        default:
                            break;
                        }
                    }
                }
            }
        }
    }
}

void MainWindow::on_btnSetInitialPose_clicked()
{
    double yaw = atan2(2.0 * (init_pose[Rw] * init_pose[Rz]), 1.0 - 2.0 * (init_pose[Rz] * init_pose[Rz]));

    m_Rviz->SetInitialPose(init_pose[Px], init_pose[Py], yaw);

    wp_indx1 = 0;
    wp_indx2 = 0;
    wp_indx3 = 0;
    if (init_flag == false)
    {
        m_Rviz->SetGoalPose(init_pose[Px], init_pose[Py], yaw);
        init_flag = true;
    }

    timerScenario->stop();
    timerTurning->stop();
    timerDocking->stop();
    flag = true;

    // viewImageSleep();

    AutoDriveFlag = true;
    timerBumper->start();
}

void MainWindow::btnGuestClicked()
{

    // viewImageSmile();
    if (enableDxl)
    {
        dxlControl->moveDxl();
        dxlControl->setHomePosition();
    }

    event_flag = 1;
    scenarioCnt = 0;
    timerScenario->start(50);
}

void MainWindow::goGuest()
{
    wp_indx1 = 0;
    double yaw = atan2(2.0 * (path1[0][Rw] * path1[0][Rz]), 1.0 - 2.0 * (path1[0][Rz] * path1[0][Rz]));

    m_Rviz->SetGoalPose(path1[0][Px], path1[0][Py], yaw);

    event_flag = 3;
    timerScenario->start(100);
}

void MainWindow::btnHomeClicked()
{
    // viewImageSmile();

    if (enableDxl)
    {
        dxlControl->moveDxl();
        dxlControl->setHomePosition();
    }

    event_flag = 5;
    des_ang = des_ang3;
    turn_direct = LEFT;
    timerTurning->start();
}

void MainWindow::goEnd()
{
    wp_indx2 = 0;
    double yaw = atan2(2.0 * (path2[0][Rw] * path2[0][Rz]), 1.0 - 2.0 * (path2[0][Rz] * path2[0][Rz]));

    m_Rviz->SetGoalPose(path2[0][Px], path2[0][Py], yaw);

    event_flag = 6;
    timerScenario->start(10);
}

void MainWindow::goHome()
{
    wp_indx3 = 0;
    // ROS_INFO("========================================== 1");
    double yaw = atan2(2.0 * (path3[0][Rw] * path3[0][Rz]), 1.0 - 2.0 * (path3[0][Rz] * path3[0][Rz]));

    // ROS_INFO("========================================== 2");
    m_Rviz->SetGoalPose(path3[0][Px], path3[0][Py], yaw);
    // ROS_INFO("========================================== 3");
    event_flag = 8;
    timerScenario->start(10);
}

void MainWindow::scenarioUpdate()
{
    switch (event_flag)
    {
    case 1:
    {
        qnode.KobukiMove(-0.1, 0.0, 0.0, 0.0, 0.0, 0.2);
        scenarioCnt++;
        if (scenarioCnt > 45)
        {
            timerScenario->stop();
            event_flag = 2;
            des_ang = des_ang1;
            turn_direct = LEFT;
            timerTurning->start();
        }
        break;
    }
    case 3:
    {
        double dist = sqrt(pow(qnode.m_TopicPacket.m_AmclPx - path1[wp_indx1][Px], 2) + pow(qnode.m_TopicPacket.m_AmclPy - path1[wp_indx1][Py], 2));
        if (dist <= 0.6)
        {
            wp_indx1++;
            if (wp_indx1 < wp_size1)
            {
                double yaw = atan2(2.0 * (path1[wp_indx1][Rw] * path1[wp_indx1][Rz]), 1.0 - 2.0 * (path1[wp_indx1][Rz] * path1[wp_indx1][Rz]));

                m_Rviz->SetGoalPose(path1[wp_indx1][Px], path1[wp_indx1][Py], yaw);
            }
        }
        if (wp_indx1 >= wp_size1)
        {
            if (qnode.m_TopicPacket.m_GoalReached)
            {
                timerScenario->stop();
                event_flag = 4;
                des_ang = des_ang2;
                turn_direct = RIGHT;
                timerTurning->start();
            }
            wp_indx1--;
        }
        break;
    }
    case 6:
    {
        double dist = sqrt(pow(qnode.m_TopicPacket.m_AmclPx - path2[wp_indx2][Px], 2) + pow(qnode.m_TopicPacket.m_AmclPy - path2[wp_indx2][Py], 2));
        if (dist <= 0.6)
        {
            wp_indx2++;
            if (wp_indx2 < wp_size2)
            {
                double yaw = atan2(2.0 * (path2[wp_indx2][Rw] * path2[wp_indx2][Rz]), 1.0 - 2.0 * (path2[wp_indx2][Rz] * path2[wp_indx2][Rz]));

                m_Rviz->SetGoalPose(path2[wp_indx2][Px], path2[wp_indx2][Py], yaw);
            }
        }
        if (wp_indx2 >= wp_size2)
        {
            if (qnode.m_TopicPacket.m_GoalReached)
            {
                timerScenario->stop();
                event_flag = 7;
                des_ang = des_ang4;
                turn_direct = RIGHT;
                timerTurning->start();
            }
            wp_indx2--;
        }
        break;
    }
    case 8:
    {
        double dist = sqrt(pow(qnode.m_TopicPacket.m_AmclPx - path3[wp_indx3][Px], 2) + pow(qnode.m_TopicPacket.m_AmclPy - path3[wp_indx3][Py], 2));
        if (dist <= 0.6)
        {
            wp_indx3++;
            if (wp_indx3 < wp_size3)
            {
                double yaw = atan2(2.0 * (path3[wp_indx3][Rw] * path3[wp_indx3][Rz]), 1.0 - 2.0 * (path3[wp_indx3][Rz] * path3[wp_indx3][Rz]));

                m_Rviz->SetGoalPose(path3[wp_indx3][Px], path3[wp_indx3][Py], yaw);
            }
        }
        if (wp_indx3 >= wp_size3)
        {
            // ROS_INFO("wp_indx3 : [%d]", wp_indx3);
            sleep(1);
            if (qnode.m_TopicPacket.m_GoalReached)
            {
                timerScenario->stop();
                btnDockingClicked();

                if (connectServer)
                {
                    QByteArray txData;
                    txData.append(QByteArray::fromRawData("\x02\x05", 2));
                    txData.append(QByteArray::fromRawData("\x05", 1));
                    txData.append(QByteArray::fromRawData("\x0D\x05", 2));
                    m_Client->socket->write(txData);
                    m_Client->socket->flush();
                }
            }
            wp_indx3--;
        }
        break;
    }
    case 9:
    {
        qnode.KobukiMove(-0.1, 0.0, 0.0, 0.0, 0.0, 0.2);
        scenarioCnt++;
        if (scenarioCnt > 50)
        {
            timerScenario->stop();
            btnHomeClicked();
        }
        break;
    }
    default:
        break;
    }
}

void MainWindow::turning()
{
    double yaw = qnode.m_TopicPacket.m_ImuYaw;

    double err = yaw - des_ang;

    // ROS_INFO("yaw =: [%f], des =: [%f], err =: [%f]", yaw, des_ang, err);

    if (abs(err) >= 0.05)
    {
        qnode.KobukiMove(0.0, 0.0, 0.0, 0.0, 0.0, turn_direct == LEFT ? 1.0 : -1.0);
    }
    else
    {
        qnode.KobukiMove(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        timerTurning->stop();

        if (event_flag == 2)
        {
            goGuest();
        }
        else if (event_flag == 4)
        {
            if (connectServer)
            {
                QByteArray txData;
                txData.append(QByteArray::fromRawData("\x02\x05", 2));
                txData.append(QByteArray::fromRawData("\x02", 1));
                txData.append(QByteArray::fromRawData("\x0D\x05", 2));
                m_Client->socket->write(txData);
            }
            // viewImageWink();
            // btnHomeClicked();
        }
        else if (event_flag == 5)
        {
            ROS_INFO("kobuki go End");
            goEnd();
        }
        else if (event_flag == 7)
        {
            ROS_INFO("kobuki go Home");
            goHome();
        }
    }
}

void MainWindow::cbEnableDxlStateChanged(int state)
{
    if (state == 2)
    {
        enableDxl = true;
        dxlControl->dxl_init();
    }
    else
    {
        enableDxl = false;
        dxlControl->dxl_deinit();
    }
}

void MainWindow::btnRunDxlClicked()
{
    if (enableDxl)
    {
        dxlControl->moveDxl();
        dxlControl->setHomePosition();
    }
}

void MainWindow::btnDockingClicked()
{
    // docking();
    ROS_INFO("docking start");

    // Create the client
    // actionlib::SimpleActionClient<kobuki_msgs::AutoDockingAction> docking_ac("dock_drive_action", true);
    docking_ac = new actionlib::SimpleActionClient<kobuki_msgs::AutoDockingAction>("dock_drive_action", true);
    docking_ac->waitForServer();

    // Create goal object
    kobuki_msgs::AutoDockingGoal goal;

    // Send the goal
    docking_ac->sendGoal(goal);

    timerDocking->start();
}

void MainWindow::docking()
{
    // int dock_state = autoDocking->spin();

    // Assign initial state
    actionlib::SimpleClientGoalState dock_state = actionlib::SimpleClientGoalState::LOST;

    dock_state = docking_ac->getState();
    ROS_INFO("%d Docking status: %s", countDocking, dock_state.toString().c_str());
    countDocking++;

    if (dock_state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Docking complete.");
        on_btnSetInitialPose_clicked();

        qnode.ResetOdom();
    }
    else
    {
        if (countDocking >= 25){
            docking_ac->cancelGoal();
            qnode.KobukiMove(0.0,0.0,0.0,0.0,0.0,0.0);
            on_btnSetInitialPose_clicked();
        }
    }
}

void MainWindow::viewImageSmile()
{
    ui->centralwidget->hide();
    ui->dock_status->close();
    backgroundWidget->show();
    labelDrawImage(
        serviceImage,
        "/root/catkin_ws/src/mobile_robot/resources/images/smile.png",
        1.5);
}

void MainWindow::viewImageWink()
{
    ui->centralwidget->hide();
    ui->dock_status->close();
    backgroundWidget->show();
    labelDrawImage(
        serviceImage,
        "/root/catkin_ws/src/mobile_robot/resources/images/wink.jpg",
        2.0);
}

void MainWindow::viewImageSleep()
{
    ui->centralwidget->hide();
    ui->dock_status->close();
    backgroundWidget->show();
    labelDrawImage(
        serviceImage,
        "/root/catkin_ws/src/mobile_robot/resources/images/sleep.jpg",
        1.0);
}

void MainWindow::viewImageWait()
{
    ui->centralwidget->hide();
    ui->dock_status->close();
    backgroundWidget->show();
    QMovie *movie = new QMovie("/root/catkin_ws/src/mobile_robot/resources/images/wait.gif");
    QSize movieSize;
    movieSize.setWidth(serviceImage->width() * 0.5);
    movieSize.setHeight(serviceImage->height() * 0.8);
    movie->setScaledSize(movieSize);
    serviceImage->setAlignment(Qt::AlignmentFlag::AlignCenter);
    serviceImage->setMovie(movie);
    movie->start();
}

void MainWindow::labelDrawImage(QLabel *label, QString imagePath, double scale)
{
    QImage *image = new QImage();
    QPixmap *buffer = new QPixmap();

    if (image->load(imagePath))
    {
        *buffer = QPixmap::fromImage(*image);
        *buffer = buffer->scaled(image->width() * scale, image->height() * scale);
    }

    label->setAlignment(Qt::AlignmentFlag::AlignCenter);
    label->setPixmap(*buffer);
}

void MainWindow::tcpWaitTimeout(){
    btnConnectServerClicked();
}


void MainWindow::bumperCheck(){
    if (qnode.m_TopicPacket.m_BumperState && countDocking == 0){
        ROS_INFO("Occurred Bumper Event");
        emergencyStop();
    }
}

void MainWindow::emergencyStop(){
    timerUIUpdate->stop();
    timerScenario->stop();
    timerTurning->stop();
    timerDocking->stop();
    timerTcpWait->stop();
    qnode.CancelGoal();
    // AutoDriveFlag = false;
    qnode.KobukiMove(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
}

} // namespace MobileRobot
