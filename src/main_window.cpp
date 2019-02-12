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

const int wp_size1 = 3;
double path1[wp_size1][4] = {
    -0.488, 0.162, 0.987, 0.158,
    -0.942, 0.368, 0.998, 0.055,
    -1.225, 0.311, 0.998, 0.055};
int wp_indx1 = 0;

const int wp_size2 = 5;
double path2[wp_size2][4] = {
    -0.702, 0.263, -0.130, 0.992,
    -0.745, 0.203, -0.108, 0.994,
    -0.484, 0.184, -0.098, 0.995,
    -0.172, 0.153, -0.127, 0.992,
    -0.002, 0.088, -0.020, 1.000};
int wp_indx2 = 0;

const int wp_size3 = 8;
double path3[wp_size3][4] = {
    -0.329, -1.133, 0.500, 0.866,
    -0.167, -0.812, 0.558, 0.830,
    0.042, -0.321, 0.685, 0.729,
    -0.004, 0.217, 0.760, 0.650,
    -0.118, 0.676, 0.767, 0.642,
    -0.287, 1.160, 0.884, 0.467,
    -0.505, 1.290, 0.936, 0.352,
    -0.600, 1.485, 0.952, 0.307};
int wp_indx3 = 0;

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
    timerUIUpdate->start(30);

    timerUpdate = new QTimer(this);
    connect(timerUpdate, SIGNAL(timeout()), this, SLOT(update()));
    timerUpdate->setInterval(1000);

    connect(ui->btnConnectServer, SIGNAL(clicked()), this, SLOT(btnConnectServer_clicked()));

    connect(ui->btnScenario1, SIGNAL(clicked()), this, SLOT(btnScenario1_clicked()));
    connect(ui->btnScenario4, SIGNAL(clicked()), this, SLOT(btnScenario4_clicked()));

    timerScenario = new QTimer(this);
    connect(timerScenario, SIGNAL(timeout()), this, SLOT(scenarioUpdate()));

    guestCome = false;
    event_flag = 0;

    //    connect(ui->pushButton, SIGNAL(clicked()), this, SLOT(btnTest_clicked()));

    m_Client = new TcpClient(this);

    connect(m_Client->socket, SIGNAL(connected()), this, SLOT(onConnectServer()));
    connect(m_Client->socket, SIGNAL(readyRead()), this, SLOT(readMessage()));

    enableDxl = false;
    connect(ui->cbEnableDxl, SIGNAL(stateChanged(int)), this, SLOT(cbEnableDxl_stateChanged(int)));
    connect(ui->btnRunDxl, SIGNAL(clicked()), this, SLOT(btnRunDxl_clicked()));

    dxlControl = new DxlControl();

    connect(ui->btnDocking, SIGNAL(clicked()), this, SLOT(btnDocking_clicked()));

    init_flag = false;

    timerTurning = new QTimer(this);
    connect(timerTurning, SIGNAL(timeout()), this, SLOT(turning()));

    timerDocking = new QTimer(this);
    connect(timerDocking, SIGNAL(timeout()), this, SLOT(docking()));

    autoDocking = new AutoDockingROS("mobile_robot");

    ros::NodeHandle n;
    autoDocking->init(n);
}

MainWindow::~MainWindow()
{
    timerUIUpdate->stop();
    timerUpdate->stop();
    delete m_Client;
    delete dxlControl;
    delete autoDocking;
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
    QSettings settings("Qt-Ros Package", "robomap");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url", QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui->line_edit_master->setText(master_url);
    ui->line_edit_host->setText(host_url);
    //ui->line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    ui->checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui->checkbox_use_environment->setChecked(checked);
    if (checked)
    {
        ui->line_edit_master->setEnabled(false);
        ui->line_edit_host->setEnabled(false);
        //ui->line_edit_topic->setEnabled(false);
    }
}

void MainWindow::WriteSettings()
{
    QSettings settings("Qt-Ros Package", "robomap");
    settings.setValue("master_url", ui->line_edit_master->text());
    settings.setValue("host_url", ui->line_edit_host->text());
    //settings.setValue("topic_name",ui->line_edit_topic->text());
    settings.setValue("use_environment_variables", QVariant(ui->checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings", QVariant(ui->checkbox_remember_settings->isChecked()));
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    WriteSettings();
    QMainWindow::closeEvent(event);
}

void MainWindow::on_btnSetInitialPose_clicked()
{
    /*
        Position(0.531, 0.237, 0.000), Orientation(0.000, 0.000, 0.009, 1.000) = Angle: 0.018
        Position(-0.685, 1.465, 0.000), Orientation(0.000, 0.000, 0.950, 0.312) = Angle: 2.506
        Position(0.104, 0.035, 0.000), Orientation(0.000, 0.000, 0.002, 1.000) = Angle: 0.004
    */
    double Px = 0.104;
    double Py = 0.035;
    double Rz = 0.002;
    double Rw = 1.000;
    double yaw = atan2(2.0 * (Rw * Rz), 1.0 - 2.0 * (Rz * Rz));

    m_Rviz->SetInitialPose(Px, Py, yaw);

    wp_indx1 = 0;
    wp_indx2 = 0;
    wp_indx3 = 0;
    if (init_flag == false)
    {
        m_Rviz->SetGoalPose(Px, Py, yaw);
        init_flag = true;
    }
}

void MainWindow::update()
{
}

void MainWindow::btnConnectServer_clicked()
{
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

    on_btnSetInitialPose_clicked();

    enableDxl = true;
    dxlControl->dxl_init();
    ui->cbEnableDxl->setChecked(enableDxl);
}

void MainWindow::readMessage()
{
    QByteArray rxData = m_Client->socket->readAll();
    if (rxData.length() > 0)
    {
        qDebug() << "Receive Data(Frome Server) : " + rxData;
        QChar *ch = new QChar[rxData.length()];
        for (int i = 0; i < rxData.length(); i++)
        {
            ch[i] = rxData.at(i);
        }
        if (ch[0] == 0x02 && ch[1] == 0x05)
        {
            if (ch[2] == 0x01)
            {
                // qDebug() << "Geust come to front UI Monitor";
                btnScenario1_clicked();
            }
            else if (ch[2] == 0x00)
            {
                btnScenario4_clicked();
                // qDebug() << "Geust leaved";
            }
        }
    }
}

void MainWindow::btnScenario1_clicked()
{

    if (enableDxl)
    {
        dxlControl->moveDxl();
        dxlControl->setHomePosition();
    }

    linear.x = -0.1;
    linear.y = 0.0;
    linear.z = 0.0;

    angular.x = 0.0;
    angular.y = 0.0;
    angular.z = -0.2;

    event_flag = 1;
    scenarioCnt = 0;
    timerScenario->start(50);
}

void MainWindow::btnScenario2_clicked()
{
    double yaw = qnode.m_TopicPacket.m_ImuYaw;
    des_ang = 3.141592;
    timerTurning->start(10);
    event_flag = 2;
    // btnScenario3_clicked();
}

void MainWindow::btnScenario3_clicked()
{
    double Px = path1[wp_indx1][0];
    double Py = path1[wp_indx1][1];
    double Rz = path1[wp_indx1][2];
    double Rw = path1[wp_indx1][3];
    double yaw = atan2(2.0 * (Rw * Rz), 1.0 - 2.0 * (Rz * Rz));

    m_Rviz->SetGoalPose(Px, Py, yaw);

    event_flag = 3;
    timerScenario->start(100);
}

void MainWindow::btnScenario4_clicked()
{

    if (enableDxl)
    {
        dxlControl->moveDxl();
        dxlControl->setHomePosition();
    }

    // linear.x = -0.1;
    // linear.y = 0.0;
    // linear.z = 0.0;

    // angular.x = 0.0;
    // angular.y = 0.0;
    // angular.z = -1.0;

    event_flag = 6;
    // scenarioCnt = 0;
    // timerScenario->start(30);

    double yaw = qnode.m_TopicPacket.m_ImuYaw;
    des_ang = 0; //3.141592/2.0;
    timerTurning->start(10);
}

void MainWindow::btnScenario4()
{
    double Px = path2[wp_indx2][0];
    double Py = path2[wp_indx2][1];
    double Rz = path2[wp_indx2][2];
    double Rw = path2[wp_indx2][3];
    double yaw = atan2(2.0 * (Rw * Rz), 1.0 - 2.0 * (Rz * Rz));

    m_Rviz->SetGoalPose(Px, Py, yaw);

    event_flag = 4;
    timerScenario->start(100);
}

void MainWindow::btnScenario5_clicked()
{
    linear.x = 0.0;
    linear.y = 0.0;
    linear.z = 0.0;

    angular.x = 0.0;
    angular.y = 0.0;
    angular.z = -1.0;

    event_flag = 7;
    scenarioCnt = 0;
    timerScenario->start(100);
}

void MainWindow::btnScenario5()
{
    double Px = path3[wp_indx3][0];
    double Py = path3[wp_indx3][1];
    double Rz = path3[wp_indx3][2];
    double Rw = path3[wp_indx3][3];
    double yaw = atan2(2.0 * (Rw * Rz), 1.0 - 2.0 * (Rz * Rz));

    m_Rviz->SetGoalPose(Px, Py, yaw);

    event_flag = 5;
    timerScenario->start(100);
}

void MainWindow::scenarioUpdate()
{
    //    qDebug() << event_flag;
    if (event_flag == 1 || event_flag == 6 || event_flag == 7)
    {
        qnode.KobukiMove(linear, angular);
        scenarioCnt++;
        if (scenarioCnt > 30)
        {
            timerScenario->stop();
            if (event_flag == 1)
            {
                event_flag = 2;
                btnScenario2_clicked();
            }
            else if (event_flag == 6)
            {
                btnScenario4();
            }
            else if (event_flag == 7)
            {
                btnScenario5();
            }
        }
    }
    else if (event_flag == 3)
    {
        double dist = sqrt(pow(qnode.m_TopicPacket.m_AmclPx - path1[wp_indx1][0], 2) + pow(qnode.m_TopicPacket.m_AmclPy - path1[wp_indx1][1], 2));
        if (dist <= 0.6)
        {
            wp_indx1++;
            if (wp_indx1 < wp_size1)
            {
                double Px = path1[wp_indx1][0];
                double Py = path1[wp_indx1][1];
                double Rz = path1[wp_indx1][2];
                double Rw = path1[wp_indx1][3];
                double yaw = atan2(2.0 * (Rw * Rz), 1.0 - 2.0 * (Rz * Rz));

                m_Rviz->SetGoalPose(Px, Py, yaw);
            }
            else
            {
                double Rz = qnode.m_TopicPacket.m_OdomRz;
                double Rw = qnode.m_TopicPacket.m_OdomRw;
                double yaw = atan2(2.0 * (Rw * Rz), 1.0 - 2.0 * (Rz * Rz));
                // double err_z = abs(Rz - qnode.m_TopicPacket.m_AmclRz);
                // double err_w = abs(Rw - qnode.m_TopicPacket.m_AmclRw);
                // qDebug() << "err_Z : " + QString::number(err_z);
                // qDebug() << "err_w : " + QString::number(err_w);
                // cout << "flag " << event_flag << " yaw : " << yaw << endl;
                // if (err_z < 0.15)
                // {
                timerScenario->stop();

                if (enableDxl)
                {
                    QByteArray txData;
                    txData.append(QByteArray::fromRawData("\x02\x05", 2));
                    txData.append(QByteArray::fromRawData("\x02", 1));
                    txData.append(QByteArray::fromRawData("\x0D\x05", 2));
                    m_Client->socket->write(txData);
                }
                sleep(5);
                btnScenario4_clicked();
                // }
            }
        }
    }
    else if (event_flag == 4)
    {
        double dist = sqrt(pow(qnode.m_TopicPacket.m_AmclPx - path2[wp_indx2][0], 2) + pow(qnode.m_TopicPacket.m_AmclPy - path2[wp_indx2][1], 2));
        if (dist <= 0.6)
        {
            wp_indx2++;
            if (wp_indx2 < wp_size2)
            {
                double Px = path2[wp_indx2][0];
                double Py = path2[wp_indx2][1];
                double Rz = path2[wp_indx2][2];
                double Rw = path2[wp_indx2][3];
                double yaw = atan2(2.0 * (Rw * Rz), 1.0 - 2.0 * (Rz * Rz));

                m_Rviz->SetGoalPose(Px, Py, yaw);
            }
            else
            {
                double Rz = qnode.m_TopicPacket.m_OdomRz;
                double Rw = qnode.m_TopicPacket.m_OdomRw;
                double yaw = atan2(2.0 * (Rw * Rz), 1.0 - 2.0 * (Rz * Rz));
                // double err_z = abs(Rz - qnode.m_TopicPacket.m_AmclRz);
                // double err_w = abs(Rw - qnode.m_TopicPacket.m_AmclRw);
                // qDebug() << "err_Z : " + QString::number(err_z);
                // qDebug() << "err_w : " + QString::number(err_w);
                // cout << "flag " << event_flag << " yaw : " << yaw << endl;
                // double Rz = path2[wp_indx2][2];
                // double err_z = abs(Rz - qnode.m_TopicPacket.m_AmclRz);
                // qDebug() << "err_Z : " + QString::number(err_z);
                // if (err_z < 1.25)
                // {
                timerScenario->stop();
                // sleep(5);
                // btnScenario5_clicked();
                // }
            }
        }
    }
    else if (event_flag == 5)
    {
        double dist = sqrt(pow(qnode.m_TopicPacket.m_AmclPx - path3[wp_indx3][0], 2) + pow(qnode.m_TopicPacket.m_AmclPy - path3[wp_indx3][1], 2));
        if (dist <= 0.6)
        {
            wp_indx3++;
            if (wp_indx3 < wp_size3)
            {
                double Px = path3[wp_indx3][0];
                double Py = path3[wp_indx3][1];
                double Rz = path3[wp_indx3][2];
                double Rw = path3[wp_indx3][3];
                double yaw = atan2(2.0 * (Rw * Rz), 1.0 - 2.0 * (Rz * Rz));

                m_Rviz->SetGoalPose(Px, Py, yaw);
            }
            else
            {

                double Rz = path3[wp_indx3][2];
                double Rw = path3[wp_indx3][3];
                double err_z = abs(Rz - qnode.m_TopicPacket.m_AmclRz);
                double err_w = abs(Rw - qnode.m_TopicPacket.m_AmclRw);
                // qDebug() << "err_Z : " + QString::number(err_z);
                // qDebug() << "err_w : " + QString::number(err_w);
                // if (err_z < 0.05)
                // {
                timerScenario->stop();
                sleep(5);
                btnDocking_clicked();
                // }
            }
        }
    }
}

void MainWindow::btnTest_clicked()
{
    double Px = qnode.m_TopicPacket.m_AmclPx;
    double Py = qnode.m_TopicPacket.m_AmclPy;
    double Rz = qnode.m_TopicPacket.m_AmclRz;
    double Rw = qnode.m_TopicPacket.m_AmclRw;
    double yaw = atan2(2.0 * (Rw * Rz), 1.0 - 2.0 * (Rz * Rz));

    //m_Rviz->SetGoalPose(Px, Py, -3.14);
    //    m_Rviz->SetGoalPose(Px, Py, ui->lineEdit_3->text().toDouble());
}

void MainWindow::turning()
{
    linear.x = 0.0;
    linear.y = 0.0;
    linear.z = 0.0;

    angular.x = 0.0;
    angular.y = 0.0;
    angular.z = 1.0;

    double yaw = qnode.m_TopicPacket.m_ImuYaw;

    double err = yaw - des_ang;

    ROS_INFO("yaw =: [%f], des =: [%f], err =: [%f]", yaw, des_ang, err);

    if (abs(err) >= 0.05)
    {
        qnode.KobukiMove(linear, angular);
    }
    else
    {
        angular.z = 0.0;
        qnode.KobukiMove(linear, angular);
        timerTurning->stop();

        if (event_flag == 2)
        {
            btnScenario3_clicked();
        }
        else if (event_flag == 6)
        {
            btnScenario4();
        }
    }
}

void MainWindow::cbEnableDxl_stateChanged(int state)
{
    //    qDebug() << state;
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

void MainWindow::btnRunDxl_clicked()
{
    if (enableDxl)
    {
        dxlControl->moveDxl();
        dxlControl->setHomePosition();
    }
}

void MainWindow::btnDocking_clicked()
{
    timerDocking->start(1000);
}

void MainWindow::docking()
{
    int dock_state = autoDocking->spin();

    ROS_INFO("auto docking result : [%d]", dock_state);
    if (dock_state == 1){
        timerDocking->stop();
        // on_btnSetInitialPose_clicked();
    }
    else if(dock_state == 2){
        linear.x = -0.3;
        linear.y = 0;
        linear.z = 0;

        angular.x = 0;
        angular.y = 0;
        angular.z = 0;

        qnode.KobukiMove(linear, angular);
    }
}

} // namespace MobileRobot
