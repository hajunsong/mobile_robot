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

double init_pose[4] = {0.124, 0.102, 0.019, 1.000};

const int wp_size1 = 6;
double path1[wp_size1][4] = {
    -0.261, 0.259, 0.980, 0.200,
    -0.230, 0.221, 0.967, 0.255,
    -0.632, 0.469, 0.950, 0.313,
    -0.927, 0.694, 0.923, 0.385,
    -1.217, 1.063, 0.879, 0.476,
    -1.326, 1.343, 0.779, 0.627
};
int wp_indx1 = 0;

const int wp_size2 = 3;
double path2[wp_size2][4] = {
    -1.501, 2.003, 0.720, 0.694,
    -1.503, 2.317, 0.635, 0.772,
    -1.456, 2.680, 0.543, 0.840
};
int wp_indx2 = 0;

const int wp_size3 = 4;
double path3[wp_size3][4] = {
    -1.501, 1.846, -0.626, 0.780,
    -1.226, 1.025, -0.375, 0.927,
    -0.606, 0.380, -0.007, 1.000,
    0.104, 0.102, 0.019, 1.000
};
int wp_indx3 = 0;

const double D2R = M_PI/180.0;
const double des_ang1 = 150*D2R;
const double des_ang2 = -165*D2R;
const double des_ang3 = 120*D2R;
const double des_ang4 = -90*D2R;

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

    autoDocking = new AutoDockingROS("mobile_robot");
    navi1 = new PathTrackingMR();

    ros::NodeHandle n;
    autoDocking->init(n);

    backgroundWidget = new QWidget(this);
    backgroundWidget->setFixedSize(1280, 800);
    backgroundWidget->setGeometry(0, 0, 1280, 800);
    backgroundWidget->hide();

    serviceImage = new QLabel(backgroundWidget);
    serviceImage->setGeometry(backgroundWidget->rect());


    on_button_connect_clicked(true);
    btnConnectServerClicked();
}

MainWindow::~MainWindow()
{
    timerUIUpdate->stop();
    delete m_Client;
    delete dxlControl;
    delete autoDocking;
    delete navi1;
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

    enableDxl = true;
    dxlControl->dxl_init();
    ui->cbEnableDxl->setChecked(enableDxl);

    connectServer = true;

    on_btnSetInitialPose_clicked();
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
                // viewImageSmile();
                btnGuestClicked();
            }
            else if (ch[2] == 0x00)
            {
                // qDebug() << "Geust leaved";
                // viewImageWink();
                btnHomeClicked();
            }
            else if (ch[2] == 0x02){
                // viewImageWait();
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
    // timerDocking->stop();

    // viewImageSleep();
}

void MainWindow::btnGuestClicked()
{
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
    double yaw = atan2(2.0 * (path1[wp_indx1][Rw] * path1[wp_indx1][Rz]), 1.0 - 2.0 * (path1[wp_indx1][Rz] * path1[wp_indx1][Rz]));

    m_Rviz->SetGoalPose(path1[wp_indx1][Px], path1[wp_indx1][Py], yaw);

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
    turn_direct = RIGHT;
    timerTurning->start();
}

void MainWindow::goEnd(){
    double yaw = atan2(2.0 * (path2[wp_indx2][Rw] * path2[wp_indx2][Rz]), 1.0 - 2.0 * (path2[wp_indx2][Rz] * path2[wp_indx2][Rz]));

    m_Rviz->SetGoalPose(path2[wp_indx2][Px], path2[wp_indx2][Py], yaw);

    event_flag = 6;
    timerScenario->start(10);
}

void MainWindow::goHome(){
    double yaw = atan2(2.0 * (path3[wp_indx3][Rw] * path3[wp_indx3][Rz]), 1.0 - 2.0 * (path3[wp_indx3][Rz] * path3[wp_indx3][Rz]));

    m_Rviz->SetGoalPose(path3[wp_indx3][Px], path3[wp_indx3][Py], yaw);

    event_flag = 8;
    timerScenario->start(10);
}

void MainWindow::scenarioUpdate()
{
    switch (event_flag)
    {
        case 1:
        {
            qnode.KobukiMove(-0.1, 0.0, 0.0, 0.0, 0.0, -0.2);
            scenarioCnt++;
            if (scenarioCnt > 60)
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
            // double vx, wz;
            // int result = navi1->move(qnode.m_TopicPacket.m_AmclPx, qnode.m_TopicPacket.m_AmclPy, &vx, &wz);

            // qnode.KobukiMove(vx, 0.0, 0.0, 0.0, 0.0, wz);

            // if (result == 1){
            //     timerScenario->stop();
            //     event_flag = 4;
            //     des_ang = des_ang2;
            //     turn_direct = LEFT;
            //     timerTurning->start();
            // }

            // ros::NodeHandle nh("~");
            // nh.param("move_base/DWAPlannerROS/local_plan min_vel_x", 0.5);
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
                if (qnode.m_TopicPacket.m_GoalReached){
                    timerScenario->stop();
                    event_flag = 4;
                    des_ang = des_ang2;
                    turn_direct = LEFT;
                    timerTurning->start();
                }
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
            if(wp_indx2 >= wp_size2){
                if (qnode.m_TopicPacket.m_GoalReached){
                    timerScenario->stop();
                    event_flag = 7;
                    des_ang = des_ang4;
                    turn_direct = RIGHT;
                    timerTurning->start();
                }
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
            if(wp_indx3 >= wp_size3){
                // ROS_INFO("wp_indx3 : [%d]", wp_indx3);
                sleep(1);
                if (qnode.m_TopicPacket.m_GoalReached){
                    timerScenario->stop();
                    // docking();
                    // timerDocking->start(100);
                    btnDockingClicked();
                }
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
        else if (event_flag == 4){
            if (connectServer)
            {
                QByteArray txData;
                txData.append(QByteArray::fromRawData("\x02\x05", 2));
                txData.append(QByteArray::fromRawData("\x02", 1));
                txData.append(QByteArray::fromRawData("\x0D\x05", 2));
                m_Client->socket->write(txData);
            }
            // viewImageWink();
            btnHomeClicked();
        }
        else if(event_flag == 5){
            goEnd();
        }
        else if(event_flag == 7){
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
    timerDocking->start(100);
}

void MainWindow::docking()
{
    timerDocking->stop();
    ROS_INFO("docking start");
    int dock_state = autoDocking->spin();

    ROS_INFO("auto docking result : [%d]", dock_state);
    if (dock_state == 1){
        on_btnSetInitialPose_clicked();
    }
    else if(dock_state == 2){
        qnode.KobukiMove(-0.25, 0.0, 0.0, 0.0, 0.0, 0.0);
        btnDockingClicked();
    }
}

void MainWindow::viewImageSmile(){
    ui->centralwidget->hide();
    ui->dock_status->close();
    backgroundWidget->show();
    labelDrawImage(
        serviceImage, 
        "/root/catkin_ws/src/mobile_robot/resources/images/smile.png", 
        1.5
    );
}

void MainWindow::viewImageWink(){
    ui->centralwidget->hide();
    ui->dock_status->close();
    backgroundWidget->show();
    labelDrawImage(
        serviceImage, 
        "/root/catkin_ws/src/mobile_robot/resources/images/wink.jpg", 
        2.0
    );
}

void MainWindow::viewImageSleep(){
    ui->centralwidget->hide();
    ui->dock_status->close();
    backgroundWidget->show();
    labelDrawImage(
        serviceImage, 
        "/root/catkin_ws/src/mobile_robot/resources/images/sleep.jpg", 
        1.0
    );
}

void MainWindow::viewImageWait(){
    ui->centralwidget->hide();
    ui->dock_status->close();
    backgroundWidget->show();
    QMovie *movie = new QMovie("/root/catkin_ws/src/mobile_robot/resources/images/wait.gif");
    QSize movieSize;
    movieSize.setWidth(serviceImage->width()*0.5);
    movieSize.setHeight(serviceImage->height()*0.8);
    movie->setScaledSize(movieSize);
    serviceImage->setAlignment(Qt::AlignmentFlag::AlignCenter);
    serviceImage->setMovie(movie);
    movie->start();
}

void MainWindow::labelDrawImage(QLabel *label, QString imagePath, double scale) {
    QImage *image = new QImage();
    QPixmap *buffer = new QPixmap();

    if (image->load(imagePath)){
        *buffer = QPixmap::fromImage(*image);
        *buffer = buffer->scaled(image->width()*scale, image->height()*scale);
    }

    label->setAlignment(Qt::AlignmentFlag::AlignCenter);
    label->setPixmap(*buffer);
}

} // namespace MobileRobot
