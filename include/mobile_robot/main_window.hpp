/**
 * @file /include/robomap/main_window.hpp
 *
 * @brief Qt based gui for robomap.
 *
 * @date November 2010
 **/
#ifndef robomap_MAIN_WINDOW_H
#define robomap_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QMainWindow>
#include <QDebug>
#include <QTime>
#include <QTimer>
#include <QStandardItemModel>

#include <rviz/render_panel.h>
#include <rviz/visualization_manager.h>
#include <rviz/tool_manager.h>

#include "src/roslaunchclass.h"

#include "ui_main_window.h"

#include "qnode.hpp"
#include "src/Rviz/createrviz.h"
#include "src/GuiUpdate/topicguiupdateclass.h"
#include "src/TcpIp/tcpipclass.h"
#include "src/Dynamixel/DxlControl.h"

#include "src/AutoDocking/auto_docking_ros.hpp"

using namespace kobuki;

#include <iostream>
using namespace std;

#define DEBUG_ENABLED   true

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace MobileRobot {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    MainWindow(int argc, char** argv, QWidget *parent = 0);
    ~MainWindow();

    void ReadSettings(); // Load up qt program settings at startup
    void WriteSettings(); // Save qt program settings when closing

    void closeEvent(QCloseEvent *event); // Overloaded function
    void showNoMasterMessage();

public Q_SLOTS:
    /******************************************
    ** Auto-connections (connectSlotsByName())
    *******************************************/
    void on_actionAbout_triggered();
    void on_button_connect_clicked(bool check );
    void on_btnSetInitialPose_clicked();
    //        void on_btnSetGoalPose_clicked();
    void on_checkbox_use_environment_stateChanged(int state);


    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically

private slots:
    void UiUpdate();
    void update();
    void btnConnectServer_clicked();
    void scenarioUpdate();
    void btnScenario1_clicked();
    void btnScenario2_clicked();
    void btnScenario3_clicked();
    void btnScenario4_clicked();
    void btnScenario4();
    void btnScenario5_clicked();
    void btnScenario5();
    void btnTest_clicked();

    void onConnectServer();
    void readMessage();

    void cbEnableDxl_stateChanged(int state);
    void btnRunDxl_clicked();

    void btnDocking_clicked();

    void turning();
    void docking();

private:
    Ui::MainWindowDesign    *ui;
    QNode                   qnode;

    CreateRviz              *m_Rviz;
    RosLaunchClass          *m_RosLaunch;
    TopicGuiUpdateClass     *m_TopicUpdate;

    QStandardItemModel      *modelMain;

    TcpClient               *m_Client;
    VelCmd                  linear, angular;

    QTimer                  *timerUIUpdate;
    QTimer                  *timerUpdate;
    QTimer                  *timerScenario;
    QTimer                  *timerTurning;
    QTimer                  *timerDocking;

    int                     scenarioCnt;
    bool                    guestCome;
    int                     event_flag;
    double                  des_ang;

    bool                    enableDxl;
    DxlControl              *dxlControl;
    bool                    init_flag;

    AutoDockingROS          *autoDocking;

};

}  // namespace robomap

#endif // robomap_MAIN_WINDOW_H
