
#include <qstandarditemmodel.h>
#include "include/mobile_robot/main_window.hpp"
#include "ui_main_window.h"

#include "topicguiupdateclass.h"

#include "include/mobile_robot/qnode.hpp"

using namespace MobileRobot;

TopicGuiUpdateClass::TopicGuiUpdateClass()
{
    uicntTopic = 0;
    prevcntTopic = 0;
    syshzTopic = 0;
}

void TopicGuiUpdateClass::Initialize(void *_ui, void *_model)
{
    Ui::MainWindowDesign* ui = (Ui::MainWindowDesign*)_ui;

    QStandardItemModel* modelTopic = (QStandardItemModel*)_model;
}

void TopicGuiUpdateClass::UIUpdate(void *_ui, void *_model, void *_RosClass)
{
    Ui::MainWindowDesign* ui = (Ui::MainWindowDesign*)_ui;
    QStandardItemModel* modelTopic = (QStandardItemModel*)_model;
    QNode* TopicInfo = (QNode*)_RosClass;

    CTopicPacket* TopicPacket = TopicInfo->GetTopicPacket();

    ui->tbMobileRobotPx->setText(QString::number(TopicPacket->m_OdomPx, 'f', 6));
    ui->tbMobileRobotPy->setText(QString::number(TopicPacket->m_OdomPy, 'f', 6));
    ui->tbMobileRobotTheta->setText(QString::number(TopicPacket->m_OdomTheta, 'f', 6));
}
