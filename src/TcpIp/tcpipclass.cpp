#include "tcpipclass.h"

TcpClient::TcpClient(QObject *parent) : QObject(parent)
{
    socket = new QTcpSocket;
}

TcpClient::~TcpClient()
{

}

void TcpClient::connectToServer(){
    socket->connectToHost(ipAddress, portNum.toInt());
}

void TcpClient::setIpAddress(QString address){
    ipAddress = address;
}

void TcpClient::setPortNum(QString port){
    portNum = port;
}
