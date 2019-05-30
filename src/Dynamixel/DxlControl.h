#pragma once

#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
using namespace std;

#include <dynamixel_sdk/dynamixel_sdk.h>

// Control table address
enum{Red=513, Green, Blue};
constexpr auto ADDR_TORQUE_ENABLE = 562;
constexpr auto ADDR_GOAL_POSITION = 596;
constexpr auto ADDR_GOAL_VELOCITY = 600;
constexpr auto ADDR_GOAL_ACCELERATION = 606;
constexpr auto ADDR_PRESENT_POSITION = 611;
constexpr auto ADDR_OPERATING_MODE = 11;
constexpr auto ADDR_LED_RED = 563;
constexpr auto ADDR_LED_GREEN = 564;
constexpr auto ADDR_LED_BLUE = 565;
constexpr auto ADDR_POSITION_P_GAIN = 594;

// Protocol version
constexpr auto PROTOCOL_VERSION = 2.0;

// Default setting
constexpr auto DXL_ID = 1;		// Is different in Dynamixel
constexpr auto BAUDRATE = 4000000;	// Is different in Dynamixel
constexpr auto DEVICENAME = "/dev/ttyUSB0";	// Is different in PC

constexpr auto TORQUE_ENABLE = 1;
constexpr auto TORQUE_DISABLE = 0;
constexpr auto OPERATING_POSITION_CONTROL = 3;
constexpr auto OPERATING_EXT_POSITION = 4;

class DxlControl
{
public:
    DxlControl();
    ~DxlControl();

    int dxl_comm_result = COMM_TX_FAIL;
    uint8_t dxl_error = 0;	// Dynamixel error
    int32_t init_pos = 0;

    void dxl_init();
    void dxl_deinit();

    void setLEDon(int addr);
    void setLEDoff();
    void setPosition(int32_t goal_position);
    void setOperateMode(uint8_t mode);
    void setHomePosition();

    int32_t getPresentPosition();

    dynamixel::PortHandler *portHandler;
    dynamixel::PacketHandler *packetHandler;

    void moveDxl();
};
