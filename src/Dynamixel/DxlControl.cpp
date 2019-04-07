#include "DxlControl.h"

DxlControl::DxlControl()
{
    init_pos = 0;
}

DxlControl::~DxlControl()
{
    dxl_deinit();
}

void DxlControl::dxl_init()
{
    portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    if (portHandler->openPort()) {
        printf("Succeeded to open the port!\n");

    }
    else {
        printf("Failed to open the port!\n");
        printf("Press any key to terminate...\n");
    }

    // Set port baudrate
    if (portHandler->setBaudRate(BAUDRATE)) {
        printf("Succeeded to change the baudrate!\n");
    }
    else {
        printf("Failed to change the baudrate!\n");
        printf("Press any key to terminate...\n");
    }

    // Check Dynamixel Torque on or off
    int torque = 0;
    dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, reinterpret_cast<uint8_t*>(&torque), &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0) {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }
    else {
        printf("Dynamixel has been successfully connected\n");
    }

    if(torque == TORQUE_ENABLE){
        packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    }

    packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_OPERATING_MODE, OPERATING_POSITION_CONTROL, &dxl_error);

    packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);

    if(init_pos == 0){
        packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION, reinterpret_cast<uint32_t*>(&init_pos), &dxl_error);
        printf("Initial Position : %d\n", init_pos);
    }

    setLEDon(Green);
}

void DxlControl::dxl_deinit()
{
    setLEDon(Red);
    // Disable Dynamixel Torque
    packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);

    // Write Dynamixel Operating mode
    packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_OPERATING_MODE, OPERATING_POSITION_CONTROL, &dxl_error);

    // Reset home position
    packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    setHomePosition();

    // Disable Dynamixel Torque
    packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);

    // Close port
    portHandler->closePort();

    printf("Dynamixel has been successfully disconnected\n");
}

void DxlControl::setLEDon(int addr)
{
    setLEDoff();
    packetHandler->write1ByteTxRx(portHandler, DXL_ID, static_cast<uint16_t>(addr), 255, &dxl_error);
}

void DxlControl::setLEDoff()
{
    packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_LED_RED, 0, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_LED_GREEN, 0, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_LED_BLUE, 0, &dxl_error);
}

void DxlControl::setPosition(int32_t goal_position)
{
    packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, static_cast<uint32_t>(goal_position), &dxl_error);
}

void DxlControl::setOperateMode(uint8_t mode)
{
    // Disable Dynamixel Torque
    packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);

    // Write Dynamixel Operating mode
    packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_OPERATING_MODE, mode, &dxl_error);

    // Reset home position
    packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
}

void DxlControl::setHomePosition()
{
    int32_t pos = 0;
    do {
        packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, init_pos, &dxl_error);
        packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION, reinterpret_cast<uint32_t*>(&pos), &dxl_error);
//        printf("Initial pos : %d, Current pos : %d\n", init_pos, pos);
    } while (abs(init_pos - pos) > 200);
}

int32_t DxlControl::getPresentPosition()
{
    int32_t present_position = 0;
    packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION, reinterpret_cast<uint32_t*>(&present_position), &dxl_error);
    return present_position;
}

void DxlControl::moveDxl(){
    setLEDon(Blue);
    int32_t pos = 0;
    int32_t offset = 40000;
    int32_t des = init_pos + offset;
    do {
        packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, des, &dxl_error);
        packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION, reinterpret_cast<uint32_t*>(&pos), &dxl_error);
//        printf("Desired pos : %d, Current pos : %d\n", des, pos);
    } while (abs(des - pos) > 100);

    des = init_pos - offset;
    pos = 0;
    do {
        packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, des, &dxl_error);
        packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION, reinterpret_cast<uint32_t*>(&pos), &dxl_error);
//        printf("Desired pos : %d, Current pos : %d\n", des, pos);
    } while (abs(des - pos) > 100);
}
