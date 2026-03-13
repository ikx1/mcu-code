#include "canopen_proto.h"

#include <string.h>

#include "can_queue.h"

static void canopen_send_standard_frame(uint16_t std_id, const uint8_t *data, uint8_t len)
{
    can_std_frame_t tx_frame = {0};

    tx_frame.std_id = std_id;
    tx_frame.dlc = len;
    if ((data != NULL) && (len > 0u))
    {
        if (len > CAN_FRAME_MAX_DATA_LEN)
        {
            len = CAN_FRAME_MAX_DATA_LEN;
            tx_frame.dlc = CAN_FRAME_MAX_DATA_LEN;
        }

        memcpy(tx_frame.data, data, len);
    }

    (void)can_queue_push_frame(&tx_frame);
}

void Control_Mode_SET(uint8_t CANopen_ID, uint8_t CANopen_mode)
{
    uint8_t data[8];

    data[0] = 0x2b;
    data[1] = 0x60;
    data[2] = 0x60;
    data[3] = 0x00;
    data[4] = CANopen_mode;
    data[5] = 0x00;
    data[6] = 0x00;
    data[7] = 0x00;

    canopen_send_standard_frame(0x600u + CANopen_ID, data, 8u);
}

void CANopen_NMT(uint8_t cmd, uint8_t CANopen_ID)
{
    uint8_t data[2];

    data[0] = cmd;
    data[1] = CANopen_ID;

    canopen_send_standard_frame(0x000u, data, 2u);
}

void SDO_Write_Data1(uint8_t CANopen_ID, uint16_t Index, uint8_t SubIndex, uint8_t DATA)
{
    uint8_t data[5];

    data[0] = 0x2f;
    data[1] = (uint8_t)(Index & 0xFFu);
    data[2] = (uint8_t)((Index >> 8) & 0xFFu);
    data[3] = SubIndex;
    data[4] = DATA;

    canopen_send_standard_frame(0x600u + CANopen_ID, data, 5u);
}

void SDO_Write_Data2(uint8_t CANopen_ID, uint16_t Index, uint8_t SubIndex, uint16_t DATA)
{
    uint8_t data[6];

    data[0] = 0x2b;
    data[1] = (uint8_t)(Index & 0xFFu);
    data[2] = (uint8_t)((Index >> 8) & 0xFFu);
    data[3] = SubIndex;
    data[4] = (uint8_t)(DATA & 0xFFu);
    data[5] = (uint8_t)((DATA >> 8) & 0xFFu);

    canopen_send_standard_frame(0x600u + CANopen_ID, data, 6u);
}

void SDO_Write_Data4(uint8_t CANopen_ID, uint16_t Index, uint8_t SubIndex, int32_t DATA)
{
    uint8_t data[8];

    data[0] = 0x23;
    data[1] = (uint8_t)(Index & 0xFFu);
    data[2] = (uint8_t)((Index >> 8) & 0xFFu);
    data[3] = SubIndex;
    data[4] = (uint8_t)(DATA & 0xFF);
    data[5] = (uint8_t)((DATA >> 8) & 0xFF);
    data[6] = (uint8_t)((DATA >> 16) & 0xFF);
    data[7] = (uint8_t)((DATA >> 24) & 0xFF);

    canopen_send_standard_frame(0x600u + CANopen_ID, data, 8u);
}

void SDO_Read_Data4(uint8_t CANopen_ID, uint16_t Index, uint8_t SubIndex)
{
    uint8_t data[5];

    data[0] = 0x40;
    data[1] = (uint8_t)(Index & 0xFFu);
    data[2] = (uint8_t)((Index >> 8) & 0xFFu);
    data[3] = SubIndex;
    data[4] = 0x00;

    canopen_send_standard_frame(0x600u + CANopen_ID, data, 5u);
}

void PDO_Write_Data6(uint8_t CANopen_ID, uint16_t rpdo1, int32_t rpdo2)
{
    uint8_t data[6];

    data[0] = (uint8_t)(rpdo1 & 0xFFu);
    data[1] = (uint8_t)((rpdo1 >> 8) & 0xFFu);
    data[2] = (uint8_t)(rpdo2 & 0xFF);
    data[3] = (uint8_t)((rpdo2 >> 8) & 0xFF);
    data[4] = (uint8_t)((rpdo2 >> 16) & 0xFF);
    data[5] = (uint8_t)((rpdo2 >> 24) & 0xFF);

    canopen_send_standard_frame(0x200u + CANopen_ID, data, 6u);
}
