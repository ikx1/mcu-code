#ifndef __CANOPEN_PROTO_H__
#define __CANOPEN_PROTO_H__

#include <stdint.h>

void Control_Mode_SET(uint8_t CANopen_ID, uint8_t CANopen_mode);
void CANopen_NMT(uint8_t cmd, uint8_t CANopen_ID);

void SDO_Write_Data1(uint8_t CANopen_ID, uint16_t Index, uint8_t SubIndex, uint8_t DATA);
void SDO_Write_Data2(uint8_t CANopen_ID, uint16_t Index, uint8_t SubIndex, uint16_t DATA);
void SDO_Write_Data4(uint8_t CANopen_ID, uint16_t Index, uint8_t SubIndex, int32_t DATA);
void SDO_Read_Data4(uint8_t CANopen_ID, uint16_t Index, uint8_t SubIndex);

void PDO_Write_Data6(uint8_t CANopen_ID, uint16_t rpdo1, int32_t rpdo2);

#endif /* __CANOPEN_PROTO_H__ */
