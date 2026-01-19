#ifndef __USER_FUNCTION_H
#define __USER_FUNCTION_H

uint8_t Serial_checksum(uint8_t *data, uint8_t len);

unsigned char BCDtoHex(unsigned char  bcd);
unsigned char HextoBCD(unsigned char hex);
unsigned short HexToBcdForTwoByte(unsigned short hex);
unsigned short BcdToHexForTwoByte(unsigned short bcd);
void bcd_to_hex_for_bytes(const void *p_src, void *p_dst, unsigned char size);
void hex_to_bcd_for_bytes(const void *p_src, void *p_dst, unsigned char size);


#endif /*__USER_FUNCTION_H*/



