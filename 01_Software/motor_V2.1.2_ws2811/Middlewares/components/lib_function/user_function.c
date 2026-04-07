#include <stdint.h>
#include <string.h>
#include <math.h>
#include "FreeRTOS.h"

#include "user_function.h"


//void *rt_calloc(size_t count,size_t size)
//{
//    void *p;
//    
//    p = pvPortMalloc(count * size);
//    
//    if(p != NULL)
//    {
//        memset(p,0, count * size);
//    }
//    
//    return p;
//}

uint8_t Serial_checksum(uint8_t *data, uint8_t len)
{
	uint8_t checksum = 0x00;
	for(uint8_t i = 0; i < (len-1); i++)
	{
		checksum += data[i];
	}
	
	return checksum;
}

unsigned char BCDtoHex(unsigned char  bcd)
{
    return (bcd & 0x0f) + (bcd >> 4) * 10;
}

unsigned char HextoBCD(unsigned char hex)
{
    if (hex > 99)
        return 0;
    
    return ((hex / 10) << 4) + hex % 10;
}

unsigned short HexToBcdForTwoByte(unsigned short hex)
{ 
    return ((HextoBCD(hex / 100) << 8) + HextoBCD(hex % 100));
}

unsigned short BcdToHexForTwoByte(unsigned short bcd)
{ 
    return (100 * BCDtoHex(bcd >> 8) + BCDtoHex(bcd & 0xff));
}

void bcd_to_hex_for_bytes(const void *p_src, void *p_dst, unsigned char size)
{
    unsigned char i;
    unsigned long long hex = 0;

    for (i=size; i>0; i--)
        hex += (unsigned long long)BCDtoHex(((unsigned char*)p_src)[i-1]) * pow(10, 2*(i-1));
    
    memcpy((unsigned char*)p_dst, &hex, size);
}

void hex_to_bcd_for_bytes(const void *p_src, void *p_dst, unsigned char size)
{ 
    unsigned char i = 0, j = 0;
    unsigned long long src = 0, bcd = 0;

    memcpy(&src, p_src, size);
    
    for (i=size; i>0; i--)
    {
        bcd |= (unsigned long long)HextoBCD(src % 100) << ((j++) * 8);
        src /= 100;
    }
    
    memcpy(p_dst, &bcd, size);
}









