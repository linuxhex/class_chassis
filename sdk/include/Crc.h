#ifndef _CRC_KYOSHO_20150320_
#define _CRC_KYOSHO_20150320_




unsigned short Utils_get_crc(unsigned char *puchMsg , unsigned short usDataLen);

bool Utils_is_crc_good( unsigned char *pushMsg, int usDataLen );

#endif//_CRC_KYOSHO_20150320_
