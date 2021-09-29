//#include "encrypt.h"
#include <stdint.h>
#include <string.h>

uint8_t tmp_str[6];

uint8_t checksum(const uint8_t* data, uint32_t len)
{
		int i;
    uint8_t sum = 0;
    
    for(i=0; i<len; i++)
    {
        sum += data[i];
    }

    return -sum; // 2's complement of all the bytes
}


uint8_t* encrypt(const uint8_t* source, uint8_t* pass)
{
	uint8_t i = 0;
	//uint8_t tmp_str[6];
	int source_length = 6;
	int pass_length = 6;
	
	//printk("bd_address=%02x %02x %02x %02x %02x %02x\r\n",*source,*(source+1),*(source+2),*(source+3),*(source+4),*(source+5));

	memset(tmp_str,0,source_length);

	for(i=0;i<source_length;++i)
	{
		tmp_str[i]=source[i]^pass[i%pass_length];
	}
	//printk("encrypt:%02x %02x %02x %02x %02x %02x\r\n",tmp_str[0],tmp_str[1],tmp_str[2],tmp_str[3],tmp_str[4],tmp_str[5]);
	return tmp_str;
}
