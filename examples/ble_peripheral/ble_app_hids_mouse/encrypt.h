#ifndef __ENCRYPT_H__
#define __ENCRYPT_H__

//#include "puart.h"

	uint8_t* encrypt(const uint8_t* source, const uint8_t* pass);
	uint8_t checksum(const uint8_t* data, uint32_t len);

#endif
