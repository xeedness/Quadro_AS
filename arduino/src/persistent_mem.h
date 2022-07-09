#ifndef PERSISTENT_MEM_H_
#define PERSISTENT_MEM_H_

#include "ASF/sam/services/flash_efc/flash_efc.h"

#define FLASH_SIZE			(0x00080000)
#define FLASH_START_ADDR	(0x00080000)
#define MAX_PROG_SIZE		(0x0007F000)

#define PARAM_START_ADDR	(FLASH_START_ADDR + MAX_PROG_SIZE)
#define MAX_PARAM_SIZE		(FLASH_SIZE - MAX_PROG_SIZE)


uint32_t store_data(const uint8_t* pucDataSrc, size_t xSize);
uint32_t read_data(const uint8_t* pucDataDst, size_t xSize);

#endif /* PERSISTENT_MEM_H_ */