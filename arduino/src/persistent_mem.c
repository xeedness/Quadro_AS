#include <string.h>
#include "persistent_mem.h"

#define NO_ERASE_FLAG	(0)
#define ERASE_FLAG		(1)

static bool bInitialized = false;

uint32_t store_data(const uint8_t* pucDataSrc, size_t xSize)
{
	//if(bInitialized == false)
	//{
		//uint32_t res = flash_init(FLASH_ACCESS_MODE_128, 6);
		//if(res == FLASH_RC_OK)
		//{
			//bInitialized = true;
		//}
	//}
	if(xSize > MAX_PARAM_SIZE)
	{
		return FLASH_RC_INVALID;
		// Bloﬂ keine exakte Fehlermeldung
	}
	return flash_write(PARAM_START_ADDR, (const void*)pucDataSrc, (uint32_t)xSize, ERASE_FLAG);
}

uint32_t read_data(const uint8_t* pucDataDst, size_t xSize)
{
	if(xSize > MAX_PARAM_SIZE)
	{
		return FLASH_RC_INVALID;
		// Bloﬂ keine exakte Fehlermeldung
	}
	
	memcpy((void*)pucDataDst, (void*)PARAM_START_ADDR, xSize);
	// Passt schon
	return FLASH_RC_OK;
}