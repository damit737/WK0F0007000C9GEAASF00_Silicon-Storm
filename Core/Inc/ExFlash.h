#ifndef __EXFLASH_IF_H
#define __EXFLASH_IF_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include "qspi.h"
	 
/* Error codes */
#define MEMORY_OK          ((uint32_t)0x00)
#define MEMORY_ERROR       ((uint32_t)0x01)
	 
#define External_Flash_Offset     0x90000000
#define Application_HeaderAddress 0x90000200
#define Application_CodeAddress   0x90000000

typedef struct{

  uint32_t crcFile;
  uint32_t fileType;
  uint32_t fileAddress;
  uint32_t dummy1;
  uint32_t dummy2;
  uint32_t dummy3;
  uint32_t fileSize;
  uint32_t crcHeader;

}sFileHeader_t;

typedef struct{
	
	enum{
		no_flash = 0,
		w25q128,
		mx25l25645g,
	}type;
	
	struct{
		uint32_t page;
		uint32_t subsector;
		uint32_t sector;
		uint32_t total;
	}size;
	
}flash_info_t;	 

void exflash_init ( void );
uint8_t ExFlash_Erase_4K ( uint32_t sector );
uint8_t ExFlash_Erase_64K ( uint32_t sector );
uint8_t ExFlash_Write ( uint32_t dstAddr, uint32_t srcAddr, uint32_t len );

extern flash_info_t flash_info;

#ifdef __cplusplus
}
#endif

#endif /* __EXFLASH_IF_H */
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
