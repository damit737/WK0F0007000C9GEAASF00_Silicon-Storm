#include <ExFlash.h>
#include "qspi.h"
#include "string.h"

flash_info_t flash_info = {
	.type = no_flash,
	.size.page = 0,
	.size.subsector = 0,
	.size.sector = 0,
	.size.total = 0,
};

extern QSPI_HandleTypeDef hqspi;

void exflash_init ( void )
{
	uint8_t buf[ 3 ];
	
	HAL_QSPI_DeInit( &hqspi );
	BSP_QSPI_Init( );
	
	if( BSP_QSPI_Reset() == QSPI_ERROR ) {

    goto fail;
  }
	
	if( BSP_QSPI_Read_ID( buf ) != QSPI_OK ) // read external flash ID
		goto fail;
	
	// W25Q128FV(SPI)
	// Maunfacturer and Device Identification
	// +----------------------------------------------------------+
	// | Maunfacturer ID | 		ID15 - ID8	  |		ID7 - ID0	  |
	// +----------------------------------------------------------+
	// |		EFh		 | 		  40h		  | 	  18h		  |
	// +----------------------------------------------------------+
	//
	//
	// MX25L25645G
	// ID Definitions
	// +----------------------------------------------------------+
	// | Maunfacturer ID | 	  Memory Type	  |  Memory Density   |
	// +----------------------------------------------------------+
	// |		C2h		 | 	 	   20h        | 		19h	      |
	// +----------------------------------------------------------+
	
	if( memcmp( buf, "\xEF\x40\x18", 3 ) == 0 )
	{
		flash_info.type 			= w25q128;
		flash_info.size.page 		= 0x100;
		flash_info.size.subsector 	= 0x1000;
		flash_info.size.sector 		= 0x10000;
		flash_info.size.total 		= 0x1000000;		
	
		HAL_QSPI_DeInit( &hqspi );
		
		BSP_QSPI_Init();
		
		Check_Status_Register( );
		
		if( BSP_QSPI_EnableMemoryMappedMode( ) == QSPI_OK )
			return;
	}
	else if( memcmp( buf, "\xC2\x20\x19", 3 ) == 0 )
	{
		flash_info.type 			= mx25l25645g;
		flash_info.size.page 		= 0x100;
		flash_info.size.subsector 	= 0x1000;
		flash_info.size.sector 		= 0x10000;
		flash_info.size.total 		= 0x2000000;		
	
		HAL_QSPI_DeInit( &hqspi );
		
		BSP_QSPI_Init( );
				
		if( BSP_QSPI_EnterQPIMode( ) != QSPI_OK )
			goto fail;
		
		if( BSP_QSPI_Enter4ByteMode( ) != QSPI_OK )
			goto fail;
		
		if( BSP_QSPI_EnableMemoryMappedMode( ) == QSPI_OK )
			return;
	}
	
	else			
	{
		flash_info.type = no_flash;
	}
	
	fail: 
	
	/**
	 * show flash error msg
	 **/
  
		
	while( 1 );
}

uint8_t ExFlash_Erase_4K ( uint32_t sector )
{	
	HAL_QSPI_DeInit( &hqspi );
	BSP_QSPI_Init( );
	
	BSP_QSPI_Erase_Sector( sector * flash_info.size.subsector );
	
	BSP_QSPI_EnableMemoryMappedMode( );

	return MEMORY_OK;
}

uint8_t ExFlash_Erase_64K ( uint32_t sector )
{
	HAL_QSPI_DeInit( &hqspi );
	BSP_QSPI_Init( );
	
	BSP_QSPI_Erase_Block( sector * flash_info.size.subsector );
	
	BSP_QSPI_EnableMemoryMappedMode( );

	return MEMORY_OK;
}

uint8_t ExFlash_Write ( uint32_t dstAddr, uint32_t srcAddr, uint32_t len )
{
	uint8_t return_v = MEMORY_OK;
	
	HAL_QSPI_DeInit( &hqspi );
	BSP_QSPI_Init( );
	
	if( BSP_QSPI_Write( ( uint8_t* )srcAddr, dstAddr, len ) == QSPI_ERROR )
	{
		return_v = MEMORY_ERROR;
	}
	
	BSP_QSPI_EnableMemoryMappedMode( );
	
	return return_v;
}
