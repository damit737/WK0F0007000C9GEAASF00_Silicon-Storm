#include <gui/model/Model.hpp>
#include <gui/model/ModelListener.hpp>

#include <ff.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int Do1Flag = 0;
uint8_t num ;

Model::Model() : modelListener(0)
{

}

void Model::tick()
{

	if(Do1Flag == 0)
	{
		if(HAL_GPIO_ReadPin(SD_CD_GPIO_Port,SD_CD_Pin) == 0)
		{
			Model::ReadSD();

		}
		Do1Flag = 1;
	}
}

void Model::ReadSD()
{
	Data_t Data = {0};

	FATFS SDFatFS;
	FIL SDFile;
	FILINFO fno2;
	FRESULT fr1;
	UINT br1;
	f_mount(&SDFatFS, "", 0);
	fr1 = f_stat("0:/elevator.txt", &fno2);
	if(fr1 == FR_OK)
	{
		f_open(&SDFile, "0:/elevator.txt", FA_OPEN_EXISTING | FA_READ);
		f_read(&SDFile, (uint8_t *)&Data, sizeof(Data), &br1);
		modelListener->notifyEvent(Data);
		f_close(&SDFile);
		f_mount(NULL,"",0);
	}
	else
	{
		f_mount(NULL,"",0);
	}

}
