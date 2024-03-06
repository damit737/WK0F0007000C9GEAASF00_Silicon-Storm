/* USER CODE BEGIN Header */

/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __EXHAREWARE_H
#define __EXHAREWARE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

typedef enum{
	BL_ON = 0x00,
	BL_OFF,
}BACKLIGHT_CONTROL;

void Backlight_Control ( BACKLIGHT_CONTROL BL_Ctrl );
void Set_Backlight_Duty ( uint8_t d );

#ifdef __cplusplus
}
#endif

#endif /* __EXHAREWARE_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
