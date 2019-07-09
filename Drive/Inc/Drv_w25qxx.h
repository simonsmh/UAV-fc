/******************** (C) COPYRIGHT 2018 DY EleTe ********************************
 * ����    �����е��
 * ����    ��www.gototi.com
 * ����    ������FLASH�洢оƬ����
**********************************************************************************/
#ifndef _DRIVER_AT45DB_H_
#define _DRIVER_AT45DB_H_

#include "common.h"
#include "BSP_Init.h"

/* Jedec Flash Information structure */
typedef struct
{
    uint8_t initialized;
    uint16_t sector_size;
    uint16_t sector_count;
    uint32_t capacity;
} flash_info_t;

void Flash_Init(void);
void Flash_SectorErase(uint32_t address,uint8_t state);
void Flash_PageRead(uint32_t address,uint8_t* buffer,  uint32_t lenght);
void Flash_PageWrite(uint32_t address,uint8_t* buffer,  uint32_t lenght);
void Flash_SectorsRead(uint32_t address,uint8_t *buffer,uint16_t count);
void Flash_SectorsWrite(uint32_t address,uint8_t *buffer,uint16_t count);
flash_info_t *Flash_GetInfo(void);

#endif

/******************* (C) COPYRIGHT 2018 DY EleTe *****END OF FILE************/
