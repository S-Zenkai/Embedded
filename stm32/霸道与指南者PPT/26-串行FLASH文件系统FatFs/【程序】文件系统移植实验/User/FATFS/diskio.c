/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2014        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control modules to the FatFs module with a defined API.       */
/*-----------------------------------------------------------------------*/

#include "diskio.h"		/* FatFs lower layer API */
#include "./flash/bsp_spi_flash.h"
//#include "usbdisk.h"	/* Example: Header file of existing USB MSD control module */
//#include "atadrive.h"	/* Example: Header file of existing ATA harddisk control module */
//#include "sdcard.h"		/* Example: Header file of existing MMC/SDC contorl module */

/* Definitions of physical drive number for each drive */
#define ATA		0	/* Example: Map ATA harddisk to physical drive 0 */
#define MMC		1	/* Example: Map MMC/SD card to physical drive 1 */
#define USB		2	/* Example: Map USB MSD to physical drive 2 */


#define SD_CARD   0
#define SPI_FLASH 1


/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
	BYTE pdrv		/* Physical drive nmuber to identify the drive */
)
{
	DSTATUS stat;
//	int result;

	switch (pdrv) {
	case SD_CARD :
//		result = ATA_disk_status();

		// translate the reslut code here

		return stat;

	case SPI_FLASH :		
	
		if (SPI_FLASH_ReadID() == sFLASH_ID)
		{
			//状态正常
			stat = 0;
		}
		else
		{
			//状态不正常
			stat = STA_NOINIT;		
		}

		return stat;
	}
	return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
	BYTE pdrv				/* Physical drive nmuber to identify the drive */
)
{
	DSTATUS stat;
//	int result;

	switch (pdrv) {
	case SD_CARD :
//		result = ATA_disk_initialize();

		// translate the reslut code here

		return stat;

	case SPI_FLASH :
		
		SPI_FLASH_Init();
		SPI_Flash_WAKEUP();
//		result = MMC_disk_initialize();
	
		return  disk_status(SPI_FLASH);

	}
	return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber to identify the drive */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	/* Sector address in LBA */
	UINT count		/* Number of sectors to read */
)
{
	DRESULT res;
//	int result;

	switch (pdrv) {
	case SD_CARD :
		// translate the arguments here

//		result = ATA_disk_read(buff, sector, count);

		// translate the reslut code here

		return res;

	case SPI_FLASH :
		/* 扇区偏移2MB，外部Flash文件系统空间放在SPI Flash后面6MB空间 */
		sector += 512;
		SPI_FLASH_BufferRead(buff,sector*4096,count*4096);

		res = RES_OK;
	
		return res;

	}

	return RES_PARERR;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if _USE_WRITE
DRESULT disk_write (
	BYTE pdrv,			/* Physical drive nmuber to identify the drive */
	const BYTE *buff,	/* Data to be written */
	DWORD sector,		/* Sector address in LBA */
	UINT count			/* Number of sectors to write */
)
{
	DRESULT res;
//	int result;

	switch (pdrv) {
	case SD_CARD :
		// translate the arguments here

//		result = ATA_disk_write(buff, sector, count);

		// translate the reslut code here

		return res;

	case SPI_FLASH :
		/* 扇区偏移2MB，外部Flash文件系统空间放在SPI Flash后面6MB空间 */
		sector += 512;
		
		//一定要先擦除再写入
		SPI_FLASH_SectorErase(sector*4096);
		SPI_FLASH_BufferWrite((u8 *)buff,sector*4096,count*4096);

		res = RES_OK;
	
		return res;


	}

	return RES_PARERR;
}
#endif


/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

#if _USE_IOCTL
DRESULT disk_ioctl (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
	DRESULT res;

	switch (pdrv) {
	case SD_CARD :

		// Process of the command for the ATA drive

		return res;

	case SPI_FLASH :
		
		switch(cmd)
		{
			//返回扇区个数
			case GET_SECTOR_COUNT:
				/* 扇区数量：1536*4096/1024/1024=6(MB) */
				*(DWORD*)buff = 1536;
				
			break;
			//返回每个扇区大小
			case GET_SECTOR_SIZE:
				*(WORD*)buff = 4096;

			break;
			//返回擦除扇区的最小个数（单位为扇区）
			case GET_BLOCK_SIZE:
			 *(WORD*)buff = 1;
			
			break;
			
		}		
			

		// Process of the command for the MMC/SD card
		res = RES_OK;
		return res;

	}

	return RES_PARERR;
}
#endif

//返回时间
DWORD get_fattime (void)
{

	return 0;
}
