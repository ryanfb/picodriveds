/*******************************************************
	Modified by Rudolph (鼓碾):
	2006/11/24
		* Improvement in speed of Write routine
		* Improvement in reliability of reading data
	2006/11/20
		* Added write routine support

 Original is EZ4FATLIB code offered by EZTEAM.
 Hardware Routines for SD card using a EZ4 series.

********************************************************/

#include <nds.h>
#include "stdio.h"
#include "string.h"


#include "io_ezsd_asm.h"

u32 SDadd;

void  SD_SendCommand (int type , unsigned int param)
{
	unsigned char ppbuf[8];
	unsigned char crc = 0 ;

	ppbuf[0] = 0x40 | (type&0xFF) ;
	ppbuf[1] = (param>>24)&0xFF ;
	ppbuf[2] = (param>>16)&0xFF ;
	ppbuf[3] = (param>>8)&0xFF ;
	ppbuf[4] = (param)&0xFF ;
	crc = crc_730(ppbuf,5);
	ppbuf[5] = (crc<<1)|0x1 ;
	SD_WriteBufferToLine(ppbuf,6);
}

bool	SD_SendCommandRS (int type, unsigned int param)
{
	u8	p[16];
	int	rtry;
	bool	ret;

	rtry = 0x1000;
	do {
		SD_SendCommand (type, param);
		ret = SD_ReadResponse(p, 6, 0x100);
		if(ret && (p[0] != type))
			ret = false;
		rtry--;
	} while(rtry && (!ret));

	if(rtry)	return true;
	return false;
}



bool 	SD_R16Response(unsigned char *ppbuf, int wait)
{
	return SD_ReadResponse(ppbuf, 6, 0x800);
}

bool 	SD_R2Response(unsigned char *ppbuf, int wait)
{
	return SD_ReadResponse(ppbuf, 17, 0x1000);
}

bool	SD_R3Response(unsigned char *ppbuf, int wait)
{
	if(SD_ReadResponse(ppbuf, 6, 0x200))
	{
		if((ppbuf[0]==0x3F)||(ppbuf[5]==0xFF))
			return true ;
	}
	return false ;
}



bool 	SD_ReadSingleBlock(unsigned int address , unsigned char *ppbuf, int len)
{
	bool	ret, rd;
	u8	pbuf[528];
	u8	crc[8];
	int	i, k;

	rd = false;
	do {
		ret = SD_SendCommandRS(17, address);	// single block read , parm = address
		if(ret) {
			SD_ReadData(pbuf, 520, 0x10000);
			for(i = 0; i < 10; i++)
				k = *(vu16*)0x9fea000;
			SD_cal_crc16(pbuf, 512, crc);
			if((*(vu32*)crc == *(vu32*)(pbuf+512)) && (*(vu32*)(crc+4) == *(vu32*)(pbuf+512+4)))
				rd = true;
		}
	} while(ret && (!rd));

	if(rd)	memcpy(ppbuf, pbuf, 512);

	return ret;
}


bool	SD_WriteSingleBlock(unsigned int address , unsigned char *ppbuf, int len)
{
	bool	ret, wr;
	u8	pbuf[528];
	int	i;
	u16	k[16];

	memcpy(pbuf, ppbuf, 512);
	SD_cal_crc16(pbuf, 512, pbuf + 512);

	wr = false;
	do {
		ret = SD_SendCommandRS(24, address);
		if(ret) {
			SD_WriteData(pbuf, 520, 0x10000);
			for(i = 0; i < 10; i++)
				k[i] = *(vu16*)0x9fea000;
			if(!(k[3] & 0x100) && (k[4] & 0x100) && !(k[5] & 0x100))
				wr = true;
		}
	} while(ret && (!wr));

	SD_SendCommandRS(13,(SDadd<<16));

	return ret;
}


void	SD_ReadLoop(int lp)
{
	int i ,k;
	for(i=0;i<lp;i++)
		k = *(vu16*)0x9FFFF40 ;
}


void	 SD_Enable()
{
	WAIT_CR &= ~0x80;
	*(vu16*)0x9fe0000 = 0xd200;
	*(vu16*)0x8000000 = 0x1500;
	*(vu16*)0x8020000 = 0xd200;
	*(vu16*)0x8040000 = 0x1500;
	*(vu16*)0x9400000 = 1;
	*(vu16*)0x9C40000 = 0x1500;
	*(vu16*)0x9fc0000 = 0x1500;
}

void	SD_Disable()
{
	*(vu16*)0x9fe0000 = 0xd200;
	*(vu16*)0x8000000 = 0x1500;
	*(vu16*)0x8020000 = 0xd200;
	*(vu16*)0x8040000 = 0x1500;  
	*(vu16*)0x9400000 = 0;
	*(vu16*)0x9C40000 = 0xd200;
	*(vu16*)0x9fc0000 = 0x1500;
  	WAIT_CR |= 0x80;
}

bool SD_initial()
{
	bool  ret ;
	unsigned char pres[32] ;

	SD_ReadLoop(147);
	SD_SendCommandRS(0,0);
	
	SD_SendCommand(55,0);			// 指示下一个命令是app命令
	ret = SD_R16Response(pres,0);
	if(pres[0]!=55 || !ret)
		return false; 

	ret = SD_SendCommandRS(41,0x0); 	//检测电压范围
	do
	{
		ret = SD_SendCommandRS(55,0);
		SD_SendCommand(41,0xFC0000);
		ret = SD_R3Response(pres,0);
	}while(pres[1]!=0x80) ;
	
	SD_SendCommand(2,0);			//Get CID 
	ret = SD_R2Response(pres,0);
//	SD_GetCIDStruct(pres,17,cid);

	do
	{
		SD_SendCommand(3,0);		//进入stand by 状态，并得到sd卡状态
		ret = SD_R16Response(pres,0);
	} while((pres[3] & 0x1E) != 0x6); 		//stand by state
	
	SDadd = pres[1] * 0x100 + pres[2] ;

/***********
	if(ret)
	{
		SDadd = pres[1]*0x100 + pres[2] ;
		SD_SendCommand(9,(SDadd<<16));//send_csd
		ret = SD_R2Response(pres);
		SD_GetCSDStruct(pres,17,csd);
	}
************/

	ret = SD_SendCommandRS(7,SDadd<<16);	//select card

	ret = SD_SendCommandRS(55,SDadd<<16); 	//app command

	ret = SD_SendCommandRS(6,2);		//00, 1 bit , 10  4 bit 

	ret = SD_SendCommandRS(16,0x200) ;	//设定一个block为512大小

	return ret ;
}

