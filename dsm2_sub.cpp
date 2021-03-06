#include "predef.h" 
#include <ctype.h>
#include <basictypes.h>
#include <system.h>
#include <constants.h>
#include <ucos.h>
#include <ucosmcfc.h>
#include <serialirq.h>
#include <stdio.h>
#include <string.h>
#include "dsm2_sub.h"
#include "sensor_config.h"
#include "car.h"


#define SM_TASK_STK (256)
#define SmOSSimpleTaskCreate(x,p) { static DWORD   func_##x_Stk[SM_TASK_STK] __attribute__( ( aligned( 4 ) ) ); OSTaskCreate(x,NULL,(void *)&func_##x_Stk[SM_TASK_STK],(void*)func_##x_Stk,p); }

OS_SEM * TheLocalSem;

extern volatile DWORD Pit_Count;
volatile DSM2_READING DSM2_Result;

DWORD LastPit;
volatile DWORD rcrx;

volatile bool bMode;
volatile int nMode;
volatile bool bGear;


static unsigned char RCBuf[64];
static unsigned char  nc;

void ParseDSMChar(char c)
{
rcrx++;
if ((LastPit+4)<=Pit_Count) nc=0;

LastPit=Pit_Count;

RCBuf[nc++]=c;

if(nc==16)
	{
	OSLock();
	  for(int i=1; i<8; i++)
	  {
		WORD w=RCBuf[i*2];
		w=w<<8;
		w|=RCBuf[(i*2)+1]; 
		int ch=(w>>10) & 0x07;
		DSM2_Result.val[ch]=(w& 0x3FF);

	  }
	  if(DSM2_Result.val[5]<400) nMode=0;
	  else if(DSM2_Result.val[5]<600) nMode=1;
	  else
	  nMode=2;
	  bGear=(DSM2_Result.val[4]>500);


	  bMode=(DSM2_Result.val[6]>500);
	  DSM2_Result.ReadingNum++;

    OSUnlock();
	if (TheLocalSem) {OSSemPost(TheLocalSem);}
	nc=0;
	}
if(nc>=19) nc=18;
}




void DSM2Task( void * pd)
{
SimpleUart( DSMUART, 125000 ); 

while(1)
{char c=sgetchar(DSMUART);
    ParseDSMChar(c);
}

}


void InitIDSM2SubSystem(WORD prio,OS_SEM * pDataSem)
{
TheLocalSem=pDataSem;
SmOSSimpleTaskCreate(DSM2Task,prio);
}




float DSM_Con(unsigned short v)
{
float f=v;
f-=512;
f*=(1.0/512);
return f;
}
