/* Rev:$Revision: 1.1 $ */
/*******************************************************************************
 *   Copyright 2009 NetBurner, Inc.  ALL RIGHTS RESERVED
 *
 *   Permission is hereby granted to purchasers of NetBurner hardware to use or
 *   modify this computer program for any use as long as the resultant program
 *   is only executed on NetBurner provided hardware.
 *
 *   No other rights to use this program or its derivatives in part or in whole
 *   are granted.
 *
 *   NetBurner makes no representation or warranties with respect to the
 *   performance of this computer program, and specifically disclaims any
 *   responsibility for any damages, special or consequential, connected with
 *   the use of this program.
 *
 *   NetBurner, Inc.
 *   5405 Morehouse Drive
 *   San Diego, CA  92121
 *   http://www.netburner.com
 *
 ******************************************************************************/
/*------------------------------------------------------------------------------
 * This is an example of a minimal network application for the SBL2e.
 *----------------------------------------------------------------------------*/
#include "predef.h"
#include <basictypes.h>         // Include for variable types
#include <serialirq.h>         // Use UART interrupts instead of polling
#include <constants.h>          // Include for constands like MAIN_PRIO
#include <system.h>             // Include for system functions
#include <netif.h>
#include <ip.h>
#include <ethernet_diag.h>
#include <autoupdate.h>
#include <i2cmaster.h>
#include <sim.h>
//#include <syslog.h>
#include <taskmon.h>
#include <math.h>

#include "imu.h"
#include "servo.h"
#include "dsm2_sub.h"
#include "sensor_config.h"
#include "gps.h"
#include "pitr_sem.h"
#include "log.h"
#include "filereporter.h"
#include "prio.h"
#include "car.h"


extern "C"
{
   void UserMain(void *pd); // prevent C++ name mangling
}

const char * AppName = "SBL2e I2C Test";



extern IPADDR ipa_syslog_addr;


#define STEER_CH (3)
#define THROTTLE_CH (2)
float fimHeading;
float figHeading;
short gzoff;
 


volatile DWORD UdpRx;
volatile char UdpChar;

 int MyUDPProcessFunction( PEFRAME pf)
 {
 PUDPPKT pUdpPkt = GetUdpPkt( pf );
 PIPPKT pIp = GetIpPkt( pf );

	if ( pUdpPkt->dstPort == 1000 )
	{
	 if(ipa_syslog_addr==0xFFFFFFFF)
		{
		 ipa_syslog_addr=pIp->ipSrc;
		}
		UdpRx++;
		UdpChar=pUdpPkt->DATA[0]; 
		return 1;
	}

	return 0;
 }


 void UdpCheck()
 {DWORD u=UdpRx;
 while(u==UdpRx) OSTimeDly(1);
 }


 OS_SEM DataSem;


extern  volatile  WORD llhcnt;
extern volatile  WORD IdCnt;
extern volatile  WORD VelCnt;
extern volatile DWORD rcrx;
extern volatile int maxn;

const int maxmx=13056;
const int minmx=-32257;
const int maxmy=21504;
const int minmy=-23553;



float CalcMagHeading(short mx, short my)
{
float x=(mx-(maxmx+minmx)/2);
x*=2.0*(1.0/(maxmx-minmx));

float y=(my-(maxmy+minmy)/2);
y*=2.0*(1.0/(maxmy-minmy));

return (180.0*atan2(x,y)/M_PI);
}


void CorrectHeading(float & fhead, float adjv, float adj_scale)
{

float err=adjv-fhead;	   //(-144.0,0)	->-144 
if (err > 180.0) err-=360.0;
if (err < -180.0) err+=360.0;
fhead+=err*adj_scale;

}

const float fGZHeadingScale_deg = (-3.8147E-05);  //(250 dps /(200 sps 32768 fs)

volatile float Target_Heading;
volatile float Offset_Heading;


//AdjustVariable SteerGain("Steer P",0.033);
//AdjustVariable SteerDGain("Steer D",-0.0001);

const double SteerGain= (0.033);
const double SteerDGain=-0.0001; 

void DoSteer()
{
SteerLoopMsg slp;
slp.deg_heading=figHeading; 
slp.adj_heading=Target_Heading+Offset_Heading;
slp.err=(slp.deg_heading-slp.adj_heading);
if(slp.err>180.0) slp.err-=360;
if(slp.err<-180.0) slp.err+=360.0;

slp.steer=(slp.err*(double)SteerGain)+((double)SteerDGain*(double)IMU_Result.gz-gzoff);
if(slp.steer>1.0) slp.steer=1.0;
if(slp.steer<-1.0) slp.steer=-1.0;

SetServo(STEER_CH,slp.steer*STEER_SIGN);
LogRecord(slp);

}

void HeadLimit(float & f)
{
  while (f>360.0) f-=360.0;
  while (f<0.0) f+=360.0;
}

/*-------------------------------------------------------------------
 * UserMain
 *-----------------------------------------------------------------*/
void UserMain(void *pd)
{
	SimpleUart(0,115200);
	assign_stdio(0);

	iprintf("Before init stack\r\n");
    
	InitializeStack();

    {
        WORD ncounts = 0;


        while ( ( !bEtherLink ) && ( ncounts < 2*TICKS_PER_SECOND ) )
        {
            ncounts++;
            OSTimeDly( 1 );
        }
    }

    EnableAutoUpdate();
	EnableTaskMonitor(); 


    OSChangePrio( MAIN_PRIO ); // set standard UserMain task priority

	//BcastSysLogPrintf("Application built on %s on %s\r\nWait...", __TIME__, __DATE__ );

	pUserUdpProcessFunction=MyUDPProcessFunction;

//	UdpCheck();
	
   
	OSSemInit(&DataSem,0);


	   									 

	LoadSensorConfig();
	   // BcastSysLogPrintf("1");
	
	InitLog();

	ImuInit(IMU_PRIO,&DataSem);
	

	InitIDSM2SubSystem(RC_PRIO,&DataSem); 

	
	InitGpsSubSystem(GPS_PRIO,&DataSem);

	
	InitServos();


    SetServo(0,0);
	SetServo(1,0);
	SetServo(2,0);
	SetServo(3,0);

	WORD LastGps=GPS_Result.ReadingNum;
	WORD LastRc =DSM2_Result.ReadingNum;
	WORD LastImu =IMU_Result.ReadingNum;
	DWORD LSecs=Secs;

	static short mmx;
	static short mmy;
	static short nmx;
	static short nmy;
	static short mgz;
    static short ngz;

	DWORD ZeroTime=Secs+5;
	long GZSum=0;
	int  GZCnt=0;
	float fmh=0.0;

	while(ZeroTime > Secs)
	{
		OSSemPend(&DataSem,20);
		if(LastImu !=IMU_Result.ReadingNum)
			{
			GZSum+=IMU_Result.gz;
			GZCnt++;
			LastImu =IMU_Result.ReadingNum; 

			if (IMU_Result.mx !=0 ) 
				fmh=CalcMagHeading(IMU_Result.mx,IMU_Result.my);
				

            }


	}
	
   gzoff=(GZSum/GZCnt);
	fimHeading=fmh;
	figHeading=fmh;


	while(1)
	 {

	OSSemPend(&DataSem,20);
	if (LastGps!=GPS_Result.ReadingNum ) 
	{
		LastGps=GPS_Result.ReadingNum;
	    LogSmGps(GPS_Result); 
		//LogTGps(bd960_Result);
	}
	
	if (LastRc !=DSM2_Result.ReadingNum)
	{
	 WORD w;
	 float f;
	  

	  if(bMode)
	  {
	   if(nMode==0) Target_Heading=50.0;
	   if(nMode==1) Target_Heading=320.0;
	   if(nMode==2) Target_Heading=230.0;
		  DoSteer();
	  }
	  else
	  {
	  w=DSM2_Result.val[1];
	  f=DSM_Con(w);
	  SetServo(STEER_CH,f*STEER_SIGN);

	  }
	   w=DSM2_Result.val[0];
	  f=DSM_Con(w);
	  SetServo(THROTTLE_CH,f);
      LastRc =DSM2_Result.ReadingNum;
	//  LogRC(DSM2_Result);
	}



	
	if(LSecs!=Secs) 
		{
		// BcastSysLogPrintf("Tick %d Iat:%ld lon:%ld SAT:%d\r\n",Secs,GPS_Result.LAT,GPS_Result.LON,GPS_Result.numSV); 
		//SysLogPrintf(ipa_syslog_addr,514,
		 LogMaxMin(mgz,mmx,mmy,ngz,nmx,nmy);
		 //static char tbuf[256];
		// siprintf(tbuf,"TCN=%ld\r\n",sim.timer[0].tcn);  
		// writestring(LOG_UART,tbuf);
		 mgz=IMU_Result.mz;
		 mmx=IMU_Result.mx;
		 mmy=IMU_Result.my;
		 ngz=IMU_Result.mz;
		 nmx=IMU_Result.mx;
		 nmy=IMU_Result.my;
		 LSecs=Secs;
		}
 

	if(LastImu !=IMU_Result.ReadingNum)
	{
	   if(IMU_Result.gz<ngz) ngz=IMU_Result.gz;
	   if(IMU_Result.gz>mgz) mgz=IMU_Result.gz;
	   	float hd=(fGZHeadingScale_deg*(IMU_Result.gz-gzoff)); 
	    fimHeading+=hd;
		figHeading+=hd;


		HeadLimit(figHeading);
		HeadLimit(fimHeading);



		if (IMU_Result.mx !=0 ) 
		{
		if(IMU_Result.mx<nmx) nmx=IMU_Result.mx;
		if(IMU_Result.mx>mmx) mmx=IMU_Result.mx;

		if(IMU_Result.my<nmy) nmy=IMU_Result.my;
		if(IMU_Result.my>mmy) mmy=IMU_Result.my;

		volatile ImuRegisters il;
		 OSLock();
		il.ax=IMU_Result.ax;
		il.ay=IMU_Result.ay;
		il.az=IMU_Result.az;
		il.gx=IMU_Result.gx;
		il.gy=IMU_Result.gy;
		il.gz=IMU_Result.gz;
		il.mx=IMU_Result.mx;
		il.my=IMU_Result.my;
		il.mz=IMU_Result.mz;
		il.t =IMU_Result.t ;
		il.ReadingNum=IMU_Result.ReadingNum;
		OSUnlock();              
		fmh=CalcMagHeading(IMU_Result.mx,IMU_Result.my);
		CorrectHeading(fimHeading,fmh,0.005);//20 times per second correct in 10 seconds so 0.005
		
		if(GPS_Result.GSpeed >25) //0.25M/sec
		{
		 float fgh=((float)GPS_Result.Heading)*1.0E-5;

		 if(GPS_Result.GSpeed >200) //12M/sec
			 CorrectHeading(figHeading,fgh,0.05);
			 else
		 if(GPS_Result.GSpeed >100) //1M/sec
		 	 CorrectHeading(figHeading,fgh,0.01);
		 	 else
			 CorrectHeading(figHeading,fgh,0.005);
		}
	    else
			CorrectHeading(figHeading,fmh,0.005);//20 times per second correct in 10 seconds so 0.005


		HeadLimit(figHeading);
		HeadLimit(fimHeading);


		il.fIhead=fimHeading;
	    il.fMhead=fmh;
		il.fgIHead=figHeading;
	    il.GHeading= GPS_Result.Heading;
		il.odo=sim.timer[0].tcn;

         LogImu(il);
		}
		LastImu =IMU_Result.ReadingNum; 

	}

 }//While

}

