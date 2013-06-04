#include "predef.h" 
#include <basictypes.h>
#include <constants.h>
#include <ucos.h>
#include <ucosmcfc.h>
#include <serialirq.h>
#include <math.h>
#include <netif.h>
#include <ip.h>
#include <udpsend.h>
#include <arp.h>
#include <string.h>
#include "gps.h"


volatile GPS_READING GPS_Result;
volatile GPS_READING TGPS_Result;



typedef struct
{
unsigned char stx;
unsigned char status;
unsigned char pt40;
unsigned char len1;
unsigned char tn;
unsigned  char pg;
unsigned  char mpg;
unsigned char data;
}__attribute__(( packed) ) rec40; 

typedef struct
{
 unsigned char type;
 unsigned char len2;
 double lattitude;
 double longitude;
 double ht;
} __attribute__(( packed) ) rec2;

typedef struct
{
 unsigned char type;
 unsigned char len2;
 unsigned long msec;
 unsigned short week;
 unsigned char nsat;
 unsigned char flag1;
 unsigned char flag2;
 unsigned char seq;
} __attribute__(( packed) ) rec1;


typedef struct
{
 unsigned char type;
 unsigned char len2;
 double ECEF_X;
 double ECEF_Y;
 double ECEF_Z;
} __attribute__(( packed) ) rec3;




typedef struct
{
 unsigned char type;
 unsigned char len2;
 unsigned char flags;
 float hspeed;
 float head; //0 to 2pi
 float vv;
} __attribute__(( packed) ) rec8;


typedef struct
{
 unsigned char type;
 unsigned char len2;
 float rms_error;
 float sigma_east;
 float sigma_north;
 float covaren;
 float sigma_up;
 float semi_major;
 float semi_minor;
 float orientaion;
 float variance;
 unsigned short epoch;
} __attribute__(( packed) ) rec12;



volatile BD960_GPS bd960_Result;
volatile BD960_GPS tbd960_Result;





static OS_SEM * pDataSem;

#ifdef BOGUS
struct GPS_READING
{
DWORD       itow;           // ms GPS Millisecond Time of Week
int         ECEF_X;         // cm ECEF X coordinate
int         ECEF_Y;         // cm ECEF Y coordinate
int         ECEF_Z;         // cm ECEF Z coordinate
int         PAcc;           // cm 3D Position Accuracy Estimate
int         ECEFVX;         // cm/s ECEF X velocity
int         ECEFVY;         // cm/s ECEF Y velocity
int         ECEFVZ;         // cm/s ECEF Z velocity
int         LON;            // 1e-07 deg Longitude
int         LAT;            // 1e-07 deg Latitude
int         HEIGHT;         // mm Height above Ellipsoid
int         HMSL;           // mm Height above mean sea level
int         VEL_N;          // cm/s  NED north velocity
int         VEL_E;          // cm/s  NED east velocity
int         VEL_D;          // cm/s  NED down velocity
int         Speed;          // cm/s  Speed (3-D)
int         GSpeed;         // cm/s  Ground Speed (2-D)
int         Heading;        // 1e-05 deg  Heading 2-D
DWORD        SAcc;           // cm/s  Speed Accuracy Estimate
DWORD        CAcc;           // deg  Course / Heading Accuracy Estimate
DWORD        Hacc;           // mm Horizontal Accuracy Estimate
DWORD        Vacc;           // mm Vertical Accuracy Estimate
BYTE         numSV;          // Number of SVs used in navigation solution
BYTE         GPSfix;         // GPSfix Type, range 0..6
WORD         week;           // GPS week
WORD         ReadingNum;  //Serial number of GPS reading
}; 

struct BD960_GPS
{
  double lattitude;
  double longitude;
  double ht;
  unsigned char r8flags;
  float hspeed;
  float head;
  float vv;
  float rms_error;
  float sigma_east;
  float sigma_north;
  float covaren;
  float sigma_up;
  float semi_major;
  float semi_minor;
  float orientaion;
  float variance;
  unsigned char flag1;
  unsigned char flag2;
};


#endif


const double LL_Scale = (1E7*180.0/M_PI); 



unsigned char * ParseRecord(unsigned char * cp, unsigned int & len)
{
if (cp[1]==0x03) return NULL; //End

unsigned int cplen=cp[1]+2;

if(cplen>=len) return NULL;

len-=cplen;

switch(cp[0])
	{
	case 1:
		{rec1 * r1=(rec1*)cp;
		TGPS_Result.numSV=r1->nsat;
		TGPS_Result.week=r1->week; 

		tbd960_Result.flag1=r1->flag1;
		tbd960_Result.flag2=r1->flag2;
		}
		break;
	   
	case 2:
		{rec2 * r2=(rec2*)cp;
		TGPS_Result.LAT=(int)(r2->lattitude*LL_Scale);
		TGPS_Result.LON=(int)(r2->longitude*LL_Scale);
		TGPS_Result.HEIGHT=(int)(r2->ht*1000.0);
		tbd960_Result.lattitude=r2->lattitude;
		tbd960_Result.longitude=r2->longitude;
		tbd960_Result.ht=r2->ht;
		}
		break;
	case 3:
		{rec3 * r3=(rec3*)cp;
		TGPS_Result.ECEF_X=(int)(r3->ECEF_X*100.0);
		TGPS_Result.ECEF_Y=(int)(r3->ECEF_Y*100.0);
  	    TGPS_Result.ECEF_Z=(int)(r3->ECEF_Z*100.0);
		}
		break;
	   
	case 8:
		{rec8 * r8=(rec8*)cp;
		tbd960_Result.hspeed=r8->hspeed;
		tbd960_Result.head=r8->head;
		tbd960_Result.r8flags=r8->flags;
		tbd960_Result.vv=r8->vv;
		TGPS_Result.GSpeed=  (int)(r8->hspeed*100.0); 
		r8->head*=180.0/M_PI;
		TGPS_Result.Heading= (int)((r8->head*1.0E5)); 
		}
		break;

	case 12:
		{
		rec12 * r12=(rec12*)cp;
		bd960_Result.rms_error   = r12->rms_error;   
		bd960_Result.sigma_east  = r12->sigma_east;  
		bd960_Result.sigma_north = r12->sigma_north; 
		bd960_Result.covaren     = r12->covaren;     
		bd960_Result.sigma_up    = r12->sigma_up;    
		bd960_Result.semi_major  = r12->semi_major;  
		bd960_Result.semi_minor  = r12->semi_minor;  
		bd960_Result.orientaion  = r12->orientaion;  
		bd960_Result.variance    = r12->variance;    

		 TGPS_Result.PAcc= (int)(r12->rms_error*100.0);
		 TGPS_Result.Hacc=TGPS_Result.PAcc;
		}
		break;

	}
cp+=cplen;
return cp;
}



#define SM_TASK_STK (512)
#define SmOSSimpleTaskCreate(x,p) { static DWORD   func_##x_Stk[SM_TASK_STK] __attribute__( ( aligned( 4 ) ) ); OSTaskCreate(x,NULL,(void *)&func_##x_Stk[SM_TASK_STK],(void*)func_##x_Stk,p); }

MACADR DestinationMac;
#define port (5017)
#define DestinationIp (0x0A010129)  //10.1.1.41

void TickleDevice()
{
DWORD data;
data=0;

if(
   (DestinationMac.phywadr[0]==0) &&
   (DestinationMac.phywadr[1]==0) &&
   (DestinationMac.phywadr[2]==0)
  )
{
	GetArp( DestinationIp, DestinationMac, 40 );
}
	   
SendScratchUdp((char *)&data,4, port,port,DestinationIp, DestinationMac );
}


volatile DWORD RxPkt;

int UdpReceiveFunction(PEFRAME pf)
{
    PUDPPKT pUdpPkt = GetUdpPkt( pf );
//    PIPPKT pIp = GetIpPkt( pf );
    if ( pUdpPkt->dstPort == port )
    {
	unsigned int len=GetUdpDataLen( pUdpPkt );
	if(len >= sizeof(rec40) )
		{
		 rec40 * pr=(rec40 *) pUdpPkt->DATA;

		 unsigned char * cp=&pr->data;
		 do
			 {
			  cp=ParseRecord(cp,len);
			 }
		 while(cp);

		}
	TGPS_Result.ReadingNum++;
	OSLock();
	memcpy((void *)&GPS_Result,(void *)&TGPS_Result,sizeof(GPS_Result));
	memcpy((void *)&bd960_Result,(void *)&tbd960_Result,sizeof(bd960_Result));
	OSUnlock();                          
	if(pDataSem)OSSemPost(pDataSem);
	RxPkt++;
	return 1;
	}
return 0;
}



void GpsTask(void *)
{
pUserUdpProcessFunction = UdpReceiveFunction;
TickleDevice();  
DWORD LastPkt=0;
DWORD LastTickle=Secs;

while(1)
{
OSTimeDly(10);
if (LastPkt==RxPkt) 
{
	TickleDevice();
	//writestring(LOG_UART,"TickleDevice\r\n");

}
else
{
// static char tbuf[256];
// sprintf(tbuf,"Lat %d Lon %d nSats %d hacc=%lu  #%lu f1=%02X f2=%0x2 nr12: %lu\r\n",GPS_Result.LAT,GPS_Result.LON,GPS_Result.numSV,GPS_Result.Hacc,GPS_Result.ReadingNum,f1,f2, nr12);
 //sprintf(tbuf,"nr1=%lu nr2=%lu nr3=%lu nr8=%lu nr12=%lu  nrq=%lu nrn=%02X\r\n",nr1,nr2,nr3,nr8,nr12,nrq,nrn);
// writestring(LOG_UART,tbuf);
LastPkt=RxPkt;
if((LastTickle+10)<Secs)
	{
	 TickleDevice();
	 LastTickle=Secs; 
	}
}
}//While


}

void InitGpsSubSystem(WORD prio,OS_SEM * pSem)
{

	pDataSem=pSem;
	SmOSSimpleTaskCreate(GpsTask,prio);

}



