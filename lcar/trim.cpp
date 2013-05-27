#include "predef.h"
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <startnet.h>
#include <ucos.h>
#include <udp.h>
#include <string.h>
#include "bd960.h"
#include "flags.h"


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
 float head;
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



volatile CombinedTrimGps Gps_Result;
volatile CombinedTrimGps TGps_Result;



void TickleDevice(WORD portnum, IPADDR ipaddr)
{
 
	   UDPPacket pkt;
	   pkt.SetSourcePort( portnum );
	   pkt.SetDestinationPort( portnum );
	   pkt.AddDataByte( 0 );
	   pkt.Send( ipaddr );

}




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
		TGps_Result.nsat=r1->nsat;
		TGps_Result.week=r1->week; 
		TGps_Result.flag1=r1->flag1; 
		TGps_Result.flag2=r1->flag2; 
		}
		break;
	   
	case 2:
		{rec2 * r2=(rec2*)cp;
		TGps_Result.lattitude=r2->lattitude;  
		TGps_Result.longitude=r2->longitude;
		TGps_Result.ht=r2->ht;
		}
		break;
	case 3:
		{rec3 * r3=(rec3*)cp;
		TGps_Result.ECEF_X=r3->ECEF_X;
		TGps_Result.ECEF_Y=r3->ECEF_Y;
  	    TGps_Result.ECEF_Z=r3->ECEF_Z;
		}
	   
	case 8:
		{rec8 * r8=(rec8*)cp;
		TGps_Result.hspeed=r8->hspeed;
		TGps_Result.head=r8->head;
		TGps_Result.vv=r8->vv;
		}
		break;

	case 12:
		{
        //rec12 * r12=(rec12*)cp;
		}
		break;
	}
cp+=cplen;
return cp;
}


volatile int ntLastStart;


void WriteObject(BYTE id, volatile void * vdata, int olen, int allways_send);

/*-------------------------------------------------------------------
 * This task will wait for incoming UDP packets and process them.
 -------------------------------------------------------------------*/
void GPSUdp( void *pd )
{

   // Create FIFO to store incoming packets and initialize it
   OS_FIFO fifo;
   OSFifoInit( &fifo );

   IPADDR ipaddr = AsciiToIp("10.1.1.40");


   // Register to listen for UDP packets on port number 'port'
   RegisterUDPFifo( 5017, &fifo );

   TickleDevice(5017,ipaddr);
int nn=0;

   while ( 1 )
   {
      // We construct a UDP packet object using the FIFO. 
      // This constructor will only return when we have received a packet
      UDPPacket upkt( &fifo,5 /* Replace this 0 with a tick count to have a time out delat */ );

      // Did we get a valid packet or just time out?
      if ( upkt.Validate() )
      {
		  bzero((void *)&TGps_Result,sizeof(TGps_Result));
		  
		  unsigned int len = upkt.GetDataSize();
		  		  
		  if(len >= sizeof(rec40) )
		  {
		   rec40 * pr=(rec40 *) upkt.GetDataBuffer();

		   unsigned char * cp=&pr->data;
		   do
			   {
			   cp=ParseRecord(cp,len);
			   }
		   while(cp);

		  }
		 nn++;
		 if(nn==256) 
			 {
			 nn=0;
			   TickleDevice(5017,ipaddr); 
			 }

		 if(TGps_Result.lattitude)
		 {
		  OSLock();
		  memcpy((void *)&Gps_Result,(void *)&TGps_Result,sizeof(TGps_Result));
		  OSUnlock();
		  OSFlagSet(&DataFlag,NEW_GPS);

		 }
      }
	  else
		  TickleDevice(5017,ipaddr); 


   }
}


bool GpsValid(volatile CombinedTrimGps & g)
{
if((g.flag1 & 0x05)==5) return true;
return false;
}




void StartGpsProcess(BYTE prio)
{
OSSimpleTaskCreatewName(GPSUdp,prio,"GPS UDP");

}



