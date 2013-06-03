#include "predef.h" 
#include <ctype.h>
#include <basictypes.h>
#include <system.h>
#include <constants.h>
#include <ucos.h>
#include <ucosmcfc.h>
#include <serialirq.h>
#include <stdio.h>
#include <smarttrap.h>
#include <serialupdate.h>
#include <string.h>
#include <Pins.h>
#include <sim.h>
#include <cfinter.h>
#include <iointernals.h>
#include "gps.h"
#include "imu.h"
#include "dsm2_sub.h"
#include "log.h"
#include "sensor_config.h"
#include "filereporter.h" 
#include "car.h"

void IRQ_WriteCharNoBlock( int portnum, char c );

volatile DWORD LogPagesWritten;
																
typedef enum tNames
{
 tu8=1,
 ti8=2,
 tu16=3,
 ti16=4,
 tu32=5,
 ti32=6,
 tpu8=7,
 tpi8=8,
 tf32=9,
 td64=10
};



//Log Descriptor Type...
#define LOG_TYPE_STRUCT (1)
#define LOG_TYPE_ITEM (2)

//Log message types   
#define MSG_TYPE  (0x11)
#define IMU_TYPE  (0x12)
#define ZERO_TYPE (0x13)
#define MAG_TYPE  (0x14)
#define ALT_TYPE  (0x15)
#define XBE_TYPE  (0x16)
#define GPS_TYPE  (0x17)
#define CFG_TYPE  (0x18)
#define DSM2_TYPE (0x19)
#define MAX_TYPE (0x1A)
#define SGPS_TYPE  (0x1B)

OS_CRIT LogShiftCrit;

#define LOG_REC_START  (0xEB)
#define LOG_REC_END    (0xEE)
#define LOG_REC_ESC  (0xEC)
#define LOG_REC_TICK (0xED)


static BOOL bBlock;


volatile DWORD LogCount;

char hexnib(BYTE b)
{
b=b&0x0F;

if (b<10) return b+'0';
return b+'A'-10;
}


void PutRawByte(BYTE b)
{

//IRQ_writechar(LOG_UART,hexnib(b>>4));
//IRQ_writechar(LOG_UART,hexnib(b &0x0f));

//	iprintf("%c",b);
//   iprintf("%02X",b);
//if(bBlock) 
	IRQ_writechar(LOG_UART,b); 	
//else
	//IRQ_WriteCharNoBlock(LOG_UART,b);

	LogCount++;
}




void PutEscapedByte(BYTE b)
{
if(b==LOG_REC_START) {PutRawByte(LOG_REC_ESC); PutRawByte(0); }
else
if(b==LOG_REC_ESC) {PutRawByte(LOG_REC_ESC); PutRawByte(1); }
else
if(b==LOG_REC_END) {PutRawByte(LOG_REC_ESC); PutRawByte(2); }
else
if(b==LOG_REC_TICK) {PutRawByte(LOG_REC_ESC); PutRawByte(3); }
else
if(b==0x11) {PutRawByte(LOG_REC_ESC); PutRawByte(4); }
else
if(b==0x13) {PutRawByte(LOG_REC_ESC); PutRawByte(5); }
else
PutRawByte(b);
}




inline void StartLogRecord() { PutRawByte(LOG_REC_START); }
inline void EndLogRecord()   { PutRawByte(LOG_REC_END); }

void LogRawRecord(BYTE msg, const unsigned char * data, int len)
{
BYTE bs=(msg);
StartLogRecord();
PutRawByte(msg);
for(int i=0; i<len; i++)
{
 bs+=data[i];
 PutEscapedByte(((PBYTE)data)[i]);
}
EndLogRecord();
PutEscapedByte(bs);
}






void StartItemIntro(BYTE id, int siz, const char * name)
{
PutRawByte(LOG_REC_START);
PutRawByte(LOG_TYPE_STRUCT);
BYTE bs=LOG_TYPE_STRUCT;
PutRawByte(id);
bs+=id;

PutEscapedByte((BYTE)(siz>>8)&0xFF);
bs+=(BYTE)(siz>>8)&0xFF;

PutEscapedByte((BYTE)(siz&0xFF));
bs+=(BYTE)(siz&0xFF);

	while(*name) 
		{bs+=*name;
		 PutEscapedByte(*name++);
		}
PutRawByte(LOG_REC_END); 
PutEscapedByte(bs);
}


//Log the base element of a structure */
void ShowBaseElement(BYTE id,PBYTE p1, PBYTE p2,int siz, enum  tNames tn, const char * el_name)
{
//iprintf("Offset : %d  size :%d  type %d  name %s\r\n",(p2-p1),siz,tn ,el_name);
int offset=(p2-p1);
PutRawByte(LOG_REC_START); 
PutRawByte(LOG_TYPE_ITEM );
BYTE bs=LOG_TYPE_ITEM;


PutEscapedByte((BYTE)id);
bs+=id;

PutEscapedByte((BYTE)tn);
bs+=tn;

PutEscapedByte((BYTE)(siz>>8)&0xFF);
bs+=(BYTE)(siz>>8)&0xFF;
PutEscapedByte((BYTE)(siz&0xFF));
bs+=(BYTE)(siz&0xFF);
PutEscapedByte((BYTE)(offset>>8)&0xFF);
bs+=(BYTE)(offset>>8)&0xFF;
PutEscapedByte((BYTE)(offset&0xFF));
bs+=(BYTE)(offset&0xFF);

 	while(*el_name) 
		{
		bs+=*el_name;
		PutEscapedByte(*el_name++);
		}
PutRawByte(LOG_REC_END); 
PutEscapedByte(bs);
}


//Generic message class...
void LogMessage(const char * msg)
{
	LogRawRecord(MSG_TYPE,( const unsigned char * )msg,strlen(msg));
}







inline static void ShowElement(BYTE id,void * vp,void * vp2, short el,const char * name)  
{
ShowBaseElement(id,(PBYTE)vp,(PBYTE)vp2,sizeof(el),ti16,name);
}


inline static void ShowElement(BYTE id,void * vp,void * vp2, int el,const char * name)
{

ShowBaseElement(id,(PBYTE)vp,(PBYTE)vp2,sizeof(el),ti32,name);
}
										

inline static void ShowElement(BYTE id,void * vp,void * vp2, long el,const char * name)
{

ShowBaseElement(id,(PBYTE)vp,(PBYTE)vp2,sizeof(el),ti32,name);
}
 


inline static void ShowElement(BYTE id,void * vp,void * vp2, unsigned short el,const char * name)
{
ShowBaseElement(id,(PBYTE)vp,(PBYTE)vp2,sizeof(el),tu16,name);
}


inline static void ShowElement(BYTE id,void * vp,void * vp2, unsigned long  el,const char * name)
{
ShowBaseElement(id,(PBYTE)vp,(PBYTE)vp2,sizeof(el),tu32,name);

}



inline static void ShowElement(BYTE id,void * vp,void * vp2, char el,const char * name)
{
ShowBaseElement(id,(PBYTE)vp,(PBYTE)vp2,sizeof(el),ti8,name);
}


inline static void ShowElement(BYTE id,void * vp,void * vp2, unsigned char el,const char * name)
{
ShowBaseElement(id,(PBYTE)vp,(PBYTE)vp2,sizeof(el),tu8,name);

}

inline static void ShowElement(BYTE id,void * vp,void * vp2, float el,const char * name)
{
ShowBaseElement(id,(PBYTE)vp,(PBYTE)vp2,sizeof(el),tf32,name);

}

/*
inline static void ShowElement(BYTE id,void * vp,void * vp2, char * el,const char * name)
{
ShowBaseElement(id,(PBYTE)vp,(PBYTE)vp2,sizeof(el),tpi8,name);
}


inline static void ShowElement(BYTE id,void * vp,void * vp2, unsigned char * el,const char * name)
{
ShowBaseElement(id,(PBYTE)vp,(PBYTE)vp2,sizeof(el),tpu8,name);
}

*/

int null_stdio_close(int fd)
{
    IoExpands[fd].read = NULL;
    IoExpands[fd].write = NULL;
    IoExpands[fd].close = NULL;
    return 0;
}

int null_stdio_read(int fd, char *buf, int nbytes)
{
    return 0;
}

int null_stdio_write(int fd, const char *buf, int nbytes)
{
return nbytes;
}


void DumpRecords();

void InitLog()
{
    IoExpands[0].extra = 0;
    IoExpands[0].read =  null_stdio_read;
    IoExpands[0].write = null_stdio_write;
    IoExpands[0].close = null_stdio_close;

	IoExpands[1].extra = 0;
	IoExpands[1].read =  null_stdio_read;
	IoExpands[1].write = null_stdio_write;
	IoExpands[1].close = null_stdio_close;

	SimpleUart(LOG_UART,115200);
	writestring(LOG_UART,"Starting log\r\n");
	bBlock=true;
	DumpRecords(); 
	bBlock=false;

}			  


/* Individual log classes... */

#define LogStart(id ,name) BYTE tid=id;   StartItemIntro(id,sizeof(item),name); 
#define LogElement(x,y) ShowElement(tid,&item,&item.x,item.x,y)






void LogRC(volatile DSM2_READING &item)
{
	LogRawRecord(DSM2_TYPE,(const unsigned char *)&item,sizeof(item)); 

}



void LogImu(volatile ImuRegisters  & item)
{
 LogRawRecord(IMU_TYPE,(const unsigned char *)&item,sizeof(item)); 
}


void LogGps(volatile GPS_READING  &item)
{
	LogRawRecord(GPS_TYPE,(const unsigned char *)&item,sizeof(item));
}


void LogSmGps(volatile GPS_READING  & gps)
{
SMGPS_READING item;

item.LON      =gps.LON;     
item.LAT      =gps.LAT;     
item.GSpeed   =gps.GSpeed;  
item.Heading  =gps.Heading; 
item.numSV    =gps.numSV;   
item.GPSfix   =gps.GPSfix;  
item. Hacc    =gps.Hacc;   
LogRawRecord(SGPS_TYPE,(const unsigned char *)&item,sizeof(item));

}



void LogConfig(volatile sensor_saved_config &item)
{
	LogRawRecord(CFG_TYPE,(const unsigned char *)&item,sizeof(item));

}

struct mmax
{
	short mgz;
	short mmx;
	short mmy;
	short ngz;
	short nmx;
	short nmy;
};



void LogMaxMin(short mgz,short mmx,short mmy,short ngz,short nmx,short nmy)
{
  mmax item;
 item.mgz=mgz;
 item.mmx=mmx;
 item.mmy=mmy;
 item.ngz=ngz;
 item.nmx=nmx;
 item.nmy=nmy;
 LogRawRecord(MAX_TYPE,(const unsigned char *)&item,sizeof(item));
}



void ShowConfig(sensor_saved_config &item)
{
	LogStart(CFG_TYPE,"Config");
	LogElement(mag_max[0]             ,"mag_max[0]");
    LogElement(mag_max[1]             ,"mag_max[1]");
	LogElement(mag_max[2]             ,"mag_max[2]");
	LogElement(mag_min[0]             ,"mag_min[0]");
	LogElement(mag_min[1]             ,"mag_min[1]");
 	LogElement(mag_min[2]             ,"mag_min[2]");
	LogElement(accel_zero[0]          ,"accel_zero[0]");
	LogElement(accel_zero[1]          ,"accel_zero[1]");
 	LogElement(accel_zero[2]          ,"accel_zero[2]");
	LogElement(default_gyro_zero[0]   ,"default_gyro_zero[0]");
    LogElement(default_gyro_zero[1]   ,"default_gyro_zero[1]");
	LogElement(default_gyro_zero[2]   ,"default_gyro_zero[2]");
	LogElement(servo_neg_lim[0]       ,"servo_neg_lim[0]");    
	LogElement(servo_neg_lim[1]       ,"servo_neg_lim[1]");    
 	LogElement(servo_neg_lim[2]       ,"servo_neg_lim[2]");    
 	LogElement(servo_neg_lim[3]       ,"servo_neg_lim[3]");    
	LogElement(servo_pos_lim[0]       ,"servo_pos_lim[0]");    
	LogElement(servo_pos_lim[1]       ,"servo_pos_lim[1]");    
	LogElement(servo_pos_lim[2]       ,"servo_pos_lim[2]");    
	LogElement(servo_pos_lim[3]       ,"servo_pos_lim[3]");    
	LogElement(servo_mid[0]           ,"servo_mid[0]");        
	LogElement(servo_mid[1]           ,"servo_mid[1]");        
	LogElement(servo_mid[2]           ,"servo_mid[2]");        
	LogElement(servo_mid[3]           ,"servo_mid[3]");        
}


void ShowMMRec(mmax & item)
{
	LogStart(MAX_TYPE,"MAX");
	LogElement(mgz,"MGZ");
	LogElement(mmx,"MMX");
	LogElement(mmy,"MMY");
	LogElement(ngz,"NGZ");
	LogElement(nmx,"NMX");
	LogElement(nmy,"NMY");
}

void ShowImuRec(ImuRegisters & item)
{
	LogStart(IMU_TYPE,"IMU");
	LogElement(ax,"Ax");
	LogElement(ay,"Ay");
	LogElement(az,"Az");
	LogElement(gx,"Gx");
	LogElement(gy,"Gy");
	LogElement(gz,"Gz");
	LogElement(mx,"Mx");
	LogElement(my,"My");
	LogElement(mz,"Mz");
	LogElement(t,"T");
	LogElement(ReadingNum,"RN");
	LogElement(fIhead,"IH");
	LogElement(fMhead,"MH");
	LogElement(GHeading,"GH");
	LogElement(odo,"Od");
}


void ShowSmGps(SMGPS_READING & item)
{
	LogStart(SGPS_TYPE,"SGPS");
	LogElement(LON    , "LON");          // 1e-07 deg Longitude                        
	LogElement(LAT    , "LAT");          // 1e-07 deg Latitude                         
	LogElement(numSV  , "numSV");        // Number of SVs used in navigation solution 
	LogElement(GSpeed , "GSpeed");       // cm/s  Ground Speed (2-D)                   
	LogElement(Heading, "Heading");       // 1e-05 deg  Heading 2-D                     
	LogElement(Hacc   , "Hacc");         // mm Horizontal Accuracy Estimate           
}

void ShowGps(GPS_READING  &item)
{
  LogStart(GPS_TYPE,"GPS");
  LogElement(  itow   , "itow");         // ms GPS Millisecond Time of Week            
  LogElement(ECEF_X , "ECEF_X");       // cm ECEF X coordinate                       
  LogElement(ECEF_Y , "ECEF_Y");       // cm ECEF Y coordinate                       
  LogElement(ECEF_Z , "ECEF_Z");       // cm ECEF Z coordinate                       
  LogElement(PAcc   , "PAcc");         // cm 3D Position Accuracy Estimate           
  LogElement(ECEFVX , "ECEFVX");       // cm/s ECEF X velocity                       
  LogElement(ECEFVY , "ECEFVY");       // cm/s ECEF Y velocity                       
  LogElement(ECEFVZ , "ECEFVZ");       // cm/s ECEF Z velocity                       
  LogElement(LON    , "LON");          // 1e-07 deg Longitude                        
  LogElement(LAT    , "LAT");          // 1e-07 deg Latitude                         
  LogElement(HEIGHT , "HEIGHT");       // mm Height above Ellipsoid                  
  LogElement(HMSL   , "HMSL");         // mm Height above mean sea level             
  LogElement(VEL_N  , "VEL_N");        // cm/s  NED north velocity                   
  LogElement(VEL_E  , "VEL_E");        // cm/s  NED east velocity                    
  LogElement(VEL_D  , "VEL_D");        // cm/s  NED down velocity                    
  LogElement(Speed  , "Speed");        // cm/s  Speed (3-D)                          
  LogElement(GSpeed , "GSpeed");       // cm/s  Ground Speed (2-D)                   
  LogElement(Heading, "Heading");       // 1e-05 deg  Heading 2-D                     
  LogElement(SAcc   , "SAcc");         // cm/s  Speed Accuracy Estimate             
  LogElement(CAcc   , "CAcc");         // deg  Course / Heading Accuracy Estimate   
  LogElement(Hacc   , "Hacc");         // mm Horizontal Accuracy Estimate           
  LogElement(Vacc   , "Vacc");         // mm Vertical Accuracy Estimate             
  LogElement(numSV  , "numSV");        // Number of SVs used in navigation solution 
  LogElement(GPSfix , "GPSfix");       // GPSfix Type, range 0..6                   
  LogElement(week   , "week");         // GPS week                                  
  LogElement(ReadingNum,"RN");
}


void ShowRC(DSM2_READING &item)
{
	LogStart(DSM2_TYPE,"DSM2");
	LogElement(val[0],"V0");
	LogElement(val[1],"V1");
	LogElement(val[2],"V2");
	LogElement(val[3],"V3");
	LogElement(val[4],"V4");
	LogElement(val[5],"V5");
	LogElement(val[6],"V6");
	LogElement(val[7],"V7");
	LogElement(ReadingNum,"RN");

}






void ShowFileVer(const char * cp)
{
PutRawByte(LOG_REC_START); 
PutRawByte(MSG_TYPE );
while(*cp) PutEscapedByte(*cp++);
PutRawByte(LOG_REC_END); 
}

void DumpRecords()
{
BYTE item[16];
ShowConfig((*((sensor_saved_config* )&item))) ;
ShowMMRec((*((mmax  *)& item)));
ShowImuRec((*((ImuRegisters * )& item))) ;
ShowGps((*((GPS_READING  * )&item)));
ShowRC((*((DSM2_READING *)&item)));
ShowSmGps(((*(SMGPS_READING *)& item))); 
FileReporter::DumpList();
LogConfig(SensorConfig);

}




FileReporter* FileReporter::pRoot;

void FileReporter::DumpList()
{
 FileReporter * pF =FileReporter::pRoot;
 while(pF)
 {
	 ShowFileVer(pF->m_pName);
	 pF=pF->m_pNext;
 }

}

void FileReporter::ShowList()
{
 FileReporter * pF =FileReporter::pRoot;
 while(pF)
 {
	 iprintf("%s\r\n",pF->m_pName);
	 pF=pF->m_pNext;
 }

}


