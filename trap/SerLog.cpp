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
#include "car.h"
#include "gps.h"
#include "imu.h"
#include "dsm2_sub.h"
#include "log.h"
#include "sensor_config.h"
#include "filereporter.h" 
#include "nav.h"
#include "mytrap.h"

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
#define MSG_TYPE  (0x11)


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

inline static void ShowElement(BYTE id,void * vp,void * vp2, double el,const char * name)
{
ShowBaseElement(id,(PBYTE)vp,(PBYTE)vp2,sizeof(el),td64,name);

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

void LogServiceTask()
{
};


#include "logcore.cpp"

