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
#include <tcp.h>
#include "gps.h"
#include "imu.h"
#include "dsm2_sub.h"
#include "log.h"
#include "sensor_config.h"
#include "filereporter.h" 
#include "car.h"
#include "nav.h"
#include "mytrap.h"

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
#define TGPS_TYPE (0x1C)
#define STEER_TYPE (0x1D)
#define HEALTH_TYPE (0x1F)
#define NAV_TYPE (0x20)
#define TRAP_TYPE (0x21)

OS_CRIT LogShiftCrit;

#define LOG_REC_START  (0xEB)
#define LOG_REC_END    (0xEE)
#define LOG_REC_ESC  (0xEC)
#define LOG_REC_TICK (0xED)


static volatile BOOL bBlock;


volatile DWORD LogCount;

char hexnib(BYTE b)
{
b=b&0x0F;

if (b<10) return b+'0';
return b+'A'-10;
}


#define TCPBLEN (1024)

struct log_to_TCP_Buffer
{
    WORD get_pos;
    WORD ack_pos;
    WORD put_pos;
    BYTE CircBuffer[TCPBLEN];
    WORD accum;
	DWORD LastForceTick;

    void PutBufChar(BYTE c)
    {
        CircBuffer[put_pos++] = c;
        if ( put_pos >= TCPBLEN )
            put_pos = 0;
        if ( put_pos == ack_pos )
        {
            put_pos--;
            if ( put_pos >= TCPBLEN )
                put_pos = ( TCPBLEN - 1 );
        }
    }

    void FreeBuf(int len)
    {
        while ( ( len ) && ( ack_pos != get_pos ) )
        {
            len--;
            ack_pos++;
            if ( ack_pos >= TCPBLEN )
            {
                ack_pos = 0;
            }
        }
    }

    int Empty()
    {
        return ( ack_pos == put_pos );
    }
    ;

    void Flush()
    {
        get_pos = 0;
        put_pos = 0;
        ack_pos = 0;
        accum = 0;
    }
    ;

    /* Ciruclar buffer layout **************************************
     ***********ack**********get********put*************** Mode 0
     ***put********ack**********get*********************** Mode 1
     *********get*******put********ack******************** Mode 2
     */

    WORD space()
    {
        WORD cpy_ack;
        WORD cpy_put;

        cpy_ack = ack_pos;
        cpy_put = put_pos;

        if ( cpy_ack > cpy_put ) /* mode 1,2 above */
        {
            return ( cpy_ack - cpy_put );
        }

        return ( ( cpy_ack + TCPBLEN ) - cpy_put );
    }


    void WriteBufferData(int tcp)
    {

        WORD cpy_get;
        WORD cpy_put;
        int force_send;

        USER_ENTER_CRITICAL();
        cpy_get = get_pos;
        cpy_put = put_pos;
        USER_EXIT_CRITICAL();

        if ( cpy_put == cpy_get )
            return;
		force_send=1;


        if ( cpy_get > cpy_put ) /* mode 1 above */
        {
            if ( write_for_callback( tcp, (const char *) ( CircBuffer + cpy_get ), ( TCPBLEN
                    - cpy_get ), force_send ) )
            {
                cpy_get = 0;
            }
        }

        if ( cpy_get < cpy_put ) /* Mode 0 or 2  above */
        {
            if ( write_for_callback( tcp, (const char *) ( CircBuffer + cpy_get ), ( cpy_put - cpy_get ), force_send ) )
            {
                cpy_get = cpy_put;
            }

        }
        USER_ENTER_CRITICAL();
        get_pos = cpy_get;
        USER_EXIT_CRITICAL();
    }
};


int lfd;
int cfd;
bool bSentBlob;

						  
log_to_TCP_Buffer  ltb;

void PutRawByte(BYTE b)
{

//IRQ_writechar(LOG_UART,hexnib(b>>4));
//IRQ_writechar(LOG_UART,hexnib(b &0x0f));

//	iprintf("%c",b);
//   iprintf("%02X",b);
if(cfd)
{
while(((bBlock)  && (ltb.space()==0 ))) OSTimeDly(2);
	
	ltb.PutBufChar(b); 
	if((LogCount & 127)==127)ltb.WriteBufferData(cfd);

}
	//IRQ_writechar(LOG_UART,b); 	
//else
	//IRQ_WriteCharNoBlock(LOG_UART,b);

	LogCount++;
}



TCPCallbacks mycallback;

WORD WinSiz(int socket) {return 1;};
void ReadDataCB(int socket, PBYTE data, int len) {};
void WriteFreeCB(int socket, const char * data_sent, int len){ ltb.FreeBuf(len);};
void CloseErrCB(int socket) {close(cfd); cfd=0; iprintf("Closed \n");};


TCPCallbacks *  MyAccept(int listen_socket, int new_socket)
{
 if(cfd) close(cfd);
 bSentBlob=false;
 cfd=new_socket;
 ltb.Flush();
 iprintf("Accept");
 return &mycallback;
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

	iprintf("Starting log\r\n");
	
	mycallback.rxwindow_size=WinSiz;
	mycallback.read_data_callback=ReadDataCB;
	mycallback.write_free_callback=WriteFreeCB;
	mycallback.closed_on_error_callback=CloseErrCB;


	lfd=listen_w_callback(0,1000 ,MyAccept);


   //bBlock=true;
	//DumpRecords(); 
	//bBlock=false;

}			  


/* Individual log classes... */


void LogServiceTask()
{
 iprintf("Service %d\r\n",Secs);
 if((cfd) && (!bSentBlob))
 {
  bSentBlob=true;
  bBlock=true;
  DumpRecords();
  bBlock=false;
 }
}




#include "logcore.cpp"
