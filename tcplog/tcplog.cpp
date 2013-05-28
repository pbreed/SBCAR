
#include <windows.h>
#include <stdio.h>
#include <conio.h>
#include <errno.h>
#include <winsock.h>
#include <conio.h>
#include <string>
#include <time.h>

using namespace std;

#pragma pack(1)

#define PRINTERROR(s) fprintf(stderr,"\n%s: %d\n", s, WSAGetLastError())

struct ConfigRecord
{
        DWORD          m_Len;           /* length of record */
        BYTE           ip_Addr[4];      /*The device IP Address */
        BYTE           ip_Mask[4];      /*The IP Address Mask */
        BYTE           ip_GateWay[4];   /*The address of the P gateway */
        BYTE           ip_TftpServer[4];/*The address of the TFTP server to load data from for debugging */
        DWORD          baud_rate;    /*The initial system Baud rate */
        unsigned char  wait_seconds; /*The number of seconds to wait before booting */
        unsigned char  bBoot_To_Application; /*True if we boot to the application, not the monitor */
        unsigned char  bException_Action; /*What should we do when we have an exception? */
        unsigned char  m_FileName[80]; /*The file name of the TFTP file to load */
        unsigned char  mac_address[6]; /*The Ethernet MAC address */
        unsigned long  m_Unused[8];
        unsigned short checksum;       /*A Checksum for this structure */
};


struct ToFromPcStruct
{
        DWORD   m_dwKeyValue;
        BYTE    m_bAction;
        ConfigRecord m_cf;
};



#define VERIFY_FROM_PC_TO_NDK (0x4255524E) //BURN
#define VERIFY_FROM_NDK_TO_PC (0x4E455442) //NETB
#define UDP_NETBURNERID_PORT (0x4E42) /*NB*/
#define NBAUTO_READ             'R'


DWORD findNB(const char * pgm)
{
	/* Setup the socket */
SOCKET sock=socket(AF_INET,SOCK_DGRAM,0);
sockaddr_in saddr;
memset(&saddr,0,sizeof(saddr));
saddr.sin_family=AF_INET;
saddr.sin_port=htons(UDP_NETBURNERID_PORT);
saddr.sin_addr.s_addr = INADDR_ANY;


/* Set up the outgoing broadcast packet */
ToFromPcStruct tfpc;
tfpc.m_dwKeyValue=htonl(VERIFY_FROM_PC_TO_NDK);
tfpc.m_bAction = NBAUTO_READ;
   


/* Now send the broad cast */
int result=bind(sock,(struct sockaddr *)&saddr,sizeof(saddr));

  sockaddr_in saddro;
  memset(&saddro,0,sizeof(saddro));
  saddro.sin_family=AF_INET;
  saddro.sin_port=htons(UDP_NETBURNERID_PORT);
  saddro.sin_addr.s_addr =INADDR_BROADCAST;
  BOOL bVal=TRUE;
  setsockopt(sock,SOL_SOCKET,SO_BROADCAST,(const char *)&bVal, sizeof(BOOL));

  result=sendto(sock,(const char  *)&tfpc,sizeof(tfpc),0,(struct sockaddr *)&saddro,sizeof(saddro));


  
fd_set readfd;
FD_ZERO(&readfd);
FD_SET(sock,&readfd);
timeval tout;

tout.tv_sec=2;
tout.tv_usec=0;


/* Now wait for each UDP packet to come back */
while(select (1,&readfd,NULL,NULL,&tout)>0)
{                           
int saddrlen=sizeof(saddr);
char buffer[1024];
result=recvfrom(sock,(char *)buffer,1024,0,(struct sockaddr *)&saddr,&saddrlen);
ToFromPcStruct * ptfpc= (ToFromPcStruct *)buffer;

/* Is the packet one of ours */
if ((result >= sizeof(tfpc)) && (ptfpc->m_dwKeyValue==htonl(VERIFY_FROM_NDK_TO_PC)))
  {
   
   DWORD actual_addr=htonl(saddr.sin_addr.s_addr);
   printf("Configured for %d.%d.%d.%d Received from %d.%d.%d.%d\n",
		  ptfpc->m_cf.ip_Addr[0],ptfpc->m_cf.ip_Addr[1],ptfpc->m_cf.ip_Addr[2],ptfpc->m_cf.ip_Addr[3],
		  actual_addr >>24,
		  (actual_addr >>16 ) & 0xFF,
		  (actual_addr >>8 ) & 0xFF,
		  (actual_addr ) & 0xFF);

   char * cp=buffer+sizeof(tfpc)+1;
   printf("Device name = %s\n",cp);
   while (*cp) cp++;
   cp++;
   printf("Program name = %s\n",cp);
   if(pgm)
   {
   if(strcmp(cp,pgm)==0)
   {
    closesocket(sock);
	return actual_addr;
   }

   }
   else
   {
	 closesocket(sock);
	 return actual_addr;
   }

 }
 FD_ZERO(&readfd);
 FD_SET(sock,&readfd);
 tout.tv_sec=2;
 tout.tv_usec=0;
};


closesocket(sock);
return 0;

}


string GetNowFilename()
{

	time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    // Visit http://www.cplusplus.com/reference/clibrary/ctime/strftime/
    // for more information about date/time format
	sprintf(buf,"LOG%02d-%02d-%04d-%02d:%02d.net",tstruct.tm_mon,tstruct.tm_mday,tstruct.tm_year+1900,tstruct.tm_hour,tstruct.tm_min);
	string s=buf;
	return s;
}



int main(int argc, char ** argv)
{

WORD wVersionRequested = MAKEWORD(1,1);
WSADATA wsaData;
// Initialize WinSock and check the version
int nRet = WSAStartup(wVersionRequested, &wsaData);
if (wsaData.wVersion != wVersionRequested)
{	
	fprintf(stderr,"\n Wrong winsocket version\n");
	return -1;
}

printf("Trying to find NetBurner...");
DWORD addr=findNB("SBL2CAR");
if(addr)
{
	 SOCKET mySocket; 
printf("Found NB\n");
   // Create a TCP/IP stream socket
	mySocket = socket(AF_INET,				// Address family
					  SOCK_STREAM,			// Socket type
					  IPPROTO_TCP);			// Protocol
	if (mySocket == INVALID_SOCKET)
	{
		PRINTERROR("B: socket()");
		return -1;
	}
	sockaddr_in saddro;
	memset(&saddro,0,sizeof(saddro));
	saddro.sin_family=AF_INET;
	saddro.sin_port=htons(1000);
	saddro.sin_addr.s_addr =htonl(addr);
	
	FILE *fout    = fopen(GetNowFilename().c_str() , "wb" );  

	nRet = connect(mySocket,(LPSOCKADDR)&saddro,sizeof(struct sockaddr));	

	if (nRet == SOCKET_ERROR)
	{
		PRINTERROR("C: socket()");
		closesocket(mySocket);
		return -1;
	}
  	else
	{
   	int sv=200000;
	nRet=setsockopt(mySocket,SOL_SOCKET,SO_RCVBUF,(const char *)&sv, sizeof(sv));
	printf("Connected! X to exit\n");
	char rxc=' ';
	while(rxc!='X')
	{
    while(_kbhit())
		{
		fd_set readfd;
		FD_ZERO(&readfd);
		FD_SET(mySocket,&readfd);
		timeval tout;
		tout.tv_sec=2;
		tout.tv_usec=0;
		if(select (1,&readfd,NULL,NULL,&tout)>0)
			{static char retbuffer[10000];
			 nRet = recv(mySocket,retbuffer,10000,0);  
			 if(nRet>0)
				{
				 fwrite(retbuffer,1,nRet,fout);
				 printf("wrote %d\n",nRet);
				}
			}



		}
	rxc=_getch();
	}
	fclose(fout);
	closesocket(mySocket);
	}

}
return 0;
}

