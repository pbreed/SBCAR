

			 


typedef struct
{
short ax;
short ay; 
short az; 
short t; 
short gx;
short gy; 
short gz; 
short mx;
short my;
short mz;
WORD ReadingNum;
float fIhead;
float fMhead;
int GHeading;
}ImuRegisters;


extern volatile	ImuRegisters IMU_Result;

void ImuInit(BYTE prio, OS_SEM * pSem);




