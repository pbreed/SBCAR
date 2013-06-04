struct DSM2_READING
{
/*short nElevator;
short nAlieron;
short nRudder;
short nThrottle;
short nGear;
short nAux2;
*/
unsigned short val[8];
WORD  ReadingNum;
};

extern volatile bool bMode;
extern volatile int nMode;
extern volatile bool bGear;


float DSM_Con(unsigned short v);
extern volatile DSM2_READING DSM2_Result;


void InitIDSM2SubSystem(WORD prio,OS_SEM * pDataSem);

