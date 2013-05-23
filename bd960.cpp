#include "predef.h" 
#include <basictypes.h>
#include <constants.h>
#include <ucos.h>
#include <ucosmcfc.h>
#include <serialirq.h>
#include <math.h>
#include "gps.h"


volatile GPS_READING GPS_Result;



static OS_SEM * pDataSem;




#define SM_TASK_STK (512)
#define SmOSSimpleTaskCreate(x,p) { static DWORD   func_##x_Stk[SM_TASK_STK] __attribute__( ( aligned( 4 ) ) ); OSTaskCreate(x,NULL,(void *)&func_##x_Stk[SM_TASK_STK],(void*)func_##x_Stk,p); }




void InitGpsSubSystem(WORD prio,OS_SEM * pSem)
{

	pDataSem=pSem;
//	SmOSSimpleTaskCreate(GpsTask,prio);

}



