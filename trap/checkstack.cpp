#include <predef.h>
#include <constants.h>
#include <basictypes.h>
#include <ucos.h>
#include <ucosmcfc.h>
#include <bsp.h>
#include <stdio.h>

extern DWORD   OSIdleTaskStk[IDLE_STK_SIZE] __attribute__( ( aligned( 4 ) ) );
extern OS_TCB OSTCBTbl[OS_MAX_TASKS];


#ifdef UCOS_STACKCHECK


DWORD CheckAStack(int &index ,int & prio)
{
index++;
while(1)
{
 while(index<OS_MAX_TASKS)
 {	
 if(OSTCBTbl[index].OSTCBPrio  )
 {
     long n = 0;
             long *cp = OSTCBTbl[index].OSTCBStkBot;
             while ( (DWORD)(*( cp++ )) == 0xDEADBEEF )
             {
                n++;
             }
	prio=OSTCBTbl[index].OSTCBPrio;
 	return 4*n;
 }
 else
 index++;
 }//Inner while
 index=0; //wrap
}//while 1
return 0xFFFF;
}

#endif


