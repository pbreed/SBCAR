
#include "predef.h" 
#include <ctype.h>
#include <basictypes.h>
#include <system.h>
#include <constants.h>
#include <ucos.h>
#include <ucosmcfc.h>
#include <sim.h>
#include <bsp.h>
#include <cfinter.h>
#include <pins.h>

#include "servo.h"
#include "car.h"

#define SERVO_ZERO  (75000)
#define SERVO_MIN  ( 50000)
#define SERVO_MAX  (100000)
#define SERVO_DELTA (25000)



extern "C"
{
   void SetIntc0( long func, int vector, int level, int prio );
   void SetIntc1( long func, int vector, int level, int prio );
}




volatile DWORD servo_value[4];

volatile int tog0;
volatile int tog1;
volatile int tog2;
volatile int tog3;
volatile DWORD icnt[4];


INTERRUPT(DTOUT0Isr,0x2700)
{
sim.timer[0].ter=3;

if(!tog0) 
	sim.timer[0]. trr=(1000000)-servo_value[0];
else
	sim.timer[0]. trr=servo_value[0]; 

tog0=tog0^1;
icnt[0]++;


}


INTERRUPT(DTOUT1Isr,0x2700)
{
sim.timer[1].ter=3;

if(!tog1) 
	sim.timer[1]. trr=(1000000)-servo_value[1];
else
	sim.timer[1]. trr=servo_value[1]; 

tog1=tog1^1;
icnt[1]++;
}

INTERRUPT(DTOUT2Isr,0x2700)
{
sim.timer[2].ter=3;

if(!tog2) 
	sim.timer[2]. trr=(1000000)-servo_value[2];
else
	sim.timer[2]. trr=servo_value[2]; 

tog2=tog2^1;
icnt[2]++;
}


INTERRUPT(DTOUT3Isr,0x2700)
{
sim.timer[3].ter=3;

if(!tog3) 
	sim.timer[3]. trr=(1000000)-servo_value[3];
else
	sim.timer[3]. trr=servo_value[3]; 

tog3=tog3^1;
icnt[3]++;
}





void InitServos()
{

#ifdef LITTLE_CAR
CPU_Pins[8].function(CPUPIN8_DTOUT0);
CPU_Pins[9].function(CPUPIN9_DTOUT1 );
CPU_Pins[13].function(CPUPIN13_DTOUT2);
CPU_Pins[14].function(CPUPIN14_DTOUT3);
#elif BIG_CAR
CPU_Pins[8].function(CPUPIN8_DTIN0);
CPU_Pins[9].function(CPUPIN9_DTOUT1 );
CPU_Pins[13].function(CPUPIN13_DTOUT2);
CPU_Pins[14].function(CPUPIN14_DTOUT3);



CPU_Pins[61].function(CPUPIN61_GPIO);   
CPU_Pins[62].function(CPUPIN62_GPIO);   
CPU_Pins[63].function(CPUPIN63_GPIO);   
CPU_Pins[64].function(CPUPIN64_GPIO);   
CPU_Pins[65].function(CPUPIN65_GPIO);   
CPU_Pins[66].function(CPUPIN66_GPIO);   
CPU_Pins[67].function(CPUPIN67_GPIO);   
CPU_Pins[68].function(CPUPIN68_GPIO);   

CPU_Pins[61].read();		      		
CPU_Pins[62].read();		      		
CPU_Pins[63].read();		      		
CPU_Pins[64].read();		      		
CPU_Pins[65].read();		      		
CPU_Pins[66].read();		      		
CPU_Pins[67].read();		      		
CPU_Pins[68].read();		      		
 

#else
#error No Car defined
#endif


	










servo_value[0]=SERVO_ZERO;
servo_value[1]=SERVO_ZERO;
servo_value[2]=SERVO_ZERO;
servo_value[3]=SERVO_ZERO;


sim.timer[0]. tmr=0x0022;  
sim.timer[0]. txmr=0; 
sim.timer[0]. ter=3;  
sim.timer[0]. trr=SERVO_ZERO; 
sim.timer[0]. tcr=0; 
sim.timer[0]. tcn=1; 

sim.timer[0].tmr=0x003B;  

SetIntc0((long)DTOUT0Isr,19,5,6);

sim.timer[1].tmr=0x0022;  
sim.timer[1].txmr=0; 
sim.timer[1].ter=3;  
sim.timer[1].trr=SERVO_ZERO; 
sim.timer[1].tcr=0; 
sim.timer[1].tcn=1; 

sim.timer[1].tmr=0x003B;  
SetIntc0((long)DTOUT1Isr,20,5,5);


sim.timer[2].tmr=0x0022;  
sim.timer[2].txmr=0; 
sim.timer[2].ter=3;  
sim.timer[2].trr=SERVO_ZERO; 
sim.timer[2].tcr=0; 
sim.timer[2].tcn=1; 

sim.timer[2].tmr=0x003B;  
SetIntc0((long)DTOUT2Isr,21,5,4);


sim.timer[3].tmr=0x0022;  
sim.timer[3].txmr=0; 
sim.timer[3].ter=3;  
sim.timer[3].trr=SERVO_ZERO; 
sim.timer[3].tcr=0; 
sim.timer[3].tcn=1; 

sim.timer[3].tmr=0x003B;  
SetIntc0((long)DTOUT3Isr,22,5,3);

}




void RawSetServo(int ch,float v) //Limits -1.0 to +1.0
{
	if ((ch<0) || (ch>3)) return;
	if(v<=-1.0) servo_value[ch]=SERVO_MIN;
	else
	if(v>=1.0) servo_value[ch]=SERVO_MAX;
	else
	if(v==0.0)servo_value[ch]=SERVO_ZERO;
	else
	{
	v*=SERVO_DELTA;
	v+=SERVO_ZERO;
	servo_value[ch]=(DWORD)v;
    }
}




void SetServo(int ch,float f)  //-1.0 to 1.0
{
	 RawSetServo(ch,f);
}

