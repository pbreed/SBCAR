
#include "predef.h"
#include <basictypes.h>         // Include for variable types
#include <constants.h>          // Include for constands like MAIN_PRIO
#include <system.h>             // Include for system functions
#include <sim.h>
#include <i2cmaster.h>
#include <ucos.h>
#include <pins.h>
#include <string.h>
#include "imu.h"
#include "pitr_sem.h"


#define MAGADDR (0x0C) //was 0x0c


void WriteReg(BYTE rg, BYTE val)
{
I2CStart( 0x68,false,2);
I2CSend(rg);
I2CSend(val);
I2CStop(20);
}

void I2CFirstThing()
{
WriteReg(0x6B,0x01); //PWR_MGMT_1 X gyro clock
WriteReg(0x37,0x0);	 //INT_PIN_CFG  Turn off I2C slave pass through
WriteReg(0x6A,0x0);	 //User control 
WriteReg(0x1b,0x0);	 //Gyro config 250 dps
WriteReg(0x1c,0x08); //ACCEL CONFIG 4 G
WriteReg(0x19,0x0);	 //SAMPLE RATE DIVIDER
WriteReg(0x37, 0x02); //Slave pass throught

OSTimeDly(2);
I2CStart(MAGADDR,false,2);
I2CSend(0x0A);
I2CSend(0x01);
I2CStop(20);
OSTimeDly(2);
}



volatile	ImuRegisters IMU_Result;
static ImuRegisters Local_IMU_Result; 
OS_SEM * LocalSem;
 
#define SM_TASK_STK (256)
#define SmOSSimpleTaskCreate(x,p) { static DWORD   func_##x_Stk[SM_TASK_STK] __attribute__( ( aligned( 4 ) ) ); OSTaskCreate(x,NULL,(void *)&func_##x_Stk[SM_TASK_STK],(void*)func_##x_Stk,p); }






BYTE rv,rv2;



void ImuRead(ImuRegisters & r, bool bDoMag)
{
I2CStart( 0x68,false,2);
I2CSend(0x3B);//59
I2CStop(20);

rv=I2CReadBuf(0x68, (PBYTE)&r, 14);

if(bDoMag)
{

I2CStart(MAGADDR,false,2);
I2CSend(0x03);
I2CStop(20);
rv2=I2CReadBuf(MAGADDR, (PBYTE)&r.mx, 6);

I2CStart(MAGADDR,false,2);
I2CSend(0x0A);
I2CSend(0x01);
I2CStop(20);
}
else
{
 r.mx=0;
 r.my=0;
 r.mz=0;
}

}


void IMU_Task(void * p)
{
OS_SEM MySem;
OSSemInit(&MySem,0);
PiterSem(&MySem, 200); 


	
while(1)
 {
   OSSemPend(&MySem,0);
   if ((Local_IMU_Result.ReadingNum %10)==0)
	  ImuRead(Local_IMU_Result,true);
   else
	   ImuRead(Local_IMU_Result,false);

   Local_IMU_Result.ReadingNum++;
   OSLock();
   memcpy((void*)&IMU_Result,&Local_IMU_Result,sizeof(IMU_Result));
   OSUnlock();
   if(LocalSem) OSSemPost(LocalSem);
 }
}



void ImuInit(BYTE prio, OS_SEM * pSem)
{
	I2CInit();

	CPU_Pins[27].function(CPUPIN27_SCL);
	CPU_Pins[28].function(CPUPIN28_SDA);

	CPU_Pins[79].function(CPUPIN79_UTXD2    );
	CPU_Pins[80].function(CPUPIN80_URXD2    );



    I2CFirstThing();
	LocalSem=pSem;
	SmOSSimpleTaskCreate(IMU_Task,prio);
}


