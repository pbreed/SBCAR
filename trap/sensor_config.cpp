#include "predef.h" 
#include <ctype.h>
#include <basictypes.h>
#include <system.h>
#include <string.h>
#include "sensor_config.h"



sensor_saved_config SensorConfig;

#define DEF_VERSION (0x12340001)


void LoadSensorConfig()
{
sensor_saved_config * pcfg=(sensor_saved_config *)GetUserParameters();
if((pcfg->version==DEF_VERSION) && (pcfg->version_2==DEF_VERSION))
	{
	memcpy(&SensorConfig,pcfg,sizeof(SensorConfig));
    }	
	else
	{//Default values 
		SensorConfig.version=DEF_VERSION;
		for(int i=0; i<3; i++)
		{
		SensorConfig.mag_max[i]=128;          
		SensorConfig.mag_min[i]=-128;          
		SensorConfig.accel_zero[i]=0;       
		SensorConfig.default_gyro_zero[i]=0;
		}
		for(int i=0; i<4; i++)
		{
		  SensorConfig.servo_neg_lim[i]=-1.04;    
		  SensorConfig.servo_pos_lim[i]=0.96;    
		  SensorConfig.servo_mid[i]=-0.04;        
		}

		SensorConfig.rx_rc_zeros_el=-16;
		SensorConfig.rx_rc_gain_el=1/((530.0+542.0)*8.0);
		SensorConfig.rx_rc_zeros_al=16;
		SensorConfig.rx_rc_gain_al=1/((512.0+495.0)*8.0);
		SensorConfig.rx_rc_zeros_rd=-3*16;
		SensorConfig.rx_rc_gain_rd=1/((515.0+527.0)*8.0);
		SensorConfig.rx_rc_zeros_th=(16*(516-526))/2;
		SensorConfig.rx_rc_gain_th=1/((516.0+526.0)*8.0);

		SensorConfig.version_2=DEF_VERSION;	
	}//Defaults 


}
void SaveSensorConfig()
{
 SaveUserParameters(&SensorConfig,sizeof(SensorConfig));
}





