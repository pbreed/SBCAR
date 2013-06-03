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

#define LogStart(id ,name) BYTE tid=id;   StartItemIntro(id,sizeof(item),name); 
#define LogElement(x,y) ShowElement(tid,&item,&item.x,item.x,y)





void LogRC(volatile DSM2_READING &item)
{
	LogRawRecord(DSM2_TYPE,(const unsigned char *)&item,sizeof(item)); 

}



void LogImu(volatile ImuRegisters  & item)
{
 LogRawRecord(IMU_TYPE,(const unsigned char *)&item,sizeof(item)); 
}


void LogGps(volatile GPS_READING  &item)
{
	LogRawRecord(GPS_TYPE,(const unsigned char *)&item,sizeof(item));
}


void LogSmGps(volatile GPS_READING  & gps)
{
SMGPS_READING item;

item.LON      =gps.LON;     
item.LAT      =gps.LAT;     
item.GSpeed   =gps.GSpeed;  
item.Heading  =gps.Heading; 
item.numSV    =gps.numSV;   
item.GPSfix   =gps.GPSfix;  
item. Hacc    =gps.Hacc;   
LogRawRecord(SGPS_TYPE,(const unsigned char *)&item,sizeof(item));

}



void LogConfig(volatile sensor_saved_config &item)
{
	LogRawRecord(CFG_TYPE,(const unsigned char *)&item,sizeof(item));

}

struct mmax
{
	short mgz;
	short mmx;
	short mmy;
	short ngz;
	short nmx;
	short nmy;
};



void LogMaxMin(short mgz,short mmx,short mmy,short ngz,short nmx,short nmy)
{
  mmax item;
 item.mgz=mgz;
 item.mmx=mmx;
 item.mmy=mmy;
 item.ngz=ngz;
 item.nmx=nmx;
 item.nmy=nmy;
 LogRawRecord(MAX_TYPE,(const unsigned char *)&item,sizeof(item));
}



void ShowConfig(sensor_saved_config &item)
{
	LogStart(CFG_TYPE,"Config");
	LogElement(mag_max[0]             ,"mag_max[0]");
    LogElement(mag_max[1]             ,"mag_max[1]");
	LogElement(mag_max[2]             ,"mag_max[2]");
	LogElement(mag_min[0]             ,"mag_min[0]");
	LogElement(mag_min[1]             ,"mag_min[1]");
 	LogElement(mag_min[2]             ,"mag_min[2]");
	LogElement(accel_zero[0]          ,"accel_zero[0]");
	LogElement(accel_zero[1]          ,"accel_zero[1]");
 	LogElement(accel_zero[2]          ,"accel_zero[2]");
	LogElement(default_gyro_zero[0]   ,"default_gyro_zero[0]");
    LogElement(default_gyro_zero[1]   ,"default_gyro_zero[1]");
	LogElement(default_gyro_zero[2]   ,"default_gyro_zero[2]");
	LogElement(servo_neg_lim[0]       ,"servo_neg_lim[0]");    
	LogElement(servo_neg_lim[1]       ,"servo_neg_lim[1]");    
 	LogElement(servo_neg_lim[2]       ,"servo_neg_lim[2]");    
 	LogElement(servo_neg_lim[3]       ,"servo_neg_lim[3]");    
	LogElement(servo_pos_lim[0]       ,"servo_pos_lim[0]");    
	LogElement(servo_pos_lim[1]       ,"servo_pos_lim[1]");    
	LogElement(servo_pos_lim[2]       ,"servo_pos_lim[2]");    
	LogElement(servo_pos_lim[3]       ,"servo_pos_lim[3]");    
	LogElement(servo_mid[0]           ,"servo_mid[0]");        
	LogElement(servo_mid[1]           ,"servo_mid[1]");        
	LogElement(servo_mid[2]           ,"servo_mid[2]");        
	LogElement(servo_mid[3]           ,"servo_mid[3]");        
}


void ShowMMRec(mmax & item)
{
	LogStart(MAX_TYPE,"MAX");
	LogElement(mgz,"MGZ");
	LogElement(mmx,"MMX");
	LogElement(mmy,"MMY");
	LogElement(ngz,"NGZ");
	LogElement(nmx,"NMX");
	LogElement(nmy,"NMY");
}

void ShowImuRec(ImuRegisters & item)
{
	LogStart(IMU_TYPE,"IMU");
	LogElement(ax,"Ax");
	LogElement(ay,"Ay");
	LogElement(az,"Az");
	LogElement(gx,"Gx");
	LogElement(gy,"Gy");
	LogElement(gz,"Gz");
	LogElement(mx,"Mx");
	LogElement(my,"My");
	LogElement(mz,"Mz");
	LogElement(t,"T");
	LogElement(ReadingNum,"RN");
	LogElement(fIhead,"IH");
	LogElement(fMhead,"MH");
	LogElement(fgIHead,"GI");
	LogElement(GHeading,"GH");
	LogElement(odo,"Od");
}


void ShowSmGps(SMGPS_READING & item)
{
	LogStart(SGPS_TYPE,"SGPS");
	LogElement(LON    , "LON");          // 1e-07 deg Longitude                        
	LogElement(LAT    , "LAT");          // 1e-07 deg Latitude                         
	LogElement(numSV  , "numSV");        // Number of SVs used in navigation solution 
	LogElement(GSpeed , "GSpeed");       // cm/s  Ground Speed (2-D)                   
	LogElement(Heading, "Heading");       // 1e-05 deg  Heading 2-D                     
	LogElement(Hacc   , "Hacc");         // mm Horizontal Accuracy Estimate           
}

void ShowGps(GPS_READING  &item)
{
  LogStart(GPS_TYPE,"GPS");
  LogElement(  itow   , "itow");         // ms GPS Millisecond Time of Week            
  LogElement(ECEF_X , "ECEF_X");       // cm ECEF X coordinate                       
  LogElement(ECEF_Y , "ECEF_Y");       // cm ECEF Y coordinate                       
  LogElement(ECEF_Z , "ECEF_Z");       // cm ECEF Z coordinate                       
  LogElement(PAcc   , "PAcc");         // cm 3D Position Accuracy Estimate           
  LogElement(ECEFVX , "ECEFVX");       // cm/s ECEF X velocity                       
  LogElement(ECEFVY , "ECEFVY");       // cm/s ECEF Y velocity                       
  LogElement(ECEFVZ , "ECEFVZ");       // cm/s ECEF Z velocity                       
  LogElement(LON    , "LON");          // 1e-07 deg Longitude                        
  LogElement(LAT    , "LAT");          // 1e-07 deg Latitude                         
  LogElement(HEIGHT , "HEIGHT");       // mm Height above Ellipsoid                  
  LogElement(HMSL   , "HMSL");         // mm Height above mean sea level             
  LogElement(VEL_N  , "VEL_N");        // cm/s  NED north velocity                   
  LogElement(VEL_E  , "VEL_E");        // cm/s  NED east velocity                    
  LogElement(VEL_D  , "VEL_D");        // cm/s  NED down velocity                    
  LogElement(Speed  , "Speed");        // cm/s  Speed (3-D)                          
  LogElement(GSpeed , "GSpeed");       // cm/s  Ground Speed (2-D)                   
  LogElement(Heading, "Heading");       // 1e-05 deg  Heading 2-D                     
  LogElement(SAcc   , "SAcc");         // cm/s  Speed Accuracy Estimate             
  LogElement(CAcc   , "CAcc");         // deg  Course / Heading Accuracy Estimate   
  LogElement(Hacc   , "Hacc");         // mm Horizontal Accuracy Estimate           
  LogElement(Vacc   , "Vacc");         // mm Vertical Accuracy Estimate             
  LogElement(numSV  , "numSV");        // Number of SVs used in navigation solution 
  LogElement(GPSfix , "GPSfix");       // GPSfix Type, range 0..6                   
  LogElement(week   , "week");         // GPS week                                  
  LogElement(ReadingNum,"RN");
}


void ShowRC(DSM2_READING &item)
{
	LogStart(DSM2_TYPE,"DSM2");
	LogElement(val[0],"V0");
	LogElement(val[1],"V1");
	LogElement(val[2],"V2");
	LogElement(val[3],"V3");
	LogElement(val[4],"V4");
	LogElement(val[5],"V5");
	LogElement(val[6],"V6");
	LogElement(val[7],"V7");
	LogElement(ReadingNum,"RN");

}






void ShowFileVer(const char * cp)
{
PutRawByte(LOG_REC_START); 
PutRawByte(MSG_TYPE );
while(*cp) PutEscapedByte(*cp++);
PutRawByte(LOG_REC_END); 
}

/*void ShowTGps(BD960_GPS & item)
{
	LogStart(TGPS_TYPE,"TGPS");
	LogElement(lattitude,"lat" );
	LogElement(longitude,"lon" );
	LogElement(ht       ,"ht" );
	LogElement( r8flags ,"r8f" );
	LogElement(hspeed    ,"hspd" );
	LogElement(head      ,"head" );
	LogElement(vv        ,"vv" );
	LogElement(rms_error ,"rmser" );
	LogElement(sigma_east ,"s_e");
	LogElement(sigma_north,"s_n");
	LogElement(covaren    ,"covar");
	LogElement(sigma_up   ,"s_up");
	LogElement(semi_major ,"s_majr");
	LogElement(semi_minor ,"s_minr");
	LogElement(orientaion ,"orient");
	LogElement(variance   ,"var");
	LogElement(flag1,"F1");
	LogElement(flag2,"F2");

}
*/


void ShowSteer(SteerLoopMsg & item)
{
	LogStart(STEER_TYPE,"STEER");
	LogElement( 	deg_heading,"CHD");
	LogElement( 	adj_heading,"THD");
	LogElement( 	err,"ERR"    );    
	LogElement( 	steer,"STEER");      
	LogElement(     dgain,"DGain");
	LogElement(     sgain,"SGain");
}


void ShowHealth(HealthRecord &item)
{
	LogStart(HEALTH_TYPE,"HEALTH");
	 LogElement(min_stack,"MIN Stack");
	 LogElement(min_pri,"MIN PRI");
	 LogElement(nGPS,"nGPS");
	 LogElement(nIMU,"nIMU");

}

void ShowNavState(NavState &item)
{
	LogStart(NAV_TYPE,"NAV");
	LogElement(dist,"Dist");
	LogElement(bearing,"Bear");
	LogElement(cross,"Cross");
	LogElement(bPassed,"pass");
}


void ShowTrap(Captured_Trap &item)
{
	LogStart(TRAP_TYPE,"TRAP");
		LogElement(vec,"VEC");
		LogElement(pc,"PC");
		LogElement(a7,"a7");
		LogElement(prio,"VEC");
}


void LogTrap()
{
LogRawRecord(TRAP_TYPE,(const unsigned char *)&cap_trap,sizeof(cap_trap)); 
cap_trap.vec=0;			   
cap_trap.pc=0;
cap_trap.a7=0;
cap_trap.prio=0;
}

void DumpRecords()
{
BYTE item[16];
ShowConfig((*((sensor_saved_config* )&item))) ;
//ShowMMRec((*((mmax  *)& item)));
ShowImuRec((*((ImuRegisters * )& item))) ;
ShowGps((*((GPS_READING  * )&item)));
ShowRC((*((DSM2_READING *)&item)));
ShowSmGps(((*(SMGPS_READING *)& item))); 
//ShowTGps(((*(BD960_GPS *)&item)));

ShowSteer(((*(SteerLoopMsg *)&item)));
ShowHealth(((*(HealthRecord *)&item)));
ShowNavState(((*(NavState *)&item)));
ShowTrap(((* (Captured_Trap *)&item)));
FileReporter::DumpList();
LogConfig(SensorConfig);
LogTrap();
}




FileReporter* FileReporter::pRoot;

void FileReporter::DumpList()
{
 FileReporter * pF =FileReporter::pRoot;
 while(pF)
 {
	 ShowFileVer(pF->m_pName);
	 pF=pF->m_pNext;
 }

}

void FileReporter::ShowList()
{
 FileReporter * pF =FileReporter::pRoot;
 while(pF)
 {
	 iprintf("%s\r\n",pF->m_pName);
	 pF=pF->m_pNext;
 }

}

//void LogTGps(volatile BD960_GPS & item)
//{
//	LogRawRecord(TGPS_TYPE,(const unsigned char *)&item,sizeof(item));
//
//}



void LogRecord(SteerLoopMsg & item)
{
   LogRawRecord(STEER_TYPE,(const unsigned char *)&item,sizeof(item));
}

void LogRecord(HealthRecord & item)
{
   LogRawRecord(HEALTH_TYPE,(const unsigned char *)&item,sizeof(item));
}





void LogRecord(NavState & item)
{
	LogRawRecord(NAV_TYPE,(const unsigned char *)&item,sizeof(item));

}



