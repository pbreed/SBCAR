void StartGpsProcess(BYTE prio);


typedef struct
{
 double lattitude; //Radians
 double longitude;//Radians
 double ht;		  //m
 unsigned long msec;
 unsigned short week;
 double ECEF_X;//m
 double ECEF_Y;//m
 double ECEF_Z;//m
 float hspeed;//m/sec
 float head; //radians
 float vv;	 //m/sec
 unsigned char flag1;
 unsigned char flag2;
 unsigned char nsat;
}CombinedTrimGps;

extern volatile CombinedTrimGps Gps_Result;


bool GpsValid(volatile CombinedTrimGps & g);

