typedef struct
{
long dest_lat;
long dest_lon;
long start_lat;
long start_lon;
double dlat;//Feet
double dlon;//Feet
double mbearing_sin;
double mbearing_cos;
double bearing;
} NavSegment;

typedef struct 
{
double dist; 	//Feet
double bearing;
double cross; //Feet Positive is too far right
bool bPassed;
}NavState;


void GetState(long clat, long clon, const NavSegment & navseg, NavState & result);
void SetupSegment(long slat, long slon, long elat, long elon, NavSegment & ns);
void LogRecord(NavState & s);


void E2NDCM (double lat, double lon, double cen[][3]);
void ConvertVeolcity(double eecfv[3],double ned[3], double cen[][3]);



#define LAT_SCALE  364566.9298 //Deg to feet
#define LONG_LAT_SCALE 0.036456693 //Count to feet


#define LON_SCALE  279501.3128 //Deg to Feet
#define LONG_LON_SCALE 0.027950131 //Count to feet



#define DECLINATION 8.916 





