void InitLog();


void LogMessage(const char * msg);

typedef struct {
double deg_heading;
double adj_heading;
double err;
double steer;
}__attribute__( ( packed ) ) SteerLoopMsg;



struct sensor_saved_config ; //forward

void LogRC(volatile DSM2_READING  &item);
void LogImu(volatile ImuRegisters  & item);
void LogGps(volatile GPS_READING  & gps);
void LogSmGps(volatile GPS_READING  & gps);
//void LogTGps(volatile BD960_GPS & gps);
void LogRecord(SteerLoopMsg & item);

void LogMaxMin(short mgz,short mmx,short mmy,short ngz,short ngx,short ngy);

extern volatile DWORD LogCount; 

