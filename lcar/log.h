void InitLog();


void LogMessage(const char * msg);

typedef struct {
float deg_heading;
float adj_heading;
float err;
float steer;
float dgain;
float sgain;
}__attribute__( ( packed ) ) SteerLoopMsg;


typedef struct {
DWORD min_stack;
int min_pri;
WORD nGPS;
WORD nIMU;
}HealthRecord;


struct sensor_saved_config ; //forward

void LogRC(volatile DSM2_READING  &item);
void LogImu(volatile ImuRegisters  & item);
void LogGps(volatile GPS_READING  & gps);
void LogSmGps(volatile GPS_READING  & gps);
//void LogTGps(volatile BD960_GPS & gps);
void LogRecord(SteerLoopMsg & item);

void LogMaxMin(short mgz,short mmx,short mmy,short ngz,short ngx,short ngy);

void LogRecord(HealthRecord & itme);
extern volatile DWORD LogCount; 

void LogServiceTask();


