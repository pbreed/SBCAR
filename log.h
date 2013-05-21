void InitLog();


void LogMessage(const char * msg);



struct sensor_saved_config ; //forward

void LogRC(volatile DSM2_READING  &item);
void LogImu(volatile ImuRegisters  & item);
void LogGps(volatile GPS_READING  & gps);
void LogSmGps(volatile GPS_READING  & gps);
void LogConfig(sensor_saved_config &sc);
void LogMaxMin(short mgz,short mmx,short mmy,short ngz,short ngx,short ngy);

extern volatile DWORD LogCount; 

