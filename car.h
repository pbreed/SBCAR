#define BIG_CAR (1)


#ifdef LITTLE_CAR
  #define DSMUART (2)
  #define LOG_UART (0)
  #define GPS_UART (1)
  
#elif BIG_CAR
  #define DSMUART (1)
  #define LOG_UART (0)

#else
#error No Car defined
#endif





