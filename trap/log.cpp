#include "car.h"
#ifdef LITTLE_CAR
#include "SerLog.cpp"
#elif BIG_CAR
#include "NetLog.cpp"
#else
#error No Car defined
#endif




