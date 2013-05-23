#include "car.h"
#ifdef LITTLE_CAR
#include "ublxgps.cpp"
#elif BIG_CAR
#include "bd960.cpp"
#else
#error No Car defined
#endif



