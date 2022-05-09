
#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>

#define BITVAL(val,bit) ((val >> bit) & 1)
#define COUNTOF(a) (sizeof(a)/sizeof(a[0]))

//#define SIMULATE_KEYS

#endif      //MAIN_H