#ifndef __SETTINGS_H
#define __SETTINGS_H


// User settings
// MOSFET driver type
//#define MP6531
#define FD6288

// Brushed mode
#define BRUSHED_MODE 0

// Vehicle mode: 1 = quad mode / eeprom load mode , 2 = crawler / thruster mode,  3 = rc car mode,  4 = like car mode but with auto reverse after stop  5 = no eeprom
#define VEHICLE_MODE 5

// Limit the max PWM inputMAX_SERVO_DEVIATION
#define MAX_SERVO_DEVIATION 90

// Stall protection on larger crafts/crawlers
#define STALL_PROTECTION 1

// Dead time for MOSFET driver -- not used right now
#define DEAD_TIME 60;


#endif
