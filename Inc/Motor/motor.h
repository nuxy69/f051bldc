#include "main.h"

extern int forward;
extern int bemf_counts;
extern int commutation_interval;
extern int advance_multiplier;
extern char comp_pwm; // TODO: needs rework so it becomes setting
extern char compit;
extern int zctimeout;
extern char prop_brake_active;
extern int sensorless;

void commutate();
void startMotor();
void forcedCommutation();
void pollingChangeCompInput();
void changeCompInput();
void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp);
void checkForZeroCross();
void playStartupTune();
void playInputTune();

void phaseAPWM();
void phaseAFLOAT();
void phaseALOW();

void phaseBPWM();
void phaseBFLOAT() ;
void phaseBLOW();

void phaseCPWM();
void phaseCFLOAT() ;
void phaseCLOW();

void comStep(int newStep);
void allOff();
void fullBrake();
void proBrake();
