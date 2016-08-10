/* 
 * state_machine.h
 * header file for control state machine
*/


#ifndef STATE_MACHINE_H__
#define STATE_MACHINE_H__
#include <stdio.h>

void state_machine_run(void);

//states
void idle(void);
void init(void);
void seeking(void);
void connecting(void);
void connected(void);
void poweroff(void);
void lowbattery(void);
void syncing(void);
void checkcharge(void);
void charging(void);
void chargecomplete(void);
#endif
