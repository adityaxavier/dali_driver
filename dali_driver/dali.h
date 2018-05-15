// dali.h
//
// This header contains definitions for the dali module
//
// Prepared: Motorola AB
//
// Functional level: Hardware
//
// Revision: R1A
//
// Rev Date Reason/description
// P1A 001110 Initial version
// R1A 010212 Released version
#ifndef DALI
#define DALI

// Initialize
extern void dali_Init(void);
// Sends the DALI address and command on the dali port
extern void dali_SendData(void);
// Interrupt handler for incoming data on the dali port
extern void dali_Start(void);
// Interrupt handler for timebase module
extern void dali_Tick(void);

#endif