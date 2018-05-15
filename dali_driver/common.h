// common.h
//
// This header contains common definitions
//
// Prepared: Motorola AB
//
// Functional level: Common
//
// Revision: R1A
//
// Rev Date Reason/description
// P1A 001023 Initial version
// P1B 010201 Added declaration for embedded ROM flash routines
// R1A 010212 Released version
#ifndef COMMON

#define COMMON
#define TRUE 1
#define FALSE 0

// Global definition
extern unsigned char address; // The DALI address
extern unsigned char command; // The DALI command
extern unsigned char answer; // The DALI answer
extern unsigned char error; // Stores the last error code

#define SRSR_ERROR 0x01 // Reset due to a hardware/software error

extern unsigned int flag; // Keep track of all events

#define NEW_DATA 0x01 // Set when new data is set
#define SEND_DATA 0x02 // Set when data is to be sent
#define ERROR_EVENT 0x04 // Set if an error has occurred
#define ANSWER_EVENT 0x08 // Set if an answer has arrived
#define DEMO_MODE 0x10 // Set if the demo mode is active

#endif